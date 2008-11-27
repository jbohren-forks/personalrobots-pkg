/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//

#include"navfn.h"


//
// create nav fn buffers 
//

NavFn::NavFn(int xs, int ys)
{  
  // create cell arrays
  obsarr = costarr = NULL;
  potarr = NULL;
  pending = NULL;
  setNavArr(xs,ys);

  // priority buffers
  pb1 = new int[PRIORITYBUFSIZE];
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];
  
  priInc = COST_NEUTRAL;	// <= COST_NEUTRAL is not breadth-first

  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // display function
  displayFn = NULL;
  displayInt = 0;
}


NavFn::~NavFn()
{
  if(obsarr)
    delete[] obsarr;
  if(costarr)
    delete[] costarr;
  if(potarr)
    delete[] potarr;
  if(pending)
    delete[] pending;
}


//
// set goal, start positions for the nav fn
//

void
NavFn::setGoal(int *g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  printf("[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int *g)
{
  start[0] = g[0];
  start[1] = g[1];
  printf("[NavFn] Setting start to %d,%d\n", start[0], start[1]);
}

//
// Set/Reset map size
//

void
NavFn::setNavArr(int xs, int ys)
{
  printf("[NavFn] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx*ny;

  if(obsarr)
    delete[] obsarr;
  if(costarr)
    delete[] costarr;
  if(potarr)
    delete[] potarr;
  if(pending)
    delete[] pending;

  obsarr = new COSTTYPE[ns];	// obstacles, 255 is obstacle
  memset(obsarr, 0, ns*sizeof(COSTTYPE));
  costarr = new COSTTYPE[ns]; // cost array, 2d config space
  memset(costarr, 0, ns*sizeof(uint16_t));
  potarr = new float[ns];	// navigation potential array
  memset(potarr, 0, ns*sizeof(float));
  pending = new bool[ns];
  memset(pending, 0, ns*sizeof(bool));
}


void
NavFn::setObs()
{
#if 0
  // set up a simple obstacle
  printf("[NavFn] Setting simple obstacle\n");
  int xx = nx/3;
  for (int i=ny/3; i<ny; i++)
    costarr[i*nx + xx] = COST_OBS;
  xx = 2*nx/3;
  for (int i=ny/3; i<ny; i++)
    costarr[i*nx + xx] = COST_OBS;

  xx = nx/4;
  for (int i=20; i<ny-ny/3; i++)
    costarr[i*nx + xx] = COST_OBS;
  xx = nx/2;
  for (int i=20; i<ny-ny/3; i++)
    costarr[i*nx + xx] = COST_OBS;
  xx = 3*nx/4;
  for (int i=20; i<ny-ny/3; i++)
    costarr[i*nx + xx] = COST_OBS;
#endif
}


// inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
			   costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
                         { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
			   costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
                         { nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
			   costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
                         { overP[overPe++]=n; pending[n]=true; }}


// Set up navigation potential arrays for new propagation

void
NavFn::setupNavFn(bool keepit)
{
  // reset values in propagation arrays
  float *pp = potarr;
  COSTTYPE *pc = costarr;
  for (int i=0; i<ns; i++)
    {
      *pp++ = POT_HIGH;
      if (!keepit) *pc = COST_NEUTRAL;
      pc++;
    }

  // outer bounds of cost array
  pc = costarr;
  for (int i=0; i<nx; i++)
    *pc++ = COST_OBS;
  pc = costarr + (ny-1)*nx;
  for (int i=0; i<nx; i++)
    *pc++ = COST_OBS;
  pc = costarr;
  for (int i=0; i<ny; i++, pc+=nx)
    *pc = COST_OBS;
  pc = costarr + nx - 1;
  for (int i=0; i<ny; i++, pc+=nx)
    *pc = COST_OBS;

  // priority buffers
  curT = COST_OBS;
  curP = pb1; 
  curPe = 0;
  nextP = pb2;
  nextPe = 0;
  overP = pb3;
  overPe = 0;
  memset(pending, 0, ns*sizeof(bool));

  // set goal
  int k = goal[0] + goal[1]*nx;
  initCost(k,0);

  // find # of obstacle cells
  pc = costarr;
  int ntot = 0;
  for (int i=0; i<ns; i++, pc++)
    {
      if (*pc >= COST_OBS)
	ntot++;			// number of cells that are obstacles
    }
  nobs = ntot;
}


// initialize a goal-type cost for starting propagation

void
NavFn::initCost(int k, float v)
{
  potarr[k] = v;
  push_cur(k+1);
  push_cur(k-1);
  push_cur(k-nx);
  push_cur(k+nx);
}


// 
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value 
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void
NavFn::updateCell(int n)
{
  // get neighbors
  float u,d,l,r;
  l = potarr[n-1];
  r = potarr[n+1];		
  u = potarr[n-nx];
  d = potarr[n+nx];
  //  printf("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n", 
  //	 potarr[n], l, r, u, d);
  //  printf("[Update] cost: %d\n", costarr[n]);

  // find lowest, and its lowest neighbor
  float ta, tc;
  if (l<r) tc=l; else tc=r;
  if (u<d) ta=u; else ta=d;

  // do planar wave update
  if (costarr[n] < COST_OBS)	// don't propagate into obstacles
    {
      float hf = (float)costarr[n]; // traversability factor
      float dc = tc-ta;		// relative cost between ta,tc
      if (dc < 0) 		// ta is lowest
	{
	  dc = -dc;
	  ta = tc;
	}

      // calculate new potential
      float pot;
      if (dc >= hf)		// if too large, use ta-only update
	pot = ta+hf;
      else			// two-neighbor interpolation update
	{
	  // use quadratic approximation
	  // might speed this up through table lookup, but still have to 
	  //   do the divide
	  float d = dc/hf;
	  float v = -0.2301*d*d + 0.5307*d + 0.7040;
	  pot = ta + hf*v;
	}

      //      printf("[Update] new pot: %d\n", costarr[n]);

      // now add affected neighbors to priority blocks
      if (pot < potarr[n])
	{
	  float le = INVSQRT2*(float)costarr[n-1];
	  float re = INVSQRT2*(float)costarr[n+1];
	  float ue = INVSQRT2*(float)costarr[n-nx];
	  float de = INVSQRT2*(float)costarr[n+nx];
	  potarr[n] = pot;
	  pot += (float)0.01;
	  if (pot < curT)	// low-cost buffer block 
	    {
	      if (l > pot+le) push_next(n-1);
	      if (r > pot+re) push_next(n+1);
	      if (u > pot+ue) push_next(n-nx);
	      if (d > pot+de) push_next(n+nx);
	    }
	  else
	    {
	      if (l > pot+le) push_over(n-1);
	      if (r > pot+re) push_over(n+1);
	      if (u > pot+ue) push_over(n-nx);
	      if (d > pot+de) push_over(n+nx);
	    }
	}

    }

}



//
// main propagation function
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFn(int cycles, bool atStart)	
{
  int nwv = 0;			// max priority block size
  int nc = 0;			// number of cells put into priority blocks
  int cycle = 0;		// which cycle we're on

  for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
    {
      // 
      if (curPe == 0 && nextPe == 0) // priority blocks empty
	break;

      // stats
      nc += curPe;
      if (curPe > nwv)
	nwv = curPe;

      // reset pending flags on current priority buffer
      int *pb = curP;
      int i = curPe;			
      while (i-- > 0)		
        pending[*(pb++)] = false;
		
      // process current priority buffer
      pb = curP; 
      i = curPe;
      while (i-- > 0)		
	updateCell(*pb++);

      if (displayInt > 0 &&  (cycle % displayInt) == 0)
	displayFn(this);

      // swap priority blocks curP <=> nextP
      curPe = nextPe;
      nextPe = 0;
      pb = curP;		// swap buffers
      curP = nextP;
      nextP = pb;

      // see if we're done with this priority level
      if (curPe == 0)
        {
          curT += priInc;	// increment priority threshold
	  curPe = overPe;	// set current to overflow block
	  overPe = 0;
          pb = curP;		// swap buffers
          curP = overP;
          overP = pb;
        }

      // check if we've hit the Start cell
      if (atStart)
	{}			// TBD

    }

  printf("[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n", 
	       cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

  if (cycle < cycles) return true; // finished up here
  else return false;
}



//
// display function setup
// <n> is the number of cycles to wait before displaying,
//     use 0 to turn it off

void
NavFn::display(void fn(NavFn *nav), int n)
{
  displayFn = fn;
  displayInt = n;
}
