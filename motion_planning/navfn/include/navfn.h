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

#ifndef _NAVFN_H
#define _NAVFN_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

// cost defs
#define COST_OBS 254		// Conor uses 255 and 254 for forbidden regions
#define COST_NEUTRAL 50
#define COSTTYPE uint16_t

// potential defs
#define POT_HIGH 1.0e10		// unassigned cell potential

// priority buffers
#define PRIORITYBUFSIZE 10000


//
// Navigation function class
// Holds buffers for costmap, navfn map
// Maps are pixel-based
// Origin is upper left, x is right, y is down
//

class NavFn
{
 public:
  // constructor
  NavFn(int nx, int ny);	// size of map
  ~NavFn();

  void setNavArr(int nx, int ny); // sets or resets the size of the map
  int nx, ny, ns;		// size of grid, in pixels

  // cell arrays
  COSTTYPE *obsarr;		// obstacle array, to be expanded to cost array
  COSTTYPE *costarr;		// cost array in 2D configuration space
  float   *potarr;		// potential array, navigation function potential
  bool    *pending;		// pending cells during propagation
  int nobs;			// number of obstacle cells

  // block priority buffers
  int *pb1, *pb2, *pb3;		// storage buffers for priority blocks
  int *curP, *nextP, *overP;	// priority buffer block ptrs
  int curPe, nextPe, overPe; // end points of arrays

  // block priority thresholds
  float curT;			// current threshold
  float priInc;			// priority threshold increment

  // goal and start positions
  void setGoal(int *goal);	
  void setStart(int *start);	
  int goal[2];
  int start[2];
  void initCost(int k, float v); // initialize cell <k> with cost <v>, for propagation

  // simple obstacle for testing
  void setObs();

  // propagation
  void updateCell(int n);	// updates the cell at index i
  void setupNavFn(bool keepit = false); // resets all nav fn arrays for propagation
  bool propNavFn(int cycles, bool atStart = false); // run propagation for <cycles> iterations,
                                            // or until Start is reached

  // display callback
  void display(void fn(NavFn *nav), int n = 100);
  int displayInt;
  void (*displayFn)(NavFn *nav);
};


#endif  // NAVFN
