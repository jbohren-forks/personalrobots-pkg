/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "../../headers.h"


#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif



//-------------------problem specific and local functions---------------------


static unsigned int inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

//examples of hash functions: map state coordinates onto a hash value
//#define GETHASHBIN(X, Y) (Y*WIDTH_Y+X) 
//here we have state coord: <X1, X2, X3, X4>
unsigned int EnvironmentNAV3DDYN::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{

  return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)) & (EnvNAV3DDYN.HashTableSize-1);
}


EnvNAV3DDYNHashEntry_t* EnvironmentNAV3DDYN::GetHashEntry(int X, int Y, int Theta)
{

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif
  
  int binid = GETHASHBIN(X, Y, Theta);
  
#if NAV3DDYN_DEBUG
  if ((int)EnvNAV3DDYN.Coord2StateIDHashTable[binid].size() > 500)
    {
      printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
	     binid, X, Y, EnvNAV3DDYN.Coord2StateIDHashTable[binid].size());


      
      PrintHashTableHist();		
    }
#endif

  //iterate over the states in the bin and select the perfect match
  for(int ind = 0; ind < (int)EnvNAV3DDYN.Coord2StateIDHashTable[binid].size(); ind++)
    {
      if( EnvNAV3DDYN.Coord2StateIDHashTable[binid][ind]->X == X 
	  && EnvNAV3DDYN.Coord2StateIDHashTable[binid][ind]->Y == Y
	  && EnvNAV3DDYN.Coord2StateIDHashTable[binid][ind]->Theta == Theta)
	{
#if TIME_DEBUG
	  time_gethash += clock()-currenttime;
#endif
	  return EnvNAV3DDYN.Coord2StateIDHashTable[binid][ind];
	}
    }

#if TIME_DEBUG	
  time_gethash += clock()-currenttime;
#endif
  
  return NULL;	  
}

void EnvironmentNAV3DDYN::PrintHashTableHist()
{
  int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;
  
  for(int  j = 0; j < EnvNAV3DDYN.HashTableSize; j++)
    {
      if((int)EnvNAV3DDYN.Coord2StateIDHashTable[j].size() == 0)
	s0++;
      else if((int)EnvNAV3DDYN.Coord2StateIDHashTable[j].size() < 50)
	s1++;
      else if((int)EnvNAV3DDYN.Coord2StateIDHashTable[j].size() < 100)
	s50++;
      else if((int)EnvNAV3DDYN.Coord2StateIDHashTable[j].size() < 200)
	s100++;
      else if((int)EnvNAV3DDYN.Coord2StateIDHashTable[j].size() < 300)
	s200++;
      else if((int)EnvNAV3DDYN.Coord2StateIDHashTable[j].size() < 400)
	s300++;
      else
	slarge++;
    }
  printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
	 s0,s1, s50, s100, s200,s300,slarge);
}

//--------------static helper functions -------------------------
static int EuclideanDistance(int X1, int Y1, int X2, int Y2)
{
  int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
  double dist = sqrt((double)sqdist);

  return (int)(NAV3DDYN_COSTMULT*dist);
  
}

//-----------interface with outside functions---------------------
const EnvNAV3DDYNConfig_t* EnvironmentNAV3DDYN::GetEnvNavConfig() {
  return &EnvNAV3DDYNCfg;
}

bool EnvironmentNAV3DDYN::InitializeEnv(const char* sEnvFile)
{
  
  FILE* fCfg = fopen(sEnvFile, "r");
  if(fCfg == NULL)
    {
      printf("ERROR: unable to open %s\n", sEnvFile);
      exit(1);
    }
  ReadConfiguration(fCfg);
  
  if(NAV3DDYN_SHRINK_FOOTPRINT){
    DiscretizeAndShrinkFootprintBoundary();
  }


  InitGeneral();
  
  
  return true;
}

bool EnvironmentNAV3DDYN::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta,
					double goalx, double goaly, double goaltheta,
					double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double nominalvel_mpersecs, double timetoturnoneunitinplace_secs)
{
  
  SetConfiguration(width, height,
		   mapdata,
		   CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, NAV3DDYN_THETADIRS),
		   CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, NAV3DDYN_THETADIRS),
		   cellsize_m, nominalvel_mpersecs, timetoturnoneunitinplace_secs, perimeterptsV);
  
  if(NAV3DDYN_SHRINK_FOOTPRINT){
    DiscretizeAndShrinkFootprintBoundary();
  }


  InitGeneral();
  
  return true;
  
}

void EnvironmentNAV3DDYN::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta,
					int goalx, int goaly, int goaltheta,
					double cellsize_m, double nominalvel_mpersecs, double timetoturnoneunitinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV) {

  EnvNAV3DDYNCfg.EnvWidth_c = width;
  EnvNAV3DDYNCfg.EnvHeight_c = height;
  EnvNAV3DDYNCfg.StartX_c = startx;
  EnvNAV3DDYNCfg.StartY_c = starty;
  EnvNAV3DDYNCfg.StartTheta = starttheta;
 
  if(EnvNAV3DDYNCfg.StartX_c < 0 || EnvNAV3DDYNCfg.StartX_c >= EnvNAV3DDYNCfg.EnvWidth_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV3DDYNCfg.StartY_c < 0 || EnvNAV3DDYNCfg.StartY_c >= EnvNAV3DDYNCfg.EnvHeight_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV3DDYNCfg.StartTheta < 0 || EnvNAV3DDYNCfg.StartTheta >= NAV3DDYN_THETADIRS) {
    printf("ERROR: illegal start coordinates for theta\n");
    exit(1);
  }
  
  EnvNAV3DDYNCfg.EndX_c = goalx;
  EnvNAV3DDYNCfg.EndY_c = goaly;
  EnvNAV3DDYNCfg.EndTheta = goaltheta;

  if(EnvNAV3DDYNCfg.EndX_c < 0 || EnvNAV3DDYNCfg.EndX_c >= EnvNAV3DDYNCfg.EnvWidth_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV3DDYNCfg.EndY_c < 0 || EnvNAV3DDYNCfg.EndY_c >= EnvNAV3DDYNCfg.EnvHeight_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV3DDYNCfg.EndTheta < 0 || EnvNAV3DDYNCfg.EndTheta >= NAV3DDYN_THETADIRS) {
    printf("ERROR: illegal goal coordinates for theta\n");
    exit(1);
  }

  EnvNAV3DDYN2Dpt_t pt;
  for(int i=0; i<robot_perimeterV.size(); i++){
    pt.x = robot_perimeterV[i].x;
    pt.y = robot_perimeterV[i].y;
    EnvNAV3DDYNCfg.FootprintPolygon.push_back(pt);
  }

  EnvNAV3DDYNCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAV3DDYNCfg.cellsize_m = cellsize_m;
  EnvNAV3DDYNCfg.timetoturnoneunitinplace_secs = timetoturnoneunitinplace_secs;

  //allocate the 2D environment
  EnvNAV3DDYNCfg.Grid2D = new char* [EnvNAV3DDYNCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAV3DDYNCfg.EnvWidth_c; x++) {
    EnvNAV3DDYNCfg.Grid2D[x] = new char [EnvNAV3DDYNCfg.EnvHeight_c];
  }
  
  //environment:
  for (int y = 0; y < EnvNAV3DDYNCfg.EnvHeight_c; y++) {
    for (int x = 0; x < EnvNAV3DDYNCfg.EnvWidth_c; x++) {
      char cval = mapdata[x+y*width];
      if(cval == 1) {
	EnvNAV3DDYNCfg.Grid2D[x][y] = 1;
      } else {
	EnvNAV3DDYNCfg.Grid2D[x][y] = 0;
      }
    }
  }
}

void EnvironmentNAV3DDYN::DiscretizeAndShrinkFootprintBoundary(){
  
  if(EnvNAV3DDYNCfg.FootprintPolygon.size() <= 1){
    return;
  }

  EnvNAV3DDYN2Dpt_t pt1 = EnvNAV3DDYNCfg.FootprintPolygon[0];
  //tack the first point onto the back of the polygon
  EnvNAV3DDYNCfg.FootprintPolygon.push_back(pt1);

  EnvNAV3DDYN2Dpt_t pt2;
  EnvNAV3DDYN2Dpt_t pt;
  vector<EnvNAV3DDYN2Dpt_t> footprintBoundary;

  double min_radius = sqrt(pow(pt1.x,2.0) + pow(pt1.y,2.0));
  double radius;

  //first descritize
  for(unsigned int i=1; i<EnvNAV3DDYNCfg.FootprintPolygon.size(); i++){
    pt2 = EnvNAV3DDYNCfg.FootprintPolygon[i];

    //create a line between pt1 and pt2 
    for(double t=0; t<=1.0; t+=0.01){

      pt.x = (1-t)*pt1.x + t*pt2.x;
      pt.y = (1-t)*pt1.y + t*pt2.y;

      radius = sqrt(pow(pt.x, 2.0) + pow(pt.y, 2.0));
      
      //check to see if this point is the closest to the center
      if(radius < min_radius)
	min_radius = radius;

      footprintBoundary.push_back(pt);
           
    }
    //shuffle the points to go to the next line
    pt1 = pt2;
  }

#if NAV3DDYN_DEBUG
  printf("Min radius: %f %f\n", min_radius,  min_radius/EnvNAV3DDYNCfg.cellsize_m);
#endif

  EnvNAV3DDYNCfg.FootprintPolygon.clear();

  //now shrink footprint
  double alpha;
  for(unsigned int i=0; i<footprintBoundary.size(); i++){
    pt = footprintBoundary[i];

    alpha = atan2(pt.y, pt.x);

    pt.x = pt.x - min_radius*cos(alpha);
    pt.y = pt.y - min_radius*sin(alpha);

    EnvNAV3DDYNCfg.FootprintPolygon.push_back(pt);

  }

  
  //now blow up all the obstacles in the environment
  int r = static_cast<int>(min_radius/EnvNAV3DDYNCfg.cellsize_m + 0.5);
  //TODO: probably can do this better
  for(int w=0; w < EnvNAV3DDYNCfg.EnvWidth_c; w++){
    for(int h=0; h < EnvNAV3DDYNCfg.EnvHeight_c; h++){
      if(EnvNAV3DDYNCfg.Grid2D[w][h]==1){
	
	//pad
	for(int x = w-r; x <= w+r; x++){
	  for(int y = h-r; y <= h+r; y++){
	    
	    if(x >= 0 && y >= 0 && x < EnvNAV3DDYNCfg.EnvWidth_c && y < EnvNAV3DDYNCfg.EnvHeight_c &&
	       EnvNAV3DDYNCfg.Grid2D[x][y] == 0){
	      EnvNAV3DDYNCfg.Grid2D[x][y] = 2;
	    }
	  }
	}
      }
    }
  }

}

void EnvironmentNAV3DDYN::PrecomputeActions()
{
#if TIME_DEBUG
  clock_t currenttime = clock();
#endif

#if NAV3DDYN_DEBUG
  printf("Precomputing Actions...\n");
#endif

  unsigned int rvind, tvind, tind, aind, dtind, ind;
  unsigned int num_actions = 0;

  unsigned int numRvValues = NAV3DDYN_NUMRV;  

  if(NAV3DDYN_NUMRV % 2 == 0){
    printf("Warning: Number of rotational velocity values should be odd, incrementing\n");
    numRvValues++;
  }

  //calculate valid set of rotational velocities
  vector<double> rvVals;
  if(numRvValues > 1){
    double rvdelta = (NAV3DDYN_MAXRV + NAV3DDYN_MAXRV)/(numRvValues-1);
    for(rvind = 0; rvind < numRvValues; rvind++)
      {
	rvVals.push_back(-NAV3DDYN_MAXRV + rvdelta*rvind);
      }
  
    rvVals[numRvValues/2] = 0;
  }else{
    rvVals.push_back(0);
  }
#if NAV3DDYN_DEBUG
  printf("Rotation Velocity values: \n");
  for(unsigned int i=0; i<rvVals.size(); i++){
    printf("\t%f\n", rvVals[i]);
  }
#endif

  //calculate valid set of translational velocities
  int numTvValues = 2;
  double tvVals[2] = {-EnvNAV3DDYNCfg.nominalvel_mpersecs, EnvNAV3DDYNCfg.nominalvel_mpersecs};
  
  //calculate a set of time values
  int numDtValues = 2;
  double dtVals[2] = {NAV3DDYN_LONGDUR, NAV3DDYN_SHORTDUR};

#if NAV3DDYN_DEBUG
  printf("Translational Velocity values: \n");
  for(unsigned int i=0; i<numTvValues; i++){
    printf("\t%f\n", tvVals[i]);
  }
#endif

  //calculate a set of turn in place actions
  int numTurnsInPlace = 2;
  int turnsInPlace[2] = {-1, 1};

  //create an initial pose
  EnvNAV3DDYNContPose_t initialPose_cont;
  initialPose_cont.X = 0;
  initialPose_cont.Y = 0;
  initialPose_cont.Theta = 0.0;

  EnvNAV3DDYNDiscPose_t initialPose_disc; 
  

  EnvNAV3DDYNContPose_t currentPose_cont; //current pose real world
  EnvNAV3DDYNDiscPose_t currentPose_disc; //current pose grid
  vector<EnvNAV3DDYNContPose_t> path_cont;

  EnvNAV3DDYNCfg.ActionsV = new EnvNAV3DDYNAction_t*[NAV3DDYN_THETADIRS];
  EnvNAV3DDYNCfg.PredActionsV = new vector<EnvNAV3DDYNAction_t*>[NAV3DDYN_THETADIRS];

  for(tind = 0; tind < NAV3DDYN_THETADIRS; tind++){
    num_actions = 0;

    initialPose_cont.Theta = (double)tind * (2.0*PI_CONST/NAV3DDYN_THETADIRS);
    initialPose_disc.X = CONTXY2DISC(initialPose_cont.X, EnvNAV3DDYNCfg.cellsize_m);
    initialPose_disc.Y = CONTXY2DISC(initialPose_cont.Y, EnvNAV3DDYNCfg.cellsize_m);
    initialPose_disc.Theta = ContTheta2Disc(initialPose_cont.Theta, NAV3DDYN_THETADIRS);

    //move the initial pose to the center of the grid point it is in
    initialPose_cont.X = DISCXY2CONT(initialPose_disc.X, EnvNAV3DDYNCfg.cellsize_m);
    initialPose_cont.Y = DISCXY2CONT(initialPose_disc.Y, EnvNAV3DDYNCfg.cellsize_m);
    initialPose_cont.Theta = DiscTheta2Cont(initialPose_disc.Theta, NAV3DDYN_THETADIRS);

    //loop through all rotation velocity values
    EnvNAV3DDYNCfg.ActionsV[tind] = new EnvNAV3DDYNAction_t[rvVals.size()*numTvValues + numTvValues + numTurnsInPlace];
    //EnvNAV3DDYNCfg.ActionsV[tind] = new EnvNAV3DDYNAction_t[rvVals.size()*numTvValues + numTvValues];
    aind = 0;
    
    //first compute the turn in place actions
     for(ind = 0; ind < numTurnsInPlace; ind++){
      EnvNAV3DDYNAction_t action;
      action.rv = 0; //this doesn't matter
      action.tv = 0;
      action.time = EnvNAV3DDYNCfg.timetoturnoneunitinplace_secs;

      //put the start and the end positions on the path
      action.path_cont.push_back(initialPose_cont);
      action.path.push_back(initialPose_disc);

      //calculate an end pose and put it on the path
      currentPose_cont.X = initialPose_cont.X;
      currentPose_cont.Y = initialPose_cont.Y;
      currentPose_cont.Theta = initialPose_cont.Theta + DiscTheta2Cont(turnsInPlace[ind], NAV3DDYN_THETADIRS);
      action.path_cont.push_back(currentPose_cont);

      currentPose_disc.X = initialPose_disc.X;
      currentPose_disc.Y = initialPose_disc.Y;
      currentPose_disc.Theta = ContTheta2Disc(currentPose_cont.Theta, NAV3DDYN_THETADIRS);
      action.path.push_back(currentPose_disc);
      
      CalculateFootprintForPath(action.path_cont, &action.footprint);
      RemoveDuplicatesFromFootprint(&action.footprint);

      action.dX = 0;
      action.dY = 0;
      action.dTheta = turnsInPlace[ind];
      action.cost = abs(turnsInPlace[ind])*EnvNAV3DDYNCfg.timetoturnoneunitinplace_secs*1000;
      
      EnvNAV3DDYNCfg.ActionsV[tind][aind] = action;

      int path_length = action.path.size();
      EnvNAV3DDYNCfg.PredActionsV[action.path[path_length-1].Theta].push_back(&(EnvNAV3DDYNCfg.ActionsV[tind][aind]));
      aind++;
#if NAV3DDYN_DEBUG
	  printf("----Adding action----\n");
	  printf("rv: %f, tv: %f\n", action.rv, action.tv);
	  printf("dX: %d, dY: %d, dTheta: %d, Cost: %d\n", action.dX, action.dY, action.dTheta, action.cost);
	  printf("Path length: %d\n", path_length);
	  printf("Path: \n");
	  for(int i = 0; i<path_length; i++){
	    printf("%d %d %d\n", action.path[i].X, action.path[i].Y, action.path[i].Theta);
	  }
	  printf("Footprint: \n");
	  for(int i = 0; i<action.footprint.size(); i++){
	    printf("%f %f\n", action.footprint[i].x, action.footprint[i].y);
	  }
	  
	  printf("Adding action to predicessor list for theta: %d\n", action.path[path_length-1].Theta);
#endif
     }


    // now compute the forward and backward moving actions
    for(rvind = 0; rvind < rvVals.size(); rvind++){

      //loop through all translational velocity values
      for(tvind = 0; tvind < numTvValues; tvind++){
	
	//try two different time durations
	for(dtind = 0; dtind < numDtValues; dtind++){
	  
	  if(dtind > 0 && fabs(rvVals[rvind]) > ERR_EPS){
	    //don't try a second time duration if its not a straight actions
	    continue;
	  }

	  EnvNAV3DDYNAction_t action;
	  action.rv = rvVals[rvind];
	  action.tv = tvVals[tvind];
	  action.time = dtVals[dtind];
	  path_cont.clear();
	  
#if NAV3DDYN_DEBUG
	  printf("Computing action for tv: %f, rv: %f, dt: %d, abs: %f\n", 
		 action.tv, action.rv, dtind, fabs(rvVals[rvind]));
#endif
	  
	  //put the initial pose on the action list
	  path_cont.push_back(initialPose_cont);
	  action.path.push_back(initialPose_disc);
	  
	  for(double ts=0; ts < dtVals[dtind]; ts+=NAV3DDYN_DURDISC){
	    //compute the new pose
	    double tvoverrv;
	    
	    if(fabs(action.rv) > ERR_EPS) //not a straight action
	      {
		tvoverrv = action.tv/action.rv;
		currentPose_cont.X = initialPose_cont.X + tvoverrv*(sin(action.rv*ts+initialPose_cont.Theta)-sin(initialPose_cont.Theta));
		currentPose_cont.Y = initialPose_cont.Y - tvoverrv*(cos(action.rv*ts+initialPose_cont.Theta)-cos(initialPose_cont.Theta));
		
	      }
	    else
	      {
		currentPose_cont.X = initialPose_cont.X + action.tv*ts*cos(initialPose_cont.Theta);
		currentPose_cont.Y = initialPose_cont.Y + action.tv*ts*sin(initialPose_cont.Theta);
		
	      }
	    currentPose_cont.Theta = action.rv*ts + initialPose_cont.Theta;
	    
	    currentPose_disc.X = CONTXY2DISC(currentPose_cont.X, EnvNAV3DDYNCfg.cellsize_m);	  
	    currentPose_disc.Y = CONTXY2DISC(currentPose_cont.Y, EnvNAV3DDYNCfg.cellsize_m);
	    currentPose_disc.Theta = ContTheta2Disc(currentPose_cont.Theta, NAV3DDYN_THETADIRS);
	    
	    path_cont.push_back(currentPose_cont);
	  }
	  
	  
	  //fix the error so that the end point is precisely the center of the cell
	  EnvNAV3DDYNContPose_t endPose_cont;
	  endPose_cont.X = DISCXY2CONT(currentPose_disc.X, EnvNAV3DDYNCfg.cellsize_m);
	  endPose_cont.Y = DISCXY2CONT(currentPose_disc.Y, EnvNAV3DDYNCfg.cellsize_m);
	  endPose_cont.Theta = DiscTheta2Cont(currentPose_disc.Theta, NAV3DDYN_THETADIRS);
	  
	  double errorx = endPose_cont.X - currentPose_cont.X;
	  double errory = endPose_cont.Y - currentPose_cont.Y;
	  double errortheta = endPose_cont.Theta - currentPose_cont.Theta;
	  if(errortheta > PI_CONST)
	    errortheta = errortheta-2*PI_CONST;
	  else if(errortheta < -PI_CONST)
	    errortheta = errortheta+2*PI_CONST;
	  
#if NAV3DDYN_DEBUG
	  printf("Error X: %f, Error Y: %f, Error Theta: %f\n", errorx, errory, errortheta);
#endif
	  
	  //now distribute the error and calculate the cost and footprint
	  int path_length = path_cont.size();
	  
	  for(int i = 1; i < path_length; i++)
	    {
	      path_cont[i].X += errorx*(((double)i)/(path_length-1));
	      path_cont[i].Y += errory*(((double)i)/(path_length-1));
	      path_cont[i].Theta += errortheta*(((double)i)/(path_length-1));
	      
	      //add this to the path
	      currentPose_disc.X = CONTXY2DISC(path_cont[i].X, EnvNAV3DDYNCfg.cellsize_m);
	      currentPose_disc.Y = CONTXY2DISC(path_cont[i].Y, EnvNAV3DDYNCfg.cellsize_m);
	      currentPose_disc.Theta = ContTheta2Disc(path_cont[i].Theta, NAV3DDYN_THETADIRS);
	      action.path.push_back(currentPose_disc);
	    }
	  
	  action.path_cont = path_cont;
	  
	  RemoveDuplicatesFromPath(&action.path);
	  CalculateFootprintForPath(path_cont, &action.footprint);
	  RemoveDuplicatesFromFootprint(&action.footprint);
	  
	  path_length = action.path.size();	

	  action.dX = action.path[path_length-1].X - initialPose_disc.X;
	  action.dY = action.path[path_length-1].Y - initialPose_disc.Y;
	  action.dTheta = action.path[path_length-1].Theta - initialPose_disc.Theta;

	  action.cost = 0;
	  for(int i = 1; i < path_length; i++){
	    action.cost += EuclideanDistance(action.path[i].X, action.path[i].Y, action.path[i-1].X, action.path[i-1].Y);
	  }
	  
	  //convert cost to a time value instead of a distance value
	  action.cost = action.cost / EnvNAV3DDYNCfg.nominalvel_mpersecs;

	  //add the action to the list of posible actions
	  EnvNAV3DDYNCfg.ActionsV[tind][aind] = action;
	  
	  //put a backwards pointer to the ending locations
	  EnvNAV3DDYNCfg.PredActionsV[action.path[path_length-1].Theta].push_back(&(EnvNAV3DDYNCfg.ActionsV[tind][aind]));
	  aind++;
#if NAV3DDYN_DEBUG
	  printf("----Adding action----\n");
	  printf("theta: %d, rv: %f, tv: %f, dt: %f\n", tind, action.rv, action.tv, dtVals[dtind]);
	  printf("dX: %d, dY: %d, dTheta: %d, Cost: %d\n", action.dX, action.dY, action.dTheta, action.cost);
	  printf("Path length: %d\n", path_length);
	  printf("Path: \n");
	  for(int i = 0; i<path_length; i++){
	    printf("%d %d %d\n", action.path[i].X, action.path[i].Y, action.path[i].Theta);
	  }
	  printf("Footprint: \n");
	  for(int i = 0; i<action.footprint.size(); i++){
	    printf("%f %f\n", action.footprint[i].x, action.footprint[i].y);
	  }
	  
	  printf("Adding action to predicessor list for theta: %d\n", action.path[path_length-1].Theta);
#endif
	  
	}
      }

    }
  }

#if TIME_DEBUG
  printf("Time to precompute actions = %.3f secs\n", (clock() - currenttime)/((double)CLOCKS_PER_SEC));
#endif

}



void EnvironmentNAV3DDYN::CalculateFootprintForPath(vector<EnvNAV3DDYNContPose_t> path, vector<EnvNAV3DDYN2Dpt_t>* footprint){
  unsigned int find;

  for(find = 0; find < path.size(); find++){
    CalculateFootprintForPose(path[find], footprint);
  }
}

/*
 * point - the point to test
 *
 * Function derived from http://ozviz.wasp.uwa.edu.au/~pbourke/geometry/insidepoly/
 */
int EnvironmentNAV3DDYN::InsideFootprint(EnvNAV3DDYN2Dpt_t pt, vector<EnvNAV3DDYN2Dpt_t>* bounding_polygon){
  
  int counter = 0;
  int i;
  double xinters;
  EnvNAV3DDYN2Dpt_t p1;
  EnvNAV3DDYN2Dpt_t p2;
  int N = bounding_polygon->size();

  p1 = bounding_polygon->at(0);
  for (i=1;i<=N;i++) {
    p2 = bounding_polygon->at(i % N);
    if (pt.y > MIN(p1.y,p2.y)) {
      if (pt.y <= MAX(p1.y,p2.y)) {
        if (pt.x <= MAX(p1.x,p2.x)) {
          if (p1.y != p2.y) {
            xinters = (pt.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
            if (p1.x == p2.x || pt.x <= xinters)
              counter++;
          }
        }
      }
    }
    p1 = p2;
  }

  if (counter % 2 == 0)
    return(0);
  else
    return(1);
#if NAV3DDYN_DEBUG
  //printf("Returning from inside footprint: %d\n", c);
#endif
  //  return c;

}

void EnvironmentNAV3DDYN::CalculateFootprintForPose(EnvNAV3DDYNContPose_t pose, vector<EnvNAV3DDYN2Dpt_t>* footprint)
{  
#if NAV3DDYN_DEBUG
  printf("---Calculating Footprint for Pose: %f %f %f---\n",
	 pose.X, pose.Y, pose.Theta);
#endif

  //handle special case where footprint is just a point
  if(EnvNAV3DDYNCfg.FootprintPolygon.size() <= 1){
    EnvNAV3DDYN2Dpt_t pt;
    pt.x = CONTXY2DISC(pose.X, EnvNAV3DDYNCfg.cellsize_m);
    pt.y = CONTXY2DISC(pose.Y, EnvNAV3DDYNCfg.cellsize_m);
    footprint->push_back(pt);
    return;
  }


  vector<EnvNAV3DDYN2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x, min_x, max_y, min_y;
  EnvNAV3DDYN2Dpt_t pt;
  for(find = 0; find < EnvNAV3DDYNCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAV3DDYNCfg.FootprintPolygon[find];
    
    //rotate and translate the point
    EnvNAV3DDYN2Dpt_t corner;
    corner.x = cos(pose.Theta)*pt.x - sin(pose.Theta)*pt.y + pose.X;
    corner.y = sin(pose.Theta)*pt.x + cos(pose.Theta)*pt.y + pose.Y;
    bounding_polygon.push_back(corner);
#if NAV3DDYN_DEBUG
    printf("Pt: %f %f, Corner: %f %f\n", pt.x, pt.y, corner.x, corner.y);
#endif
    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }

#if NAV3DDYN_DEBUG
  printf("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAV3DDYNCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAV3DDYNCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=0.1){
    for(double y=min_y; y<=max_y; y+=0.1){
      EnvNAV3DDYN2Dpt_t pt;
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAV3DDYNCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAV3DDYNCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){
#if NAV3DDYN_DEBUG
      printf("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif
	if(InsideFootprint(pt, &bounding_polygon)){
	//convert to a grid point
#if NAV3DDYN_DEBUG
	  printf("Pt Inside %f %f\n", pt.x, pt.y);
#endif
	  pt.x = discrete_x;
	  pt.y = discrete_y;
	  footprint->push_back(pt);
	  prev_inside = 1;
#if NAV3DDYN_DEBUG
	  printf("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
	}else{
	  prev_inside = 0;
	}
      }else{
#if NAV3DDYN_DEBUG
	//	printf("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }
  }
 

}

void EnvironmentNAV3DDYN::RemoveDuplicatesFromPath(vector<EnvNAV3DDYNDiscPose_t> *path){
  
#if NAV3DDYN_DEBUG
  printf("Entering function: RemoveDuplicatesFromPath - size=%d\n", path->size());
#endif

  vector<EnvNAV3DDYNDiscPose_t>::iterator new_end;

  new_end = unique(path->begin(), path->end());
  path->erase(new_end, path->end());
  

#if NAV3DDYN_DEBUG
  printf("Leaving function: RemoveDuplicatesFromPath - size=%d\n", path->size());
#endif
}

void EnvironmentNAV3DDYN::RemoveDuplicatesFromFootprint(vector<EnvNAV3DDYN2Dpt_t>* footprint)
{
  
#if NAV3DDYN_DEBUG
  printf("Entering function: RemoveDuplicatesFromFootprint\n");
#endif
  
  sort(footprint->begin(), footprint->end());

  vector<EnvNAV3DDYN2Dpt_t>::iterator new_end;
  new_end = unique(footprint->begin(), footprint->end());
  footprint->erase(new_end, footprint->end());

#if NAV3DDYN_DEBUG
  printf("Leaving function: RemoveDuplicatesFromFootprint\n");
#endif
  
}

bool EnvironmentNAV3DDYN::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  //initialize MDPCfg with the start and goal ids	
  MDPCfg->goalstateid = EnvNAV3DDYN.goalstateid;
  MDPCfg->startstateid = EnvNAV3DDYN.startstateid;
  
#if NAV3DDYN_DEBUG
  printf("Initializing start id (%d) and goal id (%d)\n", 
	 EnvNAV3DDYN.startstateid,
	 EnvNAV3DDYN.goalstateid);
#endif
  return true;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV3DDYN::SetStart(double x_m, double y_m, double theta_rad){

  int x = CONTXY2DISC(x_m, EnvNAV3DDYNCfg.cellsize_m);
  int y = CONTXY2DISC(x_m, EnvNAV3DDYNCfg.cellsize_m);
  int theta = ContTheta2Disc(theta_rad, NAV3DDYN_THETADIRS);
  
  if(!IsWithinMapCell(x,y))
    return -1;
  
  EnvNAV3DDYNHashEntry_t* OutHashEntry;
  if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y, theta);
  }
  EnvNAV3DDYN.startstateid = OutHashEntry->stateID;
  
  return EnvNAV3DDYN.startstateid;    

}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV3DDYN::SetGoal(double x_m, double y_m, double theta_rad){

  int x = CONTXY2DISC(x_m, EnvNAV3DDYNCfg.cellsize_m);
  int y = CONTXY2DISC(x_m, EnvNAV3DDYNCfg.cellsize_m);
  int theta = ContTheta2Disc(theta_rad, NAV3DDYN_THETADIRS);
  
  if(!IsWithinMapCell(x,y))
    return -1;
  
  EnvNAV3DDYNHashEntry_t* OutHashEntry;
  if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y, theta);
  }
  EnvNAV3DDYN.goalstateid = OutHashEntry->stateID;
  
  return EnvNAV3DDYN.goalstateid;    
  
}

void EnvironmentNAV3DDYN::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
  unsigned int aind, ftind;

  //get X, Y for the state
  EnvNAV3DDYNHashEntry_t* HashEntry = EnvNAV3DDYN.StateID2CoordTable[SourceStateID];

#if NAV3DDYN_DEBUG
  printf("Computing Successors: %d %d %d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
#endif
  
#if TIME_DEBUG
  clock_t currenttime = clock();
#endif
  
  //clear the successor array
  SuccIDV->clear();
  CostV->clear();


  //goal state should be absorbing
  if(SourceStateID == EnvNAV3DDYN.goalstateid)
    return;
  
  	
  //get the set of valid actions from the current location
  EnvNAV3DDYNAction_t* valid_actions = EnvNAV3DDYNCfg.ActionsV[HashEntry->Theta];
  int num_actions = NAV3DDYN_NUMRV * 2 + 2 + 2;
  SuccIDV->reserve(num_actions);
  CostV->reserve(num_actions);
 
  bool insideBounds;
  bool collisionFree;

  //iterate through actions 
  for(aind = 0; aind < num_actions; aind++){

    EnvNAV3DDYNAction_t* nav3daction = &(valid_actions[aind]);

    int newX = HashEntry->X + nav3daction->dX;
    int newY = HashEntry->Y + nav3daction->dY;
    int newTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAV3DDYN_THETADIRS);	
#if NAV3DDYN_DEBUG
    printf("Original Coords: %d %d %d, New Coords: %d %d %d\n",
	   HashEntry->X, HashEntry->Y, HashEntry->Theta,
	   newX, newY, newTheta);
#endif      

    int tempX, tempY;
    double ftX, ftY;
    insideBounds = true;
    collisionFree = true;
     //check for collisions in footprint
    for (ftind = 0; ftind < nav3daction->footprint.size(); ftind++){
      //translate the footprint
      ftX = nav3daction->footprint.at(ftind).x;
      ftY = nav3daction->footprint.at(ftind).y;
      tempX = HashEntry->X + ftX;
      tempY = HashEntry->Y + ftY;
	
      if(IsValidCell(tempX, tempY)){
	if(EnvNAV3DDYNCfg.Grid2D[tempX][tempY] != 0){
#if NAV3DDYN_DEBUG
	  printf("Pt (%d, %d) is a collision\n", tempX, tempY);
#endif
	  collisionFree = false;
	  break;
	}
      }else{
#if NAV3DDYN_DEBUG
	printf("Pt (%d, %d) is not inside bounds\n", tempX, tempY);
#endif
	insideBounds = false;
	break;
      }
     
    }
    
    //skip the invalid actions
    if(!insideBounds){ 
      continue;
    }else if(!collisionFree)
      continue;
    
    
    EnvNAV3DDYNHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(newX, newY, newTheta)) == NULL)
      {
	//have to create a new entry
	OutHashEntry = CreateNewHashEntry(newX, newY, newTheta);
      }
    
    //compute clow 
    int cost = nav3daction->cost;

#if NAV3DDYN_DEBUG
    int heuristic = GetFromToHeuristic(HashEntry->stateID, OutHashEntry->stateID);
    printf("Adding state with cost %d and heuristic %d\n", cost, heuristic);
    printf("\tFrom : %d %d %d, To: %d %d %d\n",
	   HashEntry->X, HashEntry->Y, HashEntry->Theta,
	   newX, newY, newTheta);
    if(OutHashEntry->stateID == EnvNAV3DDYN.goalstateid){
      printf("Adding goal state to list: %d %d %d\n", OutHashEntry->X, OutHashEntry->Y, OutHashEntry->Theta);
    }
#endif
    
    SuccIDV->push_back(OutHashEntry->stateID);
    CostV->push_back(cost);
  }
  
#if TIME_DEBUG
  time_getsuccs += clock()-currenttime;
#endif
  
}

void EnvironmentNAV3DDYN::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{
  unsigned int aind;
  unsigned int ftind;

#if NAV3DDYN_DEBUG
  printf("Inside GetPreds\n");
#endif

#if TIME_DEBUG
  clock_t currenttime = clock();
#endif
	
  //clear the successor array
  PredIDV->clear();
  CostV->clear();
   
  //get X, Y for the state
  EnvNAV3DDYNHashEntry_t* HashEntry = EnvNAV3DDYN.StateID2CoordTable[TargetStateID];

#if NAV3DDYN_DEBUG
  printf("Hash Entry values: %d %d %d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
#endif
  
  //no predecessors if obstacle
  if(EnvNAV3DDYNCfg.Grid2D[HashEntry->X][HashEntry->Y] != 0)
    return;

  //get the set of valid actions from the current location
  vector<EnvNAV3DDYNAction_t*> pred_actions = EnvNAV3DDYNCfg.PredActionsV[HashEntry->Theta];
#if NAV3DDYN_DEBUG
  printf("Number of actions: %d\n", pred_actions.size());
#endif
  PredIDV->reserve(pred_actions.size());
  CostV->reserve(pred_actions.size());

  bool insideBounds;
  bool collisionFree;

  //iterate through actions
  for (aind = 0; aind < pred_actions.size(); aind++)
    {

      EnvNAV3DDYNAction_t* nav3daction = pred_actions[aind];
      int predX = HashEntry->X - nav3daction->dX;
      int predY = HashEntry->Y - nav3daction->dY;
      int predTheta = NORMALIZEDISCTHETA(HashEntry->Theta - nav3daction->dTheta, NAV3DDYN_THETADIRS);	
    
#if NAV3DDYN_DEBUG
      printf("Delta Values: %d %d %d\n", nav3daction->dX, nav3daction->dY, nav3daction->dTheta);
      printf("Pred Values: %d %d %d\n", predX, predY, predTheta);
#endif
      //check for collisions in footprint
      int tempX, tempY;
      double ftX, ftY;
      insideBounds = true;
      collisionFree = true;
      for (ftind = 0; ftind < nav3daction->footprint.size(); ftind++){
	//translate the footprint
	ftX = nav3daction->footprint.at(ftind).x;
	ftY = nav3daction->footprint.at(ftind).y;
	tempX = ftX + predX;
	tempY = ftY + predY;

	if(IsValidCell(tempX, tempY)){
	  if(EnvNAV3DDYNCfg.Grid2D[tempX][tempY] != 0){
#if NAV3DDYN_DEBUG
	    	printf("Pt (%d, %d) is a collision\n", tempX, tempY);
#endif
		collisionFree = false;
	    break;
	  }
	}else{
#if NAV3DDYN_DEBUG
	  printf("Pt (%d, %d) is not inside bounds\n", tempX, tempY);
#endif
	  insideBounds = false;
	  break;
	}
      }
    
      //skip the invalid actions
      if(!insideBounds){ 
	continue;
      }else if(!collisionFree)
	continue;
      
      EnvNAV3DDYNHashEntry_t* OutHashEntry;
      if((OutHashEntry = GetHashEntry(predX, predY, predTheta)) == NULL)
	{
	  //have to create a new entry
	  OutHashEntry = CreateNewHashEntry(predX, predY, predTheta);
	}
      
      //compute clow 
      int cost = nav3daction->cost;
#if NAV3DDYN_DEBUG
      int heuristic = GetFromToHeuristic(HashEntry->stateID, OutHashEntry->stateID);
      printf("Adding pred (%d %d %d) with cost %d and heuristic value %d\n", predX, predY, predTheta, cost, heuristic);
#endif
      PredIDV->push_back(OutHashEntry->stateID);
      CostV->push_back(cost);
    }
  
#if TIME_DEBUG
  time_getsuccs += clock()-currenttime;
#endif
  
  
}


int EnvironmentNAV3DDYN::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
  return 0;
#endif
  

#if NAV3DDYN_DEBUG
  if(FromStateID >= (int)EnvNAV3DDYN.StateID2CoordTable.size() 
     || ToStateID >= (int)EnvNAV3DDYN.StateID2CoordTable.size())
    {
      printf("ERROR in EnvNAV3DDYN... function: stateID illegal\n");
      exit(1);
    }
#endif
 

  //TODO: This should actually calculate the length of the arc that the robot will drive along
 
  //get X, Y for the state
  EnvNAV3DDYNHashEntry_t* FromHashEntry = EnvNAV3DDYN.StateID2CoordTable[FromStateID];
  EnvNAV3DDYNHashEntry_t* ToHashEntry = EnvNAV3DDYN.StateID2CoordTable[ToStateID];
	

  return EuclideanDistance(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/EnvNAV3DDYNCfg.nominalvel_mpersecs;	
  
}

bool EnvironmentNAV3DDYN::UpdateCost(int x, int y, int new_status)
{

  EnvNAV3DDYNCfg.Grid2D[x][y] = new_status;
  
  return true;
}

void EnvironmentNAV3DDYN::GetCoordFromState(int stateID, int& x, int& y, int& theta) const {
  EnvNAV3DDYNHashEntry_t* HashEntry = EnvNAV3DDYN.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
}

int EnvironmentNAV3DDYN::GetStateFromCoord(int x, int y, int theta) {

  EnvNAV3DDYNHashEntry_t* OutHashEntry;
  if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
    //have to create a new entry
    OutHashEntry = CreateNewHashEntry(x, y, theta);
  }
  return OutHashEntry->stateID;
}

bool EnvironmentNAV3DDYN::IsObstacle(int x, int y)
{
  return (EnvNAV3DDYNCfg.Grid2D[x][y] != 0);
}

//--------------------Debugging Functions---------------------
void EnvironmentNAV3DDYN::PrintConfigurationToFile(const char* logFile){

#if NAV3DDYN_DEBUG
  printf("Printing configuration to file - %s\n", logFile);
#endif

  FILE* fp = fopen(logFile, "w");
  fprintf(fp, "discretization(cells):%d %d\n", EnvNAV3DDYNCfg.EnvWidth_c, EnvNAV3DDYNCfg.EnvHeight_c);
  fprintf(fp, "cellsize(meters):%f\n", EnvNAV3DDYNCfg.cellsize_m);
  fprintf(fp, "nominalvel(mpersecs):%f\n", EnvNAV3DDYNCfg.nominalvel_mpersecs);
  fprintf(fp, "timetoturn45degsinplace(secs):%f\n", EnvNAV3DDYNCfg.timetoturnoneunitinplace_secs);
  fprintf(fp, "start(m,rad):%d %d %d\n", EnvNAV3DDYNCfg.StartX_c, EnvNAV3DDYNCfg.StartY_c, EnvNAV3DDYNCfg.StartTheta);
  fprintf(fp, "end(m,rad): %d %d %d\n", EnvNAV3DDYNCfg.EndX_c, EnvNAV3DDYNCfg.EndY_c, EnvNAV3DDYNCfg.EndTheta);
  fprintf(fp, "num_footprint_corners:%d\n", EnvNAV3DDYNCfg.FootprintPolygon.size());
  fprintf(fp, "footprint:\n");
 
  for(unsigned int i=0; i<EnvNAV3DDYNCfg.FootprintPolygon.size(); i++){
    fprintf(fp, "%f %f\n", EnvNAV3DDYNCfg.FootprintPolygon[i].x, EnvNAV3DDYNCfg.FootprintPolygon[i].y);
  }

  fprintf(fp, "environment:\n");
  
  for(unsigned int h=0; h<EnvNAV3DDYNCfg.EnvHeight_c; h++){
    for(unsigned int w=0; w<EnvNAV3DDYNCfg.EnvWidth_c; w++){
      fprintf(fp, "%d ", EnvNAV3DDYNCfg.Grid2D[w][h]);
    }
    fprintf(fp, "\n");
  }

  fclose(fp);

}

void EnvironmentNAV3DDYN::PrintActionsToFile(const char* logFile){
  
#if NAV3DDYN_DEBUG
  printf("Printing actions to file - %s\n", logFile);
#endif

  FILE* fp = fopen(logFile, "w");
  unsigned int tind, aind, pind;
  unsigned int num_actions, path_length, footprint_size;
  double rv, tv;
  int time;
  EnvNAV3DDYNDiscPose_t pt;
  EnvNAV3DDYNContPose_t pt_cont;
  EnvNAV3DDYN2Dpt_t ft_pt;

  for(tind = 0; tind < NAV3DDYN_THETADIRS; tind++){
    fwrite(&tind, sizeof(tind), 1, fp);
    num_actions = NAV3DDYN_NUMRV * 2 + 2 + 2;
    //printf("Printing: %d %d\n", tind, num_actions);
    fwrite(&num_actions, sizeof(num_actions), 1, fp);
    for(aind = 0; aind < num_actions; aind++){
      rv = EnvNAV3DDYNCfg.ActionsV[tind][aind].rv;
      tv = EnvNAV3DDYNCfg.ActionsV[tind][aind].tv;
      time = EnvNAV3DDYNCfg.ActionsV[tind][aind].time;
      fwrite(&rv, sizeof(rv), 1, fp);
      fwrite(&tv, sizeof(tv), 1, fp);
      fwrite(&time, sizeof(time), 1, fp);
      path_length = EnvNAV3DDYNCfg.ActionsV[tind][aind].path.size();
      fwrite(&path_length, sizeof(path_length), 1, fp);
      for(pind = 0; pind < path_length; pind++){
	pt = EnvNAV3DDYNCfg.ActionsV[tind][aind].path[pind];
	fwrite(&pt, sizeof(pt), 1, fp);
      }
      
      path_length = EnvNAV3DDYNCfg.ActionsV[tind][aind].path_cont.size();
      fwrite(&path_length, sizeof(path_length), 1, fp);
      for(pind = 0; pind < path_length; pind++){
	pt_cont = EnvNAV3DDYNCfg.ActionsV[tind][aind].path_cont[pind];
	fwrite(&pt_cont, sizeof(pt_cont), 1, fp);
      }
      footprint_size = EnvNAV3DDYNCfg.ActionsV[tind][aind].footprint.size();
      fwrite(&footprint_size, sizeof(footprint_size), 1, fp);
      for(pind = 0; pind < footprint_size; pind++){
	ft_pt = EnvNAV3DDYNCfg.ActionsV[tind][aind].footprint[pind];
	fwrite(&ft_pt, sizeof(ft_pt), 1, fp);
      }
    } 
  }
  
  fclose(fp);

}

void EnvironmentNAV3DDYN::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if NAV3DDYN_DEBUG
  if(stateID >= (int)EnvNAV3DDYN.StateID2CoordTable.size())
    {
      printf("ERROR in EnvNAV3DDYN... function: stateID illegal (2)\n");
      exit(1);
    }
#endif

  if(fOut == NULL)
    fOut = stdout;

  EnvNAV3DDYNHashEntry_t* HashEntry = EnvNAV3DDYN.StateID2CoordTable[stateID];

  if(stateID == EnvNAV3DDYN.goalstateid && bVerbose)
    {
      fprintf(fOut, "the state is a goal state\n");
    }

  if(bVerbose)
    fprintf(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
  else
    fprintf(fOut, "%d %d %d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
  
}

void EnvironmentNAV3DDYN::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
  fprintf(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
	  time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
	  time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}

int EnvironmentNAV3DDYN::SizeofCreatedEnv()
{
  return (int)EnvNAV3DDYN.StateID2CoordTable.size();
	
}

void EnvironmentNAV3DDYN::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* goalx, double* goaly, double* goaltheta,
				      double* cellsize_m)
{
  *size_x = EnvNAV3DDYNCfg.EnvWidth_c;
  *size_y = EnvNAV3DDYNCfg.EnvHeight_c;
  
  *startx = DISCXY2CONT(EnvNAV3DDYNCfg.StartX_c, EnvNAV3DDYNCfg.cellsize_m);
  *starty = DISCXY2CONT(EnvNAV3DDYNCfg.StartY_c, EnvNAV3DDYNCfg.cellsize_m);
  *starttheta = DiscTheta2Cont(EnvNAV3DDYNCfg.StartTheta, NAV3DDYN_THETADIRS);
  *goalx = DISCXY2CONT(EnvNAV3DDYNCfg.EndX_c, EnvNAV3DDYNCfg.cellsize_m);
  *goaly = DISCXY2CONT(EnvNAV3DDYNCfg.EndY_c, EnvNAV3DDYNCfg.cellsize_m);
  *goaltheta = DiscTheta2Cont(EnvNAV3DDYNCfg.EndTheta, NAV3DDYN_THETADIRS);;
  
  *cellsize_m = EnvNAV3DDYNCfg.cellsize_m;

}

void EnvironmentNAV3DDYN::GetContPathFromStateIds(vector<int> stateIdV, vector<EnvNAV3DDYNContPose_t>* path){
  
  //check to be sure there are some states, if so then just return
  if(stateIdV.size() == 0){
    return;
  }

  int currId = stateIdV[0];
  int x, y, theta;
  double currX, currY;
  int currTheta;

  GetCoordFromState(currId, x, y, theta);
  currX = DISCXY2CONT(x, EnvNAV3DDYNCfg.cellsize_m);
  currY = DISCXY2CONT(y, EnvNAV3DDYNCfg.cellsize_m);
  currTheta = theta;
  int nextId;
  double prevX, prevY;
  int prevTheta;

  int found;
 

  vector<EnvNAV3DDYNAction_t*> actions;
  EnvNAV3DDYNContPose_t pose;

  for(unsigned int i=1; i<stateIdV.size(); i++){
    
    nextId = stateIdV[i];    
    GetCoordFromState(nextId, x, y, theta);
    
    actions = EnvNAV3DDYNCfg.PredActionsV[theta];
    
    found = 0;
    for(unsigned int aind = 0; aind < actions.size(); aind++){
      prevX = DISCXY2CONT(x - actions[aind]->dX, EnvNAV3DDYNCfg.cellsize_m);
      prevY = DISCXY2CONT(y - actions[aind]->dY, EnvNAV3DDYNCfg.cellsize_m);
      prevTheta = NORMALIZEDISCTHETA(theta - actions[aind]->dTheta, NAV3DDYN_THETADIRS);
      #if NAV3DDYN_DEBUG
      printf("Curr: %f %f %d, Prev: %f %f %d\n",
	     currX, currY, currTheta,
	     prevX, prevY, prevTheta);
      #endif
      if( max(prevX - currX, currX - prevX) < ERR_EPS && max(prevY - currY, currY - prevY) < ERR_EPS && prevTheta==currTheta){
	
	for(unsigned int pind = 0; pind < actions[aind]->path_cont.size(); pind++){
	  pose = actions[aind]->path_cont[pind];
	  pose.X += currX;
	  pose.Y += currY;
	  path->push_back(pose);
	}
	found = 1;
	break;
      }
    }

    if(found == 1){
      currX = DISCXY2CONT(x,EnvNAV3DDYNCfg.cellsize_m);
      currY = DISCXY2CONT(y,EnvNAV3DDYNCfg.cellsize_m);
      currTheta = theta;
      currId = nextId;
    }else{
      printf("Error occurred while computing path from stateIds....failed to find action for stateId %d\n", currId);
      return;
    }
  }
  
  return;
}

int EnvironmentNAV3DDYN::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
  return 0;
#endif
  
#if NAV3DDYN_DEBUG
  if(stateID >= (int)EnvNAV3DDYN.StateID2CoordTable.size())
    {
      printf("ERROR in EnvNAV3DDYN... function: stateID illegal\n");
      exit(1);
    }
#endif
  
  
  //define this function if it used in the planner (heuristic forward search would use it)
  return GetFromToHeuristic(stateID, EnvNAV3DDYN.goalstateid);
  
}

int EnvironmentNAV3DDYN::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
  return 0;
#endif
  
  
#if NAV3DDYN_DEBUG
  if(stateID >= (int)EnvNAV3DDYN.StateID2CoordTable.size())
    {
      printf("ERROR in EnvNAV3DDYN... function: stateID illegal\n");
      exit(1);
    }
#endif
  
  //define this function if it used in the planner (heuristic backward search would use it)
  return GetFromToHeuristic(EnvNAV3DDYN.startstateid, stateID);

}


//----------------Private Functions------------------------------
void EnvironmentNAV3DDYN::ReadConfiguration(FILE* fCfg)
{
    	//read in the configuration of environment and initialize  EnvNAV3DDYNCfg structure
	char sTemp[1024], sTemp1[1024];
	int dTemp;

	//discretization(cells)
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "discretization(cells):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.EnvWidth_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.EnvHeight_c = atoi(sTemp);

	//cellsize
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "cellsize(meters):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.cellsize_m = atof(sTemp);

	//Max translational velocity
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "nominalvel(mpersecs):");
	if(strcmp(sTemp1, sTemp) != 0)
	  {
	    printf("ERROR: configuration file has incorrect format\n");
	    printf("Expected %s got %s\n", sTemp1, sTemp);
	    exit(1);
	  }
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.nominalvel_mpersecs = atoi(sTemp);

	//Max rotational velocity
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "timetoturn45degsinplace(secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	  {
	    printf("ERROR: configuration file has incorrect format\n");
	    printf("Expected %s got %s\n", sTemp1, sTemp);
	    exit(1);
	  }
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.timetoturnoneunitinplace_secs = atoi(sTemp);

	//start state
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "start(m,rad):");
	if(strcmp(sTemp1, sTemp) != 0)
	  {
	    printf("ERROR: configuration file has incorrect format\n");
	    printf("Expected %s got %s\n", sTemp1, sTemp);
	    exit(1);
	  }

	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAV3DDYNCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAV3DDYNCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.StartTheta = ContTheta2Disc(atof(sTemp), NAV3DDYN_THETADIRS);


	if(EnvNAV3DDYNCfg.StartX_c < 0 || EnvNAV3DDYNCfg.StartX_c >= EnvNAV3DDYNCfg.EnvWidth_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV3DDYNCfg.StartY_c < 0 || EnvNAV3DDYNCfg.StartY_c >= EnvNAV3DDYNCfg.EnvHeight_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV3DDYNCfg.StartTheta < 0 || static_cast<unsigned int>(EnvNAV3DDYNCfg.StartTheta) >= NAV3DDYN_THETADIRS) {
		printf("ERROR: illegal start coordinates for theta\n");
		exit(1);
	}

	//end state
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "end(m,rad):");
	if(strcmp(sTemp1, sTemp) != 0)
	  {
	    printf("ERROR: configuration file has incorrect format\n");
	    printf("Expected %s got %s\n", sTemp1, sTemp);
	    exit(1);
	  }
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAV3DDYNCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAV3DDYNCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DDYNCfg.EndTheta = ContTheta2Disc(atof(sTemp), NAV3DDYN_THETADIRS);;

	if(EnvNAV3DDYNCfg.EndX_c < 0 || EnvNAV3DDYNCfg.EndX_c >= EnvNAV3DDYNCfg.EnvWidth_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV3DDYNCfg.EndY_c < 0 || EnvNAV3DDYNCfg.EndY_c >= EnvNAV3DDYNCfg.EnvHeight_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV3DDYNCfg.EndTheta < 0 || static_cast<unsigned int>(EnvNAV3DDYNCfg.EndTheta) >= NAV3DDYN_THETADIRS) {
		printf("ERROR: illegal goal coordinates for theta\n");
		exit(1);
	}

	//footprint
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "num_footprint_corners:");
	if(strcmp(sTemp1, sTemp) != 0)
	  {
	    printf("ERROR: configuration file has incorrect format\n");
	    printf("Expected %s got %s\n", sTemp1, sTemp);
	    exit(1);
	  }
	fscanf(fCfg, "%s", sTemp);
	int num_corners = atoi(sTemp);

	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "footprint:");
	if(strcmp(sTemp1, sTemp) != 0)
	  {
	    printf("ERROR: configuration file has incorrect format\n");
	    printf("Expected %s got %s\n", sTemp1, sTemp);
	    exit(1);
	  }
	
	EnvNAV3DDYN2Dpt_t pt;
	for(int c=0; c<num_corners; c++){
	  fscanf(fCfg, "%s", sTemp);
	  pt.x = atof(sTemp);
	  //	  pt.x = CONTXY2DISC(atof(sTemp),EnvNAV3DDYNCfg.cellsize_m);
	  fscanf(fCfg, "%s", sTemp);
	  pt.y = atof(sTemp);
	  //pt.y = CONTXY2DISC(atof(sTemp),EnvNAV3DDYNCfg.cellsize_m);
	  EnvNAV3DDYNCfg.FootprintPolygon.push_back(pt);
	}


	//allocate the 2D environment
	int x, y;
	EnvNAV3DDYNCfg.Grid2D = new char* [EnvNAV3DDYNCfg.EnvWidth_c];
	for (x = 0; x < EnvNAV3DDYNCfg.EnvWidth_c; x++)
	{
		EnvNAV3DDYNCfg.Grid2D[x] = new char [EnvNAV3DDYNCfg.EnvHeight_c];
	}

	//environment:
	fscanf(fCfg, "%s", sTemp);	
	for (y = 0; y < EnvNAV3DDYNCfg.EnvHeight_c; y++)
		for (x = 0; x < EnvNAV3DDYNCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				printf("ERROR: incorrect format of config file\n");
				printf("Environment definition does not match discretization specs\n");
				exit(1);
			}
			EnvNAV3DDYNCfg.Grid2D[x][y] = dTemp;
		}

#if NAV3DDYN_DEBUG
	printf("Environment: \n");
	printf("\tdiscretization(cells): %d %d\n", EnvNAV3DDYNCfg.EnvWidth_c, EnvNAV3DDYNCfg.EnvHeight_c);
	printf("\tcellsize(meters): %f\n", EnvNAV3DDYNCfg.cellsize_m);
	printf("\tnominalvel(mpersecs): %f\n", EnvNAV3DDYNCfg.nominalvel_mpersecs);
	printf("\ttimetoturnoneunitinplace(secs): %f\n", EnvNAV3DDYNCfg.timetoturnoneunitinplace_secs);
	printf("\tstart(m,rad): %f %f %d\n", DISCXY2CONT(EnvNAV3DDYNCfg.StartX_c, EnvNAV3DDYNCfg.cellsize_m), DISCXY2CONT(EnvNAV3DDYNCfg.StartY_c, EnvNAV3DDYNCfg.cellsize_m), EnvNAV3DDYNCfg.StartTheta);
	printf("\tend(m,rad): %f %f %d\n", DISCXY2CONT(EnvNAV3DDYNCfg.EndX_c, EnvNAV3DDYNCfg.cellsize_m), DISCXY2CONT(EnvNAV3DDYNCfg.EndY_c, EnvNAV3DDYNCfg.cellsize_m), EnvNAV3DDYNCfg.EndTheta);
	printf("\tnum_footprint_corners: %u\n", EnvNAV3DDYNCfg.FootprintPolygon.size());
	printf("\tfootprint\n");
	for(int c=0; c<EnvNAV3DDYNCfg.FootprintPolygon.size(); c++){
	  printf("\t\t%f %f\n", EnvNAV3DDYNCfg.FootprintPolygon[c].x, EnvNAV3DDYNCfg.FootprintPolygon[c].y);
	}
	printf("\tenvironment:\n");
	for(int h=0; h < EnvNAV3DDYNCfg.EnvHeight_c; h++){
	  printf("\t\t");
	  for(int w=0; w < EnvNAV3DDYNCfg.EnvWidth_c; w++){
	    printf("%d ", EnvNAV3DDYNCfg.Grid2D[w][h]);
	  }
	  printf("\n");
	}
#endif
}

bool EnvironmentNAV3DDYN::InitGeneral() {
  
  //Initialize other parameters of the environment
  InitializeEnvConfig();
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

void EnvironmentNAV3DDYN::InitializeEnvConfig()
{
  //precompute a list of valid actions from postion (0, 0, 0)
  PrecomputeActions();

}

void EnvironmentNAV3DDYN::InitializeEnvironment()
{
  EnvNAV3DDYNHashEntry_t* HashEntry;

  //initialize the map from Coord to StateID
  EnvNAV3DDYN.HashTableSize = 64*1024; //should be power of two
  EnvNAV3DDYN.Coord2StateIDHashTable = new vector<EnvNAV3DDYNHashEntry_t*>[EnvNAV3DDYN.HashTableSize];
	
  //initialize the map from StateID to Coord
  EnvNAV3DDYN.StateID2CoordTable.clear();

  //create start state 
  HashEntry = CreateNewHashEntry(EnvNAV3DDYNCfg.StartX_c, EnvNAV3DDYNCfg.StartY_c, EnvNAV3DDYNCfg.StartTheta);
  EnvNAV3DDYN.startstateid = HashEntry->stateID;
  
  //create goal state 
  HashEntry = CreateNewHashEntry(EnvNAV3DDYNCfg.EndX_c, EnvNAV3DDYNCfg.EndY_c, EnvNAV3DDYNCfg.EndTheta);
  EnvNAV3DDYN.goalstateid = HashEntry->stateID;
}





//------------------------------Heuristic computation--------------------------

void EnvironmentNAV3DDYN::ComputeHeuristicValues()
{
  //whatever necessary pre-computation of heuristic values is done here 
  printf("Precomputing heuristics...\n");
	


  printf("done\n");

}

/*****************************************************************/






EnvNAV3DDYNHashEntry_t* EnvironmentNAV3DDYN::CreateNewHashEntry(int X, int Y, int Theta) 
{
  int i;
  
#if TIME_DEBUG	
  clock_t currenttime = clock();
#endif

  EnvNAV3DDYNHashEntry_t* HashEntry = new EnvNAV3DDYNHashEntry_t;
  
  HashEntry->X = X;
  HashEntry->Y = Y;
  HashEntry->Theta = Theta;
  
  HashEntry->stateID = EnvNAV3DDYN.StateID2CoordTable.size();
  
  //insert into the tables
  EnvNAV3DDYN.StateID2CoordTable.push_back(HashEntry);


  //get the hash table bin
  i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta); 
  
  //insert the entry into the bin
  EnvNAV3DDYN.Coord2StateIDHashTable[i].push_back(HashEntry);
  
  //insert into and initialize the mappings
  int* entry = new int [NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for(i = 0; i < NUMOFINDICES_STATEID2IND; i++)
    {
      StateID2IndexMapping[HashEntry->stateID][i] = -1;
    }
  
  if(HashEntry->stateID != (int)StateID2IndexMapping.size()-1)
    {
      printf("ERROR in Env... function: last state has incorrect stateID\n");
      exit(1);	
    }
  
#if TIME_DEBUG
  time_createhash += clock()-currenttime;
#endif
  
  return HashEntry;
}

bool EnvironmentNAV3DDYN::IsValidCell(int X, int Y)
{
  return (X >= 0 && X < EnvNAV3DDYNCfg.EnvWidth_c && 
	  Y >= 0 && Y < EnvNAV3DDYNCfg.EnvHeight_c && 
	  EnvNAV3DDYNCfg.Grid2D[X][Y] == 0);
}

bool EnvironmentNAV3DDYN::IsWithinMapCell(int X, int Y)
{
  return (X >= 0 && X < EnvNAV3DDYNCfg.EnvWidth_c && 
	  Y >= 0 && Y < EnvNAV3DDYNCfg.EnvHeight_c);
}






//------------------------------------------------------------------------------


//------------debugging functions---------------------------------------------
bool EnvironmentNAV3DDYN::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/NAV3DDYN_THETADIRS*0.01)
    {
      int nTheta = ContTheta2Disc(theta, NAV3DDYN_THETADIRS);
      double newTheta = DiscTheta2Cont(nTheta, NAV3DDYN_THETADIRS);
      int nnewTheta = ContTheta2Disc(newTheta, NAV3DDYN_THETADIRS);
      
      fprintf(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta*180/PI_CONST, nTheta, newTheta, nnewTheta);
      
      if(nTheta != nnewTheta)
        {
	  printf("ERROR: invalid quantization\n");                     
	  return false;
        }
    }
  
  return true;
}

//---------------------------NOT YET IMPLEMENTED---------------------------------------

void EnvironmentNAV3DDYN::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  printf("ERROR in EnvNAV3DDYN... function: SetAllActionsandAllOutcomes is undefined\n");
  exit(1);
  /*
	int cost;

#if NAV3DDYN_DEBUG
	if(state->StateID >= (int)EnvNAV3DDYN.StateID2CoordTable.size())
	{
		printf("ERROR in Env... function: stateID illegal\n");
		exit(1);
	}

	if((int)state->Actions.size() != 0)
	{
		printf("ERROR in Env_setAllActionsandAllOutcomes: actions already exist for the state\n");
		exit(1);
	}
#endif
	

	//goal state should be absorbing
	if(state->StateID == EnvNAV3DDYN.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV3DDYNHashEntry_t* HashEntry = EnvNAV3DDYN.StateID2CoordTable[state->StateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV3DDYNCfg.EnvWidth_c-1 || //TODO - modify based on robot's size
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV3DDYNCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < NAV3DDYN_ACTIONWIDTH; aind++)
	{
		EnvNAV3DDYNAction_t* nav3daction = &EnvNAV3DDYNCfg.ActionsV[HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAV3DDYN_THETADIRS);

        //skip the invalid cells
        if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY))
                continue;
        }
       else if(EnvNAV3DDYNCfg.Grid2D[newX][newY] != 0) //TODO - need to modify to take robot perimeter into account
            continue;

		//skip invalid diagonal move
        if(newX != HashEntry->X && newY != HashEntry->Y) //TODO - need to modify to take robot perimeter into account
		{
			if(EnvNAV3DDYNCfg.Grid2D[HashEntry->X][newY] != 0 || EnvNAV3DDYNCfg.Grid2D[newX][HashEntry->Y] != 0)
				continue;
		}


		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

        //compute cost
		cost = nav3daction->cost;

    	EnvNAV3DDYNHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTheta);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
  */
}

void EnvironmentNAV3DDYN::SetAllPreds(CMDPSTATE* state)
{
  //implement this if the planner needs access to predecessors
  
  printf("ERROR in EnvNAV3DDYN... function: SetAllPreds is undefined\n");
  exit(1);
}

void EnvironmentNAV3DDYN::PrintEnv_Config(FILE* fOut)
{

  //implement this if the planner needs to print out EnvNAV3DDYN. configuration
  
  printf("ERROR in EnvNAV3DDYN... function: PrintEnv_Config is undefined\n");
  exit(1);
  
}

void EnvironmentNAV3DDYN::GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV)
{

  printf("ERROR in EnvNAV3DDYN... function: GetPredsofChangedEdges is undefined\n");
  exit(1);
  /*	nav2dcell_t cell;

	for(int i = 0; i < (int)changedcellsV->size(); i++)
	{
		cell = changedcellsV->at(i);
		for(int tind = 0; tind < NAV3DDYN_THETADIRS; tind++)
			preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x,cell.y,tind));
		for(int j = 0; j < 8; j++){
			int affx = cell.x + EnvNAV3DDYNCfg.dXY[j][0];
			int affy = cell.y + EnvNAV3DDYNCfg.dXY[j][1];
			if(affx < 0 || affx >= EnvNAV3DDYNCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV3DDYNCfg.EnvHeight_c)
				continue;
			for(int tind = 0; tind < NAV3DDYN_THETADIRS; tind++)
				preds_of_changededgesIDV->push_back(GetStateFromCoord(affx,affy,tind));
		}
	}
  */
}
