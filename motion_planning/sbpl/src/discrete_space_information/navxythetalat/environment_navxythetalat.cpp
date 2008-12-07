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
#include "../../sbpl/headers.h"


#if TIME_DEBUG
static clock_t time3_addallout = 0;
static clock_t time_gethash = 0;
static clock_t time_createhash = 0;
static clock_t time_getsuccs = 0;
#endif

//-----------------constructors/destructors-------------------------------
EnvironmentNAVXYTHETALAT::EnvironmentNAVXYTHETALAT()
{
	EnvNAVXYTHETALATCfg.obsthresh = ENVNAVXYTHETALAT_DEFAULTOBSTHRESH;

	grid2Dsearch = NULL;
	bNeedtoRecomputeStartHeuristics = true;


	EnvNAVXYTHETALATCfg.actionwidth = NAVXYTHETALAT_DEFAULT_ACTIONWIDTH;
}

EnvironmentNAVXYTHETALAT::~EnvironmentNAVXYTHETALAT()
{
	if(grid2Dsearch != NULL)
		delete grid2Dsearch;
	grid2Dsearch = NULL;
}

//---------------------------------------------------------------------


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
unsigned int EnvironmentNAVXYTHETALAT::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)) & (EnvNAVXYTHETALAT.HashTableSize-1);
}



void EnvironmentNAVXYTHETALAT::PrintHashTableHist()
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < EnvNAVXYTHETALAT.HashTableSize; j++)
	{
		if((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}

void EnvironmentNAVXYTHETALAT::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta,
					int goalx, int goaly, int goaltheta,
					double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV) {
  EnvNAVXYTHETALATCfg.EnvWidth_c = width;
  EnvNAVXYTHETALATCfg.EnvHeight_c = height;
  EnvNAVXYTHETALATCfg.StartX_c = startx;
  EnvNAVXYTHETALATCfg.StartY_c = starty;
  EnvNAVXYTHETALATCfg.StartTheta = starttheta;
 
  if(EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= NAVXYTHETALAT_THETADIRS) {
    printf("ERROR: illegal start coordinates for theta\n");
    exit(1);
  }
  
  EnvNAVXYTHETALATCfg.EndX_c = goalx;
  EnvNAVXYTHETALATCfg.EndY_c = goaly;
  EnvNAVXYTHETALATCfg.EndTheta = goaltheta;

  if(EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= NAVXYTHETALAT_THETADIRS) {
    printf("ERROR: illegal goal coordinates for theta\n");
    exit(1);
  }

  EnvNAVXYTHETALATCfg.FootprintPolygon = robot_perimeterV;

  EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAVXYTHETALATCfg.cellsize_m = cellsize_m;
  EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment
  EnvNAVXYTHETALATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
    EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
  }
  
  //environment:
  if (0 == mapdata) {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
	EnvNAVXYTHETALATCfg.Grid2D[x][y] = 0;
      }
    }
  }
  else {
    for (int y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++) {
      for (int x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++) {
	char cval = mapdata[x+y*width];
	if(cval == 1) {
	  EnvNAVXYTHETALATCfg.Grid2D[x][y] = 1;
	} else {
	  EnvNAVXYTHETALATCfg.Grid2D[x][y] = 0;
	}
      }
    }
  }
}

void EnvironmentNAVXYTHETALAT::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAVXYTHETALATCfg structure
	char sTemp[1024], sTemp1[1024];
	int dTemp;
	int x, y;

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
	EnvNAVXYTHETALATCfg.EnvWidth_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.EnvHeight_c = atoi(sTemp);

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
	EnvNAVXYTHETALATCfg.cellsize_m = atof(sTemp);
	
	//speeds
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "nominalvel(mpersecs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.nominalvel_mpersecs = atof(sTemp);
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "timetoturn45degsinplace(secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = atof(sTemp);


	//start(meters,rads): 
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.StartTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETALAT_THETADIRS);


	if(EnvNAVXYTHETALATCfg.StartX_c < 0 || EnvNAVXYTHETALATCfg.StartX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAVXYTHETALATCfg.StartY_c < 0 || EnvNAVXYTHETALATCfg.StartY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAVXYTHETALATCfg.StartTheta < 0 || EnvNAVXYTHETALATCfg.StartTheta >= NAVXYTHETALAT_THETADIRS) {
		printf("ERROR: illegal start coordinates for theta\n");
		exit(1);
	}

	//end(meters,rads): 
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAVXYTHETALATCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAVXYTHETALATCfg.EndTheta = ContTheta2Disc(atof(sTemp), NAVXYTHETALAT_THETADIRS);;

	if(EnvNAVXYTHETALATCfg.EndX_c < 0 || EnvNAVXYTHETALATCfg.EndX_c >= EnvNAVXYTHETALATCfg.EnvWidth_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAVXYTHETALATCfg.EndY_c < 0 || EnvNAVXYTHETALATCfg.EndY_c >= EnvNAVXYTHETALATCfg.EnvHeight_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAVXYTHETALATCfg.EndTheta < 0 || EnvNAVXYTHETALATCfg.EndTheta >= NAVXYTHETALAT_THETADIRS) {
		printf("ERROR: illegal goal coordinates for theta\n");
		exit(1);
	}


	//allocate the 2D environment
	EnvNAVXYTHETALATCfg.Grid2D = new unsigned char* [EnvNAVXYTHETALATCfg.EnvWidth_c];
	for (x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
	{
		EnvNAVXYTHETALATCfg.Grid2D[x] = new unsigned char [EnvNAVXYTHETALATCfg.EnvHeight_c];
	}

	//environment:
	fscanf(fCfg, "%s", sTemp);	
	for (y = 0; y < EnvNAVXYTHETALATCfg.EnvHeight_c; y++)
		for (x = 0; x < EnvNAVXYTHETALATCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				printf("ERROR: incorrect format of config file\n");
				exit(1);
			}
			EnvNAVXYTHETALATCfg.Grid2D[x][y] = dTemp;
		}

}


void EnvironmentNAVXYTHETALAT::ComputeReplanningDataforAction(EnvNAVXYTHETALATAction_t* action)
{
	int j;

	//iterate over all the cells involved in the action
	EnvNAVXYTHETALAT3Dcell_t startcell3d, endcell3d;
	for(int i = 0; i < (int)action->intersectingcellsV.size(); i++)
	{

		//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
		startcell3d.theta = action->starttheta;
		startcell3d.x = - action->intersectingcellsV.at(i).x;
		startcell3d.y = - action->intersectingcellsV.at(i).y;

		//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
		endcell3d.theta = NORMALIZEDISCTHETA(startcell3d.theta + action->dTheta, NAVXYTHETALAT_THETADIRS); 
		endcell3d.x = startcell3d.x + action->dX; 
		endcell3d.y = startcell3d.y + action->dY;

		//store the cells if not already there
		for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
		{
			if(affectedsuccstatesV.at(j) == endcell3d)
				break;
		}
		if (j == (int)affectedsuccstatesV.size())
			affectedsuccstatesV.push_back(endcell3d);

		for(j = 0; j < (int)affectedpredstatesV.size(); j++)
		{
			if(affectedpredstatesV.at(j) == startcell3d)
				break;
		}
		if (j == (int)affectedpredstatesV.size())
			affectedpredstatesV.push_back(startcell3d);

    }//over intersecting cells

	

	//add the centers since with h2d we are using these in cost computations
	//---intersecting cell = origin
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell3d.theta = action->starttheta;
	startcell3d.x = - 0;
	startcell3d.y = - 0;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell3d.theta = NORMALIZEDISCTHETA(startcell3d.theta + action->dTheta, NAVXYTHETALAT_THETADIRS); 
	endcell3d.x = startcell3d.x + action->dX; 
	endcell3d.y = startcell3d.y + action->dY;

	//store the cells if not already there
	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell3d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell3d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell3d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell3d);


	//---intersecting cell = outcome state
	//compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
	startcell3d.theta = action->starttheta;
	startcell3d.x = - action->dX;
	startcell3d.y = - action->dY;

	//compute the translated affected search Pose - what state has an incoming action whose intersecting cell is at 0,0
	endcell3d.theta = NORMALIZEDISCTHETA(startcell3d.theta + action->dTheta, NAVXYTHETALAT_THETADIRS); 
	endcell3d.x = startcell3d.x + action->dX; 
	endcell3d.y = startcell3d.y + action->dY;

	for(j = 0; j < (int)affectedsuccstatesV.size(); j++)
	{
		if(affectedsuccstatesV.at(j) == endcell3d)
			break;
	}
	if (j == (int)affectedsuccstatesV.size())
		affectedsuccstatesV.push_back(endcell3d);

	for(j = 0; j < (int)affectedpredstatesV.size(); j++)
	{
		if(affectedpredstatesV.at(j) == startcell3d)
			break;
	}
	if (j == (int)affectedpredstatesV.size())
		affectedpredstatesV.push_back(startcell3d);


}


//computes all the 3D states whose outgoing actions are potentially affected when cell (0,0) changes its status
//it also does the same for the 3D states whose incoming actions are potentially affected when cell (0,0) changes its status
void EnvironmentNAVXYTHETALAT::ComputeReplanningData()
{

    //iterate over all actions
	//orientations
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
    {        
        //actions
		for(int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
		{
            //compute replanning data for this action 
			ComputeReplanningDataforAction(&EnvNAVXYTHETALATCfg.ActionsV[tind][aind]);
		}
	}
}


void EnvironmentNAVXYTHETALAT::PrecomputeActions(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
	//TODO - compute and store in motionprimitiveV
	EnvNAVXYTHETALATCfg.actionwidth = motionprimitiveV->size();

	printf("Pre-computing action data...\n");
	EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t* [NAVXYTHETALAT_THETADIRS];
	EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [NAVXYTHETALAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;

	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
		EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[motionprimitiveV->size()];

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);


		//iterate over motion primitives
		for(int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
		{
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
			double endangle = sourcepose.theta + motionprimitiveV->at(aind).endtheta_rad;
			double endx = sourcepose.x + (motionprimitiveV->at(aind).endx_m*cos(sourcepose.theta) - motionprimitiveV->at(aind).endy_m*sin(sourcepose.theta));
			double endy = sourcepose.y + (motionprimitiveV->at(aind).endx_m*sin(sourcepose.theta) + motionprimitiveV->at(aind).endy_m*cos(sourcepose.theta));
			
			int endx_c = CONTXY2DISC(endx, EnvNAVXYTHETALATCfg.cellsize_m);
			int endy_c = CONTXY2DISC(endy, EnvNAVXYTHETALATCfg.cellsize_m);

			
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta = ContTheta2Disc(motionprimitiveV->at(aind).endtheta_rad, NAVXYTHETALAT_THETADIRS);
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = endx_c;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = endy_c;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.cellsize_m/EnvNAVXYTHETALATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + 
					EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			int endangle_c = NORMALIZEDISCTHETA(tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, NAVXYTHETALAT_THETADIRS);
			EnvNAVXYTHETALAT3Dpt_t pose;
			pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
			pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
			pose.theta = DiscTheta2Cont(endangle_c, NAVXYTHETALAT_THETADIRS);
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			fprintf(fDeb, "action tind=%d aind=%d: dX=%d dY=%d dTheta=%d (%.2f degs -> %.2f degs) cost=%d (mprim: %.2f %.2f %.2f)\n",
				tind, aind, 			
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, sourcepose.theta*180/PI_CONST, pose.theta*180/PI_CONST,	
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost,
				motionprimitiveV->at(aind).endx_m, motionprimitiveV->at(aind).endy_m, motionprimitiveV->at(aind).endtheta_rad);
#endif

			//add to the list of backward actions
			int targettheta = (tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta)%NAVXYTHETALAT_THETADIRS;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
			 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

		}
	}

	//now compute replanning data
	ComputeReplanningData();

	printf("done pre-computing action data based on motion primitives\n");


}

void EnvironmentNAVXYTHETALAT::PrecomputeActions()
{
	//TODO - compute and store in motionprimitiveV

	//construct list of actions
	printf("Pre-computing action data...\n");
	EnvNAVXYTHETALATCfg.ActionsV = new EnvNAVXYTHETALATAction_t* [NAVXYTHETALAT_THETADIRS];
	EnvNAVXYTHETALATCfg.PredActionsV = new vector<EnvNAVXYTHETALATAction_t*> [NAVXYTHETALAT_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < NAVXYTHETALAT_THETADIRS; tind++)
	{
		EnvNAVXYTHETALATCfg.ActionsV[tind] = new EnvNAVXYTHETALATAction_t[EnvNAVXYTHETALATCfg.actionwidth];

		//compute sourcepose
		EnvNAVXYTHETALAT3Dpt_t sourcepose;
		sourcepose.x = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.y = DISCXY2CONT(0, EnvNAVXYTHETALATCfg.cellsize_m);
		sourcepose.theta = DiscTheta2Cont(tind, NAVXYTHETALAT_THETADIRS);

		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta = aind-1; //-1,0,1
			double angle = DiscTheta2Cont(tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, NAVXYTHETALAT_THETADIRS);
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(ceil(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.cellsize_m/EnvNAVXYTHETALATCfg.nominalvel_mpersecs*sqrt((double)(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX + 
					EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY*EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY))));

			//compute intersecting cells
			EnvNAVXYTHETALAT3Dpt_t pose;
			pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
			pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
			pose.theta = angle;
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
			RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			printf("action tind=%d aind=%d: dTheta=%d (%f) dX=%d dY=%d cost=%d\n",
				tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, angle, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
				EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

			//add to the list of backward actions
			int targettheta = (tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta)%NAVXYTHETALAT_THETADIRS;
			if (targettheta < 0)
				targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
			 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

		}

		//decrease and increase angle without movement
		aind = 3;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta = -1; 
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		EnvNAVXYTHETALAT3Dpt_t pose;
		pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, NAVXYTHETALAT_THETADIRS);
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		printf("action tind=%d aind=%d: dTheta=%d (%f) dX=%d dY=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, DiscTheta2Cont(tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, NAVXYTHETALAT_THETADIRS),
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		int targettheta = (tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta)%NAVXYTHETALAT_THETADIRS;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
		 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));


		aind = 4;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].starttheta = tind;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta = 1; 
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY = 0;
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY, EnvNAVXYTHETALATCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, NAVXYTHETALAT_THETADIRS);
		EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);
		RemoveSourceFootprint(sourcepose, &EnvNAVXYTHETALATCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		printf("action tind=%d aind=%d: dTheta=%d (%f) dX=%d dY=%d cost=%d\n",
			tind, aind, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, DiscTheta2Cont(tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta, NAVXYTHETALAT_THETADIRS),
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dX, EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dY,
			EnvNAVXYTHETALATCfg.ActionsV[tind][aind].cost);
#endif

		//add to the list of backward actions
		targettheta = (tind + EnvNAVXYTHETALATCfg.ActionsV[tind][aind].dTheta)%NAVXYTHETALAT_THETADIRS;
		if (targettheta < 0)
			targettheta = targettheta + NAVXYTHETALAT_THETADIRS;
		 EnvNAVXYTHETALATCfg.PredActionsV[targettheta].push_back(&(EnvNAVXYTHETALATCfg.ActionsV[tind][aind]));

	}

	//now compute replanning data
	ComputeReplanningData();

	printf("done pre-computing action data\n");


}



void EnvironmentNAVXYTHETALAT::InitializeEnvConfig(vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
	//aditional to configuration file initialization of EnvNAVXYTHETALATCfg if necessary

	//dXY dirs
	EnvNAVXYTHETALATCfg.dXY[0][0] = -1;
	EnvNAVXYTHETALATCfg.dXY[0][1] = -1;
	EnvNAVXYTHETALATCfg.dXY[1][0] = -1;
	EnvNAVXYTHETALATCfg.dXY[1][1] = 0;
	EnvNAVXYTHETALATCfg.dXY[2][0] = -1;
	EnvNAVXYTHETALATCfg.dXY[2][1] = 1;
	EnvNAVXYTHETALATCfg.dXY[3][0] = 0;
	EnvNAVXYTHETALATCfg.dXY[3][1] = -1;
	EnvNAVXYTHETALATCfg.dXY[4][0] = 0;
	EnvNAVXYTHETALATCfg.dXY[4][1] = 1;
	EnvNAVXYTHETALATCfg.dXY[5][0] = 1;
	EnvNAVXYTHETALATCfg.dXY[5][1] = -1;
	EnvNAVXYTHETALATCfg.dXY[6][0] = 1;
	EnvNAVXYTHETALATCfg.dXY[6][1] = 0;
	EnvNAVXYTHETALATCfg.dXY[7][0] = 1;
	EnvNAVXYTHETALATCfg.dXY[7][1] = 1;

	if(motionprimitiveV == NULL)
		PrecomputeActions();
	else
		PrecomputeActions(motionprimitiveV);


}



EnvNAVXYTHETALATHashEntry_t* EnvironmentNAVXYTHETALAT::GetHashEntry(int X, int Y, int Theta)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y, Theta);
	
#if DEBUG
	if ((int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid].size() > 500)
	{
		printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid].size(); ind++)
	{
		if( EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid][ind]->X == X 
			&& EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid][ind]->Y == Y
			&& EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid][ind]->Theta == Theta)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return EnvNAVXYTHETALAT.Coord2StateIDHashTable[binid][ind];
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


EnvNAVXYTHETALATHashEntry_t* EnvironmentNAVXYTHETALAT::CreateNewHashEntry(int X, int Y, int Theta) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAVXYTHETALATHashEntry_t* HashEntry = new EnvNAVXYTHETALATHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;

	HashEntry->stateID = EnvNAVXYTHETALAT.StateID2CoordTable.size();

	//insert into the tables
	EnvNAVXYTHETALAT.StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta); 

	//insert the entry into the bin
	EnvNAVXYTHETALAT.Coord2StateIDHashTable[i].push_back(HashEntry);

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

bool EnvironmentNAVXYTHETALAT::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c && 
		EnvNAVXYTHETALATCfg.Grid2D[X][Y] < EnvNAVXYTHETALATCfg.obsthresh);
}

bool EnvironmentNAVXYTHETALAT::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAVXYTHETALATCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

bool EnvironmentNAVXYTHETALAT::IsValidConfiguration(int X, int Y, int Theta)
{
	vector<sbpl_2Dcell_t> footprint;
	EnvNAVXYTHETALAT3Dpt_t pose;

	//compute continuous pose
	pose.x = DISCXY2CONT(X, EnvNAVXYTHETALATCfg.cellsize_m);
	pose.y = DISCXY2CONT(Y, EnvNAVXYTHETALATCfg.cellsize_m);
	pose.theta = DiscTheta2Cont(Theta, NAVXYTHETALAT_THETADIRS);

	//compute footprint cells
	CalculateFootprintForPose(pose, &footprint);

	//iterate over all footprint cells
	for(int find = 0; find < (int)footprint.size(); find++)
	{
		int x = footprint.at(find).x;
		int y = footprint.at(find).y;

		if (x < 0 || x >= EnvNAVXYTHETALATCfg.EnvWidth_c ||
			y < 0 || Y >= EnvNAVXYTHETALATCfg.EnvHeight_c ||		
			EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh)
		{
			return false;
		}
	}

	return true;
}


int EnvironmentNAVXYTHETALAT::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action)
{
	sbpl_2Dcell_t cell;

	//TODO - go over bounding box (minpt and maxpt) to test validity and skip testing boundaries below, also order intersect cells so that the four farthest pts go first
	
	if(!IsValidCell(SourceX, SourceY))
		return INFINITECOST;

	int currentmaxcost = 0;
	for(int i = 0; i < (int)action->intersectingcellsV.size(); i++) 
	{
		//get the cell in the map
		cell = action->intersectingcellsV.at(i);
		cell.x = cell.x + SourceX;
		cell.y = cell.y + SourceY;
		
		//check validity
		if(!IsValidCell(cell.x, cell.y))
			return INFINITECOST;

		if(EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y] > currentmaxcost)
			currentmaxcost = EnvNAVXYTHETALATCfg.Grid2D[cell.x][cell.y];
	}

	//to ensure consistency of h2D:
	currentmaxcost = __max(currentmaxcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX][SourceY]);
	if(!IsValidCell(SourceX + action->dX, SourceY + action->dY))
		return INFINITECOST;
	currentmaxcost = __max(currentmaxcost, EnvNAVXYTHETALATCfg.Grid2D[SourceX + action->dX][SourceY + action->dY]);


	return action->cost*(currentmaxcost+1); //use cell cost as multiplicative factor
 
}



void EnvironmentNAVXYTHETALAT::InitializeEnvironment()
{
	EnvNAVXYTHETALATHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	EnvNAVXYTHETALAT.HashTableSize = 64*1024; //should be power of two
	EnvNAVXYTHETALAT.Coord2StateIDHashTable = new vector<EnvNAVXYTHETALATHashEntry_t*>[EnvNAVXYTHETALAT.HashTableSize];
	
	//initialize the map from StateID to Coord
	EnvNAVXYTHETALAT.StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.StartTheta);
	EnvNAVXYTHETALAT.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.EndTheta);
	EnvNAVXYTHETALAT.goalstateid = HashEntry->stateID;
}

double EnvironmentNAVXYTHETALAT::EuclideanDistance_m(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    return EnvNAVXYTHETALATCfg.cellsize_m*sqrt((double)sqdist);

}



void EnvironmentNAVXYTHETALAT::CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint)
{  

#if DEBUG
  printf("---Calculating Footprint for Pose: %f %f %f---\n",
	 pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(EnvNAVXYTHETALATCfg.FootprintPolygon.size() <= 1){
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, EnvNAVXYTHETALATCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, EnvNAVXYTHETALATCfg.cellsize_m);
    footprint->push_back(cell);
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
  sbpl_2Dpt_t pt = {0,0};
  for(find = 0; find < EnvNAVXYTHETALATCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAVXYTHETALATCfg.FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.theta)*pt.x - sin(pose.theta)*pt.y + pose.x;
    corner.y = sin(pose.theta)*pt.x + cos(pose.theta)*pt.y + pose.y;
    bounding_polygon.push_back(corner);
#if DEBUG
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

#if DEBUG
  printf("Footprint bounding box: %f %f %f %f\n", min_x, max_x, min_y, max_y);
#endif
  //initialize previous values to something that will fail the if condition during the first iteration in the for loop
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETALATCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETALATCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=EnvNAVXYTHETALATCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAVXYTHETALATCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAVXYTHETALATCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAVXYTHETALATCfg.cellsize_m);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){

#if DEBUG
		printf("Testing point: %f %f Discrete: %d %d\n", pt.x, pt.y, discrete_x, discrete_y);
#endif
	
		if(IsInsideFootprint(pt, &bounding_polygon)){
		//convert to a grid point

#if DEBUG
			printf("Pt Inside %f %f\n", pt.x, pt.y);
#endif

			sbpl_2Dcell_t cell;
			cell.x = discrete_x;
			cell.y = discrete_y;
			footprint->push_back(cell);
			prev_inside = 1;

#if DEBUG
			printf("Added pt to footprint: %f %f\n", pt.x, pt.y);
#endif
		}
		else{
			prev_inside = 0;
		}

      }
	  else
	  {
#if DEBUG
		//rintf("Skipping pt: %f %f\n", pt.x, pt.y);
#endif
      }
      
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;

    }//over x_min...x_max
  }
}


void EnvironmentNAVXYTHETALAT::RemoveSourceFootprint(EnvNAVXYTHETALAT3Dpt_t sourcepose, vector<sbpl_2Dcell_t>* footprint)
{  
	vector<sbpl_2Dcell_t> sourcefootprint;

	//compute source footprint
	CalculateFootprintForPose(sourcepose, &sourcefootprint);

	//now remove the source cells from the footprint
	for(int sind = 0; sind < (int)sourcefootprint.size(); sind++)
	{
		for(int find = 0; find < (int)footprint->size(); find++)
		{
			if(sourcefootprint.at(sind).x == footprint->at(find).x && sourcefootprint.at(sind).y == footprint->at(find).y)
			{
				footprint->erase(footprint->begin() + find);
				break;
			}
		}//over footprint
	}//over source



}


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAVXYTHETALAT::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	printf("Precomputing heuristics...\n");
	
	//allocated 2D grid search
	grid2Dsearch = new SBPL2DGridSearch(EnvNAVXYTHETALATCfg.EnvWidth_c, EnvNAVXYTHETALATCfg.EnvHeight_c, (float)EnvNAVXYTHETALATCfg.cellsize_m);

	printf("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNAVXYTHETALAT::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/NAVXYTHETALAT_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, NAVXYTHETALAT_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, NAVXYTHETALAT_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, NAVXYTHETALAT_THETADIRS);

		fprintf(fOut, "theta=%f(%f)->%d->%f->%d\n", theta, theta*180/PI_CONST, nTheta, newTheta, nnewTheta);

        if(nTheta != nnewTheta)
        {
            printf("ERROR: invalid quantization\n");                     
            return false;
        }
    }

  return true;
}



//-----------------------------------------------------------------------------

//-----------interface with outside functions-----------------------------------
bool EnvironmentNAVXYTHETALAT::InitializeEnv(const char* sEnvFile, vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	ReadConfiguration(fCfg);

	InitGeneral(motionprimitiveV);


	return true;
}


bool EnvironmentNAVXYTHETALAT::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	ReadConfiguration(fCfg);

	InitGeneral(NULL);


	return true;
}



bool EnvironmentNAVXYTHETALAT::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta,
					double goalx, double goaly, double goaltheta,
				    double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
					unsigned char obsthresh,  vector<SBPL_xytheta_mprimitive>* motionprimitiveV)
{
	EnvNAVXYTHETALATCfg.obsthresh = obsthresh;

	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, NAVXYTHETALAT_THETADIRS),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, NAVXYTHETALAT_THETADIRS),
					cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

	InitGeneral(motionprimitiveV);

	return true;
}


bool EnvironmentNAVXYTHETALAT::InitGeneral(vector<SBPL_xytheta_mprimitive>* motionprimitiveV) {


  //Initialize other parameters of the environment
  InitializeEnvConfig(motionprimitiveV);
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAVXYTHETALAT::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAVXYTHETALAT.goalstateid;
	MDPCfg->startstateid = EnvNAVXYTHETALAT.startstateid;

	return true;
}



int EnvironmentNAVXYTHETALAT::GetFromToHeuristic(int FromStateID, int ToStateID)
{

#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)EnvNAVXYTHETALAT.StateID2CoordTable.size() 
		|| ToStateID >= (int)EnvNAVXYTHETALAT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		exit(1);
	}
#endif

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* FromHashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[FromStateID];
	EnvNAVXYTHETALATHashEntry_t* ToHashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[ToStateID];
	

	return (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y)/EnvNAVXYTHETALATCfg.nominalvel_mpersecs);	

}


int EnvironmentNAVXYTHETALAT::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvNAVXYTHETALAT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		exit(1);
	}
#endif


	//define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvNAVXYTHETALAT.goalstateid);

}


int EnvironmentNAVXYTHETALAT::GetStartHeuristic(int stateID)
{


#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)EnvNAVXYTHETALAT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal\n");
		exit(1);
	}
#endif


	if(bNeedtoRecomputeStartHeuristics)
	{
		grid2Dsearch->search(EnvNAVXYTHETALATCfg.Grid2D, EnvNAVXYTHETALATCfg.obsthresh, 
			EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.EndY_c, SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH);
		bNeedtoRecomputeStartHeuristics = false;
	}

	EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[stateID];
	int h2D = grid2Dsearch->getlowerboundoncostfromstart_inmm(HashEntry->X, HashEntry->Y);
	int hEuclid = (int)(NAVXYTHETALAT_COSTMULT_MTOMM*EuclideanDistance_m(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.StartY_c, HashEntry->X, HashEntry->Y));
		

#if DEBUG
	fprintf(fDeb, "h2D = %d hEuclid = %d\n", h2D, hEuclid);
#endif

	//define this function if it is used in the planner (heuristic backward search would use it)
    return (int)(((double)__max(h2D,hEuclid))/EnvNAVXYTHETALATCfg.nominalvel_mpersecs); 

}



void EnvironmentNAVXYTHETALAT::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)EnvNAVXYTHETALAT.StateID2CoordTable.size())
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
	if(state->StateID == EnvNAVXYTHETALAT.goalstateid)
		return;

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[state->StateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAVXYTHETALATCfg.EnvWidth_c-1 || //TODO - modify based on robot's size
       HashEntry->Y == 0 || HashEntry->Y == EnvNAVXYTHETALATCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAVXYTHETALAT_THETADIRS);	

        //skip the invalid cells
        if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY))
                continue;
        }

		//get cost
		cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if(cost >= INFINITECOST)
            continue;

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    	EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
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
}



void EnvironmentNAVXYTHETALAT::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	printf("ERROR in EnvNAVXYTHETALAT... function: SetAllPreds is undefined\n");
	exit(1);
}


void EnvironmentNAVXYTHETALAT::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int aind;

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth); 
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);

	//goal state should be absorbing
	if(SourceStateID == EnvNAVXYTHETALAT.goalstateid)
		return;

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[SourceStateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAVXYTHETALATCfg.EnvWidth_c-1 ||  //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAVXYTHETALATCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < EnvNAVXYTHETALATCfg.actionwidth; aind++)
	{
		EnvNAVXYTHETALATAction_t* nav3daction = &EnvNAVXYTHETALATCfg.ActionsV[HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAVXYTHETALAT_THETADIRS);	

        //skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY)) 
                continue;
        }

		//get cost
		int cost = GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction);
        if(cost >= INFINITECOST)
            continue;

    	EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTheta);
		}

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNAVXYTHETALAT::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(EnvNAVXYTHETALATCfg.actionwidth); 
    CostV->reserve(EnvNAVXYTHETALATCfg.actionwidth);

	//get X, Y for the state
	EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[TargetStateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAVXYTHETALATCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAVXYTHETALATCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < (int)EnvNAVXYTHETALATCfg.PredActionsV[HashEntry->Theta].size(); aind++)
	{

		EnvNAVXYTHETALATAction_t* nav3daction = EnvNAVXYTHETALATCfg.PredActionsV[HashEntry->Theta].at(aind);

        int predX = HashEntry->X - nav3daction->dX;
		int predY = HashEntry->Y - nav3daction->dY;
		int predTheta = NORMALIZEDISCTHETA(HashEntry->Theta - nav3daction->dTheta, NAVXYTHETALAT_THETADIRS);	
	
	
		//skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(predX, predY))
                continue;
        }

		//get cost
		int cost = GetActionCost(predX, predY, predTheta, nav3daction);
	    if(cost >= INFINITECOST)
			continue;
        
    	EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(predX, predY, predTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(predX, predY, predTheta);
		}

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}



int EnvironmentNAVXYTHETALAT::SizeofCreatedEnv()
{
	return (int)EnvNAVXYTHETALAT.StateID2CoordTable.size();
	
}

void EnvironmentNAVXYTHETALAT::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)EnvNAVXYTHETALAT.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAVXYTHETALAT... function: stateID illegal (2)\n");
		exit(1);
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[stateID];

	if(stateID == EnvNAVXYTHETALAT.goalstateid && bVerbose)
	{
		fprintf(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	fprintf(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
    else
    	fprintf(fOut, "%.3f %.3f %.3f\n", DISCXY2CONT(HashEntry->X, EnvNAVXYTHETALATCfg.cellsize_m), DISCXY2CONT(HashEntry->Y,EnvNAVXYTHETALATCfg.cellsize_m), 
		DiscTheta2Cont(HashEntry->Theta, NAVXYTHETALAT_THETADIRS));

}

void EnvironmentNAVXYTHETALAT::GetCoordFromState(int stateID, int& x, int& y, int& theta) const {
  EnvNAVXYTHETALATHashEntry_t* HashEntry = EnvNAVXYTHETALAT.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
}

int EnvironmentNAVXYTHETALAT::GetStateFromCoord(int x, int y, int theta) {

   EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta);
    }
    return OutHashEntry->stateID;
}

const EnvNAVXYTHETALATConfig_t* EnvironmentNAVXYTHETALAT::GetEnvNavConfig() {
  return &EnvNAVXYTHETALATCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETALAT::SetGoal(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);

	if(!IsWithinMapCell(x,y))
	{
		printf("ERROR: trying to set a goal cell %d %d that is outside of map\n", x,y);
		return -1;
	}

    if(!IsValidConfiguration(x,y,theta))
	{
		printf("WARNING: goal configuration is invalid\n");
	}

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta);
    }
    EnvNAVXYTHETALAT.goalstateid = OutHashEntry->stateID;

	EnvNAVXYTHETALATCfg.EndX_c = x;
	EnvNAVXYTHETALATCfg.EndY_c = y;
	EnvNAVXYTHETALATCfg.EndTheta = theta;


    return EnvNAVXYTHETALAT.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAVXYTHETALAT::SetStart(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, EnvNAVXYTHETALATCfg.cellsize_m);
	int y = CONTXY2DISC(y_m, EnvNAVXYTHETALATCfg.cellsize_m); 
	int theta = ContTheta2Disc(theta_rad, NAVXYTHETALAT_THETADIRS);

	if(!IsWithinMapCell(x,y))
	{
		printf("ERROR: trying to set a start cell %d %d that is outside of map\n", x,y);
		return -1;
	}

    if(!IsValidConfiguration(x,y,theta))
	{
		printf("WARNING: start configuration %d %d %d is invalid\n", x,y,theta);
	}

    EnvNAVXYTHETALATHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta);
    }

	//need to recompute start heuristics?
	if(EnvNAVXYTHETALAT.startstateid != OutHashEntry->stateID)
		bNeedtoRecomputeStartHeuristics = true;

	//set start
    EnvNAVXYTHETALAT.startstateid = OutHashEntry->stateID;
	EnvNAVXYTHETALATCfg.StartX_c = x;
	EnvNAVXYTHETALATCfg.StartY_c = y;
	EnvNAVXYTHETALATCfg.StartTheta = theta;

    return EnvNAVXYTHETALAT.startstateid;    

}

bool EnvironmentNAVXYTHETALAT::UpdateCost(int x, int y, unsigned char newcost)
{

#if DEBUG
	fprintf(fDeb, "Cost updated for cell %d %d from old cost=%d to new cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y], newcost);
#endif

    EnvNAVXYTHETALATCfg.Grid2D[x][y] = newcost;

	bNeedtoRecomputeStartHeuristics = true;

    return true;
}


void EnvironmentNAVXYTHETALAT::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAVXYTHETALAT. configuration
	
	printf("ERROR in EnvNAVXYTHETALAT... function: PrintEnv_Config is undefined\n");
	exit(1);

}

void EnvironmentNAVXYTHETALAT::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    fprintf(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}


void EnvironmentNAVXYTHETALAT::GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;
	EnvNAVXYTHETALAT3Dcell_t affectedcell;
	EnvNAVXYTHETALATHashEntry_t* affectedHashEntry;


	for(int i = 0; i < (int)changedcellsV->size(); i++) 
	{
		cell = changedcellsV->at(i);
			
		//now iterate over all states that could potentially be affected
		for(int sind = 0; sind < (int)affectedpredstatesV.size(); sind++)
		{
			affectedcell = affectedpredstatesV.at(sind);

			//translate to correct for the offset
			affectedcell.x = affectedcell.x + cell.x;
			affectedcell.y = affectedcell.y + cell.y;

			//insert only if it was actually generated
		    affectedHashEntry = GetHashEntry(affectedcell.x, affectedcell.y, affectedcell.theta);
			if(affectedHashEntry != NULL)
				preds_of_changededgesIDV->push_back(affectedHashEntry->stateID);
		}
	}
}


bool EnvironmentNAVXYTHETALAT::IsObstacle(int x, int y)
{

#if DEBUG
	fprintf(fDeb, "Status of cell %d %d is queried. Its cost=%d\n", x,y,EnvNAVXYTHETALATCfg.Grid2D[x][y]);
#endif


	return (EnvNAVXYTHETALATCfg.Grid2D[x][y] >= EnvNAVXYTHETALATCfg.obsthresh); 

}

void EnvironmentNAVXYTHETALAT::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* goalx, double* goaly, double* goaltheta,
									  	double* cellsize_m, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs, unsigned char* obsthresh,
										vector<SBPL_xytheta_mprimitive>* mprimitiveV)
{
	*size_x = EnvNAVXYTHETALATCfg.EnvWidth_c;
	*size_y = EnvNAVXYTHETALATCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAVXYTHETALATCfg.StartX_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAVXYTHETALATCfg.StartY_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.StartTheta, NAVXYTHETALAT_THETADIRS);
	*goalx = DISCXY2CONT(EnvNAVXYTHETALATCfg.EndX_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAVXYTHETALATCfg.EndY_c, EnvNAVXYTHETALATCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(EnvNAVXYTHETALATCfg.EndTheta, NAVXYTHETALAT_THETADIRS);;

	*cellsize_m = EnvNAVXYTHETALATCfg.cellsize_m;
	*nominalvel_mpersecs = EnvNAVXYTHETALATCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs;

	*obsthresh = EnvNAVXYTHETALATCfg.obsthresh;

	//TODO - return motion primitive vector
}


bool EnvironmentNAVXYTHETALAT::PoseContToDisc(double px, double py, double pth,
					 int &ix, int &iy, int &ith) const
{
  ix = CONTXY2DISC(px, EnvNAVXYTHETALATCfg.cellsize_m);
  iy = CONTXY2DISC(py, EnvNAVXYTHETALATCfg.cellsize_m);
  ith = ContTheta2Disc(pth, NAVXYTHETALAT_THETADIRS); // ContTheta2Disc() normalizes the angle
  return (pth >= -2*PI_CONST) && (pth <= 2*PI_CONST)
    && (ix >= 0) && (ix < EnvNAVXYTHETALATCfg.EnvWidth_c)
    && (iy >= 0) && (iy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}


bool EnvironmentNAVXYTHETALAT::PoseDiscToCont(int ix, int iy, int ith,
					 double &px, double &py, double &pth) const
{
  px = DISCXY2CONT(ix, EnvNAVXYTHETALATCfg.cellsize_m);
  py = DISCXY2CONT(iy, EnvNAVXYTHETALATCfg.cellsize_m);
  pth = normalizeAngle(DiscTheta2Cont(ith, NAVXYTHETALAT_THETADIRS));
  return (ith >= 0) && (ith < NAVXYTHETALAT_THETADIRS)
    && (ix >= 0) && (ix < EnvNAVXYTHETALATCfg.EnvWidth_c)
    && (iy >= 0) && (iy < EnvNAVXYTHETALATCfg.EnvHeight_c);
}

unsigned char EnvironmentNAVXYTHETALAT::GetMapCost(int x, int y)
{
	return EnvNAVXYTHETALATCfg.Grid2D[x][y];
}

//------------------------------------------------------------------------------
