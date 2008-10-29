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
unsigned int EnvironmentNAV3DKIN::GETHASHBIN(unsigned int X1, unsigned int X2, unsigned int Theta)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)+(inthash(Theta)<<2)) & (EnvNAV3DKIN.HashTableSize-1);
}



void EnvironmentNAV3DKIN::PrintHashTableHist()
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < EnvNAV3DKIN.HashTableSize; j++)
	{
		if((int)EnvNAV3DKIN.Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)EnvNAV3DKIN.Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)EnvNAV3DKIN.Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)EnvNAV3DKIN.Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)EnvNAV3DKIN.Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)EnvNAV3DKIN.Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}

void EnvironmentNAV3DKIN::SetConfiguration(int width, int height,
					const unsigned char* mapdata,
					int startx, int starty, int starttheta,
					int goalx, int goaly, int goaltheta,
					double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV) {
  EnvNAV3DKINCfg.EnvWidth_c = width;
  EnvNAV3DKINCfg.EnvHeight_c = height;
  EnvNAV3DKINCfg.StartX_c = startx;
  EnvNAV3DKINCfg.StartY_c = starty;
  EnvNAV3DKINCfg.StartTheta = starttheta;
 
  if(EnvNAV3DKINCfg.StartX_c < 0 || EnvNAV3DKINCfg.StartX_c >= EnvNAV3DKINCfg.EnvWidth_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV3DKINCfg.StartY_c < 0 || EnvNAV3DKINCfg.StartY_c >= EnvNAV3DKINCfg.EnvHeight_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV3DKINCfg.StartTheta < 0 || EnvNAV3DKINCfg.StartTheta >= NAV3DKIN_THETADIRS) {
    printf("ERROR: illegal start coordinates for theta\n");
    exit(1);
  }
  
  EnvNAV3DKINCfg.EndX_c = goalx;
  EnvNAV3DKINCfg.EndY_c = goaly;
  EnvNAV3DKINCfg.EndTheta = goaltheta;

  if(EnvNAV3DKINCfg.EndX_c < 0 || EnvNAV3DKINCfg.EndX_c >= EnvNAV3DKINCfg.EnvWidth_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV3DKINCfg.EndY_c < 0 || EnvNAV3DKINCfg.EndY_c >= EnvNAV3DKINCfg.EnvHeight_c) {
    printf("ERROR: illegal goal coordinates\n");
    exit(1);
  }
  if(EnvNAV3DKINCfg.EndTheta < 0 || EnvNAV3DKINCfg.EndTheta >= NAV3DKIN_THETADIRS) {
    printf("ERROR: illegal goal coordinates for theta\n");
    exit(1);
  }

  EnvNAV3DKINCfg.FootprintPolygon = robot_perimeterV;

  EnvNAV3DKINCfg.nominalvel_mpersecs = nominalvel_mpersecs;
  EnvNAV3DKINCfg.cellsize_m = cellsize_m;
  EnvNAV3DKINCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;


  //allocate the 2D environment
  EnvNAV3DKINCfg.Grid2D = new char* [EnvNAV3DKINCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAV3DKINCfg.EnvWidth_c; x++) {
    EnvNAV3DKINCfg.Grid2D[x] = new char [EnvNAV3DKINCfg.EnvHeight_c];
  }
  
  //environment:
  for (int y = 0; y < EnvNAV3DKINCfg.EnvHeight_c; y++) {
    for (int x = 0; x < EnvNAV3DKINCfg.EnvWidth_c; x++) {
      char cval = mapdata[x+y*width];
      if(cval == 1) {
	EnvNAV3DKINCfg.Grid2D[x][y] = 1;
      } else {
	EnvNAV3DKINCfg.Grid2D[x][y] = 0;
      }
    }
  }
}

void EnvironmentNAV3DKIN::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAV3DKINCfg structure
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
	EnvNAV3DKINCfg.EnvWidth_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.EnvHeight_c = atoi(sTemp);

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
	EnvNAV3DKINCfg.cellsize_m = atof(sTemp);
	
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
	EnvNAV3DKINCfg.nominalvel_mpersecs = atof(sTemp);
	fscanf(fCfg, "%s", sTemp);
	strcpy(sTemp1, "timetoturn45degsinplace(secs):");
	if(strcmp(sTemp1, sTemp) != 0)
	{
		printf("ERROR: configuration file has incorrect format\n");
		printf("Expected %s got %s\n", sTemp1, sTemp);
		exit(1);
	}
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.timetoturn45degsinplace_secs = atof(sTemp);


	//start(meters,rads): 
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.StartX_c = CONTXY2DISC(atof(sTemp),EnvNAV3DKINCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.StartY_c = CONTXY2DISC(atof(sTemp),EnvNAV3DKINCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.StartTheta = ContTheta2Disc(atof(sTemp), NAV3DKIN_THETADIRS);


	if(EnvNAV3DKINCfg.StartX_c < 0 || EnvNAV3DKINCfg.StartX_c >= EnvNAV3DKINCfg.EnvWidth_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV3DKINCfg.StartY_c < 0 || EnvNAV3DKINCfg.StartY_c >= EnvNAV3DKINCfg.EnvHeight_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV3DKINCfg.StartTheta < 0 || EnvNAV3DKINCfg.StartTheta >= NAV3DKIN_THETADIRS) {
		printf("ERROR: illegal start coordinates for theta\n");
		exit(1);
	}

	//end(meters,rads): 
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.EndX_c = CONTXY2DISC(atof(sTemp),EnvNAV3DKINCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.EndY_c = CONTXY2DISC(atof(sTemp),EnvNAV3DKINCfg.cellsize_m);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV3DKINCfg.EndTheta = ContTheta2Disc(atof(sTemp), NAV3DKIN_THETADIRS);;

	if(EnvNAV3DKINCfg.EndX_c < 0 || EnvNAV3DKINCfg.EndX_c >= EnvNAV3DKINCfg.EnvWidth_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV3DKINCfg.EndY_c < 0 || EnvNAV3DKINCfg.EndY_c >= EnvNAV3DKINCfg.EnvHeight_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV3DKINCfg.EndTheta < 0 || EnvNAV3DKINCfg.EndTheta >= NAV3DKIN_THETADIRS) {
		printf("ERROR: illegal goal coordinates for theta\n");
		exit(1);
	}


	//allocate the 2D environment
	EnvNAV3DKINCfg.Grid2D = new char* [EnvNAV3DKINCfg.EnvWidth_c];
	for (x = 0; x < EnvNAV3DKINCfg.EnvWidth_c; x++)
	{
		EnvNAV3DKINCfg.Grid2D[x] = new char [EnvNAV3DKINCfg.EnvHeight_c];
	}

	//environment:
	fscanf(fCfg, "%s", sTemp);	
	for (y = 0; y < EnvNAV3DKINCfg.EnvHeight_c; y++)
		for (x = 0; x < EnvNAV3DKINCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				printf("ERROR: incorrect format of config file\n");
				exit(1);
			}
			EnvNAV3DKINCfg.Grid2D[x][y] = dTemp;
		}

}


void EnvironmentNAV3DKIN::InitializeEnvConfig()
{
	//aditional to configuration file initialization of EnvNAV3DKINCfg if necessary

	//dXY dirs
	EnvNAV3DKINCfg.dXY[0][0] = -1;
	EnvNAV3DKINCfg.dXY[0][1] = -1;
	EnvNAV3DKINCfg.dXY[1][0] = -1;
	EnvNAV3DKINCfg.dXY[1][1] = 0;
	EnvNAV3DKINCfg.dXY[2][0] = -1;
	EnvNAV3DKINCfg.dXY[2][1] = 1;
	EnvNAV3DKINCfg.dXY[3][0] = 0;
	EnvNAV3DKINCfg.dXY[3][1] = -1;
	EnvNAV3DKINCfg.dXY[4][0] = 0;
	EnvNAV3DKINCfg.dXY[4][1] = 1;
	EnvNAV3DKINCfg.dXY[5][0] = 1;
	EnvNAV3DKINCfg.dXY[5][1] = -1;
	EnvNAV3DKINCfg.dXY[6][0] = 1;
	EnvNAV3DKINCfg.dXY[6][1] = 0;
	EnvNAV3DKINCfg.dXY[7][0] = 1;
	EnvNAV3DKINCfg.dXY[7][1] = 1;



	//construct list of actions
	EnvNAV3DKINCfg.ActionsV = new EnvNAV3DKINAction_t* [NAV3DKIN_THETADIRS];
	vector<sbpl_2Dcell_t> footprint;
	//iterate over source angles
	for(int tind = 0; tind < NAV3DKIN_THETADIRS; tind++)
	{
		EnvNAV3DKINCfg.ActionsV[tind] = new EnvNAV3DKINAction_t[NAV3DKIN_ACTIONWIDTH];
		//the construction assumes that the robot first turns and then goes along this new theta
		int aind = 0;
		for(; aind < 3; aind++)
		{
			EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta = aind-1; //-1,0,1
			double angle = DiscTheta2Cont(tind + EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, NAV3DKIN_THETADIRS);
			EnvNAV3DKINCfg.ActionsV[tind][aind].dX = (int)(cos(angle) + 0.5*(cos(angle)>0?1:-1));
			EnvNAV3DKINCfg.ActionsV[tind][aind].dY = (int)(sin(angle) + 0.5*(sin(angle)>0?1:-1));
			EnvNAV3DKINCfg.ActionsV[tind][aind].cost = (int)(NAV3DKIN_COSTMULT*EnvNAV3DKINCfg.cellsize_m/EnvNAV3DKINCfg.nominalvel_mpersecs*sqrt(EnvNAV3DKINCfg.ActionsV[tind][aind].dX*EnvNAV3DKINCfg.ActionsV[tind][aind].dX + 
					EnvNAV3DKINCfg.ActionsV[tind][aind].dY*EnvNAV3DKINCfg.ActionsV[tind][aind].dY));

			//compute intersecting cells
			EnvNAV3DKIN3Dpt_t pose;
			pose.x = DISCXY2CONT(EnvNAV3DKINCfg.ActionsV[tind][aind].dX, EnvNAV3DKINCfg.cellsize_m);
			pose.y = DISCXY2CONT(EnvNAV3DKINCfg.ActionsV[tind][aind].dY, EnvNAV3DKINCfg.cellsize_m);
			pose.theta = angle;
			EnvNAV3DKINCfg.ActionsV[tind][aind].intersectingcellsV.clear();
			CalculateFootprintForPose(pose, &EnvNAV3DKINCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
			printf("action tind=%d aind=%d: dTheta=%d (%f) dX=%d dY=%d cost=%d\n",
				tind, aind, EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, angle, EnvNAV3DKINCfg.ActionsV[tind][aind].dX, EnvNAV3DKINCfg.ActionsV[tind][aind].dY,
				EnvNAV3DKINCfg.ActionsV[tind][aind].cost);
#endif
		}
		//decrease and increase angle without movement
		aind = 3;
		EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta = -1; 
		EnvNAV3DKINCfg.ActionsV[tind][aind].dX = 0;
		EnvNAV3DKINCfg.ActionsV[tind][aind].dY = 0;
		EnvNAV3DKINCfg.ActionsV[tind][aind].cost = (int)(NAV3DKIN_COSTMULT*EnvNAV3DKINCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		EnvNAV3DKIN3Dpt_t pose;
		pose.x = DISCXY2CONT(EnvNAV3DKINCfg.ActionsV[tind][aind].dX, EnvNAV3DKINCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAV3DKINCfg.ActionsV[tind][aind].dY, EnvNAV3DKINCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(tind + EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, NAV3DKIN_THETADIRS);
		EnvNAV3DKINCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAV3DKINCfg.ActionsV[tind][aind].intersectingcellsV);

#if DEBUG
		printf("action tind=%d aind=%d: dTheta=%d (%f) dX=%d dY=%d cost=%d\n",
			tind, aind, EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, DiscTheta2Cont(tind + EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, NAV3DKIN_THETADIRS),
			EnvNAV3DKINCfg.ActionsV[tind][aind].dX, EnvNAV3DKINCfg.ActionsV[tind][aind].dY,
			EnvNAV3DKINCfg.ActionsV[tind][aind].cost);
#endif

		aind = 4;
		EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta = 1; 
		EnvNAV3DKINCfg.ActionsV[tind][aind].dX = 0;
		EnvNAV3DKINCfg.ActionsV[tind][aind].dY = 0;
		EnvNAV3DKINCfg.ActionsV[tind][aind].cost = (int)(NAV3DKIN_COSTMULT*EnvNAV3DKINCfg.timetoturn45degsinplace_secs);

		//compute intersecting cells
		pose.x = DISCXY2CONT(EnvNAV3DKINCfg.ActionsV[tind][aind].dX, EnvNAV3DKINCfg.cellsize_m);
		pose.y = DISCXY2CONT(EnvNAV3DKINCfg.ActionsV[tind][aind].dY, EnvNAV3DKINCfg.cellsize_m);
		pose.theta = DiscTheta2Cont(tind + EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, NAV3DKIN_THETADIRS);
		EnvNAV3DKINCfg.ActionsV[tind][aind].intersectingcellsV.clear();
		CalculateFootprintForPose(pose, &EnvNAV3DKINCfg.ActionsV[tind][aind].intersectingcellsV);


#if DEBUG
		printf("action tind=%d aind=%d: dTheta=%d (%f) dX=%d dY=%d cost=%d\n",
			tind, aind, EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, DiscTheta2Cont(tind + EnvNAV3DKINCfg.ActionsV[tind][aind].dTheta, NAV3DKIN_THETADIRS),
			EnvNAV3DKINCfg.ActionsV[tind][aind].dX, EnvNAV3DKINCfg.ActionsV[tind][aind].dY,
			EnvNAV3DKINCfg.ActionsV[tind][aind].cost);
#endif

	}

}



EnvNAV3DKINHashEntry_t* EnvironmentNAV3DKIN::GetHashEntry(int X, int Y, int Theta)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y, Theta);
	
#if DEBUG
	if ((int)EnvNAV3DKIN.Coord2StateIDHashTable[binid].size() > 500)
	{
		printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, EnvNAV3DKIN.Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)EnvNAV3DKIN.Coord2StateIDHashTable[binid].size(); ind++)
	{
		if( EnvNAV3DKIN.Coord2StateIDHashTable[binid][ind]->X == X 
			&& EnvNAV3DKIN.Coord2StateIDHashTable[binid][ind]->Y == Y
			&& EnvNAV3DKIN.Coord2StateIDHashTable[binid][ind]->Theta == Theta)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return EnvNAV3DKIN.Coord2StateIDHashTable[binid][ind];
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


EnvNAV3DKINHashEntry_t* EnvironmentNAV3DKIN::CreateNewHashEntry(int X, int Y, int Theta) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAV3DKINHashEntry_t* HashEntry = new EnvNAV3DKINHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;
	HashEntry->Theta = Theta;

	HashEntry->stateID = EnvNAV3DKIN.StateID2CoordTable.size();

	//insert into the tables
	EnvNAV3DKIN.StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y, HashEntry->Theta); 

	//insert the entry into the bin
	EnvNAV3DKIN.Coord2StateIDHashTable[i].push_back(HashEntry);

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

bool EnvironmentNAV3DKIN::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAV3DKINCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV3DKINCfg.EnvHeight_c && 
		EnvNAV3DKINCfg.Grid2D[X][Y] == 0);
}

bool EnvironmentNAV3DKIN::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAV3DKINCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV3DKINCfg.EnvHeight_c);
}


int EnvironmentNAV3DKIN::GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAV3DKINAction_t* action)
{
	sbpl_2Dcell_t cell;

	for(int i = 0; i < action->intersectingcellsV.size(); i++)
	{
		//get the cell in the map
		cell = action->intersectingcellsV.at(i);
		cell.x = cell.x + SourceX;
		cell.y = cell.y + SourceY;
		
		//check validity
		if(!IsValidCell(cell.x, cell.y))
			return INFINITECOST;
	}

	return action->cost;
}



void EnvironmentNAV3DKIN::InitializeEnvironment()
{
	EnvNAV3DKINHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	EnvNAV3DKIN.HashTableSize = 64*1024; //should be power of two
	EnvNAV3DKIN.Coord2StateIDHashTable = new vector<EnvNAV3DKINHashEntry_t*>[EnvNAV3DKIN.HashTableSize];
	
	//initialize the map from StateID to Coord
	EnvNAV3DKIN.StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(EnvNAV3DKINCfg.StartX_c, EnvNAV3DKINCfg.StartY_c, EnvNAV3DKINCfg.StartTheta);
	EnvNAV3DKIN.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(EnvNAV3DKINCfg.EndX_c, EnvNAV3DKINCfg.EndY_c, EnvNAV3DKINCfg.EndTheta);
	EnvNAV3DKIN.goalstateid = HashEntry->stateID;
}

static int EuclideanDistance(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    double dist = sqrt((double)sqdist);
    return (int)(NAV3DKIN_COSTMULT*dist);

}


void EnvironmentNAV3DKIN::CalculateFootprintForPose(EnvNAV3DKIN3Dpt_t pose, vector<sbpl_2Dcell_t>* footprint)
{  

#if DEBUG
  printf("---Calculating Footprint for Pose: %f %f %f---\n",
	 pose.x, pose.y, pose.theta);
#endif

  //handle special case where footprint is just a point
  if(EnvNAV3DKINCfg.FootprintPolygon.size() <= 1){
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, EnvNAV3DKINCfg.cellsize_m);
    cell.y = CONTXY2DISC(pose.y, EnvNAV3DKINCfg.cellsize_m);
    footprint->push_back(cell);
    return;
  }

  vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x, min_x, max_y, min_y;
  sbpl_2Dpt_t pt;
  for(find = 0; find < EnvNAV3DKINCfg.FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = EnvNAV3DKINCfg.FootprintPolygon[find];
    
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
  int prev_discrete_x = CONTXY2DISC(pt.x, EnvNAV3DKINCfg.cellsize_m) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, EnvNAV3DKINCfg.cellsize_m) + 1;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=EnvNAV3DKINCfg.cellsize_m/3){
    for(double y=min_y; y<=max_y; y+=EnvNAV3DKINCfg.cellsize_m/3){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, EnvNAV3DKINCfg.cellsize_m);
      discrete_y = CONTXY2DISC(pt.y, EnvNAV3DKINCfg.cellsize_m);
      
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


//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAV3DKIN::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	printf("Precomputing heuristics...\n");
	


	printf("done\n");

}

//------------debugging functions---------------------------------------------
bool EnvironmentNAV3DKIN::CheckQuant(FILE* fOut) 
{

  for(double theta  = -10; theta < 10; theta += 2.0*PI_CONST/NAV3DKIN_THETADIRS*0.01)
    {
		int nTheta = ContTheta2Disc(theta, NAV3DKIN_THETADIRS);
		double newTheta = DiscTheta2Cont(nTheta, NAV3DKIN_THETADIRS);
		int nnewTheta = ContTheta2Disc(newTheta, NAV3DKIN_THETADIRS);

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
bool EnvironmentNAV3DKIN::InitializeEnv(const char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	ReadConfiguration(fCfg);

	InitGeneral();


	return true;
}



bool EnvironmentNAV3DKIN::InitializeEnv(int width, int height,
					const unsigned char* mapdata,
					double startx, double starty, double starttheta,
					double goalx, double goaly, double goaltheta,
				    double goaltol_x, double goaltol_y, double goaltol_theta,
					const vector<sbpl_2Dpt_t> & perimeterptsV,
					double cellsize_m, double nominalvel_mpersecs, double timetoturn45degsinplace_secs)
{
	//TODO - need to set the tolerance as well

	SetConfiguration(width, height,
					mapdata,
					CONTXY2DISC(startx, cellsize_m), CONTXY2DISC(starty, cellsize_m), ContTheta2Disc(starttheta, NAV3DKIN_THETADIRS),
					CONTXY2DISC(goalx, cellsize_m), CONTXY2DISC(goaly, cellsize_m), ContTheta2Disc(goaltheta, NAV3DKIN_THETADIRS),
					cellsize_m, nominalvel_mpersecs, timetoturn45degsinplace_secs, perimeterptsV);

	InitGeneral();

	return true;
}


bool EnvironmentNAV3DKIN::InitGeneral() {
  //Initialize other parameters of the environment
  InitializeEnvConfig();
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAV3DKIN::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAV3DKIN.goalstateid;
	MDPCfg->startstateid = EnvNAV3DKIN.startstateid;

	return true;
}



int EnvironmentNAV3DKIN::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)EnvNAV3DKIN.StateID2CoordTable.size() 
		|| ToStateID >= (int)EnvNAV3DKIN.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DKIN... function: stateID illegal\n");
		exit(1);
	}
#endif

	//get X, Y for the state
	EnvNAV3DKINHashEntry_t* FromHashEntry = EnvNAV3DKIN.StateID2CoordTable[FromStateID];
	EnvNAV3DKINHashEntry_t* ToHashEntry = EnvNAV3DKIN.StateID2CoordTable[ToStateID];
	

	return EuclideanDistance(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);	

}


int EnvironmentNAV3DKIN::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvNAV3DKIN.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DKIN... function: stateID illegal\n");
		exit(1);
	}
#endif


	//define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvNAV3DKIN.goalstateid);

}


int EnvironmentNAV3DKIN::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)EnvNAV3DKIN.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DKIN... function: stateID illegal\n");
		exit(1);
	}
#endif

    


	//define this function if it used in the planner (heuristic backward search would use it)
    return GetFromToHeuristic(EnvNAV3DKIN.startstateid, stateID);


}



void EnvironmentNAV3DKIN::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)EnvNAV3DKIN.StateID2CoordTable.size())
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
	if(state->StateID == EnvNAV3DKIN.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV3DKINHashEntry_t* HashEntry = EnvNAV3DKIN.StateID2CoordTable[state->StateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV3DKINCfg.EnvWidth_c-1 || //TODO - modify based on robot's size
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV3DKINCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < NAV3DKIN_ACTIONWIDTH; aind++)
	{
		EnvNAV3DKINAction_t* nav3daction = &EnvNAV3DKINCfg.ActionsV[HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAV3DKIN_THETADIRS);	

        //skip the invalid cells
        if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY))
                continue;
        }

       if(GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction) >= INFINITECOST)
            continue;

		//skip invalid diagonal move
        if(newX != HashEntry->X && newY != HashEntry->Y) //TODO - need to modify to take robot perimeter into account
		{
			if(EnvNAV3DKINCfg.Grid2D[HashEntry->X][newY] != 0 || EnvNAV3DKINCfg.Grid2D[newX][HashEntry->Y] != 0)
				continue;
		}


		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

        //compute cost
		cost = nav3daction->cost;

    	EnvNAV3DKINHashEntry_t* OutHashEntry;
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



void EnvironmentNAV3DKIN::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	printf("ERROR in EnvNAV3DKIN... function: SetAllPreds is undefined\n");
	exit(1);
}


void EnvironmentNAV3DKIN::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int aind;

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(NAV3DKIN_ACTIONWIDTH); 
    CostV->reserve(NAV3DKIN_ACTIONWIDTH);

	//goal state should be absorbing
	if(SourceStateID == EnvNAV3DKIN.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV3DKINHashEntry_t* HashEntry = EnvNAV3DKIN.StateID2CoordTable[SourceStateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV3DKINCfg.EnvWidth_c-1 ||  //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV3DKINCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < NAV3DKIN_ACTIONWIDTH; aind++)
	{
		EnvNAV3DKINAction_t* nav3daction = &EnvNAV3DKINCfg.ActionsV[HashEntry->Theta][aind];
        int newX = HashEntry->X + nav3daction->dX;
		int newY = HashEntry->Y + nav3daction->dY;
		int newTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAV3DKIN_THETADIRS);	

        //skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(newX, newY)) 
                continue;
        }

        if(GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction) >= INFINITECOST)
            continue;

		//skip invalid diagonal move
        if(newX != HashEntry->X && newY != HashEntry->Y) //TODO - need to modify to take robot perimeter into account
		{
			if(EnvNAV3DKINCfg.Grid2D[HashEntry->X][newY] != 0 || EnvNAV3DKINCfg.Grid2D[newX][HashEntry->Y] != 0)
				continue;
        }


    	EnvNAV3DKINHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY, newTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY, newTheta);
		}

        //compute clow 
        int cost = nav3daction->cost;

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNAV3DKIN::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(NAV3DKIN_ACTIONWIDTH); 
    CostV->reserve(NAV3DKIN_ACTIONWIDTH);

	//get X, Y for the state
	EnvNAV3DKINHashEntry_t* HashEntry = EnvNAV3DKIN.StateID2CoordTable[TargetStateID];
	
	//no predecessors if obstacle
	if(EnvNAV3DKINCfg.Grid2D[HashEntry->X][HashEntry->Y] != 0)
		return;

	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV3DKINCfg.EnvWidth_c-1 || //TODO - need to modify to take robot perimeter into account
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV3DKINCfg.EnvHeight_c-1)
        bTestBounds = true;

	for (aind = 0; aind < NAV3DKIN_ACTIONWIDTH; aind++)
	{
		EnvNAV3DKINAction_t* nav3daction = &EnvNAV3DKINCfg.ActionsV[HashEntry->Theta][aind];
        int predX = HashEntry->X + nav3daction->dX;
		int predY = HashEntry->Y + nav3daction->dY;
		int predTheta = NORMALIZEDISCTHETA(HashEntry->Theta + nav3daction->dTheta, NAV3DKIN_THETADIRS);	
	

		//TODO - incorrect - have to compute preds array
		
		//skip the invalid cells
		if(bTestBounds){ //TODO - need to modify to take robot perimeter into account
            if(!IsValidCell(predX, predY))
                continue;
        }

		//skip invalid diagonal move
	    if(GetActionCost(HashEntry->X, HashEntry->Y, HashEntry->Theta, nav3daction) >= INFINITECOST) //TODO -change after I have explicit backward actions
			continue;
        


    	EnvNAV3DKINHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(predX, predY, predTheta)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(predX, predY, predTheta);
		}

        //compute clow 
        int cost = nav3daction->cost;

        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}



int EnvironmentNAV3DKIN::SizeofCreatedEnv()
{
	return (int)EnvNAV3DKIN.StateID2CoordTable.size();
	
}

void EnvironmentNAV3DKIN::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)EnvNAV3DKIN.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV3DKIN... function: stateID illegal (2)\n");
		exit(1);
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAV3DKINHashEntry_t* HashEntry = EnvNAV3DKIN.StateID2CoordTable[stateID];

	if(stateID == EnvNAV3DKIN.goalstateid && bVerbose)
	{
		fprintf(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	fprintf(fOut, "X=%d Y=%d Theta=%d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);
    else
    	fprintf(fOut, "%d %d %d\n", HashEntry->X, HashEntry->Y, HashEntry->Theta);

}

void EnvironmentNAV3DKIN::GetCoordFromState(int stateID, int& x, int& y, int& theta) const {
  EnvNAV3DKINHashEntry_t* HashEntry = EnvNAV3DKIN.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
  theta = HashEntry->Theta;
}

int EnvironmentNAV3DKIN::GetStateFromCoord(int x, int y, int theta) {

   EnvNAV3DKINHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta);
    }
    return OutHashEntry->stateID;
}

const EnvNAV3DKINConfig_t* EnvironmentNAV3DKIN::GetEnvNavConfig() {
  return &EnvNAV3DKINCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV3DKIN::SetGoal(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, EnvNAV3DKINCfg.cellsize_m);
	int y = CONTXY2DISC(x_m, EnvNAV3DKINCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAV3DKIN_THETADIRS);

    if(!IsWithinMapCell(x,y))
        return -1;

    EnvNAV3DKINHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta);
    }
    EnvNAV3DKIN.goalstateid = OutHashEntry->stateID;

    return EnvNAV3DKIN.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAV3DKIN::SetStart(double x_m, double y_m, double theta_rad){

	int x = CONTXY2DISC(x_m, EnvNAV3DKINCfg.cellsize_m);
	int y = CONTXY2DISC(x_m, EnvNAV3DKINCfg.cellsize_m);
	int theta = ContTheta2Disc(theta_rad, NAV3DKIN_THETADIRS);

    if(!IsWithinMapCell(x,y))
        return -1;

    EnvNAV3DKINHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y, theta)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y, theta);
    }
    EnvNAV3DKIN.startstateid = OutHashEntry->stateID;

    return EnvNAV3DKIN.startstateid;    

}

bool EnvironmentNAV3DKIN::UpdateCost(int x, int y, int new_status)
{

    EnvNAV3DKINCfg.Grid2D[x][y] = new_status;

    return true;
}


void EnvironmentNAV3DKIN::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAV3DKIN. configuration
	
	printf("ERROR in EnvNAV3DKIN... function: PrintEnv_Config is undefined\n");
	exit(1);

}

void EnvironmentNAV3DKIN::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    fprintf(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}


void EnvironmentNAV3DKIN::GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;

	for(int i = 0; i < (int)changedcellsV->size(); i++)
	{
		cell = changedcellsV->at(i);
		for(int tind = 0; tind < NAV3DKIN_THETADIRS; tind++)
			preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x,cell.y,tind));
		for(int j = 0; j < 8; j++){
			int affx = cell.x + EnvNAV3DKINCfg.dXY[j][0];
			int affy = cell.y + EnvNAV3DKINCfg.dXY[j][1];
			if(affx < 0 || affx >= EnvNAV3DKINCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV3DKINCfg.EnvHeight_c)
				continue;
			for(int tind = 0; tind < NAV3DKIN_THETADIRS; tind++)
				preds_of_changededgesIDV->push_back(GetStateFromCoord(affx,affy,tind));
		}
	}
}


bool EnvironmentNAV3DKIN::IsObstacle(int x, int y)
{

	return (EnvNAV3DKINCfg.Grid2D[x][y] != 0);

}

void EnvironmentNAV3DKIN::GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double*starttheta, double* goalx, double* goaly, double* goaltheta,
									  	double* cellsize_m, double* nominalvel_mpersecs, double* timetoturn45degsinplace_secs)
{
	*size_x = EnvNAV3DKINCfg.EnvWidth_c;
	*size_y = EnvNAV3DKINCfg.EnvHeight_c;

	*startx = DISCXY2CONT(EnvNAV3DKINCfg.StartX_c, EnvNAV3DKINCfg.cellsize_m);
	*starty = DISCXY2CONT(EnvNAV3DKINCfg.StartY_c, EnvNAV3DKINCfg.cellsize_m);
	*starttheta = DiscTheta2Cont(EnvNAV3DKINCfg.StartTheta, NAV3DKIN_THETADIRS);
	*goalx = DISCXY2CONT(EnvNAV3DKINCfg.EndX_c, EnvNAV3DKINCfg.cellsize_m);
	*goaly = DISCXY2CONT(EnvNAV3DKINCfg.EndY_c, EnvNAV3DKINCfg.cellsize_m);
	*goaltheta = DiscTheta2Cont(EnvNAV3DKINCfg.EndTheta, NAV3DKIN_THETADIRS);;

	*cellsize_m = EnvNAV3DKINCfg.cellsize_m;
	*nominalvel_mpersecs = EnvNAV3DKINCfg.nominalvel_mpersecs;
	*timetoturn45degsinplace_secs = EnvNAV3DKINCfg.timetoturn45degsinplace_secs;
}



//------------------------------------------------------------------------------
