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


//-------------------constructor--------------------------------------------
EnvironmentNAV2D::EnvironmentNAV2D()
{
	EnvNAV2DCfg.obsthresh = ENVNAV2D_DEFAULTOBSTHRESH;
	
};



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
unsigned int EnvironmentNAV2D::GETHASHBIN(unsigned int X1, unsigned int X2)
{

	return inthash(inthash(X1)+(inthash(X2)<<1)) & (EnvNAV2D.HashTableSize-1);
}



void EnvironmentNAV2D::PrintHashTableHist()
{
	int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

	for(int  j = 0; j < EnvNAV2D.HashTableSize; j++)
	{
		if((int)EnvNAV2D.Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if((int)EnvNAV2D.Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
}

void EnvironmentNAV2D::SetConfiguration(int width, int height,
					unsigned char* mapdata,
					int startx, int starty,
					int goalx, int goaly) {
  EnvNAV2DCfg.EnvWidth_c = width;
  EnvNAV2DCfg.EnvHeight_c = height;
  EnvNAV2DCfg.StartX_c = startx;
  EnvNAV2DCfg.StartY_c = starty;
 
  if(EnvNAV2DCfg.StartX_c < 0 || EnvNAV2DCfg.StartX_c >= EnvNAV2DCfg.EnvWidth_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  if(EnvNAV2DCfg.StartY_c < 0 || EnvNAV2DCfg.StartY_c >= EnvNAV2DCfg.EnvHeight_c) {
    printf("ERROR: illegal start coordinates\n");
    exit(1);
  }
  
  EnvNAV2DCfg.EndX_c = goalx;
  EnvNAV2DCfg.EndY_c = goaly;

  //allocate the 2D environment
  EnvNAV2DCfg.Grid2D = new unsigned char* [EnvNAV2DCfg.EnvWidth_c];
  for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
    EnvNAV2DCfg.Grid2D[x] = new unsigned char [EnvNAV2DCfg.EnvHeight_c];
  }
  
  //environment:
  for (int y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++) {
    for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++) {
      unsigned char cval = mapdata[x+y*width];
	   EnvNAV2DCfg.Grid2D[x][y] = cval;
    }
  }
}

void EnvironmentNAV2D::ReadConfiguration(FILE* fCfg)
{
	//read in the configuration of environment and initialize  EnvNAV2DCfg structure
	char sTemp[1024];
	int dTemp;
	int x, y;

	//discretization(cells)
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV2DCfg.EnvWidth_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV2DCfg.EnvHeight_c = atoi(sTemp);
	
	//start(cells): 
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV2DCfg.StartX_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV2DCfg.StartY_c = atoi(sTemp);

	if(EnvNAV2DCfg.StartX_c < 0 || EnvNAV2DCfg.StartX_c >= EnvNAV2DCfg.EnvWidth_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}
	if(EnvNAV2DCfg.StartY_c < 0 || EnvNAV2DCfg.StartY_c >= EnvNAV2DCfg.EnvHeight_c)
	{
		printf("ERROR: illegal start coordinates\n");
		exit(1);
	}

	//end(cells): 
	fscanf(fCfg, "%s", sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV2DCfg.EndX_c = atoi(sTemp);
	fscanf(fCfg, "%s", sTemp);
	EnvNAV2DCfg.EndY_c = atoi(sTemp);

	if(EnvNAV2DCfg.EndX_c < 0 || EnvNAV2DCfg.EndX_c >= EnvNAV2DCfg.EnvWidth_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}
	if(EnvNAV2DCfg.EndY_c < 0 || EnvNAV2DCfg.EndY_c >= EnvNAV2DCfg.EnvHeight_c)
	{
		printf("ERROR: illegal end coordinates\n");
		exit(1);
	}


	//allocate the 2D environment
	EnvNAV2DCfg.Grid2D = new unsigned char* [EnvNAV2DCfg.EnvWidth_c];
	for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++)
	{
		EnvNAV2DCfg.Grid2D[x] = new unsigned char [EnvNAV2DCfg.EnvHeight_c];
	}

	//environment:
	fscanf(fCfg, "%s", sTemp);	
	for (y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++)
		for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++)
		{
			if(fscanf(fCfg, "%d", &dTemp) != 1)
			{
				printf("ERROR: incorrect format of config file\n");
				exit(1);
			}
			EnvNAV2DCfg.Grid2D[x][y] = dTemp;
		}

}


void EnvironmentNAV2D::InitializeEnvConfig()
{
	//aditional to configuration file initialization of EnvNAV2DCfg if necessary

	//actions
	EnvNAV2DCfg.dXY[0][0] = -1;
	EnvNAV2DCfg.dXY[0][1] = -1;
	EnvNAV2DCfg.dXY[1][0] = -1;
	EnvNAV2DCfg.dXY[1][1] = 0;
	EnvNAV2DCfg.dXY[2][0] = -1;
	EnvNAV2DCfg.dXY[2][1] = 1;
	EnvNAV2DCfg.dXY[3][0] = 0;
	EnvNAV2DCfg.dXY[3][1] = -1;
	EnvNAV2DCfg.dXY[4][0] = 0;
	EnvNAV2DCfg.dXY[4][1] = 1;
	EnvNAV2DCfg.dXY[5][0] = 1;
	EnvNAV2DCfg.dXY[5][1] = -1;
	EnvNAV2DCfg.dXY[6][0] = 1;
	EnvNAV2DCfg.dXY[6][1] = 0;
	EnvNAV2DCfg.dXY[7][0] = 1;
	EnvNAV2DCfg.dXY[7][1] = 1;


}



EnvNAV2DHashEntry_t* EnvironmentNAV2D::GetHashEntry(int X, int Y)
{

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

	int binid = GETHASHBIN(X, Y);
	
#if DEBUG
	if ((int)EnvNAV2D.Coord2StateIDHashTable[binid].size() > 500)
	{
		printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, EnvNAV2D.Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < (int)EnvNAV2D.Coord2StateIDHashTable[binid].size(); ind++)
	{
		if( EnvNAV2D.Coord2StateIDHashTable[binid][ind]->X == X 
			&& EnvNAV2D.Coord2StateIDHashTable[binid][ind]->Y == Y)
		{
#if TIME_DEBUG
			time_gethash += clock()-currenttime;
#endif
			return EnvNAV2D.Coord2StateIDHashTable[binid][ind];
		}
	}

#if TIME_DEBUG	
	time_gethash += clock()-currenttime;
#endif

	return NULL;	  
}


EnvNAV2DHashEntry_t* EnvironmentNAV2D::CreateNewHashEntry(int X, int Y) 
{
	int i;

#if TIME_DEBUG	
	clock_t currenttime = clock();
#endif

	EnvNAV2DHashEntry_t* HashEntry = new EnvNAV2DHashEntry_t;

	HashEntry->X = X;
	HashEntry->Y = Y;

	HashEntry->stateID = EnvNAV2D.StateID2CoordTable.size();

	//insert into the tables
	EnvNAV2D.StateID2CoordTable.push_back(HashEntry);


	//get the hash table bin
	i = GETHASHBIN(HashEntry->X, HashEntry->Y); 

	//insert the entry into the bin
	EnvNAV2D.Coord2StateIDHashTable[i].push_back(HashEntry);

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

bool EnvironmentNAV2D::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAV2DCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV2DCfg.EnvHeight_c && 
		EnvNAV2DCfg.Grid2D[X][Y] < EnvNAV2DCfg.obsthresh);
}

bool EnvironmentNAV2D::IsWithinMapCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAV2DCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV2DCfg.EnvHeight_c);
}




void EnvironmentNAV2D::InitializeEnvironment()
{
	EnvNAV2DHashEntry_t* HashEntry;

	//initialize the map from Coord to StateID
	EnvNAV2D.HashTableSize = 64*1024; //should be power of two
	EnvNAV2D.Coord2StateIDHashTable = new vector<EnvNAV2DHashEntry_t*>[EnvNAV2D.HashTableSize];
	
	//initialize the map from StateID to Coord
	EnvNAV2D.StateID2CoordTable.clear();

	//create start state 
	HashEntry = CreateNewHashEntry(EnvNAV2DCfg.StartX_c, EnvNAV2DCfg.StartY_c);
	EnvNAV2D.startstateid = HashEntry->stateID;

	//create goal state 
	HashEntry = CreateNewHashEntry(EnvNAV2DCfg.EndX_c, EnvNAV2DCfg.EndY_c);
	EnvNAV2D.goalstateid = HashEntry->stateID;
}

static int EuclideanDistance(int X1, int Y1, int X2, int Y2)
{
    int sqdist = ((X1-X2)*(X1-X2)+(Y1-Y2)*(Y1-Y2));
    double dist = sqrt((double)sqdist);
    return (int)(ENVNAV2D_COSTMULT*dist);

}

//------------------------------------------------------------------------------

//------------------------------Heuristic computation--------------------------

void EnvironmentNAV2D::ComputeHeuristicValues()
{
	//whatever necessary pre-computation of heuristic values is done here 
	printf("Precomputing heuristics...\n");
	


	printf("done\n");

}

//-----------interface with outside functions-----------------------------------
bool EnvironmentNAV2D::InitializeEnv(const char* sEnvFile)
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



bool EnvironmentNAV2D::InitializeEnv(int width, int height,
					unsigned char* mapdata,
					int startx, int starty,
					int goalx, int goaly, unsigned char obsthresh)
{

	EnvNAV2DCfg.obsthresh = obsthresh;

	SetConfiguration(width, height,
					mapdata,
					startx, starty,
					goalx, goaly);

	InitGeneral();

	return true;
}


bool EnvironmentNAV2D::InitGeneral() {
  //Initialize other parameters of the environment
  InitializeEnvConfig();
  
  //initialize Environment
  InitializeEnvironment();
  
  //pre-compute heuristics
  ComputeHeuristicValues();

  return true;
}

bool EnvironmentNAV2D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvNAV2D.goalstateid;
	MDPCfg->startstateid = EnvNAV2D.startstateid;

	return true;
}



int EnvironmentNAV2D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(FromStateID >= (int)EnvNAV2D.StateID2CoordTable.size() 
		|| ToStateID >= (int)EnvNAV2D.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV2D... function: stateID illegal\n");
		exit(1);
	}
#endif

	//get X, Y for the state
	EnvNAV2DHashEntry_t* FromHashEntry = EnvNAV2D.StateID2CoordTable[FromStateID];
	EnvNAV2DHashEntry_t* ToHashEntry = EnvNAV2D.StateID2CoordTable[ToStateID];
	

	return EuclideanDistance(FromHashEntry->X, FromHashEntry->Y, ToHashEntry->X, ToHashEntry->Y);	

}


int EnvironmentNAV2D::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvNAV2D.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV2D... function: stateID illegal\n");
		exit(1);
	}
#endif


	//define this function if it used in the planner (heuristic forward search would use it)
    return GetFromToHeuristic(stateID, EnvNAV2D.goalstateid);

}


int EnvironmentNAV2D::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif


#if DEBUG
	if(stateID >= (int)EnvNAV2D.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV2D... function: stateID illegal\n");
		exit(1);
	}
#endif

    


	//define this function if it used in the planner (heuristic backward search would use it)
    return GetFromToHeuristic(EnvNAV2D.startstateid, stateID);


}



void EnvironmentNAV2D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

	int cost;

#if DEBUG
	if(state->StateID >= (int)EnvNAV2D.StateID2CoordTable.size())
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
	if(state->StateID == EnvNAV2D.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[state->StateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV2DCfg.EnvWidth_c-1 || 
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV2DCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (int aind = 0; aind < ENVNAV2D_ACTIONSWIDTH; aind++)
	{
        int newX = HashEntry->X + EnvNAV2DCfg.dXY[aind][0];
        int newY = HashEntry->Y + EnvNAV2DCfg.dXY[aind][1];

        //skip the invalid cells
        if(bTestBounds){
            if(!IsValidCell(newX, newY))
                continue;
        }


		int costmult = EnvNAV2DCfg.Grid2D[newX][newY];

		//for diagonal move, take max over adjacent cells
        if(newX != HashEntry->X && newY != HashEntry->Y)
		{
			costmult = __max(costmult, 	EnvNAV2DCfg.Grid2D[HashEntry->X][newY]);
			costmult = __max(costmult, EnvNAV2DCfg.Grid2D[newX][HashEntry->Y]);
		}
		//check that it is valid
		if(costmult >= EnvNAV2DCfg.obsthresh)
			continue;

		//otherwise compute the actual cost
		cost = (costmult+1)*ENVNAV2D_COSTMULT;
        //diagonal moves are costlier
        if(newX != HashEntry->X && newY != HashEntry->Y)
            cost = (int)(sqrt(2)*cost);

		//add the action
		CMDPACTION* action = state->AddAction(aind);

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif
        

    	EnvNAV2DHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

#if TIME_DEBUG
		time3_addallout += clock()-currenttime;
#endif

	}
}



void EnvironmentNAV2D::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
	printf("ERROR in EnvNAV2D... function: SetAllPreds is undefined\n");
	exit(1);
}


void EnvironmentNAV2D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int aind;

#if TIME_DEBUG
		clock_t currenttime = clock();
#endif

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();
    SuccIDV->reserve(ENVNAV2D_ACTIONSWIDTH);
    CostV->reserve(ENVNAV2D_ACTIONSWIDTH);

	//goal state should be absorbing
	if(SourceStateID == EnvNAV2D.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[SourceStateID];
	
	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV2DCfg.EnvWidth_c-1 || 
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV2DCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (aind = 0; aind < ENVNAV2D_ACTIONSWIDTH; aind++)
	{
        int newX = HashEntry->X + EnvNAV2DCfg.dXY[aind][0];
        int newY = HashEntry->Y + EnvNAV2DCfg.dXY[aind][1];
    
        //skip the invalid cells
         if(bTestBounds){
            if(!IsValidCell(newX, newY))
                continue;
        }

		int costmult = EnvNAV2DCfg.Grid2D[newX][newY];

		//for diagonal move, take max over adjacent cells
        if(newX != HashEntry->X && newY != HashEntry->Y)
		{
			costmult = __max(costmult, 	EnvNAV2DCfg.Grid2D[HashEntry->X][newY]);
			costmult = __max(costmult, EnvNAV2DCfg.Grid2D[newX][HashEntry->Y]);
		}
		//check that it is valid
		if(costmult >= EnvNAV2DCfg.obsthresh)
			continue;

		//otherwise compute the actual cost
		int cost = (costmult+1)*ENVNAV2D_COSTMULT;
        //diagonal moves are costlier
        if(newX != HashEntry->X && newY != HashEntry->Y)
            cost = (int)(sqrt(2)*cost);
		 

    	EnvNAV2DHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY);
		}

        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif

}

void EnvironmentNAV2D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

    int aind;

#if TIME_DEBUG
	clock_t currenttime = clock();
#endif

    //clear the successor array
    PredIDV->clear();
    CostV->clear();
    PredIDV->reserve(ENVNAV2D_ACTIONSWIDTH);
    CostV->reserve(ENVNAV2D_ACTIONSWIDTH);

	//get X, Y for the state
	EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[TargetStateID];

	//no predecessors if obstacle
	if(EnvNAV2DCfg.Grid2D[HashEntry->X][HashEntry->Y] >= EnvNAV2DCfg.obsthresh)
		return;

	int targetcostmult = EnvNAV2DCfg.Grid2D[HashEntry->X][HashEntry->Y];

	//iterate through actions
    bool bTestBounds = false;
    if(HashEntry->X == 0 || HashEntry->X == EnvNAV2DCfg.EnvWidth_c-1 || 
       HashEntry->Y == 0 || HashEntry->Y == EnvNAV2DCfg.EnvHeight_c-1)
        bTestBounds = true;
	for (aind = 0; aind < ENVNAV2D_ACTIONSWIDTH; aind++)
	{
        int predX = HashEntry->X + EnvNAV2DCfg.dXY[aind][0];
        int predY = HashEntry->Y + EnvNAV2DCfg.dXY[aind][1];
    
        //skip the invalid cells
         if(bTestBounds){
            if(!IsValidCell(predX, predY))
                continue;
        }

		//compute costmult
		 int costmult = targetcostmult;
		//for diagonal move, take max over adjacent cells
        if(predX != HashEntry->X && predY != HashEntry->Y)
		{
			costmult = __max(costmult, 	EnvNAV2DCfg.Grid2D[HashEntry->X][predY]);
			costmult = __max(costmult, EnvNAV2DCfg.Grid2D[predX][HashEntry->Y]);
		}
		//check that it is valid
		if(costmult >= EnvNAV2DCfg.obsthresh)
			continue;

		//otherwise compute the actual cost
		int cost = (costmult+1)*ENVNAV2D_COSTMULT;
        //diagonal moves are costlier
        if(predX != HashEntry->X && predY != HashEntry->Y)
            cost = (int)(sqrt(2)*cost);

    	EnvNAV2DHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(predX, predY)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(predX, predY);
		}


        PredIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

#if TIME_DEBUG
		time_getsuccs += clock()-currenttime;
#endif


}



int EnvironmentNAV2D::SizeofCreatedEnv()
{
	return (int)EnvNAV2D.StateID2CoordTable.size();
	
}

void EnvironmentNAV2D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= (int)EnvNAV2D.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV2D... function: stateID illegal (2)\n");
		exit(1);
	}
#endif

	if(fOut == NULL)
		fOut = stdout;

	EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[stateID];

	if(stateID == EnvNAV2D.goalstateid && bVerbose)
	{
		fprintf(fOut, "the state is a goal state\n");
	}

    if(bVerbose)
    	fprintf(fOut, "X=%d Y=%d\n", HashEntry->X, HashEntry->Y);
    else
    	fprintf(fOut, "%d %d\n", HashEntry->X, HashEntry->Y);

}

void EnvironmentNAV2D::GetCoordFromState(int stateID, int& x, int& y) const {
  EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[stateID];
  x = HashEntry->X;
  y = HashEntry->Y;
}

int EnvironmentNAV2D::GetStateFromCoord(int x, int y) {

   EnvNAV2DHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y);
    }
    return OutHashEntry->stateID;
}

const EnvNAV2DConfig_t* EnvironmentNAV2D::GetEnvNavConfig() {
  return &EnvNAV2DCfg;
}

//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2D::SetGoal(int x, int y){

    if(!IsWithinMapCell(x,y))
        return -1;

    EnvNAV2DHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y);
    }
    EnvNAV2D.goalstateid = OutHashEntry->stateID;

    return EnvNAV2D.goalstateid;    

}


//returns the stateid if success, and -1 otherwise
int EnvironmentNAV2D::SetStart(int x, int y){

    if(!IsWithinMapCell(x,y))
        return -1;

    EnvNAV2DHashEntry_t* OutHashEntry;
    if((OutHashEntry = GetHashEntry(x, y)) == NULL){
        //have to create a new entry
        OutHashEntry = CreateNewHashEntry(x, y);
    }
    EnvNAV2D.startstateid = OutHashEntry->stateID;

    return EnvNAV2D.startstateid;    

}

bool EnvironmentNAV2D::UpdateCost(int x, int y, unsigned char newcost)
{

    EnvNAV2DCfg.Grid2D[x][y] = newcost;

    return true;
}


void EnvironmentNAV2D::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAV2D. configuration
	
	printf("ERROR in EnvNAV2D... function: PrintEnv_Config is undefined\n");
	exit(1);

}

void EnvironmentNAV2D::PrintTimeStat(FILE* fOut)
{

#if TIME_DEBUG
    fprintf(fOut, "time3_addallout = %f secs, time_gethash = %f secs, time_createhash = %f secs, time_getsuccs = %f\n",
            time3_addallout/(double)CLOCKS_PER_SEC, time_gethash/(double)CLOCKS_PER_SEC, 
            time_createhash/(double)CLOCKS_PER_SEC, time_getsuccs/(double)CLOCKS_PER_SEC);
#endif
}


void EnvironmentNAV2D::GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV)
{
	nav2dcell_t cell;

	for(int i = 0; i < (int)changedcellsV->size(); i++)
	{
		cell = changedcellsV->at(i);
		preds_of_changededgesIDV->push_back(GetStateFromCoord(cell.x,cell.y));
		for(int j = 0; j < 8; j++){
			int affx = cell.x + EnvNAV2DCfg.dXY[j][0];
			int affy = cell.y + EnvNAV2DCfg.dXY[j][1];
			if(affx < 0 || affx >= EnvNAV2DCfg.EnvWidth_c || affy < 0 || affy >= EnvNAV2DCfg.EnvHeight_c)
				continue;
			preds_of_changededgesIDV->push_back(GetStateFromCoord(affx,affy));
		}
	}
}


bool EnvironmentNAV2D::IsObstacle(int x, int y)
{

	return (EnvNAV2DCfg.Grid2D[x][y] >= EnvNAV2DCfg.obsthresh);

}

unsigned char EnvironmentNAV2D::GetMapCost(int x, int y)
{
	return EnvNAV2DCfg.Grid2D[x][y];
}



void EnvironmentNAV2D::GetEnvParms(int *size_x, int *size_y, int* startx, int* starty, int* goalx, int* goaly)
{
	*size_x = EnvNAV2DCfg.EnvWidth_c;
	*size_y = EnvNAV2DCfg.EnvHeight_c;

	*startx = EnvNAV2DCfg.StartX_c;
	*starty = EnvNAV2DCfg.StartY_c;
	*goalx = EnvNAV2DCfg.EndX_c;
	*goaly = EnvNAV2DCfg.EndY_c;
}



//------------------------------------------------------------------------------
