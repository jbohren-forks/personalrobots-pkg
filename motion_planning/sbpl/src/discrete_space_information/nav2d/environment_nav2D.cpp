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




//extern clock_t time3_addallout;
//extern clock_t time_gethash;
//extern clock_t time_createhash;

//function prototypes


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
		if(EnvNAV2D.Coord2StateIDHashTable[j].size() == 0)
			s0++;
		else if(EnvNAV2D.Coord2StateIDHashTable[j].size() < 50)
			s1++;
		else if(EnvNAV2D.Coord2StateIDHashTable[j].size() < 100)
			s50++;
		else if(EnvNAV2D.Coord2StateIDHashTable[j].size() < 200)
			s100++;
		else if(EnvNAV2D.Coord2StateIDHashTable[j].size() < 300)
			s200++;
		else if(EnvNAV2D.Coord2StateIDHashTable[j].size() < 400)
			s300++;
		else
			slarge++;
	}
	printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
		s0,s1, s50, s100, s200,s300,slarge);
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
	EnvNAV2DCfg.Grid2D = new char* [EnvNAV2DCfg.EnvWidth_c];
	for (x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++)
	{
		EnvNAV2DCfg.Grid2D[x] = new char [EnvNAV2DCfg.EnvHeight_c];
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
	clock_t currenttime = clock();

	int binid = GETHASHBIN(X, Y);
	
#if DEBUG
	if (EnvNAV2D.Coord2StateIDHashTable[binid].size() > 500)
	{
		printf("WARNING: Hash table has a bin %d (X=%d Y=%d) of size %d\n", 
			binid, X, Y, EnvNAV2D.Coord2StateIDHashTable[binid].size());
		
		PrintHashTableHist();		
	}
#endif

	//iterate over the states in the bin and select the perfect match
	for(int ind = 0; ind < EnvNAV2D.Coord2StateIDHashTable[binid].size(); ind++)
	{
		if( EnvNAV2D.Coord2StateIDHashTable[binid][ind]->X == X 
			&& EnvNAV2D.Coord2StateIDHashTable[binid][ind]->Y == Y)
		{
			//time_gethash += clock()-currenttime;
			return EnvNAV2D.Coord2StateIDHashTable[binid][ind];
		}
	}
	
	//time_gethash += clock()-currenttime;

	return NULL;	  
}


EnvNAV2DHashEntry_t* EnvironmentNAV2D::CreateNewHashEntry(int X, int Y) 
{
	int i;
	
	clock_t currenttime = clock();

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

	if(HashEntry->stateID != StateID2IndexMapping.size()-1)
	{
		printf("ERROR in Env... function: last state has incorrect stateID\n");
		exit(1);	
	}


	//time_createhash += clock()-currenttime;

	return HashEntry;
}

bool EnvironmentNAV2D::IsValidCell(int X, int Y)
{
	return (X >= 0 && X < EnvNAV2DCfg.EnvWidth_c && 
		Y >= 0 && Y < EnvNAV2DCfg.EnvHeight_c && 
		EnvNAV2DCfg.Grid2D[X][Y] == 0);
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
	EnvNAV2D.HashTableSize = 32*1024; //should be power of two
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
    return COSTMULT*dist;

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
bool EnvironmentNAV2D::InitializeEnv(char* sEnvFile)
{

	FILE* fCfg = fopen(sEnvFile, "r");
	if(fCfg == NULL)
	{
		printf("ERROR: unable to open %s\n", sEnvFile);
		exit(1);
	}
	ReadConfiguration(fCfg);

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
	if(FromStateID >= EnvNAV2D.StateID2CoordTable.size() 
		|| ToStateID >= EnvNAV2D.StateID2CoordTable.size())
	{
		printf("ERROR in EnvNAV2D... function: stateID illegal\n");
		exit(1);
	}
#endif;

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
	if(stateID >= EnvNAV2D.StateID2CoordTable.size())
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
	if(stateID >= EnvNAV2D.StateID2CoordTable.size())
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
	if(state->StateID >= EnvNAV2D.StateID2CoordTable.size())
	{
		printf("ERROR in Env... function: stateID illegal\n");
		exit(1);
	}

	if(state->Actions.size() != 0)
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
	for (int aind = 0; aind < ACTIONSWIDTH; aind++)
	{
        int newX = HashEntry->X + EnvNAV2DCfg.dXY[aind][0];
        int newY = HashEntry->Y + EnvNAV2DCfg.dXY[aind][1];

        //skip the invalid cells
        if(!IsValidCell(newX, newY))
            continue;

		//skip invalid diagonal move
        if(newX != HashEntry->X && newY != HashEntry->Y)
		{
			if(EnvNAV2DCfg.Grid2D[HashEntry->X][newY] != 0 || EnvNAV2DCfg.Grid2D[newX][HashEntry->Y] != 0)
				continue;
		}


		//add the action
		CMDPACTION* action = state->AddAction(aind);

		//clock_t currenttime = clock();
        
        cost = COSTMULT;
        //diagonal moves are costlier
        if(newX != HashEntry->X && newY != HashEntry->Y)
            cost = sqrt(2)*cost;

    	EnvNAV2DHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY);
		}
		action->AddOutcome(OutHashEntry->stateID, cost, 1.0); 

		//time3_addallout += clock()-currenttime;
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

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

	//goal state should be absorbing
	if(SourceStateID == EnvNAV2D.goalstateid)
		return;

	//get X, Y for the state
	EnvNAV2DHashEntry_t* HashEntry = EnvNAV2D.StateID2CoordTable[SourceStateID];
	
	//iterate through actions
	for (aind = 0; aind < ACTIONSWIDTH; aind++)
	{
        int newX = HashEntry->X + EnvNAV2DCfg.dXY[aind][0];
        int newY = HashEntry->Y + EnvNAV2DCfg.dXY[aind][1];
    
        //skip the invalid cells
        if(!IsValidCell(newX, newY))
            continue;

		//skip invalid diagonal move
        if(newX != HashEntry->X && newY != HashEntry->Y)
		{
			if(EnvNAV2DCfg.Grid2D[HashEntry->X][newY] != 0 || EnvNAV2DCfg.Grid2D[newX][HashEntry->Y] != 0)
				continue;
		}


    	EnvNAV2DHashEntry_t* OutHashEntry;
		if((OutHashEntry = GetHashEntry(newX, newY)) == NULL)
		{
			//have to create a new entry
			OutHashEntry = CreateNewHashEntry(newX, newY);
		}

        //compute clow
        int cost = COSTMULT;
        //diagonal moves are costlier
        if(newX != HashEntry->X && newY != HashEntry->Y)
            cost = sqrt(2)*cost;


        SuccIDV->push_back(OutHashEntry->stateID);
        CostV->push_back(cost);
	}

}


int EnvironmentNAV2D::SizeofCreatedEnv()
{
	return EnvNAV2D.StateID2CoordTable.size();
	
}

void EnvironmentNAV2D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
#if DEBUG
	if(stateID >= EnvNAV2D.StateID2CoordTable.size())
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


void EnvironmentNAV2D::PrintEnv_Config(FILE* fOut)
{

	//implement this if the planner needs to print out EnvNAV2D. configuration
	
	printf("ERROR in EnvNAV2D... function: PrintEnv_Config is undefined\n");
	exit(1);

}


//------------------------------------------------------------------------------
