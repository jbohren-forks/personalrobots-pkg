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


#define PI_CONST 3.141592653

#define XYZTO3DIND(x,y,z) ((x) + (y)*EnvROBARMCfg.EnvWidth_c + (z)*EnvROBARMCfg.EnvWidth_c*EnvROBARMCfg.EnvHeight_c)
#define DEG2RAD(d) ((d)*(PI_CONST/180.0))
#define RAD2DEG(r) ((r)*(180.0/PI_CONST))

// output a lot of debug output
#define DEBUG_A_LOT 0

// cost multiplier
#define COSTMULT 1000
#define COSTOFACTION 1000

// max number of cells traversed per movement of one joint (9 degree increments, .825m arm = 3.45)
#define CELLSPERACTION 3.45

// 1.414*(1000/3.45) = 410
#define SQRT_2_COSTMULT 410

//1.732*(1000/3.45) = 502
#define SQRT_3_COSTMULT 502

// discretization of joint angles
#define ANGLEDELTA 40.0

// number of successors to a cell
#define DIRECTIONS 26

// offset in cells to make map centered at (H/2,W/2,D/2) instead of (0,0,0) (like in gazebo)
#define X_OFFSET_C 0
#define Y_OFFSET_C 24
#define Z_OFFSET_C 0

#define OUTPUT_OBSTACLES 0

// length of gripper along x-axis in wrist frame (meters)
#define GRIPPER_LENGTHX 0.11

//padding of obstacles (meters)
#define PADDING 0.16

#define PRECOMPUTE_DH 1

#define SMOOTH_COST 0

#define VERBOSE 1

int dx[DIRECTIONS] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
int dy[DIRECTIONS] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
int dz[DIRECTIONS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

static clock_t DH_time = 0;
static clock_t KL_time = 0;
static clock_t check_collision_time = 0;
static clock_t trig_time = 0;
static clock_t DH_construct_trans_time = 0;
static clock_t DH_matrix_mult_time = 0;
static clock_t DH_fetch_trans_time = 0;
static int num_forwardkinematics = 0;

// TODO:
// -self-collision checking


//-------------------state access functions---------------------
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
unsigned int EnvironmentROBARM::GETHASHBIN(short unsigned int* coord, int numofcoord)
{

    int val = 0;

    for(int i = 0; i < numofcoord; i++)
    {
        val += inthash(coord[i]) << i;
    }

    return inthash(val) & (EnvROBARM.HashTableSize-1);
}

void EnvironmentROBARM::PrintHashTableHist()
{
    int s0=0, s1=0, s50=0, s100=0, s200=0, s300=0, slarge=0;

    for(int  j = 0; j < EnvROBARM.HashTableSize; j++)
    {
        if((int)EnvROBARM.Coord2StateIDHashTable[j].size() == 0)
                s0++;
        else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 50)
                s1++;
        else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 100)
                s50++;
        else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 200)
                s100++;
        else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 300)
                s200++;
        else if((int)EnvROBARM.Coord2StateIDHashTable[j].size() < 400)
                s300++;
        else
                slarge++;
    }
    printf("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n",
        s0,s1, s50, s100, s200,s300,slarge);
}

EnvROBARMHashEntry_t* EnvironmentROBARM::GetHashEntry(short unsigned int* coord, int numofcoord, bool bIsGoal)
{
    //clock_t currenttime = clock();

    //if it is goal
    if(bIsGoal)
    {
        return EnvROBARM.goalHashEntry;
    }

    int binid = GETHASHBIN(coord, numofcoord);
	
#if DEBUG
    if ((int)EnvROBARM.Coord2StateIDHashTable[binid].size() > 500)
    {
	printf("WARNING: Hash table has a bin %d (coord0=%d) of size %d\n", 
            binid, coord[0], EnvROBARM.Coord2StateIDHashTable[binid].size());
	
	PrintHashTableHist();		
    }
#endif

    //iterate over the states in the bin and select the perfect match
    for(int ind = 0; ind < (int)EnvROBARM.Coord2StateIDHashTable[binid].size(); ind++)
    {
        int j = 0;
        for(j = 0; j < numofcoord; j++)
        {
            if( EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
            {
                break;
            }
        }

        if (j == numofcoord)
        {
            //time_gethash += clock()-currenttime;
            return EnvROBARM.Coord2StateIDHashTable[binid][ind];
        }
    }

    //time_gethash += clock()-currenttime;

    return NULL;  
}

EnvROBARMHashEntry_t* EnvironmentROBARM::CreateNewHashEntry(short unsigned int* coord, int numofcoord, short unsigned int endeff[3], short unsigned int wrist[3], short unsigned int elbow[3], short unsigned int action) 
{
    int i;
    //clock_t currenttime = clock();
    EnvROBARMHashEntry_t* HashEntry = new EnvROBARMHashEntry_t;

    memcpy(HashEntry->coord, coord, numofcoord*sizeof(short unsigned int));
    memcpy(HashEntry->endeff, endeff, 3*sizeof(short unsigned int));
    memcpy(HashEntry->wrist, wrist, 3*sizeof(short unsigned int));
    memcpy(HashEntry->elbow, elbow, 3*sizeof(short unsigned int));
    HashEntry->action = action;

    //get rid of this
    HashEntry->endeffx = endeff[0];
    HashEntry->endeffy = endeff[1];
    HashEntry->endeffz = endeff[2];

    // assign a stateID to HashEntry to be used 
    HashEntry->stateID = EnvROBARM.StateID2CoordTable.size();

    //insert into the tables
    EnvROBARM.StateID2CoordTable.push_back(HashEntry);

    //get the hash table bin
    i = GETHASHBIN(HashEntry->coord, numofcoord);

    //insert the entry into the bin
    EnvROBARM.Coord2StateIDHashTable[i].push_back(HashEntry);

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

    //time_createhash += clock()-currenttime;
    return HashEntry;
}
//--------------------------------------------------------------

//---------------heuristic values computation-------------------

//compute straight line distance from x,y,z to every other cell
void EnvironmentROBARM::getDistancetoGoal(int* HeurGrid, int goalx, int goaly, int goalz)
{
    for(int x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for(int y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            for(int z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
            {
		HeurGrid[XYZTO3DIND(x,y,z)] = (COSTMULT/CELLSPERACTION)*sqrt((goalx-x)*(goalx-x) + (goaly-y)*(goaly-y) +  (goalz-z)*(goalz-z));
//                 printf("HeurGrid(%i,%i,%i) -> (%i,%i,%i): %i\n",x,y,z,goalx,goaly,goalz,HeurGrid[XYZTO3DIND(x,y,z)]);
            }
        }
    }
}

void EnvironmentROBARM::ReInitializeState3D(State3D* state)
{
	state->g = INFINITECOST;
	state->iterationclosed = 0;
}

void EnvironmentROBARM::InitializeState3D(State3D* state, short unsigned int x, short unsigned int y, short unsigned int z)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
    state->x = x;
    state->y = y;
    state->z = z;
}

void EnvironmentROBARM::Create3DStateSpace(State3D**** statespace3D)
{
    int x,y,z;

    //allocate a statespace for 3D search
    *statespace3D = new State3D** [EnvROBARMCfg.EnvWidth_c];
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
	(*statespace3D)[x] = new State3D* [EnvROBARMCfg.EnvHeight_c];
	for(y = 0; y < EnvROBARMCfg.EnvWidth_c; y++)
	{
	    (*statespace3D)[x][y] = new State3D [EnvROBARMCfg.EnvDepth_c];
	    for(z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
	    {
		InitializeState3D(&(*statespace3D)[x][y][z],x,y,z);
	    }
	}
    }
}

void EnvironmentROBARM::Delete3DStateSpace(State3D**** statespace3D)
{
	int x,y;

	//delete the 3D statespace
	for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
	{
	   for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
		delete [] (*statespace3D)[x][y];

	   delete [] (*statespace3D)[x];
	}
	delete *statespace3D;
}

void EnvironmentROBARM::ComputeHeuristicValues()
{
    printf("Running 3D BFS to compute heuristics\n");
    clock_t currenttime = clock();

    //allocate memory
    int hsize = XYZTO3DIND(EnvROBARMCfg.EnvWidth_c-1, EnvROBARMCfg.EnvHeight_c-1,EnvROBARMCfg.EnvDepth_c-1)+1;
    EnvROBARM.Heur = new int [hsize]; 

    //now compute the heuristics for the ONLY goal location
    State3D*** statespace3D;	
    Create3DStateSpace(&statespace3D);

    Search3DwithQueue(statespace3D, EnvROBARM.Heur, EnvROBARMCfg.EndEffGoalX_c, EnvROBARMCfg.EndEffGoalY_c, EnvROBARMCfg.EndEffGoalZ_c);

    Delete3DStateSpace(&statespace3D);
    printf("completed in %.3f seconds.\n", double(clock()-currenttime) / CLOCKS_PER_SEC);
}

void EnvironmentROBARM::Search3DwithQueue(State3D*** statespace, int* HeurGrid, short unsigned int searchstartx, short unsigned int searchstarty, short unsigned int searchstartz)
{
    State3D* ExpState;
    int newx, newy, newz, x,y,z;
    unsigned int g_temp;

    //create a queue
    queue<State3D*> Queue; 

    //initialize to infinity all 
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
	for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
	{
	    for (z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
	    {
		HeurGrid[XYZTO3DIND(x,y,z)] = INFINITECOST;
		ReInitializeState3D(&statespace[x][y][z]);
	    }
	}
    }

    //initialization
    statespace[searchstartx][searchstarty][searchstartz].g = 0;	
    Queue.push(&statespace[searchstartx][searchstarty][searchstartz]);

    //expand all of the states
    while((int)Queue.size() > 0)
    {
	//get the state to expand
	ExpState = Queue.front();
	
	Queue.pop();

	//it may be that the state is already closed
	if(ExpState->iterationclosed == 1)
	    continue;

	//close it
	ExpState->iterationclosed = 1;

//         printf("ExpState: %u,%u,%u, g-cost:%i \n",ExpState->x,ExpState->y, ExpState->z, ExpState->g);

	//set the corresponding heuristics
	HeurGrid[XYZTO3DIND(ExpState->x, ExpState->y, ExpState->z)] = ExpState->g;

	//iterate through neighbors
	for(int d = 0; d < DIRECTIONS; d++)
	{
	    newx = ExpState->x + dx[d];
	    newy = ExpState->y + dy[d];
	    newz = ExpState->z + dz[d];

	    //make sure it is inside the map and has no obstacle
	    if(0 > newx || newx >= EnvROBARMCfg.EnvWidth_c ||
		0 > newy || newy >= EnvROBARMCfg.EnvHeight_c ||
		0 > newz || newz >= EnvROBARMCfg.EnvDepth_c ||
		EnvROBARMCfg.Grid3D[newx][newy][newz] == 1)
		    continue;

	    if(statespace[newx][newy][newz].iterationclosed == 0)
	    {
		//insert into the stack
		Queue.push(&statespace[newx][newy][newz]);

		//set the g-value
		if (ExpState->x != newx && ExpState->y != newy && ExpState->z != newz)
                    g_temp = ExpState->g + SQRT_3_COSTMULT;
		else if ((ExpState->y != newy && ExpState->z != newz) ||
		          (ExpState->x != newx && ExpState->z != newz) ||
		          (ExpState->x != newx && ExpState->y != newy))
                    g_temp = ExpState->g + SQRT_2_COSTMULT;
		else
                    g_temp = ExpState->g + COSTMULT/CELLSPERACTION;

                if(statespace[newx][newy][newz].g > g_temp)
                    statespace[newx][newy][newz].g = g_temp;
//              printf("%i: %u,%u,%u, g-cost: %i\n",d,newx,newy,newz,statespace[newx][newy][newz].g);
	    }
	}
    }
}

/*precompute euclidean distance from every cell to every cell
void EnvironmentROBARM::ComputeHeuristicValues()
{
    printf("Using Euclidean distance to compute heuristics\n");

    clock_t currenttime = clock();

    //allocate memory
    int hsize = XYZTO3DIND(EnvROBARMCfg.EnvWidth_c-1, EnvROBARMCfg.EnvHeight_c-1,EnvROBARMCfg.EnvDepth_c-1)+1;
    EnvROBARM.Heur = new int* [hsize];
//     printf("hsize = %i\n",hsize);
    for(int i = 0; i < hsize; i++)
    {
        EnvROBARM.Heur[i] = new int [hsize];
//         printf("%i ", i);
    }
    printf("Done allocating memory for heuristic grid\n");

    //now compute the heuristics for each goal location
    for(int x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for(int y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
//             printf+_("hind = ");
            for(int z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
            {   
                int hind = XYZTO3DIND(x,y,z);
//                 printf("%i ",hind);  
                //euclidean distance
                getDistancetoGoal(&EnvROBARM.Heur[hind][0], x, y, z);
            }
//             printf("\n");
        }
//         printf("h for %d computed\n", x);
    }
    printf("heuristic computation time = %.3f secs\n", (clock()-currenttime)/((double)CLOCKS_PER_SEC));
    printf("Successfully computed heuristics\n");
}
*/
//--------------------------------------------------------------


//--------------printing routines-------------------------------
//temporary
void EnvironmentROBARM::PrintHeurGrid()
{
    int x,y;	

    printf("z=22\n");
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
	for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
	{
// 	    for(z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
// 	    {
		printf("%3.0u ",EnvROBARM.Heur[XYZTO3DIND(x,y,22)]/1000);
// 	    }
	}
	printf("\n");
    }

    printf("z=24\n");
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
	for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
	{
// 	    for(z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
// 	    {
		printf("%3.0u ",EnvROBARM.Heur[XYZTO3DIND(x,y,24)]/1000);
// 	    }
	}
	printf("\n");
    }
}

void EnvironmentROBARM::printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal)
{
    double angles[NUMOFLINKS];
    int dangles[NUMOFLINKS];
    int i;
    short unsigned int x,y,z;
    short unsigned int wrist[3];
    short unsigned int elbow[3];

    ComputeContAngles(coord, angles);

    if(bVerbose)
    {
        for (i = 0; i < NUMOFLINKS; i++)
        {
            dangles[i] = angles[i]*(180/PI_CONST) + 0.999999;
        }
    }

    if(bVerbose)
    	fprintf(fOut, "angles: ");
    for(i = 0; i < NUMOFLINKS; i++)
    {
        if(!bLocal)
            fprintf(fOut, "%i ", dangles[i]);
        else
        {
            if(i > 0)
                fprintf(fOut, "%i ", dangles[i]-dangles[i-1]);
            else
                fprintf(fOut, "%i ", dangles[i]);
        }
    }
// 	for(i = 0; i < NUMOFLINKS; i++)
// 	{
//         if(!bLocal)
//     		fprintf(fOut, "%1.2f ", angles[i]);
//         else
//         {
// 		    if(i > 0)
// 			    fprintf(fOut, "%1.2f ", angles[i]-angles[i-1]);
// 		    else
// 			    fprintf(fOut, "%1.2f ", angles[i]);
//         }
// 	}

    ComputeEndEffectorPos(angles, &x, &y, &z, wrist, elbow);
    if(bGoal)
    {
        x = EnvROBARMCfg.EndEffGoalX_c;
        y = EnvROBARMCfg.EndEffGoalY_c;
        z = EnvROBARMCfg.EndEffGoalZ_c;
    }
    if(bVerbose)
    	fprintf(fOut, "   endeff: %d %d %d", x,y,z);
    else
    	fprintf(fOut, "%d %d %d", x,y,z);

    fprintf(fOut, "\n");
}

/*
void EnvironmentROBARM::PrintPathRec(State* currentstate, State* searchstartstate, int* cost)
{
	if(currentstate == searchstartstate)
	{
		return;
	}

	if(currentstate->g > currentstate->v)
	{
		printf("ERROR: an underconsistent state is encountered on the path\n");

#if PLANNER_TYPE == ARA_PLANNER_TYPE 
		PrintState(currentstate, stdout);
#endif
		exit(1);
	}

	//cost of the transition to the previous state
	*cost = *cost + currentstate->costtobestnextstate;
	if(*cost > 100000 || currentstate->g < currentstate->bestnextstate->v +
		currentstate->costtobestnextstate || 
		!IsValidCoord(currentstate->bestnextstate->statecoord))
	{
#if PLANNER_TYPE == ARA_PLANNER_TYPE
		printf("currentstate\n");
		PrintState(currentstate, stdout);
		printf("nextstate\n");
		PrintState(currentstate->bestnextstate, stdout);
#endif
		printf("next statecoord IsValid=%d\n", 
			IsValidCoord(currentstate->bestnextstate->statecoord));
		printf("ERROR: cost of the path too high=%d or incorrect transition\n", *cost);
		exit(1);
	}
	//transition itself
	currentstate = currentstate->bestnextstate;

	

	//proceed recursively
#if FORWARD_SEARCH
	PrintPathRec(currentstate, searchstartstate, cost);
	printangles(fSol, currentstate);
#else
	printangles(fSol, currentstate);
	PrintPathRec(currentstate, searchstartstate, cost);
#endif

}


//prints found path and some statistics
int EnvironmentROBARM::PrintPathStat(double erreps, int* pathcost)
{
	State *goalstate, *startstate;
	short unsigned int coord[NUMOFLINKS];
	int cost = 0;
	int inconssize;

	goalstate = RobArmStateSpace.goalstate;
	
	ComputeCoord(RobArmStateSpace.currentangle, coord);
	startstate = GetState(coord);

#if FORWARD_SEARCH
	if(goalstate == NULL || goalstate->bestnextstate == NULL)
	{
		printf("No path is found\n");
		*pathcost = INFINITECOST;
		return 0;
	}
	PrintPathRec(goalstate, startstate, &cost);
	printangles(fSol, goalstate);
#else
	if(startstate == NULL || startstate->bestnextstate == NULL)
	{
		printf("No path is found\n");
		*pathcost = INFINITECOST;
		return 0;
	}
	int goalh = Heuristic(goalstate, startstate);
	if(startstate->g+startstate->h < goalh)
	{
		printf("ERROR: invalid heuristic. goalh(updated)=%d startg=%d\n",
			goalh, startstate->g);
#if PLANNER_TYPE == ARA_PLANNER_TYPE
		PrintState(startstate, stdout);
#endif
		exit(1);
	}
	else
		printf("goalh=%d startg=%d\n", goalstate->h, startstate->g);
	printangles(fSol, startstate);
	PrintPathRec(startstate, goalstate, &cost);
#endif

	fprintf(fSol, "path cost=%d at eps=%f\n", cost, erreps);
	printf("path cost=%d\n", cost);

#if PLANNER_TYPE == ANYASTAR_PLANNER_TYPE
	inconssize = 0;
#else
	inconssize = RobArmStateSpace.inconslist->currentsize;
#endif

	printf("open size=%d, incons size=%d\n", 
		RobArmStateSpace.heap->currentsize, inconssize);
	fprintf(fSol, "open size=%d, incons size=%d\n", 
		RobArmStateSpace.heap->currentsize, inconssize);

	fprintf(fSol1, "**************\n");
	fflush(fStat);

	*pathcost = cost;

	return 1;
}

void EnvironmentROBARM::PrintInfo()
{
	int i;
	unsigned int statespace_size = 1;
	double fsize = 1.0;

	fprintf(fSol, "statespace dim=%d size= <", NUMOFLINKS);
	fprintf(fSol1, "%d\n", NUMOFLINKS);
	for(i = 0; i < NUMOFLINKS; i++)
	{
		statespace_size *= RobArmStateSpace.anglevals[i];
		fprintf(fSol, "%d ", RobArmStateSpace.anglevals[i]);
		fprintf(fSol1, "%f ", EnvROBARMCfg.LinkLength_m[i]);
		fsize = fsize*RobArmStateSpace.anglevals[i];
	}
	fprintf(fSol, "> => %g\n", fsize);
	fprintf(fSol1, "\n");

	printf("dimensionality: %d statespace: %u (over 10^%d)\n", NUMOFLINKS, statespace_size, 
		(int)(log10(fsize)));
}

void PrintCoord(short unsigned int coord[NUMOFLINKS], FILE* fOut)
{
	for(int i = 0; i < NUMOFLINKS; i++)
		fprintf(fOut, "%d ", coord[i]);
	fprintf(fOut, "\n");
}
*/
//--------------------------------------------------------------


//---------------Additional domain specific functions-----------
void EnvironmentROBARM::ReadConfiguration(FILE* fCfg)
{
    char sTemp[1024];
    int x, y, z, i, object_width, object_height, object_depth;
    double xd,yd,zd;
    short unsigned int pX, pY, pZ;
    short unsigned int wrist[3];
    short unsigned int elbow[3];

    //environmentsize(meters)
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.EnvWidth_m = atof(sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.EnvHeight_m = atof(sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.EnvDepth_m = atof(sTemp);

    //discretization(cells)
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.EnvWidth_c = atoi(sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.EnvHeight_c = atoi(sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.EnvDepth_c = atof(sTemp);

    //set additional parameters
    EnvROBARMCfg.GridCellWidth = EnvROBARMCfg.EnvWidth_m/EnvROBARMCfg.EnvWidth_c;
    if(EnvROBARMCfg.GridCellWidth != EnvROBARMCfg.EnvHeight_m/EnvROBARMCfg.EnvHeight_c ||
       EnvROBARMCfg.GridCellWidth != EnvROBARMCfg.EnvDepth_m/EnvROBARMCfg.EnvDepth_c)
    {
	printf("ERROR: The cell should be square\n");
	exit(1);
    }

    //basexyz(cells): 
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.BaseX_c = atoi(sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.BaseY_c = atoi(sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.BaseZ_c = atoi(sTemp);

    //linklengths(meters): 
    fscanf(fCfg, "%s", sTemp);	
    for(i = 0; i < NUMOFLINKS; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.LinkLength_m[i] = atof(sTemp);
    }

    //linkstartangles(degrees): 
    fscanf(fCfg, "%s", sTemp);	
    for(i = 0; i < NUMOFLINKS; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.LinkStartAngles_d[i] = atoi(sTemp);
    }

    //endeffectorgoal(cells) or linkgoalangles(degrees) or endeffectorgoal(meters):
    fscanf(fCfg, "%s", sTemp);
    if(strcmp(sTemp, "endeffectorgoal(cells):") == 0)
    {
        //only endeffector is specified
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.EndEffGoalX_c = atoi(sTemp);
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.EndEffGoalY_c = atoi(sTemp);
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.EndEffGoalZ_c = atoi(sTemp);

        //set goalangle to invalid number
        EnvROBARMCfg.LinkGoalAngles_d[0] = INVALID_NUMBER;
    }
    else if(strcmp(sTemp, "linkgoalangles(degrees):") == 0)
    {
        double goalangles[NUMOFLINKS];

        //linkgoalangles(degrees): 90.0 180.0 90.0		
        for(i = 0; i < NUMOFLINKS; i++)
        {
	    fscanf(fCfg, "%s", sTemp);
	    EnvROBARMCfg.LinkGoalAngles_d[i] = atoi(sTemp);
        }

        //compute endeffectorgoal(cells):
        for(i = 0; i < NUMOFLINKS; i++)
        {
            // convert goal angles to radians
            goalangles[i] = PI_CONST*(EnvROBARMCfg.LinkGoalAngles_d[i]/180.0);
        }
        ComputeEndEffectorPos(goalangles, &EnvROBARMCfg.EndEffGoalX_c, &EnvROBARMCfg.EndEffGoalY_c, &EnvROBARMCfg.EndEffGoalZ_c, wrist, elbow);
    }
    else if(strcmp(sTemp, "endeffectorgoal(meters):") == 0)
    {
        //only endeffector is specified
        fscanf(fCfg, "%s", sTemp);
        xd = atof(sTemp);
        fscanf(fCfg, "%s", sTemp);
        yd = atof(sTemp);
        fscanf(fCfg, "%s", sTemp);
        zd = atof(sTemp);

	ContXYZ2Cell(xd,yd,zd, &EnvROBARMCfg.EndEffGoalX_c, &EnvROBARMCfg.EndEffGoalY_c, &EnvROBARMCfg.EndEffGoalZ_c);

	//set goalangle to invalid number
        EnvROBARMCfg.LinkGoalAngles_d[0] = INVALID_NUMBER;
    }
    else
    {
        printf("ERROR: invalid string encountered=%s\n", sTemp);
        exit(1);
    }

    //linktwist(degrees)
    fscanf(fCfg, "%s", sTemp);
    for(i = 0; i < NUMOFLINKS_DH; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.DH_alpha[i] = PI_CONST*(atof(sTemp)/180.0);
    }

    //linklength(meters)
    fscanf(fCfg, "%s", sTemp);   
    for(i = 0; i < NUMOFLINKS_DH; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.DH_a[i] = atof(sTemp);
    }

    //linkoffset(meters)
    fscanf(fCfg, "%s", sTemp);   
    for(i = 0; i < NUMOFLINKS_DH; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.DH_d[i] = atof(sTemp);
    }

    //jointangles(degrees)
    fscanf(fCfg, "%s", sTemp);
    for(i = 0; i < NUMOFLINKS_DH; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.DH_theta[i] = DEG2RAD(atof(sTemp));
    }

    //posmotorlimits(degrees)
    fscanf(fCfg, "%s", sTemp);
    for(i = 0; i < NUMOFLINKS; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.PosMotorLimits[i] = DEG2RAD(atof(sTemp));
    }

    //negmotorlimits(degrees)
    fscanf(fCfg, "%s", sTemp);
    for(i = 0; i < NUMOFLINKS; i++)
    {
        fscanf(fCfg, "%s", sTemp);
        EnvROBARMCfg.NegMotorLimits[i] = DEG2RAD(atof(sTemp));
    }

    //allocate the 3D environment & fill set all cells to zero
    EnvROBARMCfg.Grid3D = new char** [EnvROBARMCfg.EnvWidth_c];
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        EnvROBARMCfg.Grid3D[x] = new char* [EnvROBARMCfg.EnvHeight_c];
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            EnvROBARMCfg.Grid3D[x][y] = new char [EnvROBARMCfg.EnvDepth_c];
            for (z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
            {// 		if(pZ_min < 0)
//                     pZ_min = 0;
                EnvROBARMCfg.Grid3D[x][y][z] = 0;
            }
        }
    }

    //environment:
    fscanf(fCfg, "%s", sTemp);
    if(strcmp(sTemp, "environment:") == 0)
    {
        fscanf(fCfg, "%s", sTemp);
        while(!feof(fCfg) && strlen(sTemp) != 0)
        {
            if(strcmp(sTemp, "cube(meters):") == 0)
            {
                fscanf(fCfg, "%s", sTemp);
                xd = atof(sTemp);
                fscanf(fCfg, "%s", sTemp);
                yd = atof(sTemp);
                fscanf(fCfg, "%s", sTemp);
                zd = atof(sTemp);
                ContXYZ2Cell(xd,yd,zd,&pX,&pY,&pZ);

                fscanf(fCfg, "%s", sTemp);
                object_width = abs((atof(sTemp)+PADDING) / EnvROBARMCfg.GridCellWidth);

                fscanf(fCfg, "%s", sTemp);
                object_height = abs((atof(sTemp)+PADDING) / EnvROBARMCfg.GridCellWidth);

                fscanf(fCfg, "%s", sTemp);
                object_depth = abs((atof(sTemp)+ PADDING) / EnvROBARMCfg.GridCellWidth);

                short unsigned int pX_max = pX + object_width/2;
                short unsigned int pX_min = pX - object_width/2;
                short unsigned int pY_max = pY + object_height/2;
                short unsigned int pY_min = pY - object_height/2;
                short unsigned int pZ_max = pZ + object_depth/2;
                short unsigned int pZ_min = pZ - object_depth/2;

                 // bounds checking, cutoff object if need be
		if(pX_max > EnvROBARMCfg.EnvWidth_c-1)
                    pX_max = EnvROBARMCfg.EnvWidth_c-1;
		if(pY_max > EnvROBARMCfg.EnvHeight_c-1)
                    pY_max = EnvROBARMCfg.EnvHeight_c-1;
		if(pZ_max > EnvROBARMCfg.EnvDepth_c-1)
                    pZ_max = EnvROBARMCfg.EnvDepth_c-1;

                // assign the cells occupying the obstacle to 1
                for (y = pY_min; y <= pY_max; y++)
                    for (x = pX_min; x <= pX_max; x++)
                    {
                        for (z = pZ_min; z <= pZ_max; z++)
                            EnvROBARMCfg.Grid3D[x][y][z] = 1;
                    }

                printf("Cubic Obstacle: (%i,%i,%i) (cell) H: %i  W: %i  D: %i\n",pX,pY,pZ,object_width,object_height, object_depth);
            }
            else if (strcmp(sTemp, "cube(cells):") == 0)
            {
                fscanf(fCfg, "%s", sTemp);
                pX = atoi(sTemp);
                fscanf(fCfg, "%s", sTemp);
                pY = atoi(sTemp);
                fscanf(fCfg, "%s", sTemp);
                pZ = atoi(sTemp);

                fscanf(fCfg, "%s", sTemp);
                object_width = abs(atoi(sTemp));

                fscanf(fCfg, "%s", sTemp);
                object_height = abs(atof(sTemp));

                fscanf(fCfg, "%s", sTemp);
                object_depth = abs(atof(sTemp));

                 // bounds checking, cutoff object if need be
                if(pX + object_width > EnvROBARMCfg.EnvWidth_c)
                    object_width = object_width - (pX + object_width - EnvROBARMCfg.EnvWidth_c);

                if(pY + object_height > EnvROBARMCfg.EnvHeight_c)
                    object_height = object_height - (pY + object_height - EnvROBARMCfg.EnvHeight_c);

                if(pZ + object_depth > EnvROBARMCfg.EnvDepth_c)
                    object_depth = object_depth - (pZ + object_depth - EnvROBARMCfg.EnvDepth_c);

                // assign the cells occupying the obstacle to 1
                for (y = pY; y < pY + object_height; y++)
                    for (x = pX; x < pX + object_width; x++)
                    {
                        for (z = pZ; z < pZ + object_depth; z++)
                            EnvROBARMCfg.Grid3D[x][y][z] = 1;
                    }

                printf("Cubic Obstacle: (%i,%i,%i) (cell) H: %i W: %i D: %i\n",pX,pY,pZ,object_width,object_height, object_depth);
            }
            else
            {
                printf("ERROR: Environment Config file contains unknown object type.\n");
                exit(1);
            }
            fscanf(fCfg, "%s", sTemp);
        }
    }

    //convert shoulder base from cell to real world coords
    Cell2ContXY(EnvROBARMCfg.BaseX_c, EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c, &(EnvROBARMCfg.BaseX_m), &(EnvROBARMCfg.BaseY_m), &(EnvROBARMCfg.BaseZ_m)); 

    printf("Config file successfully read.\n");
}

void EnvironmentROBARM::PrintConfiguration()
{
    int i;
    double pX, pY, pZ;
    short unsigned int xu, yu, zu;
    short unsigned int wrist[3];
    short unsigned int elbow[3];

    printf("\n\nEnvironment/Robot Details:\n");
    printf("Grid Cell Width: %.2f cm\n",EnvROBARMCfg.GridCellWidth*100);

    Cell2ContXY(EnvROBARMCfg.BaseX_c,EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c,&pX, &pY, &pZ);
    printf("Shoulder Base: %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",EnvROBARMCfg.BaseX_c,EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c,pX,pY,pZ);

    ComputeEndEffectorPos(EnvROBARMCfg.LinkStartAngles_d, &xu, &yu, &zu, wrist, elbow);
    Cell2ContXY(elbow[0],elbow[1],elbow[2],&pX, &pY, &pZ);
    printf("Elbow Start:   %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",elbow[0],elbow[1],elbow[2],pX,pY,pZ);

    Cell2ContXY(wrist[0],wrist[1],wrist[2],&pX, &pY, &pZ);
    printf("Wrist Start:   %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",wrist[0],wrist[1],wrist[2],pX,pY,pZ);

    Cell2ContXY(xu,yu,zu,&pX, &pY, &pZ);
    printf("End Effector Start: %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",xu, yu, zu,pX,pY,pZ);

    Cell2ContXY(EnvROBARMCfg.EndEffGoalX_c,EnvROBARMCfg.EndEffGoalY_c, EnvROBARMCfg.EndEffGoalZ_c, &pX, &pY, &pZ);
    printf("End Effector Goal:  %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",EnvROBARMCfg.EndEffGoalX_c,EnvROBARMCfg.EndEffGoalY_c, EnvROBARMCfg.EndEffGoalZ_c, pX,pY,pZ);

#if OUTPUT_OBSTACLES
    int x, y, z;
    int sum = 0;
    printf("\nObstacles:\n");
    for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
	for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
        {
	    for (z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
	    {
		sum += EnvROBARMCfg.Grid3D[x][y][z];
		if (EnvROBARMCfg.Grid3D[x][y][z] == 1)
		    printf("(%i,%i,%i) ",x,y,z);
	    }
        }
    printf("\nThe occupancy grid contains %i obstacle cells.\n",sum); 
#endif

    printf("\nDH Parameters:\n");
    printf("LinkTwist: ");
    for(i=0; i < NUMOFLINKS_DH; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_alpha[i]);
    printf("\nLinkLength: ");
    for(i=0; i < NUMOFLINKS_DH; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_a[i]);
    printf("\nLinkOffset: ");
    for(i=0; i < NUMOFLINKS_DH; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_d[i]);
    printf("\nJointAngles: ");
    for(i=0; i < NUMOFLINKS_DH; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_theta[i]);

    printf("\n\nMotor Limits:\n");
    printf("PosMotorLimits: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%.2f  ",EnvROBARMCfg.PosMotorLimits[i]);
    printf("\nNegMotorLimits: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%.2f  ",EnvROBARMCfg.NegMotorLimits[i]);
    printf("\n\n");
}

void EnvironmentROBARM::DiscretizeAngles()
{
	int i;
//     double HalfGridCell = EnvROBARMCfg.GridCellWidth/2.0;
	for(i = 0; i < NUMOFLINKS; i++)
	{
            EnvROBARMCfg.angledelta[i] = (2.0*PI_CONST) / ANGLEDELTA; 
            EnvROBARMCfg.anglevals[i] = ANGLEDELTA;

//         if (EnvROBARMCfg.LinkLength_m[i] > 0)
//             EnvROBARMCfg.angledelta[i] =  2*asin(HalfGridCell/EnvROBARMCfg.LinkLength_m[i]);
//         else
//             EnvROBARMCfg.angledelta[i] = ANGLEDELTA;

//         EnvROBARMCfg.anglevals[i] = (int)(2.0*PI_CONST/EnvROBARMCfg.angledelta[i]+0.99999999);

#if DEBUG_A_LOT
        printf("angledelta[%i]: %f, angleval[%i]: %i\n",i, EnvROBARMCfg.angledelta[i], i, EnvROBARMCfg.anglevals[i]);
#endif
    }
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
void EnvironmentROBARM::ComputeContAngles(short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS])
{
	int i;

	for(i = 0; i < NUMOFLINKS; i++)
	{
            angle[i] = coord[i]*EnvROBARMCfg.angledelta[i];
	}
}

void EnvironmentROBARM::ComputeCoord(double angle[NUMOFLINKS], short unsigned int coord[NUMOFLINKS])
{
	int i;

	for(i = 0; i < NUMOFLINKS; i++)
	{
		coord[i] = (int)((angle[i] + EnvROBARMCfg.angledelta[i]*0.5)/EnvROBARMCfg.angledelta[i]);
		if(coord[i] == EnvROBARMCfg.anglevals[i])
			coord[i] = 0;
	}
}

// convert a cell in the occupancy grid to point in real world 
void EnvironmentROBARM::Cell2ContXY(int x, int y, int z, double *pX, double *pY, double *pZ)
{
    *pX = (x - X_OFFSET_C) * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth*0.5;
    *pY = (y - Y_OFFSET_C) * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth*0.5;
    *pZ = (z - Z_OFFSET_C) *EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth*0.5;
}

// convert a point in real world to a cell in occupancy grid 
void EnvironmentROBARM::ContXYZ2Cell(double x, double y, double z, short unsigned int *pX, short unsigned int *pY, short unsigned int *pZ)
{
    //take the nearest cell
    *pX = (int)((x/EnvROBARMCfg.GridCellWidth) + X_OFFSET_C);
    if( x < 0) *pX = 0;
    if( *pX >= EnvROBARMCfg.EnvWidth_c) *pX = EnvROBARMCfg.EnvWidth_c-1;

    *pY = (int)((y/EnvROBARMCfg.GridCellWidth) + Y_OFFSET_C);
    if( y + (Y_OFFSET_C*EnvROBARMCfg.GridCellWidth) < 0) *pY = 0;
    if( *pY >= EnvROBARMCfg.EnvHeight_c) *pY = EnvROBARMCfg.EnvHeight_c-1;

    *pZ = (int)((z/EnvROBARMCfg.GridCellWidth) + Z_OFFSET_C);
    if( z < 0) *pZ = 0;
    if( *pZ >= EnvROBARMCfg.EnvDepth_c) *pZ = EnvROBARMCfg.EnvDepth_c-1;
}

//returns 1 if end effector within space, 0 otherwise
int EnvironmentROBARM::ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int* pX, short unsigned int* pY, short unsigned int* pZ, short unsigned int wrist[3], short unsigned int elbow[3])
{
    num_forwardkinematics++;

    clock_t currenttime = clock();
    double x,y,z;
    int retval = 1;

    //convert angles from positive values in radians (from 0->6.28) to centered around 0
    for (int i = 0; i < NUMOFLINKS; i++)
    {
        if(angles[i] >= PI_CONST)
            angles[i] = -2.0*PI_CONST + angles[i];
    }

    if(EnvROBARMCfg.use_DH)
    {
//         boost::numeric::ublas::matrix<double> T[NUMOFLINKS_DH];
//         getTransformations_robarm7d(T,angles);

        GetDHTransformations(angles);

        //get position of elbow
        x = EnvROBARMCfg.T_array[0][3][3] + EnvROBARMCfg.BaseX_m;
        y = EnvROBARMCfg.T_array[1][3][3] + EnvROBARMCfg.BaseY_m;
        z = EnvROBARMCfg.T_array[2][3][3] + EnvROBARMCfg.BaseZ_m;

        ContXYZ2Cell(x, y, z, &(elbow[0]), &(elbow[1]), &(elbow[2]));
//         printf("x:%.3f y: %.3f z: %.3f\n",x,y,z);

        //get position of wrist
        x = EnvROBARMCfg.T_array[0][3][5] + EnvROBARMCfg.BaseX_m;
        y = EnvROBARMCfg.T_array[1][3][5] + EnvROBARMCfg.BaseY_m;
        z = EnvROBARMCfg.T_array[2][3][5] + EnvROBARMCfg.BaseZ_m;

        ContXYZ2Cell(x, y, z, &(wrist[0]), &(wrist[1]), &(wrist[2]));
//         printf("x:%.3f y: %.3f z: %.3f\n",x,y,z);

        //get position of tip of gripper
        x = EnvROBARMCfg.T_array[0][3][7] + EnvROBARMCfg.BaseX_m;
        y = EnvROBARMCfg.T_array[1][3][7] + EnvROBARMCfg.BaseY_m;
        z = EnvROBARMCfg.T_array[2][3][7] + EnvROBARMCfg.BaseZ_m;

        ContXYZ2Cell(x, y, z, pX, pY, pZ);
//         printf("x:%.3f y: %.3f z: %.3f\n",x,y,z);

        // if end effector is out of bounds then return 0
        if(*pX >= EnvROBARMCfg.EnvWidth_c || *pY >= EnvROBARMCfg.EnvHeight_c || *pZ >= EnvROBARMCfg.EnvDepth_c)
            retval =  0;

//         printf("elbow: %i,%i,%i   wrist: %i,%i,%i   gripper: %i,%i,%i\n", elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2],*pX, *pY, *pZ);

        DH_time += clock() - currenttime;
    }
    else
    {
        //get position of elbow
        ComputeForwardKinematics(angles, 4, &x, &y, &z);
        ContXYZ2Cell(x, y, z, &(elbow[0]), &(elbow[1]), &(elbow[2]));

        //get position of wrist
        ComputeForwardKinematics(angles, 6, &x, &y, &z);
        ContXYZ2Cell(x, y, z, &(wrist[0]), &(wrist[1]), &(wrist[2]));

        //get position of tip of gripper
        ComputeForwardKinematics(angles, 7, &x, &y, &z);
        ContXYZ2Cell(x, y, z, pX, pY, pZ);

        // check upper bounds
        if(*pX >= EnvROBARMCfg.EnvWidth_c || *pY >= EnvROBARMCfg.EnvHeight_c || *pZ >= EnvROBARMCfg.EnvDepth_c)
            retval =  0;

#if DEBUG_A_LOT
        for(int m = 0; m <= NUMOFLINKS; m++)
        {
            ComputeForwardKinematics(angles,m, &x, &y, &z);
            printf("Joint %i in frame 0: %.3f %.3f %.3f\n",m,x,y,z);
        }
#endif
        KL_time += clock() - currenttime;
    }

    return retval;
}

//if pTestedCells is NULL, then the tested points are not saved and it is more
//efficient as it returns as soon as it sees first invalid point
int EnvironmentROBARM::IsValidLineSegment(double x0, double y0, double z0, double x1, double y1, double z1, char ***Grid3D, vector<CELLV>* pTestedCells)
{
    bresenham_param_t params;
    int nX, nY, nZ; 
    short unsigned int nX0, nY0, nZ0, nX1, nY1, nZ1;
    int retvalue = 1;
    CELLV tempcell;

    //FIX THIS
    //make sure the line segment is inside the environment
//     if(x0 < 0 || x0 >= EnvROBARMCfg.EnvWidth_m ||
//         x1 < 0 || x1 >= EnvROBARMCfg.EnvWidth_m ||
//         y0 < 0 || y0 >= EnvROBARMCfg.EnvHeight_m ||
//         y1 < 0 || y1 >= EnvROBARMCfg.EnvHeight_m ||
//         z0 < 0 || z0 >= EnvROBARMCfg.EnvDepth_m ||
//         z1 < 0 || z1 >= EnvROBARMCfg.EnvDepth_m)
//             return 0;

    ContXYZ2Cell(x0, y0, z0, &nX0, &nY0, &nZ0);
    ContXYZ2Cell(x1, y1, z1, &nX1, &nY1, &nZ1);

    //iterate through the points on the segment
    get_bresenham_parameters(nX0, nY0, nZ0, nX1, nY1, nZ1, &params);
    do {
	    get_current_point(&params, &nX, &nY, &nZ);
	    if(Grid3D[nX][nY][nZ] == 1)
	    {
		    if(pTestedCells == NULL)
			return 0;
		    else
			retvalue = 0;
	    }
    
	    //insert the tested point
	    if(pTestedCells)
	    {
		tempcell.bIsObstacle = (Grid3D[nX][nY][nZ] == 1);
		tempcell.x = nX;
		tempcell.y = nY;
		tempcell.z = nZ;
		pTestedCells->push_back(tempcell);
	    }
    } while (get_next_point(&params));

    return retvalue;
}

int EnvironmentROBARM::IsValidLineSegment(short unsigned int x0, short unsigned int y0, short unsigned int z0, short unsigned int x1, short unsigned int y1, short unsigned int z1, char ***Grid3D, vector<CELLV>* pTestedCells)
{
    bresenham_param_t params;
    int nX, nY, nZ; 
    int retvalue = 1;
    CELLV tempcell;

    //make sure the line segment is inside the environment - no need to check < 0
    if(x0 >= EnvROBARMCfg.EnvWidth_c ||
       x1 >= EnvROBARMCfg.EnvWidth_c ||
       y0 >= EnvROBARMCfg.EnvHeight_c ||
       y1 >= EnvROBARMCfg.EnvHeight_c ||
       z0 >= EnvROBARMCfg.EnvDepth_c ||
       z1 >= EnvROBARMCfg.EnvDepth_c)
    {
//         printf("IsValidLineSegment: %i %i %i --> %i %i %i is out of bounds.\n", x0,y0,z0,x1,y1,z1);
        return 0;
    }

    //iterate through the points on the segment
    get_bresenham_parameters(x0, y0, z0, x1, y1, z1, &params);
    do {
        get_current_point(&params, &nX, &nY, &nZ);
        if(Grid3D[nX][nY][nZ] == 1)
        {
            if(pTestedCells == NULL)
                return 0;
            else
            {
//                 printf("IsValidLineSegment: %i %i %i is an obstacle\n",nX,nY,nZ);
                retvalue = 0;
            }
        }

        //insert the tested point
        if(pTestedCells)
        {
            tempcell.bIsObstacle = (Grid3D[nX][nY][nZ] == 1);
            tempcell.x = nX;
            tempcell.y = nY;
            tempcell.z = nZ;
            pTestedCells->push_back(tempcell);
        }
    } while (get_next_point(&params));

    return retvalue;
}

int EnvironmentROBARM::IsValidCoord(short unsigned int coord[NUMOFLINKS], char*** Grid3D, vector<CELLV>* pTestedCells)
{
    double angles[NUMOFLINKS];
    int retvalue = 1;
    short unsigned int endeffx, endeffy,endeffz;
    short unsigned int wrist[3];
    short unsigned int elbow[3];

    if(Grid3D == NULL)
        Grid3D = EnvROBARMCfg.Grid3D;

    ComputeContAngles(coord, angles);

    // check motor limits
    if(EnvROBARMCfg.enforce_motor_limits)
    {
        //shoulder pan - Left is Positive Direction
        if (angles[0] > EnvROBARMCfg.PosMotorLimits[0] && angles[0] < 6.283-EnvROBARMCfg.NegMotorLimits[0])
            return 0;
        //shoulder pitch - Down is Positive Direction
        if (angles[1] > EnvROBARMCfg.PosMotorLimits[1] && angles[1] < 6.283-EnvROBARMCfg.NegMotorLimits[1])
            return 0;
        //upperarm roll
        if (angles[2] > EnvROBARMCfg.PosMotorLimits[2] && angles[2] < 6.283-EnvROBARMCfg.NegMotorLimits[2])
            return 0;
        //elbow flex - Down is Positive Direction
        if (angles[3] > EnvROBARMCfg.PosMotorLimits[3] && angles[3] < 6.283-EnvROBARMCfg.NegMotorLimits[3])
            return 0;
        //forearm roll
        if (angles[4] > EnvROBARMCfg.PosMotorLimits[4] && angles[4] < 6.283-EnvROBARMCfg.NegMotorLimits[4])
            return 0;
        //wrist flex - Down is Positive Direction
        if (angles[5] > EnvROBARMCfg.PosMotorLimits[5] && angles[5] < 6.283-EnvROBARMCfg.NegMotorLimits[5])
            return 0;
        //wrist roll
        if (angles[6] > EnvROBARMCfg.PosMotorLimits[6] && angles[6] < 6.283-EnvROBARMCfg.NegMotorLimits[6])
            return 0;
    }

    clock_t currenttime = clock();

    // check if only end effector is valid
    if (EnvROBARMCfg.endeff_check_only)
    {
        //just check whether end effector is in valid position
        if(ComputeEndEffectorPos(angles, &endeffx, &endeffy, &endeffz, wrist, elbow) == false)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }

        //bounds checking on upper bound (short unsigned int cannot be less than 0, so just check maxes)
        if(endeffx >= EnvROBARMCfg.EnvWidth_c || endeffy >= EnvROBARMCfg.EnvHeight_c || endeffz >= EnvROBARMCfg.EnvDepth_c)   
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }

        if(pTestedCells)
        {
            CELLV tempcell;
            tempcell.bIsObstacle = Grid3D[endeffx][endeffy][endeffz];
            tempcell.x = endeffx;
            tempcell.y = endeffy;
            tempcell.z = endeffz;

            pTestedCells->push_back(tempcell);
        }

        //check end effector is not hitting obstacle
        if(Grid3D[endeffx][endeffy][endeffz] == 1)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }
    }
    else // check if elbow and wrist are valid as well
    {
        //check whether end effector is in valid position
        if(ComputeEndEffectorPos(angles, &endeffx, &endeffy, &endeffz, wrist, elbow) == false)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }
        //bounds checking on upper bound (short unsigned int cannot be less than 0, so just check maxes)
        if(endeffx >= EnvROBARMCfg.EnvWidth_c || endeffy >= EnvROBARMCfg.EnvHeight_c || endeffz >= EnvROBARMCfg.EnvDepth_c)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }

        //check the validity of the corresponding line segments
        if(!IsValidLineSegment(EnvROBARMCfg.BaseX_c,EnvROBARMCfg.BaseY_c,EnvROBARMCfg.BaseZ_c, elbow[0],elbow[1],elbow[2],Grid3D, pTestedCells) ||
            !IsValidLineSegment(elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2],Grid3D, pTestedCells) ||
            !IsValidLineSegment(wrist[0],wrist[1],wrist[2],endeffx,endeffy, endeffz,Grid3D, pTestedCells))
        {
            if(pTestedCells == NULL)
            {
                check_collision_time += clock() - currenttime;
                return 0;
            }
            else
                retvalue = 0;
        }
//     printf("elbow: (%u,%u,%u)  wrist:(%u,%u,%u) endeff:(%u,%u,%u)\n", elbow[0],elbow[1],elbow[2],wrist[0], wrist[1], wrist[2], endeffx, endeffy,endeffz);
    }
    check_collision_time += clock() - currenttime;

    //check self collision
//     XYZ p1,p2,p3,p4,pa,pb;
//     double mua,mub;
//     p1.x = (double)endeffx;
//     p1.y = (double)endeffy;
//     p1.z = (double)endeffz;
//     p2.x = (double)wrist[0];
//     p2.y = (double)wrist[1];
//     p2.z = (double)wrist[2];
//     p3.x = (double)elbow[0];
//     p3.y = (double)elbow[1];
//     p3.z = (double)elbow[2];
//     p4.x = (double)EnvROBARMCfg.BaseX_c;
//     p4.y = (double)EnvROBARMCfg.BaseY_c;
//     p4.z = (double)EnvROBARMCfg.BaseZ_c;
// 
//     if(!LineLineIntersect(p1,p2, p3,p4,&pa,&pb, &mua, &mub))
//         printf("Gripper does not intersect with upperarm\n");
//     else
//     {
//         printf("pa: %.3f %.3f %.3f  pb: %.3f %.3f %.3f\n", pa.x,pa.y,pa.z,pb.x,pb.y,pb.z);
//         double dist = sqrt((pa.x-pb.x)*(pa.x-pb.x) + (pa.y-pb.y)*(pa.y-pb.y) + (pa.z-pb.z)*(pa.z-pb.z));
//         cout << dist << endl;
//         exit(1);
//     }

//     POINT A, B, pA, pB;
//     VECTOR dA, dB;
//     A.px = endeffx;
//     A.py = endeffy;
//     A.pz = endeffz;
// 
//     if (wrist[0] > endeffx)
//         dA.dx = wrist[0] - endeffx;
//     else
//         dA.dx = endeffx - wrist[0];
// 
//     if (wrist[1] > endeffy)
//         dA.dy = wrist[1] - endeffy;
//     else
//         dA.dy = endeffy - wrist[1];
// 
//     if (wrist[2] > endeffz)
//         dA.dz = endeffz - wrist[2];
//     else
//         dA.dz = wrist[2] - endeffz;
// 
// 
//     B.px = (double)elbow[0];
//     B.py = (double)elbow[1];
//     B.pz = (double)elbow[2];
// 
//     if ((double)elbow[0] > (double)EnvROBARMCfg.BaseX_c)
//         dB.dx = (double)EnvROBARMCfg.BaseX_c - (double)elbow[0];
//     else
//         dB.dx = (double)elbow[0] - (double)EnvROBARMCfg.BaseX_c;
// 
//     if ((double)elbow[1] > (double)EnvROBARMCfg.BaseY_c)
//         dB.dy = (double)elbow[1] - (double)EnvROBARMCfg.BaseY_c;
//     else
//         dB.dy = (double)EnvROBARMCfg.BaseY_c - (double)elbow[1];
// 
//     if ((double)elbow[2] > (double)EnvROBARMCfg.BaseZ_c)
//         dB.dz = (double)elbow[2] - (double)EnvROBARMCfg.BaseZ_c;
//     else
//         dB.dz = (double)EnvROBARMCfg.BaseZ_c - (double)elbow[2];
// 
//     printf("A: %.3f %.3f %.3f B: %.3f %.3f %.3f\n",A.px,A.py,A.pz,B.px,B.py,B.pz);
//     printf("dA: %.3f %.3f %.3f dB: %.3f %.3f %.3f\n",dA.dx,dA.dy,dA.dz,dB.dx,dB.dy,dB.dz);
//     if(line_line_closest_points3d ( &pA, &pB, &A, &dA, &B, &dB ) == 2);
//     {
//         double dist = sqrt((pA.px-pB.px)*(pA.px-pB.px) + (pA.py-pB.py)*(pA.py-pB.py) + (pA.pz-pB.pz)*(pA.pz-pB.pz));
//         printf("pA: %.3f %.3f %.3f    pB: %.3f %.3f %.3f    dist: %.3f\n", pA.px,pA.py,pA.pz,pB.px,pB.py,pB.pz,dist);
//         exit(1);
//     }

    return retvalue;
}

int EnvironmentROBARM::IsValidCoord(short unsigned int coord[NUMOFLINKS], EnvROBARMHashEntry_t* arm)
{
    double angles[NUMOFLINKS];
    int retvalue = 1;

    char*** Grid3D= EnvROBARMCfg.Grid3D;
    vector<CELLV>* pTestedCells = NULL;

    ComputeContAngles(coord, angles);

    // check motor limits
    if(EnvROBARMCfg.enforce_motor_limits)
    {
        //shoulder pan - Left is Positive Direction
        if (angles[0] > EnvROBARMCfg.PosMotorLimits[0] && angles[0] < 6.283-EnvROBARMCfg.NegMotorLimits[0])
            return 0;
        //shoulder pitch - Down is Positive Direction
        if (angles[1] > EnvROBARMCfg.PosMotorLimits[1] && angles[1] < 6.283-EnvROBARMCfg.NegMotorLimits[1])
            return 0;
        //upperarm roll
        if (angles[2] > EnvROBARMCfg.PosMotorLimits[2] && angles[2] < 6.283-EnvROBARMCfg.NegMotorLimits[2])
            return 0;
        //elbow flex - Down is Positive Direction
        if (angles[3] > EnvROBARMCfg.PosMotorLimits[3] && angles[3] < 6.283-EnvROBARMCfg.NegMotorLimits[3])
            return 0;
        //forearm roll
        if (angles[4] > EnvROBARMCfg.PosMotorLimits[4] && angles[4] < 6.283-EnvROBARMCfg.NegMotorLimits[4])
            return 0;
        //wrist flex - Down is Positive Direction
        if (angles[5] > EnvROBARMCfg.PosMotorLimits[5] && angles[5] < 6.283-EnvROBARMCfg.NegMotorLimits[5])
            return 0;
        //wrist roll
        if (angles[6] > EnvROBARMCfg.PosMotorLimits[6] && angles[6] < 6.283-EnvROBARMCfg.NegMotorLimits[6])
            return 0;
    }

    clock_t currenttime = clock();

    // check if only end effector is valid
    if (EnvROBARMCfg.endeff_check_only)
    {
        //just check whether end effector is in valid position
//         if(ComputeEndEffectorPos(angles, &endeffx, &endeffy, &endeffz, wrist, elbow) == false)
//         {
//             check_collision_time += clock() - currenttime;
//             return 0;
//         }

        //bounds checking on upper bound (short unsigned int cannot be less than 0, so just check maxes)
        if(arm->endeffx >= EnvROBARMCfg.EnvWidth_c || arm->endeffy >= EnvROBARMCfg.EnvHeight_c || arm->endeffz >= EnvROBARMCfg.EnvDepth_c)   
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }

        if(pTestedCells)
        {
            CELLV tempcell;
            tempcell.bIsObstacle = Grid3D[arm->endeffx][arm->endeffy][arm->endeffz];
            tempcell.x = arm->endeffx;
            tempcell.y = arm->endeffy;
            tempcell.z = arm->endeffz;

            pTestedCells->push_back(tempcell);
        }

        //check end effector is not hitting obstacle
        if(Grid3D[arm->endeffx][arm->endeffy][arm->endeffz] == 1)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }
    }
    else // check if elbow and wrist are valid as well
    {
        //check whether end effector is in valid position
//         if(ComputeEndEffectorPos(angles, &endeffx, &endeffy, &endeffz, wrist, elbow) == false)
//         {
//             check_collision_time += clock() - currenttime;
//             return 0;
//         }
        //bounds checking on upper bound (short unsigned int cannot be less than 0, so just check maxes)
        if(arm->endeffx >= EnvROBARMCfg.EnvWidth_c || arm->endeffy >= EnvROBARMCfg.EnvHeight_c || arm->endeffz >= EnvROBARMCfg.EnvDepth_c)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }

        //check the validity of the corresponding line segments
        if(!IsValidLineSegment(EnvROBARMCfg.BaseX_c,EnvROBARMCfg.BaseY_c,EnvROBARMCfg.BaseZ_c, arm->elbow[0],arm->elbow[1],arm->elbow[2], Grid3D, pTestedCells) ||
            !IsValidLineSegment(arm->elbow[0],arm->elbow[1],arm->elbow[2],arm->wrist[0],arm->wrist[1],arm->wrist[2], Grid3D, pTestedCells) ||
            !IsValidLineSegment(arm->wrist[0],arm->wrist[1],arm->wrist[2],arm->endeffx,arm->endeffy, arm->endeffz, Grid3D, pTestedCells))
        {
            if(pTestedCells == NULL)
            {
                check_collision_time += clock() - currenttime;
                return 0;
            }
            else
                retvalue = 0;
        }
//     printf("elbow: (%u,%u,%u)  wrist:(%u,%u,%u) endeff:(%u,%u,%u)\n", elbow[0],elbow[1],elbow[2],wrist[0], wrist[1], wrist[2], endeffx, endeffy,endeffz);
    }
    check_collision_time += clock() - currenttime;

    return retvalue;
}

int EnvironmentROBARM::cost(short unsigned int state1coord[], short unsigned int state2coord[])
{
    if(!IsValidCoord(state1coord) || !IsValidCoord(state2coord))
        return INFINITECOST;

#if UNIFORM_COST
    return 1*COSTOFACTION;
#else

    int i;
    //the cost becomes higher as we are closer to the base
    for(i = 0; i < NUMOFLINKS; i++)
    {
        if(state1coord[i] != state2coord[i])
            return (NUMOFLINKS-i)*COSTOFACTION;  //return (NUMOFLINKS-i)*(NUMOFLINKS-i);
    }

    printf("ERROR: cost on the same states is called:\n");
    //printangles(stdout, state1coord);
    //printangles(stdout, state2coord);

    exit(1);
#endif
}

//added so it would not have to recompute forward kinematics
//add a second bool variable for for first state for backwards case
int EnvironmentROBARM::cost(short unsigned int state1coord[], short unsigned int state2coord[],bool bState2IsGoal)
{
    EnvROBARMHashEntry_t* HashEntry1 = GetHashEntry(state1coord, NUMOFLINKS, false);
    EnvROBARMHashEntry_t* HashEntry2 = GetHashEntry(state2coord, NUMOFLINKS, bState2IsGoal);

    //why does the goal return as invalid from IsValidCoord?
    if (!bState2IsGoal)
    {
        if(!IsValidCoord(state1coord,HashEntry1) || !IsValidCoord(state2coord,HashEntry2))
            return INFINITECOST;
    }
    else
    {
        if(!IsValidCoord(state1coord,HashEntry1))
            return INFINITECOST;
    }

#if UNIFORM_COST
    return 1*COSTOFACTION;
#else

    int i;
    //the cost becomes higher as we are closer to the base
    for(i = 0; i < NUMOFLINKS; i++)
    {
        if(state1coord[i] != state2coord[i])
            return (NUMOFLINKS-i)*COSTOFACTION;  //return (NUMOFLINKS-i)*(NUMOFLINKS-i);
    }

    printf("ERROR: cost on the same states is called:\n");
    //printangles(stdout, state1coord);
    //printangles(stdout, state2coord);

    exit(1);
#endif
}

void EnvironmentROBARM::InitializeEnvConfig()
{
	//find the discretization for each angle and store the discretization
	DiscretizeAngles();
}

bool EnvironmentROBARM::InitializeEnvironment()
{
    short unsigned int coord[NUMOFLINKS];
    double startangles[NUMOFLINKS];
    double angles[NUMOFLINKS];
    short unsigned int wrist[3];
    short unsigned int elbow[3];
    short unsigned int endeff[3];
    int i;

    //initialize the map from Coord to StateID
    EnvROBARM.HashTableSize = 32*1024; //should be power of two
    EnvROBARM.Coord2StateIDHashTable = new vector<EnvROBARMHashEntry_t*>[EnvROBARM.HashTableSize];

    //initialize the map from StateID to Coord
    EnvROBARM.StateID2CoordTable.clear();

    //initialize the angles of the start states
    for(i = 0; i < NUMOFLINKS; i++)
    {
        startangles[i] = PI_CONST*(EnvROBARMCfg.LinkStartAngles_d[i]/180.0);
    }

    ComputeCoord(startangles, coord);
    ComputeContAngles(coord, angles);
    ComputeEndEffectorPos(angles, &endeff[0], &endeff[1], &endeff[2], wrist, elbow);

    cout << "Starting End Effector Position: " << endeff[0] << " " << endeff[1] << " " << endeff[2] << endl;

    //create the start state
    EnvROBARM.startHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeff, wrist, elbow,0);

    //create the goal state 
    //initialize the coord of goal state
    for(i = 0; i < NUMOFLINKS; i++)
    {
	coord[i] = 0;
    }

    endeff[0] = EnvROBARMCfg.EndEffGoalX_c;
    endeff[1] = EnvROBARMCfg.EndEffGoalY_c;
    endeff[2] = EnvROBARMCfg.EndEffGoalZ_c;
    EnvROBARM.goalHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeff, wrist, elbow, 0);

    if(!IsValidCoord(EnvROBARM.startHashEntry->coord) )
        printf("Start Hash Entry is invalid\n");

    //check the validity of both goal and start configurations
    //testing for EnvROBARMCfg.EndEffGoalX_c < 0  and EnvROBARMCfg.EndEffGoalY_c < 0 is useless since they are unsigned 
    if(!IsValidCoord(EnvROBARM.startHashEntry->coord) || EnvROBARMCfg.EndEffGoalX_c >= EnvROBARMCfg.EnvWidth_c ||
	EnvROBARMCfg.EndEffGoalY_c >= EnvROBARMCfg.EnvHeight_c || EnvROBARMCfg.EndEffGoalZ_c >= EnvROBARMCfg.EnvDepth_c)
    {
        printf("Either start or goal configuration is invalid\n");
        return false;
    }

    if(EnvROBARMCfg.Grid3D[EnvROBARMCfg.EndEffGoalX_c][EnvROBARMCfg.EndEffGoalY_c][EnvROBARMCfg.EndEffGoalZ_c] == 1)
    {
	printf("End Effector Goal is invalid\n");
        exit(1);
    }

    //for now heuristics are not set
    EnvROBARM.Heur = NULL;

    return true;
}

//----------------------------------------------------------------------


//-----------interface with outside functions---------------------------
bool EnvironmentROBARM::InitializeEnv(const char* sEnvFile)
{
    // default values - temporary solution
    EnvROBARMCfg.use_DH = 1;
    EnvROBARMCfg.enforce_motor_limits = 1;
    EnvROBARMCfg.dijkstra_heuristic = 1;
    EnvROBARMCfg.endeff_check_only = 0;

    //parse the configuration file
    FILE* fCfg = fopen(sEnvFile, "r");
    if(fCfg == NULL)
    {
	printf("ERROR: unable to open %s\n", sEnvFile);
	exit(1);
    }
    ReadConfiguration(fCfg);

    //temporary
    char parFile[] = "params.cfg";
    fCfg = fopen(parFile, "r");
    if(fCfg == NULL)
	printf("ERROR: unable to open %s.....using defaults.\n", parFile);

    ReadParams(fCfg);
    fclose(fCfg);

    //start ros node
    if (!EnvROBARMCfg.use_DH)
        InitializeKinNode();
    else
        //pre-compute DH Transformations
        ComputeDHTransformations();

    //Initialize other parameters of the environment
    InitializeEnvConfig();

    //initialize Environment
    if(InitializeEnvironment() == false)
        return false;

    //pre-compute heuristics
    ComputeHeuristicValues();

    char velFile[] = "vel.cfg";
    fCfg  = fopen(velFile, "r");
    if(fCfg == NULL)
	printf("ERROR: unable to open %s.....using defaults.\n", velFile);

    //parse successor actions file
    ReadSuccActionsFile(fCfg);

    //pre-compute action to action costs
    ComputeActionCosts();

#if VERBOSE

    printf("USE_DH: %i\n", EnvROBARMCfg.use_DH);
    printf("ENFORCE_MOTOR_LIMITS: %i\n", EnvROBARMCfg.enforce_motor_limits);
    printf("DIJKSTRA_HEURISTIC: %i\n", EnvROBARMCfg.dijkstra_heuristic);
    printf("ENDEFF_CHECK_ONLY: %i\n", EnvROBARMCfg.endeff_check_only);
    printf("\n");

    //output environment data
    PrintConfiguration();

//     printf("2: elbow: %i,%i,%i   wrist: %i,%i,%i   gripper: %i,%i,%i\n", elbow[0],elbow[1],elbow[2], wrist[0],wrist[1],wrist[2],pX, pY, pZ);
    printf("Start to Goal Heuristic: %i \n", GetFromToHeuristic(EnvROBARM.startHashEntry->stateID, EnvROBARM.goalHashEntry->stateID));

    //output successor actions
    OutputActions();

    //output action costs
    OutputActionCostTable();
#endif

    return true;
}

bool EnvironmentROBARM::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
	MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
	MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;

	return true;
}

int EnvironmentROBARM::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    int h;

#if USE_HEUR==0
    return 0;
#endif

#if DEBUG
    if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() || ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
    {
        printf("ERROR in EnvROBARM... function: stateID illegal\n");
	exit(1);
    }
#endif

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
    EnvROBARMHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

    //Dijkstra's algorithm
    if(EnvROBARMCfg.dijkstra_heuristic)
        h = EnvROBARM.Heur[XYZTO3DIND(FromHashEntry->endeffx, FromHashEntry->endeffy, FromHashEntry->endeffz)];
    //euclidean distance
    else 
        h = (COSTMULT/CELLSPERACTION)*sqrt((FromHashEntry->endeffx-ToHashEntry->endeffx)*(FromHashEntry->endeffx-ToHashEntry->endeffx) + 
                                    (FromHashEntry->endeffy-ToHashEntry->endeffy)*(FromHashEntry->endeffy-ToHashEntry->endeffy) + 
                                    (FromHashEntry->endeffz-ToHashEntry->endeffz)*(FromHashEntry->endeffz-ToHashEntry->endeffz));

#if DEBUG_A_LOT
    printf("GetFromToHeuristic(#%i(%i,%i,%i) -> #%i(%i,%i,%i)) H cost: %i\n",FromStateID, FromHashEntry->endeffx, FromHashEntry->endeffy, FromHashEntry->endeffz, ToStateID, ToHashEntry->endeffx, ToHashEntry->endeffy, ToHashEntry->endeffz, h);
#endif

    return h;
}

int EnvironmentROBARM::GetGoalHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
	{
		printf("ERROR in EnvROBARM... function: stateID illegal\n");
		exit(1);
	}
#endif


	//define this function if it used in the planner (heuristic forward search would use it)

	printf("ERROR in EnvROBARM..function: GetGoalHeuristic is undefined\n");
	exit(1);
}

int EnvironmentROBARM::GetStartHeuristic(int stateID)
{
#if USE_HEUR==0
	return 0;
#endif

#if DEBUG
	if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
	{
		printf("ERROR in EnvROBARM... function: stateID illegal\n");
		exit(1);
	}
#endif

	//define this function if it used in the planner (heuristic backward search would use it)
	printf("ERROR in EnvROBARM... function: GetStartHeuristic is undefined\n");
	exit(1);

	return 0;
}

int EnvironmentROBARM::SizeofCreatedEnv()
{
	return EnvROBARM.StateID2CoordTable.size();
	
}

void EnvironmentROBARM::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
{
    bool bLocal = false;

#if DEBUG
    if(stateID >= (int)EnvROBARM.StateID2CoordTable.size())
    {
	printf("ERROR in EnvROBARM... function: stateID illegal (2)\n");
	exit(1);
    }
#endif

    if(fOut == NULL)
        fOut = stdout;

    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

    bool bGoal = false;
    if(stateID == EnvROBARM.goalHashEntry->stateID)
        bGoal = true;

    if(stateID == EnvROBARM.goalHashEntry->stateID && bVerbose)
    {
	fprintf(fOut, "the state is a goal state\n");
	bGoal = true;
    }

    if(bLocal)
    {
	printangles(fOut, HashEntry->coord, bGoal, bVerbose, true);
    }
    else
    {
	if(!bGoal)
	    printangles(fOut, HashEntry->coord, bGoal, bVerbose, false);
	else
	    printangles(fOut, EnvROBARMCfg.goalcoords, bGoal, bVerbose, false);
    }
}

//get the goal as a successor of source state at given cost
//if costtogoal = -1 then the succ is chosen
void EnvironmentROBARM::PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/, FILE* fOut /*=NULL*/)
{
    short unsigned int succcoord[NUMOFLINKS];
    double angles[NUMOFLINKS];
    short unsigned int endeffx, endeffy, endeffz;
    short unsigned int wrist[3];
    short unsigned int elbow[3];
    int i, inc;

    if(fOut == NULL)
        fOut = stdout;

    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

    //default coords of successor
    for(i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];	

    //iterate through successors of s
    for (i = 0; i < NUMOFLINKS; i++)
    {
        //increase and decrease in ith angle
        for(inc = -1; inc < 2; inc = inc+2)
        {
	    if(inc == -1)
	    {
		if(HashEntry->coord[i] == 0)
                    succcoord[i] = EnvROBARMCfg.anglevals[i]-1;
		else
		    succcoord[i] = HashEntry->coord[i] + inc;
	    }
	    else
	    {
		succcoord[i] = (HashEntry->coord[i] + inc)%
		EnvROBARMCfg.anglevals[i];
	    }

	    //skip invalid successors
	    if(!IsValidCoord(succcoord))
                continue;

	    ComputeContAngles(succcoord, angles);
	    ComputeEndEffectorPos(angles, &endeffx, &endeffy, &endeffz, wrist, elbow);
	    if(endeffx == EnvROBARMCfg.EndEffGoalX_c && endeffy == EnvROBARMCfg.EndEffGoalY_c && endeffz ==  EnvROBARMCfg.EndEffGoalZ_c)
	    {
		if(cost(HashEntry->coord,succcoord) == costtogoal || costtogoal == -1)
		{

		    if(bVerbose)
			fprintf(fOut, "the state is a goal state\n");
		    printangles(fOut, succcoord, true, bVerbose, bLocal);
		    return;
		}
	   }
        }

        //restore it back
        succcoord[i] = HashEntry->coord[i];
    }
}

void EnvironmentROBARM::PrintEnv_Config(FILE* fOut)
{
	//implement this if the planner needs to print out EnvROBARM. configuration
	
	printf("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
	exit(1);
}

void EnvironmentROBARM::PrintHeader(FILE* fOut)
{
    fprintf(fOut, "%d\n", NUMOFLINKS);
    for(int i = 0; i < NUMOFLINKS; i++)
        fprintf(fOut, "%.3f ", EnvROBARMCfg.LinkLength_m[i]);
    fprintf(fOut, "\n");
}

/*
void EnvironmentROBARM::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int i, inc;
    short unsigned int succcoord[NUMOFLINKS];

    double angles[NUMOFLINKS];
    EnvROBARMHashEntry_t SuccTemp;

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

    //goal state should be absorbing
    if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
        return;

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

    //default coords of successor
    for(i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    ComputeContAngles(succcoord, angles);

    //iterate through successors of s
    for (i = 0; i < NUMOFLINKS; i++)
    {
        //increase and decrease in ith angle
        for(inc = -1; inc < 2; inc = inc+2)
        {
            if(inc == -1)
            {
                if(HashEntry->coord[i] == 0)
                        succcoord[i] = EnvROBARMCfg.anglevals[i]-1;
                else
                        succcoord[i] = HashEntry->coord[i] + inc;
            }
            else
            {
                succcoord[i] = (HashEntry->coord[i] + inc)%
                        EnvROBARMCfg.anglevals[i];
            }

            //get the successor
            EnvROBARMHashEntry_t* OutHashEntry;
            bool bSuccisGoal = false;

            //have to create a new entry
            ComputeContAngles(succcoord, angles);
            if(ComputeEndEffectorPos(angles, &SuccTemp.endeffx, &SuccTemp.endeffy, &SuccTemp.endeffz, SuccTemp.wrist, SuccTemp.elbow) == false)
                continue;

            //skip invalid successors
            if(!IsValidCoord(succcoord,&SuccTemp))
                continue;

            //if s is within 3 cells of the goal
            if(abs(HashEntry->endeffx - EnvROBARMCfg.EndEffGoalX_c) < 3 || abs(HashEntry->endeffy - EnvROBARMCfg.EndEffGoalY_c) < 3 || abs(HashEntry->endeffz - EnvROBARMCfg.EndEffGoalZ_c) < 3) 
            {
                if(SuccTemp.endeffx == EnvROBARMCfg.EndEffGoalX_c && SuccTemp.endeffy == EnvROBARMCfg.EndEffGoalY_c && SuccTemp.endeffz == EnvROBARMCfg.EndEffGoalZ_c)
                {
                    bSuccisGoal = true;
                    printf("goal succ is generated\n");

                    for (int j = 0; j < NUMOFLINKS; j++)
                        EnvROBARMCfg.goalcoords[j] = succcoord[j];
		}
            }
            if((OutHashEntry = GetHashEntry(succcoord, NUMOFLINKS, bSuccisGoal)) == NULL)
            {
                //have to create a new entry
                OutHashEntry = CreateNewHashEntry(succcoord, NUMOFLINKS, SuccTemp.endeffx, SuccTemp.endeffy, SuccTemp.endeffz, SuccTemp.wrist, SuccTemp.elbow);
            }
            SuccIDV->push_back(OutHashEntry->stateID);
            CostV->push_back(cost(HashEntry->coord,succcoord,bSuccisGoal));
        }
        //restore it back
        succcoord[i] = HashEntry->coord[i];
    }
}
*/

//get successors based on parsed successor actions
void EnvironmentROBARM::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    int i, inc, a;
    short unsigned int succcoord[NUMOFLINKS];

    double angles[NUMOFLINKS], s_angles[NUMOFLINKS];
    EnvROBARMHashEntry_t SuccTemp;

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

    //goal state should be absorbing
    if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
        return;

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];

    ComputeContAngles(HashEntry->coord, s_angles);

    //default coords of successor
    for(i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    ComputeContAngles(succcoord, angles);

    //iterate through successors of s (possible actions)
    for (i = 0; i < EnvROBARMCfg.nVelSucc; i++)
    {
        //increase and decrease in ith angle
        for(inc = -1; inc < 2; inc = inc+2)
        {
            if(inc == -1)
            {
                for(a = 0; a < NUMOFLINKS; a++)
                {
                    if((HashEntry->coord[a] == 0 && EnvROBARMCfg.succ_vel[i][a] != 0) || (HashEntry->coord[a] - EnvROBARMCfg.succ_vel[i][a] < 0))
                        succcoord[a] = EnvROBARMCfg.anglevals[a] - EnvROBARMCfg.succ_vel[i][a];
                    else
                        succcoord[a] = HashEntry->coord[a] - EnvROBARMCfg.succ_vel[i][a];
                }
            }
            else
            {
                for(a = 0; a < NUMOFLINKS; a++)
                    succcoord[a] = (HashEntry->coord[a] + int(EnvROBARMCfg.succ_vel[i][a])) % EnvROBARMCfg.anglevals[a];
            }

            //get the successor
            EnvROBARMHashEntry_t* OutHashEntry;
            bool bSuccisGoal = false;

            //have to create a new entry
            ComputeContAngles(succcoord, angles);
            if(ComputeEndEffectorPos(angles, &SuccTemp.endeffx, &SuccTemp.endeffy, &SuccTemp.endeffz, SuccTemp.wrist, SuccTemp.elbow) == false)
                continue;

            //skip invalid successors
            if(!IsValidCoord(succcoord,&SuccTemp))
                continue;

            //if s is within 3 cells of the goal - 3 should be changed to number of cells moved by largest action
            if(abs(HashEntry->endeffx - EnvROBARMCfg.EndEffGoalX_c) < 3 || abs(HashEntry->endeffy - EnvROBARMCfg.EndEffGoalY_c) < 3 || abs(HashEntry->endeffz - EnvROBARMCfg.EndEffGoalZ_c) < 3) 
            {
                if(SuccTemp.endeffx == EnvROBARMCfg.EndEffGoalX_c && SuccTemp.endeffy == EnvROBARMCfg.EndEffGoalY_c && SuccTemp.endeffz == EnvROBARMCfg.EndEffGoalZ_c)
                {
                    bSuccisGoal = true;
                    printf("goal succ is generated\n");

                    for (int j = 0; j < NUMOFLINKS; j++)
                        EnvROBARMCfg.goalcoords[j] = succcoord[j];
		}
            }

            if((OutHashEntry = GetHashEntry(succcoord, NUMOFLINKS, bSuccisGoal)) == NULL)
            {
                //have to create a new entry
                short unsigned int endeff[3];
                endeff[0] = SuccTemp.endeffx;
                endeff[1] = SuccTemp.endeffy;
                endeff[2] = SuccTemp.endeffz;
                OutHashEntry = CreateNewHashEntry(succcoord, NUMOFLINKS, endeff, SuccTemp.wrist, SuccTemp.elbow, i);
            }

            SuccIDV->push_back(OutHashEntry->stateID);
            CostV->push_back(cost(HashEntry->coord,OutHashEntry->coord,bSuccisGoal) + EnvROBARMCfg.ActionCosts[HashEntry->action][OutHashEntry->action]);
        }
    }
}

void EnvironmentROBARM::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

    printf("ERROR in EnvROBARM... function: GetPreds is undefined\n");
    exit(1);
}

void EnvironmentROBARM::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{


    printf("ERROR in EnvROBARM..function: SetAllActionsandOutcomes is undefined\n");
    exit(1);
}

void EnvironmentROBARM::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
    printf("ERROR in EnvROBARM... function: SetAllPreds is undefined\n");
    exit(1);
}

int EnvironmentROBARM::GetEdgeCost(int FromStateID, int ToStateID)
{

#if DEBUG
	if(FromStateID >= (int)EnvROBARM.StateID2CoordTable.size() 
		|| ToStateID >= (int)EnvROBARM.StateID2CoordTable.size())
	{
		printf("ERROR in EnvROBARM... function: stateID illegal\n");
		exit(1);
	}
#endif

	//get X, Y for the state
	EnvROBARMHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
	EnvROBARMHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];
	

	return cost(FromHashEntry->coord, ToHashEntry->coord);

}

bool EnvironmentROBARM::AreEquivalent(int State1ID, int State2ID)
{

	EnvROBARMHashEntry_t* HashEntry1 = EnvROBARM.StateID2CoordTable[State1ID];
	EnvROBARMHashEntry_t* HashEntry2 = EnvROBARM.StateID2CoordTable[State2ID];

    return (HashEntry1->endeffx == HashEntry2->endeffx && HashEntry1->endeffy == HashEntry2->endeffy && HashEntry1->endeffz == HashEntry2->endeffz);

}


//------------------------------------------------------------------------------
void EnvironmentROBARM::outputangles(double angles[NUMOFLINKS], bool degrees)
{
    if (!degrees)
        printf("angles: %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f\n", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7]);
    else
    {
        for (int i = 0; i < NUMOFLINKS; i++)
            angles[i] = angles[i]*(180/PI_CONST) + 0.999999;

        printf("angles: %i, %i, %i, %i, %i, %i, %i, %i\n", (int)angles[0], (int)angles[1], (int)angles[2], (int)angles[3], (int)angles[4], (int)angles[5], (int)angles[6], (int)angles[7]);
    }
}

void EnvironmentROBARM::outputjointpositions(EnvROBARMHashEntry_t* H)
{
    printf("elbow: (%u,%u,%u)  wrist:(%u,%u,%u) endeff:(%u,%u,%u)\n", H->elbow[0], H->elbow[1],H->elbow[2],H->wrist[0],H->wrist[1],H->wrist[2],H->endeffx, H->endeffy,H->endeffz);
}

//display action cost table
void EnvironmentROBARM::OutputActionCostTable()
{
    int x,y;
    printf("\nAction Cost Table\n");
    for (x = 0; x < EnvROBARMCfg.nVelSucc; x++)
    {
        printf("%i: ",x); 
        for (y = 0; y < EnvROBARMCfg.nVelSucc; y++)
            printf("%i  ",EnvROBARMCfg.ActionCosts[x][y]);
        printf("\n");
    }
    printf("\n");
}

//display successor actions
void EnvironmentROBARM::OutputActions()
{
    int x,y;
    printf("\nSuccessor Actions\n");
    for (x=0; x < EnvROBARMCfg.nVelSucc; x++)
    {
        printf("%x:  ",x);
        for(y=0; y < NUMOFLINKS; y++)
            printf("%.2f  ",EnvROBARMCfg.succ_vel[x][y]);

        printf("\n");
    }
}

void EnvironmentROBARM::CloseKinNode()
{
//     ros::fini();
//     sleep(1);
    printf("\n# calls to ComputeEndEffectorPos(): %i\n",num_forwardkinematics);

    printf("\n");
    if(EnvROBARMCfg.use_DH)
    {
        printf("DH Computation:\n");
        printf("Total: %3.2f sec\n", DH_time/(double)CLOCKS_PER_SEC);
//         printf("Trigonometry: %3.2f sec\n", trig_time/(double)CLOCKS_PER_SEC);
//         printf("Construct transformation: %3.2f sec\n",DH_construct_trans_time/(double)CLOCKS_PER_SEC);
//         printf("Multiply transformations: %3.2f sec\n",DH_matrix_mult_time/(double)CLOCKS_PER_SEC);
//         printf("Fetching transformations: %3.2f sec\n",DH_fetch_trans_time/(double)CLOCKS_PER_SEC);
        printf("\n");
    }
    else
        printf("Kinematics Library time: %3.2f sec\n", KL_time/(double)CLOCKS_PER_SEC);

    printf("Total check for collision time: %3.2f sec\n", check_collision_time/(double)CLOCKS_PER_SEC);
    printf("\n");
}

//parse algorithm parameter file - add to it
void EnvironmentROBARM::ReadParams(FILE* fCfg)
{
    char sTemp[1024];

    // USE_DH
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.use_DH = atoi(sTemp);
    // ENFORCE_MOTOR_LIMITS
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.enforce_motor_limits = atoi(sTemp);
    // DIJKSTRA_HEURISTIC
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.dijkstra_heuristic = atoi(sTemp);
    // ENDEFF_CHECK_ONLY
    fscanf(fCfg, "%s", sTemp);
    fscanf(fCfg, "%s", sTemp);
    EnvROBARMCfg.endeff_check_only = atoi(sTemp);
}

//parse velocities file
void EnvironmentROBARM::ReadSuccActionsFile(FILE* fCfg)
{
    char sTemp[1024];
    int x,y,nrows,ncols;

    //parse velocities
    fscanf(fCfg, "%s", sTemp);
    if(strcmp(sTemp, "Actions:") == 0)
    {
        fscanf(fCfg, "%s", sTemp);
        nrows = atoi(sTemp);
        fscanf(fCfg, "%s", sTemp);
        ncols = atoi(sTemp);

        EnvROBARMCfg.nVelSucc = nrows;

        if(ncols != NUMOFLINKS)
            if (ncols > NUMOFLINKS)
                printf("WARNING: Actions file contain more joint angles than expected number of joints. It will be truncated.\n");
            else
            {
                printf("Error: Not enough angles in actions file for expected system.  Exiting.\n");
                exit(1); 
            }

        //initialize EnvROBARM.succ_vel & parse config file
        EnvROBARMCfg.succ_vel = new double* [nrows];
        for (x=0; x < nrows; x++)
        {
            EnvROBARMCfg.succ_vel[x] = new double [ncols];
            for(y=0; y < ncols; y++)
            {
                 fscanf(fCfg, "%s", sTemp);
                 if(!feof(fCfg) && strlen(sTemp) != 0)
                    EnvROBARMCfg.succ_vel[x][y] = atoi(sTemp);
                else
                {
                    printf("ERROR: End of actions file reached prematurely.\n");
                    exit(1);
                }
            }
        }
    }
    else
    {
        printf("ERROR: Actions file contains unknown object type.\n");
        exit(1);
    }
}



//----------------------Forward Kinematics ----------------------------------

//needed when using kinematic library
void EnvironmentROBARM::InitializeKinNode()
{
    clock_t currenttime = clock();

    char *c_filename = getenv("ROS_PACKAGE_PATH");
    std::stringstream filename;
    filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;
    EnvROBARMCfg.pr2_kin.loadXML(filename.str());

    EnvROBARMCfg.left_arm = EnvROBARMCfg.pr2_kin.getSerialChain("left_arm");    
    assert(EnvROBARMCfg.left_arm);    
    EnvROBARMCfg.pr2_config = new JntArray(EnvROBARMCfg.left_arm->num_joints_);

//     printf("Node initialized.\n");
    /*std::string pr2Content;
    calcFK_armplanner.get_param("robotdesc/pr2",pr2Content);  
    EnvROBARMCfg.pr2_kin.loadString(pr2Content.c_str());*/

    KL_time += clock() - currenttime;
}

//uses ros's KL Library
void EnvironmentROBARM::ComputeForwardKinematics(double *angles, int f_num, double  *x, double *y, double *z)
{
    KDL::Vector gripper(0.0, 0.0, GRIPPER_LENGTHX);

    for(int i = 0; i < NUMOFLINKS; i++)
        (*EnvROBARMCfg.pr2_config)(i) = angles[i];

    Frame f;
    EnvROBARMCfg.left_arm->computeFK((*EnvROBARMCfg.pr2_config),f,f_num);

    //add translation from wrist to end of fingers
    if(f_num == 7)
        f.p = f.M*gripper + f.p;

    //translate xyz coordinates from shoulder frame to base frame
    *x = f.p[0] + EnvROBARMCfg.BaseX_m;
    *y = f.p[1] + EnvROBARMCfg.BaseY_m;
    *z = f.p[2] + EnvROBARMCfg.BaseZ_m;
}

//first method tried (called by getTransformations_robarm7d())
//takes in DH paramters and returns DH matrix 
void EnvironmentROBARM::getDHMatrix(boost::numeric::ublas::matrix<double>*T, double alpha, double a, double d, double theta)
{

    clock_t currenttime = clock();
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_alpha = cos(alpha);
    double s_alpha = sin(alpha);
    trig_time += clock() - currenttime;

    T->resize(4,4);

    currenttime = clock();
    (*T)(0,0) = c_theta;
    (*T)(0,1) = -s_theta*c_alpha;
    (*T)(0,2) = s_theta*s_alpha;
    (*T)(0,3) = a*c_theta;

    (*T)(1,0) = s_theta;
    (*T)(1,1) = c_theta*c_alpha;
    (*T)(1,2) = -c_theta*s_alpha;
    (*T)(1,3) = a*s_theta;

    (*T)(2,0) = 0;
    (*T)(2,1) = s_alpha;
    (*T)(2,2) = c_alpha;
    (*T)(2,3) = d;

    (*T)(3,0) = 0;
    (*T)(3,1) = 0;
    (*T)(3,2) = 0;
    (*T)(3,3) = 1;
    DH_construct_trans_time += clock() - currenttime;
}

//first method tried - very inefficient (DEPRECATED)
//get transformation matrix from each each coordinate frame to the shoulder frame (DEPRECATED)
void EnvironmentROBARM::getTransformations_robarm7d(boost::numeric::ublas::matrix<double> *T, double angles[NUMOFLINKS])
{
    double theta[NUMOFLINKS_DH];
    clock_t currenttime;

    theta[0] = EnvROBARMCfg.DH_theta[0];
    theta[1] = angles[0] + EnvROBARMCfg.DH_theta[1];
    theta[2] = angles[1] + EnvROBARMCfg.DH_theta[2];
    theta[3] = angles[2] + EnvROBARMCfg.DH_theta[3];
    theta[4] = angles[3] + EnvROBARMCfg.DH_theta[4];
    theta[5] = angles[4] + EnvROBARMCfg.DH_theta[5];
    theta[6] = angles[5] + EnvROBARMCfg.DH_theta[6];
    theta[7] = angles[6] + EnvROBARMCfg.DH_theta[7];

    // compute transformation matrices and premultiply
    for(int i = 0; i < NUMOFLINKS_DH; i++)
    {
//         printf("\nFrame: %d \n",i);
#if PRECOMPUTE_DH
//         GetPrecomputedDHMatrix(&(T[i]), theta[i],i);
        GetDHMatrix(theta[i], i);
#else 
        getDHMatrix(&(T[i]),EnvROBARMCfg.DH_alpha[i],EnvROBARMCfg.DH_a[i],EnvROBARMCfg.DH_d[i],theta[i]);
#endif

//         printf("\n\nFrame: %d  theta: %.3f\n",i,theta[i]);
//         for (int x = 0; x < 4; x++)
//         {
//             printf("\n");
//             for (int y=0; y < 4; y++)
//                 printf("%1.3f  ",(T[i])(x,y));
//         }

//         printf("\n");
//         for (int x = 0; x < 4; x++)
//         {
//             printf("\n");
//             for (int y=0; y < 4; y++)
//                 printf("%1.3f  ",(T[i])(x,y)); //printf("%1.3f  ",(EnvROBARMCfg.T_DH[x][y][i]));
//         }


        if(i > 0)
        {
            currenttime = clock();
//             T[i] = prod(T[i-1],T[i]);
            Mult4x4(i-1, i, i);
            DH_matrix_mult_time += clock() - currenttime;
        }

//         printf("\nOld Method\n");
//         for (int x = 0; x < 4; x++)
//         {
//             printf("\n");
//             for (int y=0; y < 4; y++)
//                 printf("%1.3f  ",EnvROBARMCfg.T_array[x][y][i]);
//         }
    }
//         printf("\n----------------------------------------------\n");
}

//Precompute DH matrices for all 8 frames with 360 degree resolution 
void EnvironmentROBARM::ComputeDHTransformations()
{
    int i,n,theta;
    double c_alpha,s_alpha;

    //precompute cos and sin tables
    for (i = 0; i < 360; i++)
    {
        EnvROBARMCfg.cos_r[i] = cos(DEG2RAD(i));
        EnvROBARMCfg.sin_r[i] = sin(DEG2RAD(i));
    }

    // assign transformation matrices 
    for (n = 0; n < NUMOFLINKS_DH; n++)
    {
        c_alpha = cos(EnvROBARMCfg.DH_alpha[n]);
        s_alpha = sin(EnvROBARMCfg.DH_alpha[n]);

        theta=0;
        for (i = 0; i < NUM_TRANS_MATRICES*4; i+=4)
        {
            EnvROBARMCfg.T_DH[i][0][n] = EnvROBARMCfg.cos_r[theta];
            EnvROBARMCfg.T_DH[i][1][n] = -EnvROBARMCfg.sin_r[theta]*c_alpha; 
            EnvROBARMCfg.T_DH[i][2][n] = EnvROBARMCfg.sin_r[theta]*s_alpha;
            EnvROBARMCfg.T_DH[i][3][n] = EnvROBARMCfg.DH_a[n]*EnvROBARMCfg.cos_r[theta];

            EnvROBARMCfg.T_DH[i+1][0][n] = EnvROBARMCfg.sin_r[theta];
            EnvROBARMCfg.T_DH[i+1][1][n] = EnvROBARMCfg.cos_r[theta]*c_alpha;
            EnvROBARMCfg.T_DH[i+1][2][n] = -EnvROBARMCfg.cos_r[theta]*s_alpha;
            EnvROBARMCfg.T_DH[i+1][3][n] = EnvROBARMCfg.DH_a[n]*EnvROBARMCfg.sin_r[theta];


            EnvROBARMCfg.T_DH[i+2][0][n] = 0.0;
            EnvROBARMCfg.T_DH[i+2][1][n] = s_alpha;
            EnvROBARMCfg.T_DH[i+2][2][n] = c_alpha;
            EnvROBARMCfg.T_DH[i+2][3][n] = EnvROBARMCfg.DH_d[n];

            EnvROBARMCfg.T_DH[i+3][0][n] = 0.0;
            EnvROBARMCfg.T_DH[i+3][1][n] = 0.0;
            EnvROBARMCfg.T_DH[i+3][2][n] = 0.0;
            EnvROBARMCfg.T_DH[i+3][3][n] = 1.0;

            theta++;
        }
    }

}

//second method -replaced getDHMatrix() (DEPRECATED)
//fetches DH matrix from precomputed matrices and assigns to matrix object
void EnvironmentROBARM::GetPrecomputedDHMatrix(boost::numeric::ublas::matrix<double>*T, double theta, int frame)
{
    clock_t currenttime = clock();
    T->resize(4,4);

    //convert to degrees
    theta=theta*(180.0/PI_CONST);

    if (theta < 0)
        theta = 360.0+theta;

    //round to nearest integer
    int t = theta + 0.5;

    //multiply by 4 to get to correct position (bad format)
    t = t*4;

    (*T)(0,0) = EnvROBARMCfg.T_DH[t][0][frame];
    (*T)(0,1) = EnvROBARMCfg.T_DH[t][1][frame];
    (*T)(0,2) = EnvROBARMCfg.T_DH[t][2][frame];
    (*T)(0,3) = EnvROBARMCfg.T_DH[t][3][frame];

    (*T)(1,0) = EnvROBARMCfg.T_DH[t+1][0][frame];
    (*T)(1,1) = EnvROBARMCfg.T_DH[t+1][1][frame];
    (*T)(1,2) = EnvROBARMCfg.T_DH[t+1][2][frame];
    (*T)(1,3) = EnvROBARMCfg.T_DH[t+1][3][frame];

    (*T)(2,0) = EnvROBARMCfg.T_DH[t+2][0][frame];
    (*T)(2,1) = EnvROBARMCfg.T_DH[t+2][1][frame];
    (*T)(2,2) = EnvROBARMCfg.T_DH[t+2][2][frame];
    (*T)(2,3) = EnvROBARMCfg.T_DH[t+2][3][frame];

    (*T)(3,0) = EnvROBARMCfg.T_DH[t+3][0][frame];
    (*T)(3,1) = EnvROBARMCfg.T_DH[t+3][1][frame];
    (*T)(3,2) = EnvROBARMCfg.T_DH[t+3][2][frame];
    (*T)(3,3) = EnvROBARMCfg.T_DH[t+3][3][frame];

    DH_fetch_trans_time += clock() - currenttime;
}

//third method - replaced GetPrecomputedDHMatrix() (DEPRECATED)
//Doesn't use a boost matrix object, fetching and copying still takes too long
void EnvironmentROBARM::GetDHMatrix(double theta, int frame)
{
    clock_t currenttime = clock();

    //convert to degrees
    theta=theta*(180.0/PI_CONST);

    if (theta < 0)
        theta = 360.0+theta;

    //round to nearest integer
    int t = theta + 0.5;

    //multiply by 4 to get to correct position (bad format)
    t = t*4;

    EnvROBARMCfg.T_array[0][0][frame] = EnvROBARMCfg.T_DH[t][0][frame];
    EnvROBARMCfg.T_array[0][1][frame] = EnvROBARMCfg.T_DH[t][1][frame];
    EnvROBARMCfg.T_array[0][2][frame] = EnvROBARMCfg.T_DH[t][2][frame];
    EnvROBARMCfg.T_array[0][3][frame] = EnvROBARMCfg.T_DH[t][3][frame];

    EnvROBARMCfg.T_array[1][0][frame] = EnvROBARMCfg.T_DH[t+1][0][frame];
    EnvROBARMCfg.T_array[1][1][frame] = EnvROBARMCfg.T_DH[t+1][1][frame];
    EnvROBARMCfg.T_array[1][2][frame] = EnvROBARMCfg.T_DH[t+1][2][frame];
    EnvROBARMCfg.T_array[1][3][frame] = EnvROBARMCfg.T_DH[t+1][3][frame];

    EnvROBARMCfg.T_array[2][0][frame] = EnvROBARMCfg.T_DH[t+2][0][frame];
    EnvROBARMCfg.T_array[2][1][frame] = EnvROBARMCfg.T_DH[t+2][1][frame];
    EnvROBARMCfg.T_array[2][2][frame] = EnvROBARMCfg.T_DH[t+2][2][frame];
    EnvROBARMCfg.T_array[2][3][frame] = EnvROBARMCfg.T_DH[t+2][3][frame];

    EnvROBARMCfg.T_array[3][0][frame] = EnvROBARMCfg.T_DH[t+3][0][frame];
    EnvROBARMCfg.T_array[3][1][frame] = EnvROBARMCfg.T_DH[t+3][1][frame];
    EnvROBARMCfg.T_array[3][2][frame] = EnvROBARMCfg.T_DH[t+3][2][frame];
    EnvROBARMCfg.T_array[3][3][frame] = EnvROBARMCfg.T_DH[t+3][3][frame];

    DH_fetch_trans_time += clock() - currenttime;
}

//multiples 4x4 transformation matrices  (ONLY USED BY DEPRECATED FUNCTION)
//A: index of first matrix  B: index of second matrix  C: index of goal matrix (usually = B) 
void EnvironmentROBARM::Mult4x4(int A, int B, int C)
{
    int i,j,k;
    double sum;
    double out[4][4];

    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
        {
            sum = 0;
            for (k=0; k<4; k++)
                sum = sum + EnvROBARMCfg.T_array[i][k][A] * EnvROBARMCfg.T_array[k][j][B];
            out[i][j] = sum;
        }
    }

    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
            EnvROBARMCfg.T_array[i][j][C] = out[i][j];
    }
}

//get transformation matrix from each each coordinate frame to the shoulder frame
void EnvironmentROBARM::GetDHTransformations(double angles[NUMOFLINKS])
{
//     clock_t currenttime;
    double theta[NUMOFLINKS_DH];
    int t,x,y,k;
    double sum;

    theta[0] = EnvROBARMCfg.DH_theta[0];
    theta[1] = angles[0] + EnvROBARMCfg.DH_theta[1];
    theta[2] = angles[1] + EnvROBARMCfg.DH_theta[2];
    theta[3] = angles[2] + EnvROBARMCfg.DH_theta[3];
    theta[4] = angles[3] + EnvROBARMCfg.DH_theta[4];
    theta[5] = angles[4] + EnvROBARMCfg.DH_theta[5];
    theta[6] = angles[5] + EnvROBARMCfg.DH_theta[6];
    theta[7] = angles[6] + EnvROBARMCfg.DH_theta[7];

    // compute transformation matrices and premultiply
    for(int i = 0; i < NUMOFLINKS_DH; i++)
    {
        //convert theta to degrees
        theta[i]=theta[i]*(180.0/PI_CONST);

        //make sure theta is not negative
        if (theta[i] < 0)
            theta[i] = 360.0+theta[i];

        //round to nearest integer
        t = theta[i] + 0.5;

        //multiply by 4 to get to correct position in T_DH
        t = t*4;

        //multiply by previous transformations to put in shoulder frame
        if(i > 0)
        {
//             currenttime = clock();
            for (x=0; x<4; x++)
            {
                for (y=0; y<4; y++)
                {
                    sum = 0;
                    for (k=0; k<4; k++)
                        sum += EnvROBARMCfg.T_array[x][k][i-1] * EnvROBARMCfg.T_DH[t+k][y][i];

                    EnvROBARMCfg.T_array[x][y][i] = sum;
                }
            }
//             DH_matrix_mult_time += clock() - currenttime;
        }
        else
        {
            for (x=0; x<4; x++)
                for (y=0; y<4; y++)
                    EnvROBARMCfg.T_array[x][y][0] = EnvROBARMCfg.T_DH[t+x][y][0];
        }
    }
}

//just used to compare the KL with DH convention
//use to compare link lengths
void EnvironmentROBARM::ValidateDH2KinematicsLibrary()
{
    double angles[NUMOFLINKS] = {0, .314, 0, -0.7853, 0, 0, 0};
    short unsigned int pX1, pX2;
    short unsigned int pY1, pY2;
    short unsigned int pZ1, pZ2;
    short unsigned int wrist1[3],wrist2[3];
    short unsigned int elbow1[3],elbow2[3];

    //compare DH vs KL
    for (double i = -1; i < 1; i+=.005)
    {
        angles[0] = i*2;
        angles[1] += i;
        angles[2] = 0;
        angles[3] = 0;
        angles[4] = i/3;
        angles[5] = -i/8;
        angles[6] = i/5;

        //DH
        EnvROBARMCfg.use_DH = 1;
        ComputeEndEffectorPos(angles, &pX1, &pY1, &pZ1, wrist1, elbow1);
        //KL
        EnvROBARMCfg.use_DH = 0;
        ComputeEndEffectorPos(angles, &pX2, &pY2, &pZ2, wrist2, elbow2);

        if(pX1 != pX2 || pY1 != pY2 || pZ1 != pZ2 || wrist1[0] != wrist2[0] || wrist1[1] != wrist2[1] || wrist1[2] != wrist2[2] ||
            elbow1[0] != elbow2[0] || elbow1[1] != elbow2[1] || elbow1[2] != elbow2[2])
        {
            printf("ERROR -->  ");
            printf("DH: %i,%i,%i   KL: %i,%i,%i  angles: ",pX1,pY1,pZ1,pX2,pY2,pZ2);
            for (int j = 0; j < 7; j++)
                printf("%.3f  ",angles[j]);
            printf("\n");
            printf("      elbow --> DH: %i,%i,%i   KL: %i,%i,%i\n",elbow1[0],elbow1[1],elbow1[2],elbow2[0],elbow2[1],elbow2[2]);
            printf("      wrist --> DH: %i,%i,%i   KL: %i,%i,%i\n",wrist1[0],wrist1[1],wrist1[2],wrist2[0],wrist2[1],wrist2[2]);
        }
        else
            printf("DH: %i,%i,%i   KL: %i,%i,%i\n",pX1,pY1,pZ1,pX2,pY2,pZ2);
    }
}

//precompute action costs
void EnvironmentROBARM::ComputeActionCosts()
{
    int i,x,y;
    double temp = 0.0;
    EnvROBARMCfg.ActionCosts = new int* [EnvROBARMCfg.nVelSucc];
    for (x = 0; x < EnvROBARMCfg.nVelSucc; x++)
    {
        EnvROBARMCfg.ActionCosts[x] = new int[EnvROBARMCfg.nVelSucc];
        for (y = 0; y < EnvROBARMCfg.nVelSucc; y++)
        {
            temp = 0.0;
            for (i = 0; i < NUMOFLINKS; i++)
            {
                if(EnvROBARMCfg.succ_vel[x][i] == 0 || EnvROBARMCfg.succ_vel[y][i] == 0)
                    temp += ((EnvROBARMCfg.succ_vel[x][i]-EnvROBARMCfg.succ_vel[y][i])*(EnvROBARMCfg.succ_vel[x][i]-EnvROBARMCfg.succ_vel[y][i]));
            }
            EnvROBARMCfg.ActionCosts[x][y] = temp * SMOOTH_COST;
        }
    }
}



