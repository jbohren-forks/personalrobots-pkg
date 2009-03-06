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

#include <boost/thread/mutex.hpp>
#include <sbpl_arm_planner/headers.h>

#define PI_CONST 3.141592653
#define DEG2RAD(d) ((d)*(PI_CONST/180.0))
#define RAD2DEG(r) ((r)*(180.0/PI_CONST))

// cost multiplier
#define COSTMULT 1000

// distance from goal 
#define MAX_EUCL_DIJK_m .03

//number of elements in rotation matrix
#define SIZE_ROTATION_MATRIX 9

// discretization of joint angles
#define ANGLEDELTA 360.0

#define GRIPPER_LENGTH_M .10

#define OUTPUT_OBSTACLES 0
#define VERBOSE 0
#define OUPUT_DEGREES 0
#define DEBUG_VALID_COORD 0
#define DEBUG_CHECK_GOALRPY 0
#define DEBUG_JOINTSPACE_PLANNING 0
#define DEBUG_CHECK_LINE_SEGMENT 0

//for ComputeForwardKinematics_DH - to rotate DH frames to PR2 frames
#define ELBOW_JOINT 2
#define WRIST_JOINT 4
#define ENDEFF 6

//continuous joints
#define FOREARM_ROLL 4
#define WRIST_ROLL 6

//temporary hack to figure out if obstacles are in the arm's reachable workspace (meters)
#define ARM_WORKSPACE 1.0

// number of successors to each cell (for dyjkstra's heuristic)
#define DIRECTIONS 26

int dx[DIRECTIONS] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
int dy[DIRECTIONS] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
int dz[DIRECTIONS] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

static clock_t DH_time = 0;
static clock_t KL_time = 0;
static clock_t check_collision_time = 0;
static int num_forwardkinematics = 0;
// static clock_t time_gethash = 0;
// static int num_GetHashEntry = 0;


/*------------------------------------------------------------------------*/
                        /* State Access Functions */
/*------------------------------------------------------------------------*/
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

unsigned int EnvironmentROBARM3D::GETHASHBIN(short unsigned int* coord, int numofcoord)
{

    int val = 0;

    for(int i = 0; i < numofcoord; i++)
    {
        val += inthash(coord[i]) << i;
    }

    return inthash(val) & (EnvROBARM.HashTableSize-1);
}

void EnvironmentROBARM3D::PrintHashTableHist()
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

EnvROBARMHashEntry_t* EnvironmentROBARM3D::GetHashEntry(short unsigned int* coord, int numofcoord, short unsigned int action, bool bIsGoal)
{
//     num_GetHashEntry++;
//     clock_t currenttime = clock();

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

        if(EnvROBARMCfg.use_smooth_actions)
        {
            //first check if it is the correct action
            if (EnvROBARM.Coord2StateIDHashTable[binid][ind]->action != action)
                continue;
        }

        for(j = 0; j < numofcoord; j++)
        {
            if(EnvROBARM.Coord2StateIDHashTable[binid][ind]->coord[j] != coord[j]) 
            {
                break;
            }
        }

        if (j == numofcoord)
        {
//             time_gethash += clock()-currenttime;
            return EnvROBARM.Coord2StateIDHashTable[binid][ind];
        }
    }

//     time_gethash += clock()-currenttime;

    return NULL;
}

EnvROBARMHashEntry_t* EnvironmentROBARM3D::CreateNewHashEntry(short unsigned int* coord, int numofcoord, short unsigned int endeff[3], short unsigned int action, double orientation[3][3])
{
    int i;
    //clock_t currenttime = clock();
    EnvROBARMHashEntry_t* HashEntry = new EnvROBARMHashEntry_t;

    memcpy(HashEntry->coord, coord, numofcoord*sizeof(short unsigned int));
    memcpy(HashEntry->endeff, endeff, 3*sizeof(short unsigned int));

    if(orientation != NULL)
        memcpy(HashEntry->orientation, orientation, 9*sizeof(double));

    HashEntry->action = action;

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


/*------------------------------------------------------------------------*/
                       /* Heuristic Computation */
/*------------------------------------------------------------------------*/
//compute straight line distance from x,y,z to every other cell
void EnvironmentROBARM3D::getDistancetoGoal(int* HeurGrid, int goalx, int goaly, int goalz)
{
    for(int x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for(int y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            for(int z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
                HeurGrid[XYZTO3DIND(x,y,z)] = (EnvROBARMCfg.cost_per_cell)*sqrt((goalx-x)*(goalx-x) + (goaly-y)*(goaly-y) +  (goalz-z)*(goalz-z));
        }
    }
}

int EnvironmentROBARM3D::XYZTO3DIND(int x, int y, int z)
{
    if(EnvROBARMCfg.lowres_collision_checking)
        return ((x) + (y)*EnvROBARMCfg.LowResEnvWidth_c + (z)*EnvROBARMCfg.LowResEnvWidth_c*EnvROBARMCfg.LowResEnvHeight_c);
    else
        return ((x) + (y)*EnvROBARMCfg.EnvWidth_c + (z)*EnvROBARMCfg.EnvWidth_c*EnvROBARMCfg.EnvHeight_c);
}

void EnvironmentROBARM3D::ReInitializeState3D(State3D* state)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
}

void EnvironmentROBARM3D::InitializeState3D(State3D* state, short unsigned int x, short unsigned int y, short unsigned int z)
{
    state->g = INFINITECOST;
    state->iterationclosed = 0;
    state->x = x;
    state->y = y;
    state->z = z;
}

void EnvironmentROBARM3D::Create3DStateSpace(State3D**** statespace3D)
{
    int x,y,z;
    int width = EnvROBARMCfg.EnvWidth_c;
    int height = EnvROBARMCfg.EnvHeight_c;
    int depth = EnvROBARMCfg.EnvDepth_c;
    
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        width = EnvROBARMCfg.LowResEnvWidth_c;
        height = EnvROBARMCfg.LowResEnvHeight_c;
        depth = EnvROBARMCfg.LowResEnvDepth_c;
    }
    
    //allocate a statespace for 3D search
    *statespace3D = new State3D** [width];
    for (x = 0; x < width; x++)
    {
        (*statespace3D)[x] = new State3D* [height];
        for(y = 0; y < height; y++)
        {
            (*statespace3D)[x][y] = new State3D [depth];
            for(z = 0; z < depth; z++)
            {
                InitializeState3D(&(*statespace3D)[x][y][z],x,y,z);
            }
        }
    }
}

void EnvironmentROBARM3D::Delete3DStateSpace(State3D**** statespace3D)
{
    int x,y;
    int width = EnvROBARMCfg.EnvWidth_c;
    int height = EnvROBARMCfg.EnvHeight_c;

    if(EnvROBARMCfg.lowres_collision_checking)
    {
        width = EnvROBARMCfg.LowResEnvWidth_c;
        height = EnvROBARMCfg.LowResEnvHeight_c;
    }

	//delete the 3D statespace
    for (x = 0; x < width; x++)
    {
        for (y = 0; y < height; y++)
            delete [] (*statespace3D)[x][y];

        delete [] (*statespace3D)[x];
    }
    delete *statespace3D;
}

void EnvironmentROBARM3D::ComputeHeuristicValues()
{
    printf("Running 3D BFS to compute heuristics\n");
    clock_t currenttime = clock();
    int hsize;

    if(EnvROBARMCfg.lowres_collision_checking)
        hsize = XYZTO3DIND(EnvROBARMCfg.LowResEnvWidth_c-1, EnvROBARMCfg.LowResEnvHeight_c-1,EnvROBARMCfg.LowResEnvDepth_c-1)+1;
    else
        hsize = XYZTO3DIND(EnvROBARMCfg.EnvWidth_c-1, EnvROBARMCfg.EnvHeight_c-1,EnvROBARMCfg.EnvDepth_c-1)+1;

    //allocate memory
    EnvROBARM.Heur = new int [hsize]; 

    //now compute the heuristics for all of the goal locations
    State3D*** statespace3D;	
    Create3DStateSpace(&statespace3D);

    Search3DwithQueue(statespace3D, EnvROBARM.Heur, EnvROBARMCfg.EndEffGoals_c);
//     Search3DwithQueue(statespace3D, EnvROBARM.Heur, EnvROBARMCfg.EndEffGoalX_c, EnvROBARMCfg.EndEffGoalY_c, EnvROBARMCfg.EndEffGoalZ_c);
//     Search3DwithHeap(statespace3D, EnvROBARM.Heur, EnvROBARMCfg.EndEffGoalX_c, EnvROBARMCfg.EndEffGoalY_c, EnvROBARMCfg.EndEffGoalZ_c);

    Delete3DStateSpace(&statespace3D);
    printf("completed in %.3f seconds.\n", double(clock()-currenttime) / CLOCKS_PER_SEC);
}

void EnvironmentROBARM3D::Search3DwithQueue(State3D*** statespace, int* HeurGrid, short unsigned int searchstartx, short unsigned int searchstarty, short unsigned int searchstartz)
{
    State3D* ExpState;
    int newx, newy, newz, x,y,z;
    unsigned int g_temp;
    char*** Grid3D = EnvROBARMCfg.Grid3D;
    short unsigned int width = EnvROBARMCfg.EnvWidth_c;
    short unsigned int height = EnvROBARMCfg.EnvHeight_c;
    short unsigned int depth = EnvROBARMCfg.EnvDepth_c;

    //use low resolution grid if enabled
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        searchstartx *= (EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth);
        searchstarty *= (EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth);
        searchstartz *= (EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth);

        width = EnvROBARMCfg.LowResEnvWidth_c;
        height = EnvROBARMCfg.LowResEnvHeight_c;
        depth = EnvROBARMCfg.LowResEnvDepth_c;

        Grid3D = EnvROBARMCfg.LowResGrid3D;
    }

    //create a queue
    queue<State3D*> Queue; 

    int b = 0;
    //initialize to infinity all 
    for (x = 0; x < width; x++)
    {
        for (y = 0; y < height; y++)
        {
            for (z = 0; z < depth; z++)
            {
                HeurGrid[XYZTO3DIND(x,y,z)] = INFINITECOST;
                ReInitializeState3D(&statespace[x][y][z]);
                b++;
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

        //set the corresponding heuristics
        HeurGrid[XYZTO3DIND(ExpState->x, ExpState->y, ExpState->z)] = ExpState->g;

        //iterate through neighbors
        for(int d = 0; d < DIRECTIONS; d++)
        {
            newx = ExpState->x + dx[d];
            newy = ExpState->y + dy[d];
            newz = ExpState->z + dz[d];

            //make sure it is inside the map and has no obstacle
            if(0 > newx || newx >= width || 0 > newy || newy >= height || 0 > newz || newz >= depth || Grid3D[newx][newy][newz] == 1)
//             if(0 > newx || newx >= width || 0 > newy || newy >= height || 0 > newz || newz >= depth || Grid3D[newx][newy][newz] >= EnvROBARMCfg.ObstacleCost)
            {
                continue;
            }

            if(statespace[newx][newy][newz].iterationclosed == 0)
            {
                //insert into the stack
                Queue.push(&statespace[newx][newy][newz]);

                //set the g-value
                if (ExpState->x != newx && ExpState->y != newy && ExpState->z != newz)
                    g_temp = ExpState->g + EnvROBARMCfg.cost_sqrt3_move;
                else if ((ExpState->y != newy && ExpState->z != newz) ||
                          (ExpState->x != newx && ExpState->z != newz) ||
                          (ExpState->x != newx && ExpState->y != newy))
                    g_temp = ExpState->g + EnvROBARMCfg.cost_sqrt2_move;
                else
                    g_temp = ExpState->g + EnvROBARMCfg.cost_per_cell;

                if(statespace[newx][newy][newz].g > g_temp)
                    statespace[newx][newy][newz].g = g_temp;
//                 printf("%i: %u,%u,%u, g-cost: %i\n",d,newx,newy,newz,statespace[newx][newy][newz].g);
            }
        }
    }
}

void EnvironmentROBARM3D::Search3DwithQueue(State3D*** statespace, int* HeurGrid, short unsigned int ** EndEffGoals_c)
{
    State3D* ExpState;
    int newx, newy, newz, x,y,z;
    short unsigned int goal_xyz[3];
    unsigned int g_temp;
    char*** Grid3D = EnvROBARMCfg.Grid3D;
    short unsigned int width = EnvROBARMCfg.EnvWidth_c;
    short unsigned int height = EnvROBARMCfg.EnvHeight_c;
    short unsigned int depth = EnvROBARMCfg.EnvDepth_c;

    //use low resolution grid if enabled
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        width = EnvROBARMCfg.LowResEnvWidth_c;
        height = EnvROBARMCfg.LowResEnvHeight_c;
        depth = EnvROBARMCfg.LowResEnvDepth_c;

        Grid3D = EnvROBARMCfg.LowResGrid3D;
    }

    //create a queue
    queue<State3D*> Queue; 

    int b = 0;
    //initialize to infinity all 
    for (x = 0; x < width; x++)
    {
        for (y = 0; y < height; y++)
        {
            for (z = 0; z < depth; z++)
            {
                HeurGrid[XYZTO3DIND(x,y,z)] = INFINITECOST;
                ReInitializeState3D(&statespace[x][y][z]);
                b++;
            }
        }
    }

    //initialization - throw starting states on queue with g cost = 0
    for(int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
    {
        goal_xyz[0] = EndEffGoals_c[i][0];
        goal_xyz[1] = EndEffGoals_c[i][1];
        goal_xyz[2] = EndEffGoals_c[i][2];

        if(EnvROBARMCfg.lowres_collision_checking)
        {
            goal_xyz[0] *= (EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth);
            goal_xyz[1] *= (EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth);
            goal_xyz[2] *= (EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth);
        }

        statespace[goal_xyz[0]][goal_xyz[1]][goal_xyz[2]].g = 0;
        Queue.push(&statespace[goal_xyz[0]][goal_xyz[1]][goal_xyz[2]]);
    }

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

        //set the corresponding heuristics
        HeurGrid[XYZTO3DIND(ExpState->x, ExpState->y, ExpState->z)] = ExpState->g;

        //iterate through neighbors
        for(int d = 0; d < DIRECTIONS; d++)
        {
            newx = ExpState->x + dx[d];
            newy = ExpState->y + dy[d];
            newz = ExpState->z + dz[d];

            //make sure it is inside the map and has no obstacle
            if(0 > newx || newx >= width || 0 > newy || newy >= height || 0 > newz || newz >= depth || Grid3D[newx][newy][newz] >= EnvROBARMCfg.ObstacleCost)
            {
                continue;
            }

            if(statespace[newx][newy][newz].iterationclosed == 0)
            {
                //insert into the stack
                Queue.push(&statespace[newx][newy][newz]);

                //set the g-value
                if (ExpState->x != newx && ExpState->y != newy && ExpState->z != newz)
                    g_temp = ExpState->g + EnvROBARMCfg.cost_sqrt3_move;
                else if ((ExpState->y != newy && ExpState->z != newz) ||
                          (ExpState->x != newx && ExpState->z != newz) ||
                          (ExpState->x != newx && ExpState->y != newy))
                    g_temp = ExpState->g + EnvROBARMCfg.cost_sqrt2_move;
                else
                    g_temp = ExpState->g + EnvROBARMCfg.cost_per_cell;

                if(statespace[newx][newy][newz].g > g_temp)
                    statespace[newx][newy][newz].g = g_temp;
//                 printf("%i: %u,%u,%u, g-cost: %i\n",d,newx,newy,newz,statespace[newx][newy][newz].g);
            }
        }
    }
}

/*
void EnvironmentROBARM3D::Search3DwithHeap(State3D*** statespace, int* HeurGrid, int searchstartx, int searchstarty, int searchstartz)
{
    CKey key;	
    CHeap* heap;
    State3D* ExpState;
    int newx, newy, newz, x,y,z;
    unsigned int g_temp;

    //create a heap
    heap = new CHeap; 

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
    key.key[0] = 0;
    heap->insertheap(&statespace[searchstartx][searchstarty][searchstartz], key);


    //expand all of the states
    while(!heap->emptyheap())
{
        //get the state to expand
        ExpState = heap->deleteminheap();

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
                EnvROBARMCfg.Grid3D[newx][newy][newz] >= EnvROBARMCfg.ObstacleCost)
                    continue;

            if(statespace[newx][newy][newz].g > ExpState->g + 1 &&
                    EnvROBARMCfg.Grid3D[newx][newy][newz] == 0)
{
                statespace[newx][newy][newz].g = ExpState->g + 1;
                key.key[0] = statespace[newx][newy][newz].g;
                if(statespace[newx][newy][newz].heapindex == 0)
                    heap->insertheap(&statespace[newx][newy][newz], key);
                else
                    heap->updateheap(&statespace[newx][newy][newz], key);
}

}
}

    //delete the heap	
    delete heap;
}

//--------------------------------------------------------------
*/


/*------------------------------------------------------------------------*/
                        /* Domain Specific Functions */
/*------------------------------------------------------------------------*/
void EnvironmentROBARM3D::ReadConfiguration(FILE* fCfg)
{
    char sTemp[1024];
    int x,i;
    double obs[6];

    fscanf(fCfg, "%s", sTemp);
    while(!feof(fCfg) && strlen(sTemp) != 0)
    {
        if(strcmp(sTemp, "environmentsize(meters):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.EnvWidth_m = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.EnvHeight_m = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.EnvDepth_m = atof(sTemp);
        }
        else if(strcmp(sTemp, "discretization(cells):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.EnvWidth_c = atoi(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.EnvHeight_c = atoi(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.EnvDepth_c = atoi(sTemp);

            //Low-Res Environment for Colision Checking
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.LowResEnvWidth_c = atoi(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.LowResEnvHeight_c = atoi(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.LowResEnvDepth_c = atoi(sTemp);

            //set additional parameters
            EnvROBARMCfg.GridCellWidth = EnvROBARMCfg.EnvWidth_m / double(EnvROBARMCfg.EnvWidth_c);
            EnvROBARMCfg.LowResGridCellWidth = EnvROBARMCfg.EnvWidth_m / double(EnvROBARMCfg.LowResEnvWidth_c);

            //temporary - I have no clue why the commented line doesn't work
            double a = (EnvROBARMCfg.EnvWidth_m / double(EnvROBARMCfg.EnvWidth_c));
            double b = (EnvROBARMCfg.EnvHeight_m / double(EnvROBARMCfg.EnvHeight_c));
            double c = (EnvROBARMCfg.EnvDepth_m / double(EnvROBARMCfg.EnvDepth_c));

            printf("%f %f %f\n",a,b,c);
//             printf("%i %i %i\n",(a != b),(a != c),(b != c));
//             printf("m: %f %f %f\n",EnvROBARMCfg.EnvWidth_m,EnvROBARMCfg.EnvHeight_m,EnvROBARMCfg.EnvDepth_m);
//             printf("c: %f %f %f\n",double(EnvROBARMCfg.EnvWidth_c),double(EnvROBARMCfg.EnvHeight_c),double(EnvROBARMCfg.EnvDepth_c));
            if((a != b) || (a != c) || (b != c))
            //if((EnvROBARMCfg.EnvWidth_m / double(EnvROBARMCfg.EnvWidth_c)) != (EnvROBARMCfg.EnvHeight_m / double(EnvROBARMCfg.EnvHeight_c)))
            //   || EnvROBARMCfg.GridCellWidth != EnvROBARMCfg.EnvDepth_m/EnvROBARMCfg.EnvDepth_c)
            {
                printf("ERROR: The cell should be square\n");
                exit(1);
            }

            //allocate memory for environment grid
            InitializeEnvGrid();
        }
        else if(strcmp(sTemp, "basexyz(cells):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.BaseX_c = atoi(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.BaseY_c = atoi(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.BaseZ_c = atoi(sTemp);

            //convert shoulder base from cell to real world coords
            Cell2ContXYZ(EnvROBARMCfg.BaseX_c, EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c, &(EnvROBARMCfg.BaseX_m), &(EnvROBARMCfg.BaseY_m), &(EnvROBARMCfg.BaseZ_m)); 
        }
        else if(strcmp(sTemp, "basexyz(meters):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.BaseX_m = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.BaseY_m = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.BaseZ_m = atof(sTemp);
            ContXYZ2Cell(EnvROBARMCfg.BaseX_m, EnvROBARMCfg.BaseY_m, EnvROBARMCfg.BaseZ_m, &(EnvROBARMCfg.BaseX_c), &(EnvROBARMCfg.BaseY_c), &(EnvROBARMCfg.BaseZ_c)); 
        }
        else if(strcmp(sTemp, "linklengths(meters):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.LinkLength_m[i] = atof(sTemp);
            }
        }
        else if(strcmp(sTemp, "linkstartangles(degrees):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.LinkStartAngles_d[i] = atof(sTemp);
            }
        }
        else if (strcmp(sTemp, "endeffectorgoal(cells):") == 0)
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
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.LinkGoalAngles_d[i] = atoi(sTemp);
            }

            //so the goal's location can be calculated after the initialization completes
            EnvROBARMCfg.JointSpaceGoal = 1;
        }
        else if(strcmp(sTemp, "linkgoalangles(radians):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.LinkGoalAngles_d[i] = RAD2DEG(atof(sTemp));
            }

                //so the goal's location can be calculated after the initialization completes
            EnvROBARMCfg.JointSpaceGoal = 1;
        }
        else if(strcmp(sTemp, "endeffectorgoal(meters):") == 0)
        {
            EnvROBARMCfg.PlanInJointSpace = false;

            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.nEndEffGoals = atoi(sTemp);

            //allocate memory
            EnvROBARMCfg.EndEffGoals_m = new double * [EnvROBARMCfg.nEndEffGoals];
            for (i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
                EnvROBARMCfg.EndEffGoals_m[i] = new double [6];

            for(i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
            {
                for(int k = 0; k < 6; k++)
                {
                    fscanf(fCfg, "%s", sTemp);
                    EnvROBARMCfg.EndEffGoals_m[i][k] = atof(sTemp);
                }
            }
        }
        else if(strcmp(sTemp, "endeffectorgoal(meters-rot):") == 0)
        {
            EnvROBARMCfg.PlanInJointSpace = false;

            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.nEndEffGoals = atoi(sTemp);

            //allocate memory
            EnvROBARMCfg.EndEffGoals_m = new double * [EnvROBARMCfg.nEndEffGoals];
            for (i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
                EnvROBARMCfg.EndEffGoals_m[i] = new double [12];

            for(i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
            {
                for(int k = 0; k < 12; k++)
                {
                    fscanf(fCfg, "%s", sTemp);
                    EnvROBARMCfg.EndEffGoals_m[i][k] = atof(sTemp);
                }
            }
        }
        else if(strcmp(sTemp, "jointspacegoal(radians):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.nJointSpaceGoals = atoi(sTemp);

            //allocate memory
            EnvROBARMCfg.JointSpaceGoals = new double * [EnvROBARMCfg.nJointSpaceGoals];
            for (i = 0; i < EnvROBARMCfg.nJointSpaceGoals; i++)
                EnvROBARMCfg.JointSpaceGoals[i] = new double [NUMOFLINKS];

            for(i = 0; i < EnvROBARMCfg.nJointSpaceGoals; i++)
            {
                for(int k = 0; k < NUMOFLINKS; k++)
                {
                    fscanf(fCfg, "%s", sTemp);
                    EnvROBARMCfg.JointSpaceGoals[i][k] = atof(sTemp);
                }
            }
            EnvROBARMCfg.PlanInJointSpace = true;
        }
        else if(strcmp(sTemp, "linktwist(degrees):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.DH_alpha[i] = PI_CONST*(atof(sTemp)/180.0);
            }
        }
        else if(strcmp(sTemp, "linklength(meters):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.DH_a[i] = atof(sTemp);
//                 EnvROBARMCfg.LinkLength_m[i] = atof(sTemp);
            }
        }
        else if(strcmp(sTemp, "linkoffset(meters):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.DH_d[i] = atof(sTemp);
            }
        }
        else if(strcmp(sTemp, "jointangle(degrees):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.DH_theta[i] = DEG2RAD(atof(sTemp));
            }
        }
        else if(strcmp(sTemp,"posmotorlimits(degrees):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.PosMotorLimits[i] = DEG2RAD(atof(sTemp));
            }
        }
        else if(strcmp(sTemp,"posmotorlimits(radians):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.PosMotorLimits[i] = atof(sTemp);
            }
        }
        else if(strcmp(sTemp,"negmotorlimits(degrees):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.NegMotorLimits[i] = DEG2RAD(atof(sTemp));
            }
        }
        else if(strcmp(sTemp,"negmotorlimits(radians):") == 0)
        {
            for(i = 0; i < NUMOFLINKS; i++)
            {
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.NegMotorLimits[i] = atof(sTemp);
            }
        }
        else if(strcmp(sTemp, "goalorientationMOE:") == 0)
        {
            for(x = 0; x < 3; x++)
            {
                //acceptable margin of error of that row
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.GoalOrientationMOE[x][0] = atof(sTemp);
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.GoalOrientationMOE[x][1] = atof(sTemp);
                fscanf(fCfg, "%s", sTemp);
                EnvROBARMCfg.GoalOrientationMOE[x][2] = atof(sTemp);
            }
        }
        else if(strcmp(sTemp, "goalorientationMOE-RPY:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.GoalRPY_MOE[0] = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.GoalRPY_MOE[1] = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.GoalRPY_MOE[2] = atof(sTemp);
        }
        else if(strcmp(sTemp, "environment:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            while(!feof(fCfg) && strlen(sTemp) != 0)
            {
                if(strcmp(sTemp, "cube(meters):") == 0)
                {
                    for(i = 0; i < 6; i++)
                    {
                        fscanf(fCfg, "%s", sTemp);
                        obs[i] = atof(sTemp);
                    }
                    AddObstacleToGrid(obs, 0, EnvROBARMCfg.Grid3D, EnvROBARMCfg.GridCellWidth);

                    if(EnvROBARMCfg.lowres_collision_checking)
                        AddObstacleToGrid(obs, 0, EnvROBARMCfg.LowResGrid3D, EnvROBARMCfg.LowResGridCellWidth);
                }
                else if (strcmp(sTemp, "cube(cells):") == 0)
                {
                    for(i = 0; i < 6; i++)
                    {
                        fscanf(fCfg, "%s", sTemp);
                        obs[i] = atof(sTemp);
                    }
                    AddObstacleToGrid(obs, 1, EnvROBARMCfg.Grid3D, EnvROBARMCfg.GridCellWidth);

                    if(EnvROBARMCfg.lowres_collision_checking)
                        AddObstacleToGrid(obs, 1, EnvROBARMCfg.LowResGrid3D, EnvROBARMCfg.LowResGridCellWidth);
                }
                else
                {
                    printf("ERROR: Environment Config file contains unknown object type.\n");
                    exit(1);
                }
                fscanf(fCfg, "%s", sTemp);
            }
        }
        else
        {
            printf("ERROR: Unknown parameter name in environment config file: %s.\n", sTemp);
        }
        fscanf(fCfg, "%s", sTemp);
    }

    printf("Parsed environment config file.\n");
}

//parse algorithm parameter file
void EnvironmentROBARM3D::ReadParamsFile(FILE* fCfg)
{
    char sTemp[1024];
    int x,y,nrows,ncols;

    fscanf(fCfg, "%s", sTemp);
    while(!feof(fCfg) && strlen(sTemp) != 0)
    {
        if(strcmp(sTemp, "Use_DH_for_FK:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.use_DH = atoi(sTemp);
        }
        else if(strcmp(sTemp, "Enforce_Motor_Limits:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.enforce_motor_limits = atoi(sTemp);
        }
        else if(strcmp(sTemp, "Use_Dijkstra_for_Heuristic:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.dijkstra_heuristic = atoi(sTemp);
        }
        else if(strcmp(sTemp, "Collision_Checking_on_EndEff_only:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.endeff_check_only = atoi(sTemp);
        }
        else if(strcmp(sTemp, "Obstacle_Padding(meters):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.padding = atof(sTemp);
        }
        else if(strcmp(sTemp, "Smoothing_Weight:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.smoothing_weight = atof(sTemp);
        }
        else if(strcmp(sTemp, "Use_Path_Smoothing:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.use_smooth_actions = atoi(sTemp);
        }
        else if(strcmp(sTemp, "Keep_Gripper_Upright_for_Whole_Path:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.enforce_upright_gripper = atoi(sTemp);
        }
        else if(strcmp(sTemp, "Check_Orientation_of_EndEff_at_Goal:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.checkEndEffGoalOrientation = atoi(sTemp);
        }
        else if(strcmp(sTemp,"ARAPlanner_Epsilon:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.epsilon = atof(sTemp);
        }
        else if(strcmp(sTemp,"Length_of_Grasped_Cylinder(meters):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.grasped_object_length_m = atof(sTemp);
        }
        else if(strcmp(sTemp,"Cylinder_in_Gripper:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.object_grasped = atoi(sTemp);
        }
        else if(strcmp(sTemp,"EndEffGoal_MarginOfError(meters):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.goal_moe_m = atof(sTemp);
        }
        else if(strcmp(sTemp,"LowRes_Collision_Checking:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.lowres_collision_checking = atoi(sTemp);
        }
        else if(strcmp(sTemp,"MultiRes_Successor_Actions:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.multires_succ_actions = atoi(sTemp);
        }
        else if(strcmp(sTemp,"Increasing_Cell_Costs_Near_Obstacles:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.variable_cell_costs = atoi(sTemp);
        }
        else if(strcmp(sTemp,"GoalRPY_MarginOfError(rad):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.GoalRPY_MOE[0] = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.GoalRPY_MOE[1] = atof(sTemp);
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.GoalRPY_MOE[2] = atof(sTemp);
        }
        else if(strcmp(sTemp,"HighResActions_Distance_Threshold(cells):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.HighResActionsThreshold_c = atof(sTemp);
        }
        else if(strcmp(sTemp,"Use_Angular_Distance_Cost:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.angular_dist_cost = atoi(sTemp);
        }
        else if(strcmp(sTemp,"Angular_Distance_Weight:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.AngularDist_Weight = atof(sTemp);
        }
        else if(strcmp(sTemp,"Dist_from_goal_to_plan_for_RPY(meters):") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.ApplyRPYCost_m = atof(sTemp);
        }
        else if(strcmp(sTemp,"AngularDistance_Exponential_Coefficient:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.ExpCoefficient = atof(sTemp);
        }
        else if(strcmp(sTemp,"Use_Max_of_Dual_Heuristics:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.dual_heuristics = atoi(sTemp);
        }
        else if(strcmp(sTemp,"PlanInJointSpace:") == 0)
        {
            fscanf(fCfg, "%s", sTemp);
            EnvROBARMCfg.PlanInJointSpace = atoi(sTemp);
        }
        //parse actions - it must be the last thing in the file
        else if(strcmp(sTemp, "Actions:") == 0)
        {
            break;
        }
        else
        {
            printf("Error: Invalid Field name (%s) in parameter file.\n",sTemp);
            exit(1);
        }
        fscanf(fCfg, "%s", sTemp);
    }

    // parse successor actions
    fscanf(fCfg, "%s", sTemp);
    nrows = atoi(sTemp);
    fscanf(fCfg, "%s", sTemp);
    ncols = atoi(sTemp);

    EnvROBARMCfg.nLowResActions = ncols;
    EnvROBARMCfg.nSuccActions = nrows;

    //initialize EnvROBARM.SuccActions & parse config file
    EnvROBARMCfg.SuccActions = new double* [nrows];
    for (x=0; x < nrows; x++)
    {
        EnvROBARMCfg.SuccActions[x] = new double [ncols];
        for(y=0; y < NUMOFLINKS; y++)
        {
            fscanf(fCfg, "%s", sTemp);
            if(!feof(fCfg) && strlen(sTemp) != 0)
                EnvROBARMCfg.SuccActions[x][y] = atoi(sTemp);
            else
            {
                printf("ERROR: End of parameter file reached prematurely. Check for newline.\n");
                exit(1);
            }
        }
    }
}

bool EnvironmentROBARM3D::SetEnvParameter(char* parameter, double value)
{
    if(EnvROBARMCfg.bInitialized == true)
    {
        printf("ERROR: all parameters must be set before initialization of the environment\n");
        return false;
    }

    printf("setting parameter %s to %2.1f\n", parameter, value);

    if(strcmp(parameter, "useDHforFK") == 0)
    {
        EnvROBARMCfg.use_DH = value;
    }
    else if(strcmp(parameter, "enforceMotorLimits") == 0)
    {
        EnvROBARMCfg.enforce_motor_limits = value;
    }
    else if(strcmp(parameter, "useDijkstraHeuristic") == 0)
    {
        EnvROBARMCfg.dijkstra_heuristic = value;
    }
    else if(strcmp(parameter, "paddingSize") == 0)
    {
        EnvROBARMCfg.padding = value;
    }
    else if(strcmp(parameter, "smoothingWeight") == 0)
    {
        EnvROBARMCfg.smoothing_weight = value;
    }
    else if(strcmp(parameter, "usePathSmoothing") == 0)
    {
        EnvROBARMCfg.use_smooth_actions = value;
    }
    else if(strcmp(parameter, "uprightGripperOnly") == 0)
    {
        EnvROBARMCfg.enforce_upright_gripper = value;
    }
    else if(strcmp(parameter, "use6DOFGoal") == 0)
    {
        EnvROBARMCfg.checkEndEffGoalOrientation = value;
    }
    else if(strcmp(parameter,"goalPosMOE") == 0)
    {
        EnvROBARMCfg.goal_moe_m = value;
    }
    else if(strcmp(parameter,"useFastCollisionChecking") == 0)
    {
        EnvROBARMCfg.lowres_collision_checking = value;
    }
    else if(strcmp(parameter,"useMultiResActions") == 0)
    {
        EnvROBARMCfg.multires_succ_actions = value;
    }
    else if(strcmp(parameter,"useHigherCostsNearObstacles") == 0)
    {
        EnvROBARMCfg.variable_cell_costs = value;
    }
    else
    {
        printf("ERROR: invalid parameter %s\n", parameter);
        return false;
    }

    return true;
}

void EnvironmentROBARM3D::InitializeEnvGrid()
{
    int x, y, z;

    // High-Res Grid - allocate the 3D environment & fill set all cells to zero
    EnvROBARMCfg.Grid3D = new char** [EnvROBARMCfg.EnvWidth_c];
    EnvROBARMCfg.Grid3D_temp = new char** [EnvROBARMCfg.EnvWidth_c];
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        EnvROBARMCfg.Grid3D[x] = new char* [EnvROBARMCfg.EnvHeight_c];
        EnvROBARMCfg.Grid3D_temp[x] = new char* [EnvROBARMCfg.EnvHeight_c];
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            EnvROBARMCfg.Grid3D[x][y] = new char [EnvROBARMCfg.EnvDepth_c];
            EnvROBARMCfg.Grid3D_temp[x][y] = new char [EnvROBARMCfg.EnvDepth_c];
            for (z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
            {
                EnvROBARMCfg.Grid3D[x][y][z] = 0;
                EnvROBARMCfg.Grid3D_temp[x][y][z] = 0;
            }
        }
    }

    //Low-Res Grid - for faster collision checking
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        // Low-Res Grid - allocate the 3D environment & fill set all cells to zero
        EnvROBARMCfg.LowResGrid3D = new char** [EnvROBARMCfg.LowResEnvWidth_c];
        EnvROBARMCfg.LowResGrid3D_temp = new char** [EnvROBARMCfg.LowResEnvWidth_c];
        for (x = 0; x < EnvROBARMCfg.LowResEnvWidth_c; x++)
        {
            EnvROBARMCfg.LowResGrid3D[x] = new char* [EnvROBARMCfg.LowResEnvHeight_c];
            EnvROBARMCfg.LowResGrid3D_temp[x] = new char* [EnvROBARMCfg.LowResEnvHeight_c];
            for (y = 0; y < EnvROBARMCfg.LowResEnvHeight_c; y++)
            {
                EnvROBARMCfg.LowResGrid3D[x][y] = new char [EnvROBARMCfg.LowResEnvDepth_c];
                EnvROBARMCfg.LowResGrid3D_temp[x][y] = new char [EnvROBARMCfg.LowResEnvDepth_c];
                for (z = 0; z < EnvROBARMCfg.LowResEnvDepth_c; z++)
                {
                    EnvROBARMCfg.LowResGrid3D[x][y][z] = 0;
                    EnvROBARMCfg.LowResGrid3D_temp[x][y][z] = 0;
                }
            }
        }
        printf("Allocated LowResGrid3D.\n");
    }
}

void EnvironmentROBARM3D::DiscretizeAngles()
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
    }
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
void EnvironmentROBARM3D::ComputeContAngles(short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS])
{
    int i;
    for(i = 0; i < NUMOFLINKS; i++)
    {
        angle[i] = coord[i]*EnvROBARMCfg.angledelta[i];
    }

}

void EnvironmentROBARM3D::ComputeCoord(double angle[NUMOFLINKS], short unsigned int coord[NUMOFLINKS])
{
    int i;
    double pos_angle;

    for(i = 0; i < NUMOFLINKS; i++)
    {
        //NOTE: Added 3/1/09
        pos_angle = angle[i];
        if(pos_angle < 0)
            pos_angle += 2*PI_CONST;

        coord[i] = (int)((pos_angle + EnvROBARMCfg.angledelta[i]*0.5)/EnvROBARMCfg.angledelta[i]);
//         coord[i] = (int)((angle[i] + EnvROBARMCfg.angledelta[i]*0.5)/EnvROBARMCfg.angledelta[i]);
        if(coord[i] == EnvROBARMCfg.anglevals[i])
            coord[i] = 0;
    }
}

// convert a cell in the occupancy grid to point in real world 
void EnvironmentROBARM3D::Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ)
{
    // offset the arm in the map so that it is placed in the middle of the world 
    int xoffset_c = EnvROBARMCfg.EnvWidth_c-1 - ARM_WORKSPACE/EnvROBARMCfg.GridCellWidth;  //arm can't reach that far behind it, so offset X by arm length
    int yoffset_c = (EnvROBARMCfg.EnvHeight_c-1) / 2;
    int zoffset_c = (EnvROBARMCfg.EnvDepth_c-1) / 2;

    *pX = (x - xoffset_c) * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth*0.5;
    *pY = (y - yoffset_c) * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth*0.5;
    *pZ = (z - zoffset_c) *EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.GridCellWidth*0.5;
}

// convert a cell in the occupancy grid to point in real world
void EnvironmentROBARM3D::Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ, double gridcell_m)
{
    // offset the arm in the map so that it is placed in the middle of the world 
    int xoffset_c = ((EnvROBARMCfg.EnvWidth_m/gridcell_m)-1) - ARM_WORKSPACE/gridcell_m;  //arm can't reach that far behind it, so offset X by arm length
    int yoffset_c = ((EnvROBARMCfg.EnvHeight_m/gridcell_m)-1) / 2;
    int zoffset_c = ((EnvROBARMCfg.EnvDepth_m/gridcell_m-1)) / 2;

    *pX = (x - xoffset_c) * gridcell_m + gridcell_m*0.5;
    *pY = (y - yoffset_c) * gridcell_m + gridcell_m*0.5;
    *pZ = (z - zoffset_c) * gridcell_m + gridcell_m*0.5;
}

// convert a point in real world to a cell in occupancy grid 
void EnvironmentROBARM3D::ContXYZ2Cell(double x, double y, double z, short unsigned int *pX, short unsigned int *pY, short unsigned int *pZ)
{
    // offset the arm in the map so that it is placed in the middle of the world
    int xoffset_c = EnvROBARMCfg.EnvWidth_c-1  - ARM_WORKSPACE/EnvROBARMCfg.GridCellWidth;  //arm can't reach that far behind it, so offset X by arm length
    int yoffset_c = (EnvROBARMCfg.EnvHeight_c-1) / 2;
    int zoffset_c = (EnvROBARMCfg.EnvDepth_c-1) / 2;

    //take the nearest cell
    *pX = (int)((x/EnvROBARMCfg.GridCellWidth) + xoffset_c);
    if(x + (xoffset_c*EnvROBARMCfg.GridCellWidth) < 0) *pX = 0;
    if(*pX >= EnvROBARMCfg.EnvWidth_c) *pX = EnvROBARMCfg.EnvWidth_c-1;

    *pY = (int)((y/EnvROBARMCfg.GridCellWidth) + yoffset_c);
    if(y + (yoffset_c*EnvROBARMCfg.GridCellWidth) < 0) *pY = 0;
    if(*pY >= EnvROBARMCfg.EnvHeight_c) *pY = EnvROBARMCfg.EnvHeight_c-1;

    *pZ = (int)((z/EnvROBARMCfg.GridCellWidth) + zoffset_c);
    if(z + (zoffset_c*EnvROBARMCfg.GridCellWidth) < 0) *pZ = 0;
    if(*pZ >= EnvROBARMCfg.EnvDepth_c) *pZ = EnvROBARMCfg.EnvDepth_c-1;
}

void EnvironmentROBARM3D::ContXYZ2Cell(double* xyz, double gridcellwidth, int dims_c[3], short unsigned int *pXYZ)
{
    // offset the arm in the map so that it is placed in the middle of the world
    int xoffset_c = dims_c[0]-1 - ARM_WORKSPACE/gridcellwidth;
    int yoffset_c = (dims_c[1]-1) / 2;
    int zoffset_c = (dims_c[2]-1) / 2;

    //take the nearest cell
    pXYZ[0] = (int)(xyz[0]/gridcellwidth + xoffset_c);
    if(xyz[0] + (xoffset_c * gridcellwidth) < 0)
        pXYZ[0] = 0;
    if(pXYZ[0] >= dims_c[0])
        pXYZ[0] = dims_c[0]-1;

    pXYZ[1] = (int)((xyz[1]/gridcellwidth) + yoffset_c);
    if(xyz[1] + (yoffset_c * gridcellwidth) < 0)
        pXYZ[1] = 0;
    if(pXYZ[1] >= dims_c[1])
        pXYZ[1] = dims_c[1]-1;

    pXYZ[2] = (int)((xyz[2]/gridcellwidth) + zoffset_c);
    if(xyz[2] + (zoffset_c * gridcellwidth) < 0) 
        pXYZ[2] = 0;
    if(pXYZ[2] >= dims_c[2]) 
        pXYZ[2] = dims_c[2]-1;
}

void EnvironmentROBARM3D::HighResGrid2LowResGrid(short unsigned int * XYZ_hr, short unsigned int * XYZ_lr)
{
    double scale = EnvROBARMCfg.GridCellWidth / EnvROBARMCfg.LowResGridCellWidth;

    XYZ_lr[0] = XYZ_hr[0] * scale;
    XYZ_lr[1] = XYZ_hr[1] * scale;
    XYZ_lr[2] = XYZ_hr[2] * scale;
}

/*//add obstacles to grid (right now just supports cubes)

void EnvironmentROBARM3D::AddObstacleToGrid(double* obstacle, int type, char*** grid, double gridcell_m)
{
    int x, y, z, pX_max, pX_min, pY_max, pY_min, pZ_max, pZ_min;
    int padding_c = EnvROBARMCfg.padding*2 / gridcell_m + 0.5;
    short unsigned int obstacle_c[6] = {0};
    int dims_c[3] = {EnvROBARMCfg.EnvWidth_c, EnvROBARMCfg.EnvHeight_c, EnvROBARMCfg.EnvDepth_c};

    if(gridcell_m == EnvROBARMCfg.LowResGridCellWidth)
{
        dims_c[0] = EnvROBARMCfg.LowResEnvWidth_c;
        dims_c[1] = EnvROBARMCfg.LowResEnvHeight_c;
        dims_c[2] = EnvROBARMCfg.LowResEnvDepth_c;
}

    //cube(meters)
    if (type == 0)
{
        double obs[3] = {obstacle[0],obstacle[1],obstacle[2]};
        short unsigned int obs_c[3] = {obstacle_c[0],obstacle_c[1],obstacle_c[2]};

        ContXYZ2Cell(obs, gridcell_m, dims_c, obs_c);
        //ContXYZ2Cell(obstacle[0],obstacle[1],obstacle[2],&(obstacle_c[0]),&(obstacle_c[1]),&(obstacle_c[2]));

        //get dimensions (width,height,depth)
        obstacle_c[3] = abs(obstacle[3]+EnvROBARMCfg.padding*2) / gridcell_m + 0.5;
        obstacle_c[4] = abs(obstacle[4]+EnvROBARMCfg.padding*2) / gridcell_m + 0.5;
        obstacle_c[5] = abs(obstacle[5]+ EnvROBARMCfg.padding*2) / gridcell_m + 0.5;

        pX_max = obs_c[0] + obstacle_c[3]/2;
        pX_min = obs_c[0] - obstacle_c[3]/2;
        pY_max = obs_c[1] + obstacle_c[4]/2;
        pY_min = obs_c[1] - obstacle_c[4]/2;
        pZ_max = obs_c[2] + obstacle_c[5]/2;
        pZ_min = obs_c[2] - obstacle_c[5]/2;
}
    //cube(cells)
    else if(type == 1)
{
        //convert to short unsigned int from double
        obstacle_c[0] = obstacle[0];
        obstacle_c[1] = obstacle[1];
        obstacle_c[2] = obstacle[2];
        obstacle_c[3] = abs(obstacle[3] + padding_c);
        obstacle_c[4] = abs(obstacle[4] + padding_c);
        obstacle_c[5] = abs(obstacle[5] + padding_c);

        pX_max = obstacle_c[0] + obstacle_c[3]/2;
        pX_min = obstacle_c[0] - obstacle_c[3]/2;
        pY_max = obstacle_c[1] + obstacle_c[4]/2;
        pY_min = obstacle_c[1] - obstacle_c[4]/2;
        pZ_max = obstacle_c[2] + obstacle_c[5]/2;
        pZ_min = obstacle_c[2] - obstacle_c[5]/2;
}
    else
{
        printf("Error: Attempted to add undefined type of obstacle to grid.\n");
        return; 
}

    // bounds checking, cutoff object if out of bounds
    if(pX_max > dims_c[0] - 1)
        pX_max = dims_c[0] - 1;
    if (pX_min < 0)
        pX_min = 0;
    if(pY_max > dims_c[1] - 1)
        pY_max = dims_c[1] - 1;
    if (pY_min < 0)
        pY_min = 0;
    if(pZ_max > dims_c[2] - 1)
        pZ_max = dims_c[2] - 1;
    if (pZ_min < 0)
        pZ_min = 0;

    // assign the cells occupying the obstacle to 1
    int b = 0;
    for (y = pY_min; y <= pY_max; y++)
{
        for (x = pX_min; x <= pX_max; x++)
{
            for (z = pZ_min; z <= pZ_max; z++)
{
                grid[x][y][z] = 1;
                b++;
}
}
}
//     printf("%i %i %i %i %i %i\n", obstacle_c[0],obstacle_c[1],obstacle_c[2],obstacle_c[3],obstacle_c[4],obstacle_c[5]);
//     printf("Obstacle %i cells\n",b);
}
*/

void EnvironmentROBARM3D::AddObstacleToGrid(double* obstacle, int type, char*** grid, double gridcell_m)
{

    int x, y, z, pX_max, pX_min, pY_max, pY_min, pZ_max, pZ_min;
//     int padding_c = EnvROBARMCfg.padding*2 / gridcell_m + 0.5;  //why multiplied by 2?
    int padding_c = (EnvROBARMCfg.padding / gridcell_m) + 0.5;
    short unsigned int obstacle_c[6] = {0}, obs_c[3];
    int dims_c[3] = {EnvROBARMCfg.EnvWidth_c, EnvROBARMCfg.EnvHeight_c, EnvROBARMCfg.EnvDepth_c};
    double ob[3];

    if(gridcell_m == EnvROBARMCfg.LowResGridCellWidth)
    {
        dims_c[0] = EnvROBARMCfg.LowResEnvWidth_c;
        dims_c[1] = EnvROBARMCfg.LowResEnvHeight_c;
        dims_c[2] = EnvROBARMCfg.LowResEnvDepth_c;
    }

    //cube(meters)
    if (type == 0)
    {
        double obs[3] = {obstacle[0],obstacle[1],obstacle[2]};
        ContXYZ2Cell(obs, gridcell_m, dims_c, obs_c);

        //get dimensions of obstacles in cells (width,height,depth)
        obstacle_c[3] = obstacle[3] / gridcell_m + 0.5;
        obstacle_c[4] = obstacle[4] / gridcell_m + 0.5;
        obstacle_c[5] = obstacle[5] / gridcell_m + 0.5;

        pX_max = obs_c[0] + obstacle_c[3]/2 + padding_c;
        pX_min = obs_c[0] - obstacle_c[3]/2 - padding_c;
        pY_max = obs_c[1] + obstacle_c[4]/2 + padding_c;
        pY_min = obs_c[1] - obstacle_c[4]/2 - padding_c;
        pZ_max = obs_c[2] + obstacle_c[5]/2 + padding_c;
        pZ_min = obs_c[2] - obstacle_c[5]/2 - padding_c;
    }

    //cube(cells)
    else if(type == 1)
    {
        //convert to short unsigned int from double
        obstacle_c[0] = obstacle[0];
        obstacle_c[1] = obstacle[1];
        obstacle_c[2] = obstacle[2];
        obstacle_c[3] = abs(obstacle[3] + padding_c);
        obstacle_c[4] = abs(obstacle[4] + padding_c);
        obstacle_c[5] = abs(obstacle[5] + padding_c);

        pX_max = obstacle_c[0] + obstacle_c[3]/2;
        pX_min = obstacle_c[0] - obstacle_c[3]/2;
        pY_max = obstacle_c[1] + obstacle_c[4]/2;
        pY_min = obstacle_c[1] - obstacle_c[4]/2;
        pZ_max = obstacle_c[2] + obstacle_c[5]/2;
        pZ_min = obstacle_c[2] - obstacle_c[5]/2;
    }
    else
    {
        printf("Error: Attempted to add undefined type of obstacle to grid.\n");
        return; 
    }

    // bounds checking, cutoff object if out of bounds
    if(pX_max > dims_c[0] - 1)
        pX_max = dims_c[0] - 1;
    if (pX_min < 0)
        pX_min = 0;
    if(pY_max > dims_c[1] - 1)
        pY_max = dims_c[1] - 1;
    if (pY_min < 0)
        pY_min = 0;
    if(pZ_max > dims_c[2] - 1)
        pZ_max = dims_c[2] - 1;
    if (pZ_min < 0)
        pZ_min = 0;

//     printf("[AddObstacleToGrid] %.2f %.2f %.2f (meters)  -> %d %d %d (cells)\n",obstacle[0],obstacle[1],obstacle[2],obs_c[0],obs_c[1],obs_c[2]);
//     printf("[AddObstacleToGrid] %d %d %d %d %d %d (cells)\n",pX_min,pX_max,pY_min,pY_max,pZ_min,pZ_max);

    // assign the cells occupying the obstacle to ObstacleCost

    for (x = pX_min; x <= pX_max; x++)
    {
        for (y = pY_min; y <= pY_max; y++)
        {
            for (z = pZ_min; z <= pZ_max; z++)
            {
                grid[x][y][z] = EnvROBARMCfg.ObstacleCost;
            }
        }
    }

    //output collision map debugging  - stupid way of doing this but fine for now!
    if(EnvROBARMCfg.lowres_collision_checking && gridcell_m == EnvROBARMCfg.LowResGridCellWidth)
    {
        Cell2ContXYZ(obs_c[0],obs_c[1],obs_c[2],&(ob[0]),&(ob[1]),&(ob[2]));
        EnvROBARMCfg.cubes.resize(EnvROBARMCfg.cubes.size()+1);
        EnvROBARMCfg.cubes.back().resize(6);
        EnvROBARMCfg.cubes.back()[0] = ob[0];
        EnvROBARMCfg.cubes.back()[1] = ob[1];
        EnvROBARMCfg.cubes.back()[2] = ob[2];
        EnvROBARMCfg.cubes.back()[3] = ((double)pX_max - (double)pX_min)*gridcell_m;
        EnvROBARMCfg.cubes.back()[4] = ((double)pY_max - (double)pY_min)*gridcell_m;
        EnvROBARMCfg.cubes.back()[5] = ((double)pZ_max - (double)pZ_min)*gridcell_m;
    }
    else if(!EnvROBARMCfg.lowres_collision_checking)
    {
        Cell2ContXYZ(obs_c[0],obs_c[1],obs_c[2],&(ob[0]),&(ob[1]),&(ob[2]));
        EnvROBARMCfg.cubes.resize(EnvROBARMCfg.cubes.size()+1);
        EnvROBARMCfg.cubes.back().resize(6);
        EnvROBARMCfg.cubes.back()[0] = ob[0];
        EnvROBARMCfg.cubes.back()[1] = ob[1];
        EnvROBARMCfg.cubes.back()[2] = ob[2];
        EnvROBARMCfg.cubes.back()[3] = ((double)pX_max - (double)pX_min)*gridcell_m;
        EnvROBARMCfg.cubes.back()[4] = ((double)pY_max - (double)pY_min)*gridcell_m;
        EnvROBARMCfg.cubes.back()[5] = ((double)pZ_max - (double)pZ_min)*gridcell_m;
    }

    //variable cell costs
    if(EnvROBARMCfg.variable_cell_costs)
    {
        //apply cost to cells close to obstacles
        pX_min -= EnvROBARMCfg.medCostRadius_c;
        pX_max += EnvROBARMCfg.medCostRadius_c;
        pY_min -= EnvROBARMCfg.medCostRadius_c;
        pY_max += EnvROBARMCfg.medCostRadius_c;
        pZ_min -= EnvROBARMCfg.medCostRadius_c;
        pZ_max += EnvROBARMCfg.medCostRadius_c;

        // bounds checking, cutoff object if out of bounds
        if(pX_max > dims_c[0] - 1)
            pX_max = dims_c[0] - 1;
        if (pX_min < 0)
            pX_min = 0;
        if(pY_max > dims_c[1] - 1)
            pY_max = dims_c[1] - 1;
        if (pY_min < 0)
            pY_min = 0;
        if(pZ_max > dims_c[2] - 1)
            pZ_max = dims_c[2] - 1;
        if (pZ_min < 0)
            pZ_min = 0;

        // assign the cells close to the obstacle to medObstacleCost
        for (y = pY_min; y <= pY_max; y++)
        {
            for (x = pX_min; x <= pX_max; x++)
            {
                for (z = pZ_min; z <= pZ_max; z++)
                {
                    if(grid[x][y][z] < EnvROBARMCfg.medObstacleCost)
                    {
                        grid[x][y][z] = EnvROBARMCfg.medObstacleCost;
                    }
                }
            }
        }

        //apply cost to cells less close to obstacles
        pX_min -= EnvROBARMCfg.lowCostRadius_c;
        pX_max += EnvROBARMCfg.lowCostRadius_c;
        pY_min -= EnvROBARMCfg.lowCostRadius_c;
        pY_max += EnvROBARMCfg.lowCostRadius_c;
        pZ_min -= EnvROBARMCfg.lowCostRadius_c;
        pZ_max += EnvROBARMCfg.lowCostRadius_c;

        // bounds checking, cutoff object if out of bounds
        if(pX_max > dims_c[0] - 1)
            pX_max = dims_c[0] - 1;
        if (pX_min < 0)
            pX_min = 0;
        if(pY_max > dims_c[1] - 1)
            pY_max = dims_c[1] - 1;
        if (pY_min < 0)
            pY_min = 0;
        if(pZ_max > dims_c[2] - 1)
            pZ_max = dims_c[2] - 1;
        if (pZ_min < 0)
            pZ_min = 0;

        // assign the cells close to the obstacle to lowObstacleCost
        for (y = pY_min; y <= pY_max; y++)
        {
            for (x = pX_min; x <= pX_max; x++)
            {
                for (z = pZ_min; z <= pZ_max; z++)
                {
                    if(grid[x][y][z] < EnvROBARMCfg.lowObstacleCost)
                    {
                        grid[x][y][z] = EnvROBARMCfg.lowObstacleCost;
                    }
                }
            }
        }
    }
}

//returns 1 if end effector within space, 0 otherwise
int EnvironmentROBARM3D::ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int endeff[3], short unsigned int wrist[3], short unsigned int elbow[3], double orientation[3][3])
{
    num_forwardkinematics++;
    clock_t currenttime = clock();
    double x,y,z;
    int retval = 1;

    //convert angles from positive values in radians (from 0->6.28) to centered around 0 (not really needed)
//     printf("[ComputeEndEffectorPos] angles: ");
    for (int i = 0; i < NUMOFLINKS; i++)
    {
        if(angles[i] >= PI_CONST)
            angles[i] = -2.0*PI_CONST + angles[i];
//         printf("%1.2f  ", angles[i]);
    }
//     printf("\n");

    //use DH or Kinematics Library for forward kinematics
    if(EnvROBARMCfg.use_DH)
    {
        ComputeForwardKinematics_DH(angles);

//         printf("shoulder: %1.4f %1.4f %1.4f\n",EnvROBARMCfg.BaseX_m,EnvROBARMCfg.BaseY_m,EnvROBARMCfg.BaseZ_m);
        //get position of elbow
        x = EnvROBARM.Trans[0][3][ELBOW_JOINT] + EnvROBARMCfg.BaseX_m;
        y = EnvROBARM.Trans[1][3][ELBOW_JOINT] + EnvROBARMCfg.BaseY_m;
        z = EnvROBARM.Trans[2][3][ELBOW_JOINT] + EnvROBARMCfg.BaseZ_m;
//         printf("elbow: %1.4f %1.4f %1.4f\n",x,y,z);
        ContXYZ2Cell(x, y, z, &(elbow[0]), &(elbow[1]), &(elbow[2]));

        //get position of wrist
        x = EnvROBARM.Trans[0][3][WRIST_JOINT] + EnvROBARMCfg.BaseX_m;
        y = EnvROBARM.Trans[1][3][WRIST_JOINT] + EnvROBARMCfg.BaseY_m;
        z = EnvROBARM.Trans[2][3][WRIST_JOINT] + EnvROBARMCfg.BaseZ_m;
//         printf("wrist: %1.4f %1.4f %1.4f\n",x,y,z);
        ContXYZ2Cell(x, y, z, &(wrist[0]), &(wrist[1]), &(wrist[2]));

        //get position of gripper
        x = EnvROBARM.Trans[0][3][ENDEFF] + EnvROBARMCfg.BaseX_m;
        y = EnvROBARM.Trans[1][3][ENDEFF] + EnvROBARMCfg.BaseY_m;
        z = EnvROBARM.Trans[2][3][ENDEFF] + EnvROBARMCfg.BaseZ_m;
//         printf("endeff: %1.4f %1.4f %1.4f\n",x,y,z);
        ContXYZ2Cell(x, y, z, &(endeff[0]),&(endeff[1]),&(endeff[2]));


//         printf("[ComputeEndEffectorPos] endeff: (%u %u %u)  wrist: (%u %u %u) elbow: (%u %u %u)\n",endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);

        // if end effector is out of bounds then return 0
        if(endeff[0] >= EnvROBARMCfg.EnvWidth_c || endeff[1] >= EnvROBARMCfg.EnvHeight_c || endeff[2] >= EnvROBARMCfg.EnvDepth_c)
        {
            printf("[ComputeEndEffectorPos] endeff: (%u %u %u)  wrist: (%u %u %u) elbow: (%u %u %u)\n",endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);
            retval =  0;
        }

        DH_time += clock() - currenttime;
    }
    else    //use Kinematics Library
    {
        //get position of elbow
        ComputeForwardKinematics_ROS(angles, 4, &x, &y, &z);
//         printf("elbow: %1.4f %1.4f %1.4f\n",x,y,z);
        ContXYZ2Cell(x, y, z, &(elbow[0]), &(elbow[1]), &(elbow[2]));

        //get position of wrist
        ComputeForwardKinematics_ROS(angles, 6, &x, &y, &z);
//         printf("wrist: %1.4f %1.4f %1.4f\n",x,y,z);
        ContXYZ2Cell(x, y, z, &(wrist[0]), &(wrist[1]), &(wrist[2]));

        //get position of tip of gripper
        ComputeForwardKinematics_ROS(angles, 7, &x, &y, &z);
//         printf("endeff: %1.4f %1.4f %1.4f\n",x,y,z);
        ContXYZ2Cell(x, y, z, &(endeff[0]), &(endeff[1]), &(endeff[2]));

        // check upper bounds
        if(endeff[0] >= EnvROBARMCfg.EnvWidth_c || endeff[1] >= EnvROBARMCfg.EnvHeight_c || endeff[2] >= EnvROBARMCfg.EnvDepth_c)
            retval =  0;

//         printf("[ComputeEndEffectorPos] endeff: (%u %u %u)  wrist: (%u %u %u) elbow: (%u %u %u)\n",endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);

        KL_time += clock() - currenttime;
    }

    // check if orientation of gripper is upright (for the whole path)
    if(EnvROBARMCfg.enforce_upright_gripper)
    {
        if (EnvROBARM.Trans[2][0][7] < 1.0 - EnvROBARMCfg.gripper_orientation_moe)
            retval = 0;
    }

    //store the orientation of gripper
    if(EnvROBARMCfg.use_DH)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                orientation[i][j] = EnvROBARM.Trans[i][j][ENDEFF];
            }
        }
    }

    return retval;
}

//returns end effector position in meters
int EnvironmentROBARM3D::ComputeEndEffectorPos(double angles[NUMOFLINKS], double endeff_m[3])
{
    int retval = 1;

    //convert angles from positive values in radians (from 0->6.28) to centered around 0
    for (int i = 0; i < NUMOFLINKS; i++)
    {
        if(angles[i] >= PI_CONST)
            angles[i] = -2.0*PI_CONST + angles[i];
    }

    //use DH or Kinematics Library for forward kinematics
    if(EnvROBARMCfg.use_DH)
    {
        ComputeForwardKinematics_DH(angles);

        //get position of tip of gripper
        endeff_m[0] = EnvROBARM.Trans[0][3][ENDEFF] + EnvROBARMCfg.BaseX_m;
        endeff_m[1] = EnvROBARM.Trans[1][3][ENDEFF] + EnvROBARMCfg.BaseY_m;
        endeff_m[2] = EnvROBARM.Trans[2][3][ENDEFF] + EnvROBARMCfg.BaseZ_m;
    }
    else
        ComputeForwardKinematics_ROS(angles, 7, &(endeff_m[0]), &(endeff_m[1]), &(endeff_m[2]));

    return retval;
}

//returns end effector position in cells
int EnvironmentROBARM3D::ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int endeff[3])
{
    int retval = 1;
    double endeff_m[3];

    //convert angles from positive values in radians (from 0->6.28) to centered around 0
    for (int i = 0; i < NUMOFLINKS; i++)
    {
        if(angles[i] >= PI_CONST)
            angles[i] = -2.0*PI_CONST + angles[i];
    }

    //use DH or Kinematics Library for forward kinematics
    if(EnvROBARMCfg.use_DH)
    {
        ComputeForwardKinematics_DH(angles);

        //get position of tip of gripper
        endeff_m[0] = EnvROBARM.Trans[0][3][ENDEFF] + EnvROBARMCfg.BaseX_m;
        endeff_m[1] = EnvROBARM.Trans[1][3][ENDEFF] + EnvROBARMCfg.BaseY_m;
        endeff_m[2] = EnvROBARM.Trans[2][3][ENDEFF] + EnvROBARMCfg.BaseZ_m;
    }
    else
        ComputeForwardKinematics_ROS(angles, 7, &(endeff_m[0]), &(endeff_m[1]), &(endeff_m[2]));

    ContXYZ2Cell(endeff_m[0],endeff_m[1],endeff_m[2], &(endeff[0]),&(endeff[1]),&(endeff[2]));
    return retval;
}

//if pTestedCells is NULL, then the tested points are not saved and it is more
//efficient as it returns as soon as it sees first invalid point
int EnvironmentROBARM3D::IsValidLineSegment(double x0, double y0, double z0, double x1, double y1, double z1, char*** Grid3D, vector<CELLV>* pTestedCells)
{
    bresenham_param_t params;
    int nX, nY, nZ; 
    short unsigned int nX0, nY0, nZ0, nX1, nY1, nZ1;
    int retvalue = 1;
    CELLV tempcell;

    //make sure the line segment is inside the environment
    if(x0 < 0 || x0 >= EnvROBARMCfg.EnvWidth_m || x1 < 0 || x1 >= EnvROBARMCfg.EnvWidth_m ||
       y0 < 0 || y0 >= EnvROBARMCfg.EnvHeight_m || y1 < 0 || y1 >= EnvROBARMCfg.EnvHeight_m ||
       z0 < 0 || z0 >= EnvROBARMCfg.EnvDepth_m || z1 < 0 || z1 >= EnvROBARMCfg.EnvDepth_m)
    {
        return 0;
    }

    ContXYZ2Cell(x0, y0, z0, &nX0, &nY0, &nZ0);
    ContXYZ2Cell(x1, y1, z1, &nX1, &nY1, &nZ1);

    //iterate through the points on the segment
    get_bresenham_parameters3d(nX0, nY0, nZ0, nX1, nY1, nZ1, &params);
    do {
        get_current_point3d(&params, &nX, &nY, &nZ);
        if(Grid3D[nX][nY][nZ] >= EnvROBARMCfg.ObstacleCost)
        {
            if(pTestedCells == NULL)
                return 0;
            else
                retvalue = 0;
        }

            //insert the tested point
        if(pTestedCells)
        {
            tempcell.bIsObstacle = (Grid3D[nX][nY][nZ] >= EnvROBARMCfg.ObstacleCost);
            tempcell.x = nX;
            tempcell.y = nY;
            tempcell.z = nZ;
            pTestedCells->push_back(tempcell);
        }
    } while (get_next_point3d(&params));

    return retvalue;
}

int EnvironmentROBARM3D::IsValidLineSegment(short unsigned int x0, short unsigned int y0, short unsigned int z0, short unsigned int x1, short unsigned int y1, short unsigned int z1, char*** Grid3D, vector<CELLV>* pTestedCells)
{
    bresenham_param_t params;
    int nX, nY, nZ; 
    int retvalue = 1;
    CELLV tempcell;

    short unsigned int width = EnvROBARMCfg.EnvWidth_c;
    short unsigned int height = EnvROBARMCfg.EnvHeight_c;
    short unsigned int depth = EnvROBARMCfg.EnvDepth_c;

    //use low resolution grid if enabled
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        width = EnvROBARMCfg.LowResEnvWidth_c;
        height = EnvROBARMCfg.LowResEnvHeight_c;
        depth = EnvROBARMCfg.LowResEnvDepth_c;
    }

    //make sure the line segment is inside the environment - no need to check < 0
    if(x0 >= width || x1 >= width ||
       y0 >= height || y1 >= height ||
       z0 >= depth || z1 >= depth)
    {
#if DEBUG_CHECK_LINE_SEGMENT
        printf("[IsValidLineSegment] line: (%u %u %u) --> (%u %u %u) is out of bounds.\n",x0,y0,z0,x1,y1,z1);
#endif
        return 0;
    }

//     printf("xyz1: %d %d %d  --> xyz2: %d %d %d\n",x0,y0,z0,x1,y1,z1);

    //iterate through the points on the segment
    get_bresenham_parameters3d(x0, y0, z0, x1, y1, z1, &params);
    do {
        get_current_point3d(&params, &nX, &nY, &nZ);
//         printf("point: %d %d %d\n",nX,nY,nZ);
//         for (int x = nX-10; x < width; x++) 
//         {
//             for (int y = nY-10; y < nY+10; y++)
//                 printf("%d ", Grid3D[x][y][nZ]);
//             printf("\n");
//         }
//         printf("\n");


        if(Grid3D[nX][nY][nZ] >= EnvROBARMCfg.ObstacleCost)
        {
#if DEBUG_CHECK_LINE_SEGMENT
            double pX,pY,pZ;
            Cell2ContXYZ(nX,nY,nZ,&pX,&pY,&pZ, EnvROBARMCfg.GridCellWidth);
            printf("[IsValidLineSegment] point (%u %u %u cells) %.2f %.2f %.2f (meters) is colliding with an obstacle.\n",nX,nY,nZ,pX,pY,pZ);
#endif
            if(pTestedCells == NULL)
                return 0;
            else
                retvalue = 0;
        }

        //insert the tested point
        if(pTestedCells)
        {
            tempcell.bIsObstacle = (Grid3D[nX][nY][nZ] >= EnvROBARMCfg.ObstacleCost);
            tempcell.x = nX;
            tempcell.y = nY;
            tempcell.z = nZ;
            pTestedCells->push_back(tempcell);
        }
    } while (get_next_point3d(&params));

    return retvalue;
}

int EnvironmentROBARM3D::IsValidCoord(short unsigned int coord[NUMOFLINKS], short unsigned int endeff_pos[3], short unsigned int wrist_pos[3], short unsigned int elbow_pos[3], double orientation[3][3])
{
    //for stats
    clock_t currenttime = clock();

    int grid_dims[3] = {EnvROBARMCfg.EnvWidth_c, EnvROBARMCfg.EnvHeight_c, EnvROBARMCfg.EnvDepth_c};
    short unsigned int endeff[3] = {endeff_pos[0],endeff_pos[1],endeff_pos[2]};
    short unsigned int wrist[3] = {wrist_pos[0],wrist_pos[1],wrist_pos[2]};
    short unsigned int elbow[3] = {elbow_pos[0], elbow_pos[1], elbow_pos[2]};
    short unsigned int shoulder[3] = {EnvROBARMCfg.BaseX_c, EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c};

//     double fingertips_g[3] = {0, 0, GRIPPER_LENGTH_M / EnvROBARMCfg.GridCellWidth};
    double fingertips_g[3] = {GRIPPER_LENGTH_M / EnvROBARMCfg.GridCellWidth, 0, 0};
    short unsigned int fingertips_s[3] = {0};

    //use high resolution grid by default
    char*** Grid3D = EnvROBARMCfg.Grid3D;

    double angles[NUMOFLINKS], angles_0[NUMOFLINKS];
    int retvalue = 1;
    vector<CELLV>* pTestedCells = NULL;
    ComputeContAngles(coord, angles);

    //for low resolution collision checking
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        grid_dims[0] = EnvROBARMCfg.LowResEnvWidth_c;
        grid_dims[1] = EnvROBARMCfg.LowResEnvHeight_c;
        grid_dims[2] = EnvROBARMCfg.LowResEnvDepth_c;
        Grid3D = EnvROBARMCfg.LowResGrid3D;

        HighResGrid2LowResGrid(endeff, endeff);
        HighResGrid2LowResGrid(wrist, wrist);
        HighResGrid2LowResGrid(elbow, elbow);
        HighResGrid2LowResGrid(shoulder, shoulder);

//         printf("[IsValidCoord] endeff:(%u,%u,%u) wrist:(%u,%u,%u) elbow: (%u,%u,%u)   \n", 
//                endeff[0], endeff[1], endeff[2],  wrist[0], wrist[1], wrist[2], elbow[0],elbow[1],elbow[2]);

        //NOTE changed gripper to be along X AXIS
        fingertips_g[0] = GRIPPER_LENGTH_M / EnvROBARMCfg.LowResGridCellWidth;
    }

//             printf("[IsValidCoord] endeff:(%u,%u,%u) wrist:(%u,%u,%u) elbow: (%u,%u,%u)   \n", 
//                endeff[0], endeff[1], endeff[2],  wrist[0], wrist[1], wrist[2], elbow[0],elbow[1],elbow[2]);

    // check motor limits
    if(EnvROBARMCfg.enforce_motor_limits)
    {
        //convert angles from positive values in radians (from 0->6.28) to centered around 0
//         printf("[IsValidCoord] angles: ");
        for (int i = 0; i < NUMOFLINKS; i++)
        {
            angles_0[i] = angles[i];
            if(angles[i] >= PI_CONST)
                angles_0[i] = -2.0*PI_CONST + angles[i];

//             printf("%.2f ",angles[i]);
        }
//         printf("\n");

        //shoulder pan - Left is Positive Direction
        if (angles_0[0] > EnvROBARMCfg.PosMotorLimits[0] || angles_0[0] < EnvROBARMCfg.NegMotorLimits[0])
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Shoulder Pan motor limit failed: %2.3f.\n",angles_0[0]);
#endif
            return 0;
        }
        //shoulder pitch - Down is Positive Direction
        if (angles_0[1] > EnvROBARMCfg.PosMotorLimits[1] || angles_0[1] < EnvROBARMCfg.NegMotorLimits[1])
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Shoulder Pitch motor limit failed: %2.3f.\n",angles_0[1]);
#endif
            return 0;
        }
        //upperarm roll
        if (angles_0[2] > EnvROBARMCfg.PosMotorLimits[2] || angles_0[2] < EnvROBARMCfg.NegMotorLimits[2])
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Upperarm Roll motor limit failed: %2.3f.\n",angles_0[2]);;
#endif
            return 0;
        }

        //elbow flex - Down is Positive Direction
        if (angles_0[3] > EnvROBARMCfg.PosMotorLimits[3] || angles_0[3] < EnvROBARMCfg.NegMotorLimits[3])
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Elbow Flex motor limit failed: %2.3f.\n",angles_0[3]);
#endif
            return 0;
        }

//         //forearm roll
//         if (angles_0[4] > EnvROBARMCfg.PosMotorLimits[4] || angles_0[4] < EnvROBARMCfg.NegMotorLimits[4])
//         {
// #if DEBUG_VALID_COORD
//             printf("[IsValidCoord] Forearm Roll motor limit failed: %2.3f.\n",angles_0[4]);
// #endif
//             return 0;
//         }

        //wrist flex - Down is Positive Direction
        if (angles_0[5] > EnvROBARMCfg.PosMotorLimits[5] || angles_0[5] < EnvROBARMCfg.NegMotorLimits[5])
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Wrist Flex motor limit failed: %2.3f.\n",angles_0[5]);
#endif
            return 0;
        }

        //wrist roll
//         if (angles_0[6] > EnvROBARMCfg.PosMotorLimits[6] || angles_0[6] < EnvROBARMCfg.NegMotorLimits[6])
//         {
// #if DEBUG_VALID_COORD
//             printf("[IsValidCoord] Wrist Roll motor limit failed: %2.3f.\n",angles_0[6]);
// #endif
//             return 0;
//         }
    }

    // check if only end effector position is valid
    if (EnvROBARMCfg.endeff_check_only)
    {
        //bounds checking on upper bound (short unsigned int cannot be less than 0, so just check maxes)
        if(endeff[0] >= grid_dims[0] || endeff[1] >= grid_dims[1] || endeff[2] >= grid_dims[2])
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }

        if(pTestedCells)
        {
            CELLV tempcell;
            tempcell.bIsObstacle = Grid3D[endeff[0]][endeff[1]][endeff[2]];
            tempcell.x = endeff[0];
            tempcell.y = endeff[1];
            tempcell.z = endeff[2];

            pTestedCells->push_back(tempcell);
        }

        //check end effector is not hitting obstacle
        if(Grid3D[endeff[0]][endeff[1]][endeff[2]] >= EnvROBARMCfg.ObstacleCost)
        {
            check_collision_time += clock() - currenttime;
            return 0;
        }
    }

    // check if full arm with object in gripper are valid as well
    else 
    {
        //bounds checking on upper bound of map - should really only check end effector not other joints
        if(endeff[0] >= grid_dims[0] || endeff[1] >= grid_dims[1] || endeff[2] >= grid_dims[2] ||
           wrist[0] >= grid_dims[0] || wrist[1] >= grid_dims[1] || wrist[2] >= grid_dims[2] ||
           elbow[0] >= grid_dims[0] || elbow[1] >= grid_dims[1] || elbow[2] >= grid_dims[2])
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Arm joint bounds checking failed.\n");
#endif
            check_collision_time += clock() - currenttime;
            return 0;
        }

        //check if joints are hitting obstacle - is not needed because IsValidLineSegment checks it
        if(Grid3D[endeff[0]][endeff[1]][endeff[2]] >= EnvROBARMCfg.ObstacleCost ||
           Grid3D[wrist[0]][wrist[1]][wrist[2]] >= EnvROBARMCfg.ObstacleCost ||
           Grid3D[elbow[0]][elbow[1]][elbow[2]] >= EnvROBARMCfg.ObstacleCost)
        {
#if DEBUG_VALID_COORD
            printf("[IsValidCoord] Arm joint collision checking failed.\n");
#endif
            check_collision_time += clock() - currenttime;
            return 0;
        }

//        //check the validity of the corresponding arm links (line segments)
//         if(!IsValidLineSegment(shoulder[0],shoulder[1],shoulder[2], elbow[0],elbow[1],elbow[2], Grid3D, pTestedCells) ||
//             !IsValidLineSegment(elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2], Grid3D, pTestedCells) ||
//             !IsValidLineSegment(wrist[0],wrist[1],wrist[2],endeff[0],endeff[1], endeff[2], Grid3D, pTestedCells))
//         {
//             if(pTestedCells == NULL)
//             {
// #if DEBUG_VALID_COORD
//                 printf("[IsValidCoord] shoulder: (%i %i %i)\n",shoulder[0],shoulder[1],shoulder[2]);
//                 printf("[IsValidCoord] elbow: (%i %i %i)\n",elbow[0],elbow[1],elbow[2]);
//                 printf("[IsValidCoord] wrist: (%i %i %i)\n",wrist[0],wrist[1],wrist[2]);
//                 printf("[IsValidCoord] endeff: (%i %i %i)\n",endeff[0],endeff[1],endeff[2]);
//                 printf("[IsValidCoord] Arm link collision checking failed.\n");
// #endif
//                 check_collision_time += clock() - currenttime;
//                 return 0;
//             }
//             else
//             {
// #if DEBUG_VALID_COORD
//                 printf("[IsValidCoord] Arm link collision checking failed..\n");
// #endif
//                 retvalue = 0;
//             }
//         }


        if(!IsValidLineSegment(shoulder[0],shoulder[1],shoulder[2], elbow[0],elbow[1],elbow[2], Grid3D, pTestedCells))
        {
            if(pTestedCells == NULL)
            {
#if DEBUG_VALID_COORD
                printf("[IsValidCoord] Upperarm collision checking failed (%d %d %d) -- > (%d %d %d).\n",shoulder[0],shoulder[1],shoulder[2],elbow[0],elbow[1],elbow[2]);
#endif
                check_collision_time += clock() - currenttime;
                return 0;
            }
            else
            {
#if DEBUG_VALID_COORD
                printf("[IsValidCoord] Upperarm collision checking failed (%d %d %d) -- > (%d %d %d).\n",shoulder[0],shoulder[1],shoulder[2],elbow[0],elbow[1],elbow[2]);
#endif
                retvalue = 0;
            }
        }
        pTestedCells = NULL;
        if(!IsValidLineSegment(elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2], Grid3D, pTestedCells))
        {
            if(pTestedCells == NULL)
            {
#if DEBUG_VALID_COORD
                printf("[IsValidCoord] Forearm collision checking failed (%d %d %d) -- > (%d %d %d).\n",elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2]);
#endif
                check_collision_time += clock() - currenttime;
                return 0;
            }
            else
            {
#if DEBUG_VALID_COORD
                printf("[IsValidCoord] Forearm collision checking failed (%d %d %d) -- > (%d %d %d).\n",elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2]);
#endif
                retvalue = 0;
            }
        }
        pTestedCells = NULL;
        if(!IsValidLineSegment(wrist[0],wrist[1],wrist[2],endeff[0],endeff[1],endeff[2], Grid3D, pTestedCells))
        {
            if(pTestedCells == NULL)
            {
#if DEBUG_VALID_COORD
                printf("[IsValidCoord] end effector collision checking failed (%d %d %d) -- > (%d %d %d).\n",wrist[0],wrist[1],wrist[2],endeff[0],endeff[1],endeff[2]);
#endif
                check_collision_time += clock() - currenttime;
                return 0;
            }
            else
            {
#if DEBUG_VALID_COORD
                printf("[IsValidCoord] end effector collision checking failed (%d %d %d) -- > (%d %d %d).\n",wrist[0],wrist[1],wrist[2],endeff[0],endeff[1],endeff[2]);
#endif
                retvalue = 0;
            }
        }

//         //check if gripper is clear of obstacles
//         if(EnvROBARMCfg.use_DH)
//         {
//             //get position of fingertips in shoulder frame (assuming gripper is closed)
//             //P_objectInshoulder = R_gripperToshoulder * P_objectIngripper + P_gripperInshoulder
//             fingertips_s[0] = (orientation[0][0]*fingertips_g[0] + orientation[0][1]*fingertips_g[1] + orientation[0][2]*fingertips_g[2]) + endeff[0];
//             fingertips_s[1] = (orientation[1][0]*fingertips_g[0] + orientation[1][1]*fingertips_g[1] + orientation[1][2]*fingertips_g[2]) + endeff[1];
//             fingertips_s[2] = (orientation[2][0]*fingertips_g[0] + orientation[2][1]*fingertips_g[1] + orientation[2][2]*fingertips_g[2]) + endeff[2];
// 
//             pTestedCells = NULL;
//             if(!IsValidLineSegment(endeff[0],endeff[1],endeff[2],fingertips_s[0], fingertips_s[1],fingertips_s[2], Grid3D, pTestedCells))
//             {
//                 if(pTestedCells == NULL)
//                 {
// #if DEBUG_VALID_COORD
//                     printf("[IsValidCoord] fingertips collision checking failed (%d %d %d) -- > (%d %d %d).\n",endeff[0],endeff[1],endeff[2],fingertips_s[0], fingertips_s[1],fingertips_s[2]);
// #endif
//                     check_collision_time += clock() - currenttime;
//                     return 0;
//                 }
//                 else
//                 {
// #if DEBUG_VALID_COORD
//                     printf("[IsValidCoord] fingertips collision checking failed (%d %d %d) -- > (%d %d %d).\n",endeff[0],endeff[1],endeff[2],fingertips_s[0], fingertips_s[1],fingertips_s[2]);
// #endif
//                     retvalue = 0;
//                 }
//             }
//         }

//         printf("[IsValidCoord] elbow: (%u,%u,%u)  wrist:(%u,%u,%u) endeff:(%u,%u,%u) fingertips: (%u %u %u)\n", elbow[0],elbow[1],elbow[2],
//                     wrist[0], wrist[1], wrist[2], endeff[0], endeff[1], endeff[2], fingertips_s[0], fingertips_s[1], fingertips_s[2]);

        //check line segment of object in gripper for collision
        if(EnvROBARMCfg.object_grasped)
        {
            double objectAbove_g[3] = {0}, objectBelow_g[3] = {0};
            short unsigned int objectAbove_s[3], objectBelow_s[3]; //these types are wrong

            //the object is along the gripper's x-axis
            objectBelow_g[0] = -EnvROBARMCfg.grasped_object_length_m / EnvROBARMCfg.GridCellWidth;

            //for now the gripper is gripping the top of the cylindrical object 
            //which means, the end effector is the bottom point of the object

            //get position of one end of object in shoulder frame
            //P_objectInshoulder = R_gripperToshoulder * P_objectIngripper + P_gripperInshoulder
            objectAbove_s[0] = (orientation[0][0]*objectAbove_g[0] + orientation[0][1]*objectAbove_g[1] + orientation[0][2]* objectAbove_g[2]) + endeff[0];
            objectAbove_s[1] = (orientation[1][0]*objectAbove_g[0] + orientation[1][1]*objectAbove_g[1] + orientation[1][2]* objectAbove_g[2]) + endeff[1];
            objectAbove_s[2] = (orientation[2][0]*objectAbove_g[0] + orientation[2][1]*objectAbove_g[1] + orientation[2][2]* objectAbove_g[2]) + endeff[2];

            objectBelow_s[0] = (orientation[0][0]*objectBelow_g[0] + orientation[0][1]*objectBelow_g[1] + orientation[0][2]* objectBelow_g[2]) + endeff[0];
            objectBelow_s[1] = (orientation[1][0]*objectBelow_g[0] + orientation[1][1]*objectBelow_g[1] + orientation[1][2]* objectBelow_g[2]) + endeff[1];
            objectBelow_s[2] = (orientation[2][0]*objectBelow_g[0] + orientation[2][1]*objectBelow_g[1] + orientation[2][2]* objectBelow_g[2]) + endeff[2];

//             printf("[IsValidCoord] objectAbove:(%.0f %.0f %.0f) objectBelow: (%.0f %.0f %.0f)\n",objectAbove_g[0], objectAbove_g[1], objectAbove_g[2], objectBelow_g[0], objectBelow_g[1], objectBelow_g[2]);
//             printf("[IsValidCoord] objectAbove:(%u %u %u) objectBelow: (%u %u %u)\n",objectAbove_s[0], objectAbove_s[1], objectAbove_s[2], objectBelow_s[0], objectBelow_s[1], objectBelow_s[2]);

            //check if the line is a valid line segment
            pTestedCells = NULL;
            if(!IsValidLineSegment(objectBelow_s[0],objectBelow_s[1],objectBelow_s[2],objectAbove_s[0], objectAbove_s[1],objectAbove_s[2], Grid3D, pTestedCells))
            {
                if(pTestedCells == NULL)
                {
                    check_collision_time += clock() - currenttime;
                    return 0;
                }
                else
                {
                    retvalue = 0;
                }
            }
            //printf("[IsValidCoord] elbow: (%u,%u,%u)  wrist:(%u,%u,%u) endeff:(%u,%u,%u) object: (%u %u %u)\n", elbow[0],elbow[1],elbow[2],
            //        wrist[0],arm-> wrist[1], wrist[2], endeff[0], endeff[1], endeff[2], objectBelow_s[0], objectBelow_s[1], objectBelow_s[2]);
        }

    }

    check_collision_time += clock() - currenttime;
    return retvalue;
}

int EnvironmentROBARM3D::cost(short unsigned int state1coord[], short unsigned int state2coord[])
{
//     if(!IsValidCoord(state1coord) || !IsValidCoord(state2coord))
//         return INFINITECOST;

#if UNIFORM_COST
    return 1*COSTMULT;
#else
//     printf("%i\n",1 * COSTMULT * (EnvROBARMCfg.Grid3D[HashEntry2->endeff[0]][HashEntry1->endeff[1]][HashEntry1->endeff[2]] + 1));
    //temporary
    return 1*COSTMULT;

#endif
}

int EnvironmentROBARM3D::cost(EnvROBARMHashEntry_t* HashEntry1, EnvROBARMHashEntry_t* HashEntry2, bool bState2IsGoal)
{
    //why does the goal return as invalid from IsValidCoord?
//     if (!bState2IsGoal)
//     {
//         if(!IsValidCoord(HashEntry1->coord,HashEntry1) || !IsValidCoord(HashEntry2->coord,HashEntry2))
//             return INFINITECOST;
//     }
//     else
//     {
//         if(!IsValidCoord(HashEntry1->coord,HashEntry1))
//             return INFINITECOST;
//     }

#if UNIFORM_COST
    return 1*COSTMULT;
#else
//     printf("%i\n",1 * COSTMULT * (EnvROBARMCfg.Grid3D[HashEntry2->endeff[0]][HashEntry1->endeff[1]][HashEntry1->endeff[2]] + 1));
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        short unsigned int endeff[3];
        HighResGrid2LowResGrid(HashEntry2->endeff, endeff);
        return 1 * COSTMULT * (EnvROBARMCfg.LowResGrid3D[endeff[0]][endeff[1]][endeff[2]] + 1);
    }
    else
        return 1 * COSTMULT * (EnvROBARMCfg.Grid3D[HashEntry2->endeff[0]][HashEntry1->endeff[1]][HashEntry1->endeff[2]] + 1);
#endif
}

bool EnvironmentROBARM3D::RemoveGoal(int goal_index)
{
    int retval = 1;

    if(EnvROBARMCfg.PlanInJointSpace)
    {
        //check if there is more then one goal
        if(EnvROBARMCfg.nJointSpaceGoals <= 1)
        {
            EnvROBARMCfg.nJointSpaceGoals = 0;
            EnvROBARMCfg.bGoalIsSet = false;
            return 0;
        }

        //if not the last goal in the list, overwrite the goal to be removed by the last goal
        if(goal_index  < EnvROBARMCfg.nJointSpaceGoals-1)
        {
            for(int i = 0; i < NUMOFLINKS; i++)
            {
                EnvROBARMCfg.JointSpaceGoals[goal_index][i] = EnvROBARMCfg.JointSpaceGoals[EnvROBARMCfg.nJointSpaceGoals-1][i];
            }
        }

        //delete last goal in list
        delete [] EnvROBARMCfg.JointSpaceGoals[EnvROBARMCfg.nJointSpaceGoals-1];

        //update number of goals
        EnvROBARMCfg.nJointSpaceGoals--;
    }
    else
    {
        //check if there is more then one goal
        if(EnvROBARMCfg.nEndEffGoals <= 1)
        {
            EnvROBARMCfg.nEndEffGoals = 0;
            EnvROBARMCfg.bGoalIsSet = false;
            return 0;
        }

        //if not the last goal in the list, overwrite the goal to be removed by the last goal
        if(goal_index  < EnvROBARMCfg.nEndEffGoals-1)
        {
            for(int i = 0; i < 6; i++)
            {
                EnvROBARMCfg.EndEffGoals_m[goal_index][i] = EnvROBARMCfg.EndEffGoals_m[EnvROBARMCfg.nEndEffGoals-1][i];
                if(i < 3)
                {
                    EnvROBARMCfg.EndEffGoals_c[goal_index][i] = EnvROBARMCfg.EndEffGoals_c[EnvROBARMCfg.nEndEffGoals-1][i];
                    EnvROBARMCfg.EndEffGoalRPY[goal_index][i] = EnvROBARMCfg.EndEffGoalRPY[EnvROBARMCfg.nEndEffGoals-1][i];
                }
            }
        }

        //delete last goal in list
        delete [] EnvROBARMCfg.EndEffGoals_m[EnvROBARMCfg.nEndEffGoals-1];
        delete [] EnvROBARMCfg.EndEffGoals_c[EnvROBARMCfg.nEndEffGoals-1];
        delete [] EnvROBARMCfg.EndEffGoalRPY[EnvROBARMCfg.nEndEffGoals-1];

        //update number of goals
        EnvROBARMCfg.nEndEffGoals--;
    }

    return retval;
}

void EnvironmentROBARM3D::UpdateEnvironment()
{
    int x, y, z;
    int b = 0, c = 0;
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            for (z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
            {
                EnvROBARMCfg.Grid3D[x][y][z] = EnvROBARMCfg.Grid3D_temp[x][y][z];

                if(EnvROBARMCfg.Grid3D[x][y][z] > 0)
                    c++;
            }
        }
    }

    if(EnvROBARMCfg.lowres_collision_checking)
    {
        for (x = 0; x < EnvROBARMCfg.LowResEnvWidth_c; x++)
        {
            for (y = 0; y < EnvROBARMCfg.LowResEnvHeight_c; y++)
            {
                for (z = 0; z < EnvROBARMCfg.LowResEnvDepth_c; z++)
                {
                    EnvROBARMCfg.LowResGrid3D[x][y][z] = EnvROBARMCfg.LowResGrid3D_temp[x][y][z];

                    if(EnvROBARMCfg.LowResGrid3D[x][y][z] > 0)
                        b++;
                }
            }
        }
    }

    printf("[UpdateEnvironment] There are %d obstacle cells in the low resolution planning grid.\n",b);
    printf("[UpdateEnvironment] There are %d obstacle cells in the high resolution planning grid.\n",c);
}

void EnvironmentROBARM3D::InitializeEnvConfig()
{
    //find the discretization for each angle and store the discretization
    DiscretizeAngles();
}

bool EnvironmentROBARM3D::InitializeEnvironment()
{
    short unsigned int coord[NUMOFLINKS] = {0};
    short unsigned int endeff[3] = {0};
    double orientation[3][3];

    //initialize the map from Coord to StateID
    EnvROBARM.HashTableSize = 32*1024; //should be power of two
    EnvROBARM.Coord2StateIDHashTable = new std::vector<EnvROBARMHashEntry_t*>[EnvROBARM.HashTableSize];

    //initialize the map from StateID to Coord
    EnvROBARM.StateID2CoordTable.clear();

    //create an empty start state
    EnvROBARM.startHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeff, 0, orientation);

    //initialize the angles of the start states
    if(!SetStartJointConfig(EnvROBARMCfg.LinkStartAngles_d, 0))
        printf("[InitializeEnvironment] Start state was not created.\n");

    //create the goal state
    EnvROBARM.goalHashEntry = CreateNewHashEntry(coord, NUMOFLINKS, endeff, 0, NULL);

    //create the goal state
    if(EnvROBARMCfg.bGoalIsSet)
    {
        EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[0][0];
        EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[0][1];
        EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[0][2];
    }

    //for now heuristics are not set
    EnvROBARM.Heur = NULL;

    return true;
}
//----------------------------------------------------------------------


/*------------------------------------------------------------------------*/
                /* Interface with Outside Functions */
/*------------------------------------------------------------------------*/
EnvironmentROBARM3D::EnvironmentROBARM3D()
{
    /* FK & COLLISION CHECKING */
    EnvROBARMCfg.bInitialized = false;
    EnvROBARMCfg.use_DH = 1;
    EnvROBARMCfg.enforce_motor_limits = 1;
    EnvROBARMCfg.endeff_check_only = 0;
    EnvROBARMCfg.object_grasped = 0;
    EnvROBARMCfg.grasped_object_length_m = .1;
    EnvROBARMCfg.enforce_upright_gripper = 0;

    /* SUCCESSOR ACTIONS */
    EnvROBARMCfg.use_smooth_actions = 1;
    EnvROBARMCfg.smoothing_weight = 0.0;
    EnvROBARMCfg.HighResActionsThreshold_c = 2;
    EnvROBARMCfg.lowres_collision_checking = 0;
    EnvROBARMCfg.multires_succ_actions = 1;

    /* HEURISTIC */
    EnvROBARMCfg.dijkstra_heuristic = 1;
    EnvROBARMCfg.ApplyRPYCost_m = .1;
    EnvROBARMCfg.dual_heuristics = 0;

    /* GOALS */
    EnvROBARMCfg.bGoalIsSet = false;
    EnvROBARMCfg.gripper_orientation_moe = .0175; // 0.0125;
    EnvROBARMCfg.goal_moe_m = .1;
    EnvROBARMCfg.checkEndEffGoalOrientation = 0;
    EnvROBARMCfg.JointSpaceGoal = 0;
    EnvROBARMCfg.nEndEffGoals = 0;
    EnvROBARMCfg.GoalRPY_MOE[0] = 1;
    EnvROBARMCfg.GoalRPY_MOE[1] = 1;
    EnvROBARMCfg.GoalRPY_MOE[2] = 1;
    EnvROBARMCfg.uninformative_heur_dist_m = .04;

    /* COSTS & COST MAP */
    EnvROBARMCfg.variable_cell_costs = false;
    EnvROBARMCfg.padding = 0.04;
    EnvROBARMCfg.ObstacleCost = 1;  //TODO Change this to 10
    EnvROBARMCfg.medObstacleCost = 1;
    EnvROBARMCfg.lowObstacleCost = 1;
    EnvROBARMCfg.medCostRadius_c = 1;
    EnvROBARMCfg.lowCostRadius_c = 1;
    EnvROBARMCfg.CellsPerAction = -1;

    /* ENVIRONMENT */
    EnvROBARMCfg.GridCellWidth = .01;   //default map has 1cm resolution
    EnvROBARMCfg.LowResGridCellWidth = 2 * EnvROBARMCfg.GridCellWidth;

    //default map size is geared towards a planted 1 meter long robotic arm
    EnvROBARMCfg.EnvWidth_m = 1.4;
    EnvROBARMCfg.EnvHeight_m = 2;
    EnvROBARMCfg.EnvDepth_m = 2;

    EnvROBARMCfg.EnvWidth_c = EnvROBARMCfg.EnvWidth_m / EnvROBARMCfg.GridCellWidth;
    EnvROBARMCfg.EnvHeight_c = EnvROBARMCfg.EnvHeight_m / EnvROBARMCfg.GridCellWidth;
    EnvROBARMCfg.EnvDepth_c = EnvROBARMCfg.EnvDepth_m / EnvROBARMCfg.GridCellWidth;

    //Low-Res Environment for Colision Checking
    EnvROBARMCfg.LowResEnvWidth_c = EnvROBARMCfg.EnvWidth_m / EnvROBARMCfg.LowResGridCellWidth;
    EnvROBARMCfg.LowResEnvHeight_c = EnvROBARMCfg.EnvHeight_m / EnvROBARMCfg.LowResGridCellWidth;
    EnvROBARMCfg.LowResEnvDepth_c = EnvROBARMCfg.EnvDepth_m / EnvROBARMCfg.LowResGridCellWidth;

    //base of arm is usually at center of environment
    EnvROBARMCfg.BaseX_m = 0;
    EnvROBARMCfg.BaseY_m = 0;
    EnvROBARMCfg.BaseZ_m = 0;
    ContXYZ2Cell(EnvROBARMCfg.BaseX_m, EnvROBARMCfg.BaseY_m, EnvROBARMCfg.BaseZ_m, &(EnvROBARMCfg.BaseX_c), &(EnvROBARMCfg.BaseY_c), &(EnvROBARMCfg.BaseZ_c));

    /* JOINT SPACE PLANNING */
    EnvROBARMCfg.PlanInJointSpace = false;
    for (int i = 0; i < NUMOFLINKS; i++)
        EnvROBARMCfg.JointSpaceGoalMOE[i] = 0.18;

    //hard coded temporarily (meters)
    EnvROBARMCfg.arm_length = ARM_WORKSPACE;

    EnvROBARMCfg.bPlanning  = false;
    EnvROBARMCfg.bUpdatePlanningGrid = false;
    
    
//     EnvROBARMCfg.bigcubes.resize(1,6);
//     EnvROBARMCfg.cubes.resize(0,6);
}

bool EnvironmentROBARM3D::InitializeEnv(const char* sEnvFile)
{

    //parse the parameter file - temporary
    char parFile[] = "params.cfg";
    FILE* fCfg = fopen(parFile, "r");
    if(fCfg == NULL)
        printf("ERROR: unable to open %s.....using defaults.\n", parFile);

    ReadParamsFile(fCfg);
    fclose(fCfg);

    //parse the configuration file
    fCfg = fopen(sEnvFile, "r");
    if(fCfg == NULL)
    {
        printf("ERROR: unable to open %s\n", sEnvFile);
        exit(1);
    }
    ReadConfiguration(fCfg);
    fclose(fCfg);

    //initialize forward kinematics
    if (!EnvROBARMCfg.use_DH)
        //start ros node
        InitializeKinNode();
    else
        //pre-compute DH Transformations
        ComputeDHTransformations();

    //if goal is in joint space, do the FK on the goal joint angles
    if(EnvROBARMCfg.JointSpaceGoal)
    {
        short unsigned int wrist[3],elbow[3],endeff[3];
        double goalangles[NUMOFLINKS];
        double orientation[3][3];

        // convert goal angles to radians
        for(int i = 0; i < NUMOFLINKS; i++)
            goalangles[i] = PI_CONST*(EnvROBARMCfg.LinkGoalAngles_d[i]/180.0);

        ComputeEndEffectorPos(goalangles, endeff, wrist, elbow, orientation);
        EnvROBARMCfg.EndEffGoalX_c = endeff[0];
        EnvROBARMCfg.EndEffGoalY_c = endeff[1];
        EnvROBARMCfg.EndEffGoalZ_c = endeff[2];
    }

    //initialize other parameters of the environment
    InitializeEnvConfig();

    //initialize Environment
    if(InitializeEnvironment() == false)
        return false;

//     vector<CELLV>* pTestedCells = NULL;
//     short unsigned int x0,y0,z0,x1,y1,z1;
//     x0 = 18;
//     y0 = 5;
//     z0 = 68;
//     x1 = 32;
//     y1 = 96;
//     z1 = 4; 
//     IsValidLineSegment(x0,  y0, z0, x1, y1, z1, EnvROBARMCfg.Grid3D, pTestedCells);
//     exit(1);

    //pre-compute action-to-action costs
    ComputeActionCosts();

    //compute the cost per cell to be used by heuristic
    ComputeCostPerCell();

    //set goals
    if(EnvROBARMCfg.PlanInJointSpace)
    {
        if(!SetJointSpaceGoals(EnvROBARMCfg.JointSpaceGoals, EnvROBARMCfg.nJointSpaceGoals))
        {
            printf("Failed to set joint space goals.\n");
            exit(1);
        }
    }
    else
    {
        SetEndEffGoals(EnvROBARMCfg.EndEffGoals_m, 0, EnvROBARMCfg.nEndEffGoals,1);
        EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[0][0];
        EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[0][1];
        EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[0][2];
        ComputeCostPerRadian();
    }

#if VERBOSE
    //output environment data
    PrintConfiguration();

    printf("Use DH Convention for FK: %i\n", EnvROBARMCfg.use_DH);
    printf("Enforce Motor Limits: %i\n", EnvROBARMCfg.enforce_motor_limits);
    printf("Use Dijkstra for Heuristic Function: %i\n", EnvROBARMCfg.dijkstra_heuristic);
    printf("EndEffector Collision Check Only: %i\n", EnvROBARMCfg.endeff_check_only);
    printf("Smoothness Weight: %.3f\n", EnvROBARMCfg.smoothing_weight);
    printf("Obstacle Padding(meters): %.2f\n", EnvROBARMCfg.padding);
    printf("\n");

    //output successor actions
    OutputActions();

    //output action costs
    if(EnvROBARMCfg.use_smooth_actions)
        OutputActionCostTable();
#endif

    //set Environment is Initialized flag(so fk can be used)
    EnvROBARMCfg.bInitialized = 1;

    return true;
}

bool EnvironmentROBARM3D::InitEnvFromFilePtr(FILE* eCfg, FILE* pCfg)
{
    //parse the parameter file
    ReadParamsFile(pCfg);

    //parse the environment configuration file
    ReadConfiguration(eCfg);

    //initialize forward kinematics
    if (!EnvROBARMCfg.use_DH)
        //start ros node
        InitializeKinNode();
    else
        //pre-compute DH Transformations
        ComputeDHTransformations();

    //if goal is in joint space, do the FK on the goal joint angles
    if(EnvROBARMCfg.JointSpaceGoal)
    {
        short unsigned int wrist[3],elbow[3],endeff[3];
        double goalangles[NUMOFLINKS];
        double orientation[3][3];

        // convert goal angles to radians
        for(int i = 0; i < NUMOFLINKS; i++)
            goalangles[i] = PI_CONST*(EnvROBARMCfg.LinkGoalAngles_d[i]/180.0);

        ComputeEndEffectorPos(goalangles, endeff, wrist, elbow, orientation);
        EnvROBARMCfg.EndEffGoalX_c = endeff[0];
        EnvROBARMCfg.EndEffGoalY_c = endeff[1];
        EnvROBARMCfg.EndEffGoalZ_c = endeff[2];
    }

    //initialize other parameters of the environment
    InitializeEnvConfig();

    //initialize Environment
    if(InitializeEnvironment() == false)
        return false;

    //pre-compute action-to-action costs
    ComputeActionCosts();

    //compute the cost per cell to be used by heuristic
    ComputeCostPerCell();

//     ComputeCostPerRadian();

    EnvROBARMCfg.bPlanning = false;
    EnvROBARMCfg.bUpdatePlanningGrid = false;
        
#if VERBOSE
    //output environment data
    PrintConfiguration();

    printf("Use DH Convention for FK: %i\n", EnvROBARMCfg.use_DH);
    printf("Enforce Motor Limits: %i\n", EnvROBARMCfg.enforce_motor_limits);
    printf("Use Dijkstra for Heuristic Function: %i\n", EnvROBARMCfg.dijkstra_heuristic);
    printf("EndEffector Collision Check Only: %i\n", EnvROBARMCfg.endeff_check_only);
    printf("Smoothness Weight: %.3f\n", EnvROBARMCfg.smoothing_weight);
    printf("Obstacle Padding(meters): %.2f\n", EnvROBARMCfg.padding);
    printf("\n");

    //output successor actions
    OutputActions();

    //output action costs
    if(EnvROBARMCfg.use_smooth_actions)
        OutputActionCostTable();
#endif

    //set Environment is Initialized flag(so fk can be used)
    EnvROBARMCfg.bInitialized = 1;

    printf("[InitEnvFromFilePtr] Environment has been initialized.\n");
    return true;
}

bool EnvironmentROBARM3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
	//initialize MDPCfg with the start and goal ids	
    MDPCfg->goalstateid = EnvROBARM.goalHashEntry->stateID;
    MDPCfg->startstateid = EnvROBARM.startHashEntry->stateID;

    return true;
}

int EnvironmentROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    if(EnvROBARMCfg.PlanInJointSpace)
        return GetJointSpaceHeuristic(FromStateID,ToStateID);

    int h, closest_goal;

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

    //Dijkstra's algorithm
    if(EnvROBARMCfg.dijkstra_heuristic)
    {
        //lowres collision checking
        if(EnvROBARMCfg.lowres_collision_checking)
        {
            short unsigned int endeff[3];
            HighResGrid2LowResGrid(FromHashEntry->endeff, endeff);
            h = EnvROBARM.Heur[XYZTO3DIND(endeff[0], endeff[1], endeff[2])];

            double dist_to_goal_c = GetDistToClosestGoal(FromHashEntry->endeff, &closest_goal);

            //if the distance to the goal is very small, then we need a more informative heuristic when using the lowres collision detection
            if(dist_to_goal_c * EnvROBARMCfg.GridCellWidth < MAX_EUCL_DIJK_m)
            {
                double h_eucl_cost = dist_to_goal_c * EnvROBARMCfg.cost_per_cell;

                if (h_eucl_cost > h)
                    h =  h_eucl_cost;
            }
        }
        //highres collision checking
        else
            h = EnvROBARM.Heur[XYZTO3DIND(FromHashEntry->endeff[0], FromHashEntry->endeff[1], FromHashEntry->endeff[2])];
    }

    //euclidean distance
    else
    {
        h = GetDistToClosestGoal(FromHashEntry->endeff, &closest_goal) * EnvROBARMCfg.cost_per_cell;
    }

//     printf("GetFromToHeuristic(#%i(%i,%i,%i) -> #%i(%i,%i,%i)) H cost: %i\n",FromStateID, FromHashEntry->endeff[0], FromHashEntry->endeff[1], FromHashEntry->endeff[2], ToStateID, ToHashEntry->endeff[0], ToHashEntry->endeff[1], ToHashEntry->endeff[2], h);
    return h;
}

//this function should be optimized once a heuristic function is chosen - it is intended for testing
int EnvironmentROBARM3D::GetFromToHeuristic(int FromStateID, int ToStateID, double FromRPY[3], double ToRPY[3])
{
#if USE_HEUR==0
    return 0;
#endif

    int h = 0,closest_goal;
    double /*rdiff,*/ pdiff,ydiff, ang_eucl_dist;
    double edist_to_goal_m;

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
    short unsigned int FromEndEff[3] = {FromHashEntry->endeff[0], FromHashEntry->endeff[1], FromHashEntry->endeff[2]};

    if(EnvROBARMCfg.lowres_collision_checking)
    {
        HighResGrid2LowResGrid(FromHashEntry->endeff, FromEndEff);
    }

    //endeff is close enough to goal & dual-heuristics are enabled
    if(EnvROBARMCfg.dual_heuristics && EnvROBARMCfg.checkEndEffGoalOrientation)
    {
        //distance to closest goal in meters
        edist_to_goal_m = GetDistToClosestGoal(FromHashEntry->endeff, &closest_goal) * EnvROBARMCfg.GridCellWidth;

        //get heuristic
        if(EnvROBARMCfg.dijkstra_heuristic)
        {
            h = EnvROBARM.Heur[XYZTO3DIND(FromEndEff[0], FromEndEff[1], FromEndEff[2])];
        }
        else
        {
            h = edist_to_goal_m * 1000.0 * EnvROBARMCfg.cost_per_mm;
        }

        //if close enough to goal, factor in orientation
        if(edist_to_goal_m < EnvROBARMCfg.ApplyRPYCost_m)
        {
//             rdiff = fabs(angles::shortest_angular_distance(FromRPY[0], EnvROBARMCfg.EndEffGoalRPY[closest_goal][0]));
            pdiff = fabs(angles::shortest_angular_distance(FromRPY[1], EnvROBARMCfg.EndEffGoalRPY[closest_goal][1]));
            ydiff = fabs(angles::shortest_angular_distance(FromRPY[2], EnvROBARMCfg.EndEffGoalRPY[closest_goal][2]));
//             ang_eucl_dist = sqrt(rdiff*rdiff + pdiff*pdiff + ydiff*ydiff);

            //assume you can fix the roll at the end
            ang_eucl_dist = sqrt(pdiff*pdiff + ydiff*ydiff);

            int h_ang_eucl_cost = ang_eucl_dist * EnvROBARMCfg.cost_per_rad;


            //try adding them together 
//             h += h_ang_eucl_cost;

            printf("xyzdist: %.3f  rpydist: %.3f  h_dij: %i  h_eucl: %i closest_goal: %i\n",
                   edist_to_goal_m, ang_eucl_dist, h, h_ang_eucl_cost,closest_goal);

            //use max(heuristic_xyz, heuristic_rpy)
            if(h_ang_eucl_cost > h)
            {
                h = h_ang_eucl_cost;
            }

//             printf("pitch1: %.3f yaw1: %.3f pitch2: %.3f yaw2: %.3f pdiff: %.3f ydiff: %.3f\n",FromRPY[1],FromRPY[2],ToRPY[1],ToRPY[2], pdiff,ydiff);
        }
    }

    //either not close enough to goal or dual-heuristics is disabled
    else
    {
        //dijkstra's
        if(EnvROBARMCfg.dijkstra_heuristic)
        {
            h = EnvROBARM.Heur[XYZTO3DIND(FromEndEff[0], FromEndEff[1], FromEndEff[2])];

            double dist_to_goal_c = GetDistToClosestGoal(FromEndEff,&closest_goal);

            //if the distance to the goal is very small, then we need a more informative heuristic when using the lowres collision detection
            if(dist_to_goal_c * EnvROBARMCfg.GridCellWidth < MAX_EUCL_DIJK_m)
            {
                double h_eucl_cost = dist_to_goal_c * EnvROBARMCfg.cost_per_cell;

                if (h_eucl_cost > h)
                    h =  h_eucl_cost;
            }
        }
        //euclidean
        else
        {
            h = GetDistToClosestGoal(FromEndEff,&closest_goal) * EnvROBARMCfg.cost_per_cell;
        }
    }

//     printf("GetFromToHeuristic(#%i(%i,%i,%i) -> #%i(%i,%i,%i)) H cost: %i\n",FromStateID, FromHashEntry->endeff[0], FromHashEntry->endeff[1], FromHashEntry->endeff[2], ToStateID, ToHashEntry->endeff[0], ToHashEntry->endeff[1], ToHashEntry->endeff[2], h);
    return h;
}

int EnvironmentROBARM3D::GetGoalHeuristic(int stateID)
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

int EnvironmentROBARM3D::GetStartHeuristic(int stateID)
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

double EnvironmentROBARM3D::GetEpsilon()
{
    return EnvROBARMCfg.epsilon;
}

int EnvironmentROBARM3D::SizeofCreatedEnv()
{
    return EnvROBARM.StateID2CoordTable.size();
	
}

void EnvironmentROBARM3D::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/)
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
        PrintAnglesWithAction(fOut, HashEntry, bGoal, bVerbose, false);
    }
}

//get the goal as a successor of source state at given cost
//if costtogoal = -1 then the succ is chosen
void EnvironmentROBARM3D::PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/, FILE* fOut /*=NULL*/)
{
    printf("[PrintSuccGoal] WARNING: This function has not been updated to deal with multiple goals.\n");

    short unsigned int succcoord[NUMOFLINKS];
    double angles[NUMOFLINKS],orientation[3][3];
    short unsigned int endeff[3],wrist[3],elbow[3];
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

            ComputeContAngles(succcoord, angles);
            ComputeEndEffectorPos(angles,endeff,wrist,elbow,orientation);

            //skip invalid successors
            if(!IsValidCoord(succcoord,endeff,wrist,elbow,orientation))
                continue;

            if(endeff[0] == EnvROBARMCfg.EndEffGoalX_c && endeff[1] == EnvROBARMCfg.EndEffGoalY_c && endeff[2] ==  EnvROBARMCfg.EndEffGoalZ_c)
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

void EnvironmentROBARM3D::PrintEnv_Config(FILE* fOut)
{
	//implement this if the planner needs to print out EnvROBARM. configuration
	
    printf("ERROR in EnvROBARM... function: PrintEnv_Config is undefined\n");
    exit(1);
}

void EnvironmentROBARM3D::PrintHeader(FILE* fOut)
{
    fprintf(fOut, "%d\n", NUMOFLINKS);
    for(int i = 0; i < NUMOFLINKS; i++)
        fprintf(fOut, "%.3f ", EnvROBARMCfg.LinkLength_m[i]);
    fprintf(fOut, "\n");
}

/* GetSuccs - 2/26/09 - with bad angular distance costs
void EnvironmentROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    short unsigned int succcoord[NUMOFLINKS];
    short unsigned int goal_moe_c = EnvROBARMCfg.goal_moe_m / EnvROBARMCfg.GridCellWidth + .999999;
    short unsigned int k, wrist[3], elbow[3], endeff[3];
    double orientation [3][3];
    int i, inc, a, correct_orientation;
    double angles[NUMOFLINKS], s_angles[NUMOFLINKS],endeff_m[3];

    //added support for checking RPY and applying a funny 'heuristic' to the angular distance to the goal orientation 2/25/09
    double rot[3][3];
    double rpy1[3],rpy2[3],goal_rpy[3];
    double dist_to_goal;
    double exp_weight;
//     int heuristic_weight=1;
    double min_ang_dist;
    int closest_rpy;

    //to support two sets of succesor actions
    int actions_i_min = 0, actions_i_max = EnvROBARMCfg.nLowResActions;

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

    //goal state should be absorbing
    if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
{
        printf("goal state is absorbing...\n");
        return;
}

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];
    ComputeContAngles(HashEntry->coord, s_angles);

    //default coords of successor
    for(i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    ComputeContAngles(succcoord, angles);

    //check if cell is close to enough to goal to use higher resolution actions
    if(EnvROBARMCfg.multires_succ_actions)
{
        if (GetDistToClosestGoal((HashEntry->endeff)) <= EnvROBARMCfg.HighResActionsThreshold_c)
{
            actions_i_min = EnvROBARMCfg.nLowResActions;
            actions_i_max = EnvROBARMCfg.nSuccActions;
}
}

    //iterate through successors of s (possible actions)
    for (i = actions_i_min; i < actions_i_max; i++)
{
        //increase and decrease in ith angle
        for(inc = -1; inc < 2; inc = inc+2)
{
            if(inc == -1)
{
                for(a = 0; a < NUMOFLINKS; a++)
{
                    //if the joint is at 0deg and the next action will decrement it
                    if(HashEntry->coord[a] == 0 && EnvROBARMCfg.SuccActions[i][a] != 0)
                        succcoord[a] =  EnvROBARMCfg.anglevals[a] - EnvROBARMCfg.SuccActions[i][a];
                    //the joint's current position, when decremented by n degrees will go below 0
                    else if(HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a] < 0)
                        succcoord[a] =  EnvROBARMCfg.anglevals[a] + (HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a]);
                    else
                        succcoord[a] = HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a];
}
}
            else
{
                for(a = 0; a < NUMOFLINKS; a++)
                    succcoord[a] = (HashEntry->coord[a] + int(EnvROBARMCfg.SuccActions[i][a])) % EnvROBARMCfg.anglevals[a];
}

            //get the successor
            EnvROBARMHashEntry_t* OutHashEntry;
            bool bSuccisGoal = false;

            //have to create a new entry
            ComputeContAngles(succcoord, angles);

            //get forward kinematics
            if(ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation) == false)
{
                continue;
}

            //do collision checking
            if(!IsValidCoord(succcoord, endeff, wrist, elbow, orientation))
{
                continue;
}

            //check if within goal_moe_c cells of the goal
            for(k = 0; k < EnvROBARMCfg.nEndEffGoals; k++)
{
                //get both RPY solutions - only needed for 6DoF goal search but put here for debugging
                for (int w=0; w<3; w++)
                    for (int q=0; q<3; q++)
                        rot[w][q] = orientation[w][q];  //NOT transposeD orientation matrix

                getRPY(rot, &rpy1[0], &rpy1[1], &rpy1[2], 1);
                getRPY(rot, &rpy2[0], &rpy2[1], &rpy2[2], 2);

                goal_rpy[0] = EnvROBARMCfg.EndEffGoalRPY[0][0];
                goal_rpy[1] = EnvROBARMCfg.EndEffGoalRPY[0][1];
                goal_rpy[2] = EnvROBARMCfg.EndEffGoalRPY[0][2];

                if(fabs(endeff[0] - EnvROBARMCfg.EndEffGoals_c[k][0]) < goal_moe_c && 
                   fabs(endeff[1] - EnvROBARMCfg.EndEffGoals_c[k][1]) < goal_moe_c && 
                   fabs(endeff[2] - EnvROBARMCfg.EndEffGoals_c[k][2]) < goal_moe_c)
{
                    //heuristic_weight = 0;   //added 2/25/09 - a hack

#if DEBUG_CHECK_GOALRPY
                    printf("Within %.1f cm of goal: ", EnvROBARMCfg.goal_moe_m*100);
                    for(int r=0; r < NUMOFLINKS; r++)
                        printf("%1.3f ",angles[r]);
                    printf("\n");
#endif
                    //6 DoF Goal Search - check if end effector has the correct orientation in the shoulder frame
                    if(EnvROBARMCfg.checkEndEffGoalOrientation)
{
                        correct_orientation = 0;

#if DEBUG_CHECK_GOALRPY
                        printf("rpy1: (%1.3f %1.3f %1.3f)   goal: (%1.3f %1.3f %1.3f)  ",rpy1[0],rpy1[1],rpy1[2],EnvROBARMCfg.EndEffGoalRPY[k][0],EnvROBARMCfg.EndEffGoalRPY[k][1],EnvROBARMCfg.EndEffGoalRPY[k][2]);
                        printf("rpy1 error: %1.3f %1.3f %1.3f   (margin of error: %1.3f %1.3f %1.3f)\n",angles::shortest_angular_distance(rpy1[0], EnvROBARMCfg.EndEffGoalRPY[k][0]),
                               angles::shortest_angular_distance(rpy1[1], EnvROBARMCfg.EndEffGoalRPY[k][1]), angles::shortest_angular_distance(rpy1[2], EnvROBARMCfg.EndEffGoalRPY[k][2]),
                               EnvROBARMCfg.GoalRPY_MOE[0],EnvROBARMCfg.GoalRPY_MOE[1],EnvROBARMCfg.GoalRPY_MOE[2]);
                        printf("rpy2: (%1.3f %1.3f %1.3f)   goal: (%1.3f %1.3f %1.3f)  ",rpy2[0],rpy2[1],rpy2[2],EnvROBARMCfg.EndEffGoalRPY[k][0],EnvROBARMCfg.EndEffGoalRPY[k][1],EnvROBARMCfg.EndEffGoalRPY[k][2]);
                        printf("rpy2 error: %1.3f %1.3f %1.3f   (margin of error: %1.3f %1.3f %1.3f)\n",angles::shortest_angular_distance(rpy2[0], EnvROBARMCfg.EndEffGoalRPY[k][0]),
                               angles::shortest_angular_distance(rpy2[1], EnvROBARMCfg.EndEffGoalRPY[k][1]), angles::shortest_angular_distance(rpy2[2], EnvROBARMCfg.EndEffGoalRPY[k][2]),
                               EnvROBARMCfg.GoalRPY_MOE[0],EnvROBARMCfg.GoalRPY_MOE[1],EnvROBARMCfg.GoalRPY_MOE[2]);
#endif

                        //compare RPY1 to goal RPY and see if it is within the MOE
                        if(fabs(angles::shortest_angular_distance(rpy1[0], EnvROBARMCfg.EndEffGoalRPY[k][0])) < EnvROBARMCfg.GoalRPY_MOE[0] &&
                           fabs(angles::shortest_angular_distance(rpy1[1], EnvROBARMCfg.EndEffGoalRPY[k][1])) < EnvROBARMCfg.GoalRPY_MOE[1] &&
                           fabs(angles::shortest_angular_distance(rpy1[2], EnvROBARMCfg.EndEffGoalRPY[k][2])) < EnvROBARMCfg.GoalRPY_MOE[2])
{
                            correct_orientation = 1;
#if DEBUG_CHECK_GOALRPY
                            ComputeEndEffectorPos(angles,endeff_m);
                            printf("goal reached-> endeff: (%1.3f %1.3f %1.3f)  rpy1: (%1.3f %1.3f %1.3f)\n",endeff_m[0],endeff_m[1],endeff_m[2],rpy1[0],rpy1[1],rpy1[2]);
#endif
}

                        //compare RPY2 to goal RPY and see if it is within the MOE
                        if((fabs(angles::shortest_angular_distance(rpy2[0], EnvROBARMCfg.EndEffGoalRPY[k][0])) < EnvROBARMCfg.GoalRPY_MOE[0] &&
                            fabs(angles::shortest_angular_distance(rpy2[1], EnvROBARMCfg.EndEffGoalRPY[k][1])) < EnvROBARMCfg.GoalRPY_MOE[1] &&
                            fabs(angles::shortest_angular_distance(rpy2[2], EnvROBARMCfg.EndEffGoalRPY[k][2])) < EnvROBARMCfg.GoalRPY_MOE[2]))
{
                            correct_orientation = 1;
#if DEBUG_CHECK_GOALRPY
                            ComputeEndEffectorPos(angles,endeff_m);
                            printf("goal reached-> endeff: (%1.3f %1.3f %1.3f)  rpy2: (%1.3f %1.3f %1.3f)\n",endeff_m[0],endeff_m[1],endeff_m[2],rpy2[0],rpy2[1],rpy2[2]);
#endif
}

                        if(correct_orientation == 1)
{
                            bSuccisGoal = true;
                            // printf("goal succ is generated\n");
                            for (int j = 0; j < NUMOFLINKS; j++)
{
                                EnvROBARMCfg.goalcoords[j] = succcoord[j];
                                EnvROBARM.goalHashEntry->coord[j] = succcoord[j];
}
                            EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[k][0];
                            EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[k][1];
                            EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[k][2];
                            EnvROBARM.goalHashEntry->action = i;
}
}

                    //3DoF goal position
                    else
{
                        bSuccisGoal = true;
                        // printf("goal succ is generated\n");
                        for (int j = 0; j < NUMOFLINKS; j++)
{
                            EnvROBARMCfg.goalcoords[j] = succcoord[j];
                            EnvROBARM.goalHashEntry->coord[j] = succcoord[j];
}
                        EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[k][0];
                        EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[k][1];
                        EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[k][2];
                        EnvROBARM.goalHashEntry->action = i;
                        //printf("goal reached-> endeff: (%u %u %u)\n",EnvROBARM.goalHashEntry->endeff[0],EnvROBARM.goalHashEntry->endeff[1],EnvROBARM.goalHashEntry->endeff[2]);

#if DEBUG_CHECK_GOALRPY
                        printf("rpy1: (%1.3f %1.3f %1.3f)\n",rpy1[0],rpy1[1],rpy1[2]);
                        printf("rpy2: (%1.3f %1.3f %1.3f)\n",rpy2[0],rpy2[1],rpy2[2]);

                        printf("goal angles: ");
                        for(int p=0; p<NUMOFLINKS;p++)
                            printf("%1.3f ",angles[p]);
                        printf("\n");
#endif
}
}
}

            //check if hash entry already exists, if not then create one
            if((OutHashEntry = GetHashEntry(succcoord, NUMOFLINKS, i, bSuccisGoal)) == NULL)
{
                OutHashEntry = CreateNewHashEntry(succcoord, NUMOFLINKS, endeff, i, orientation);
}

            SuccIDV->push_back(OutHashEntry->stateID);

            if(!EnvROBARMCfg.angular_dist_cost)
{
                CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) + GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID) + EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);
}
            else
{
                //goal orientation (only set to handle one goal configuration for now)
                goal_rpy[0] = EnvROBARMCfg.EndEffGoalRPY[0][0];
                goal_rpy[1] = EnvROBARMCfg.EndEffGoalRPY[0][1];
                goal_rpy[2] = EnvROBARMCfg.EndEffGoalRPY[0][2];

                //get minimum angular distance to goal orientation
                min_ang_dist = getAngularEuclDist(rpy1,goal_rpy);
                closest_rpy = 1;
                if(getAngularEuclDist(rpy2,goal_rpy) < min_ang_dist)
{
                    closest_rpy = 2;
    //                 printf("rpy2 is closer than rpy1 to goal_rpy\n");
                    min_ang_dist = getAngularEuclDist(rpy2,goal_rpy);
}

                //distance of end effector to closest goal position
                dist_to_goal = GetDistToClosestGoal(OutHashEntry->endeff);

                //if the end effector is not close to the goal, don't take the current orientation of the end effector into account
                if (dist_to_goal > EnvROBARMCfg.HighResActionsThreshold_c)
{
                    CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) + GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID) + EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);
}
                else
{
                    //exponential weight on the angular distance cost (as the endeff gets closer to goal the weight increases)
                    exp_weight = exp(-dist_to_goal * EnvROBARMCfg.ExpCoefficient);

                    CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) + //cost from start (g-value)
                                    GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID) //* (1-exp_weight) * heuristic_weight  //position heuristic
                                    EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action] + //smoothing
                                    COSTMULT * min_ang_dist * EnvROBARMCfg.AngularDist_Weight * exp_weight); //angular distance
#if DEBUG_CHECK_GOALRPY
                    printf("exp_weight: %.4f, angular weight: %.4f, angular distance: %.3f, g-value: %i, heuristic: %i, ",
                            exp_weight,
                            COSTMULT*min_ang_dist*EnvROBARMCfg.AngularDist_Weight*exp_weight,
                            min_ang_dist,
                            cost(HashEntry,OutHashEntry,bSuccisGoal),
                            GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID)); // * heuristic_weight * (1-exp_weight) );

                    if(EnvROBARMCfg.use_smooth_actions)
                        printf("smoothing: %i, ", EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);

                    if(closest_rpy == 1)
                        printf("rpy1: %.3f %.3f %.3f\n",rpy1[0],rpy1[1],rpy1[2]);
                    else
                        printf("rpy2: %.3f %.3f %.3f\n",rpy2[0],rpy2[1],rpy2[2]);
#endif
}
}
            // printf("%i %i %i --> %i %i %i,  g: %i  h: %i\n",HashEntry->endeff[0],HashEntry->endeff[1],HashEntry->endeff[2],endeff[0],endeff[1],endeff[2],cost(HashEntry,OutHashEntry,bSuccisGoal),GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID)); 
}
}
}
                                                                                */

void EnvironmentROBARM3D::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    if(EnvROBARMCfg.PlanInJointSpace)
    {
        GetJointSpaceSuccs(SourceStateID, SuccIDV, CostV);
        return;
    }

    if(!EnvROBARMCfg.bGoalIsSet)
        return;

    short unsigned int succcoord[NUMOFLINKS];
    short unsigned int goal_moe_c = EnvROBARMCfg.goal_moe_m / EnvROBARMCfg.GridCellWidth + .999999;
    short unsigned int k, wrist[3], elbow[3], endeff[3];
    double orientation [3][3];
    int i,y, inc, a, correct_orientation, closest_goal;
    double angles[NUMOFLINKS], s_angles[NUMOFLINKS];//,endeff_m[3];

    //added support for checking RPY and applying a funny 'heuristic' to the angular distance to the goal orientation 2/25/09
    double rpy1[3],rpy2[3],goal_rpy[3];

    //to support two sets of succesor actions
    int actions_i_min = 0, actions_i_max = EnvROBARMCfg.nLowResActions;

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

    //goal state should be absorbing
    if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    {
        printf("goal state is absorbing...\n");
        return;
    }

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];
    ComputeContAngles(HashEntry->coord, s_angles);

    //default coords of successor
    for(i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    ComputeContAngles(succcoord, angles);

    //check if cell is close to enough to goal to use higher resolution actions
    if(EnvROBARMCfg.multires_succ_actions)
    {
        if (GetDistToClosestGoal(HashEntry->endeff,&closest_goal) <= EnvROBARMCfg.HighResActionsThreshold_c)
        {
            actions_i_min = EnvROBARMCfg.nLowResActions;
            actions_i_max = EnvROBARMCfg.nSuccActions;
        }
    }

    //iterate through successors of s (possible actions)
    for (i = actions_i_min; i < actions_i_max; i++)
    {
        //increase and decrease in ith angle
        for(inc = -1; inc < 2; inc = inc+2)
        {
            if(inc == -1)
            {
                for(a = 0; a < NUMOFLINKS; a++)
                {
                    //if the joint is at 0deg and the next action will decrement it
                    if(HashEntry->coord[a] == 0 && EnvROBARMCfg.SuccActions[i][a] != 0)
                        succcoord[a] =  EnvROBARMCfg.anglevals[a] - EnvROBARMCfg.SuccActions[i][a];
                    //the joint's current position, when decremented by n degrees will go below 0
                    else if(HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a] < 0)
                        succcoord[a] =  EnvROBARMCfg.anglevals[a] + (HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a]);
                    else
                        succcoord[a] = HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a];
                }
            }
            else
            {
                for(a = 0; a < NUMOFLINKS; a++)
                    succcoord[a] = (HashEntry->coord[a] + int(EnvROBARMCfg.SuccActions[i][a])) % EnvROBARMCfg.anglevals[a];
            }

            //get the successor
            EnvROBARMHashEntry_t* OutHashEntry;
            bool bSuccisGoal = false;

            //have to create a new entry
            ComputeContAngles(succcoord, angles);

            //get forward kinematics
            if(ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation) == false)
            {
//                 printf("Invalid succ: endeff (%i %i %i) wrist(%i %i %i)\n",
//                        endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2]);
                continue;
            }

            //do collision checking
            if(!IsValidCoord(succcoord, endeff, wrist, elbow, orientation))
            {
//                 printf("Invalid succ: %i %i %i %i %i %i %i\n",
//                        succcoord[0],succcoord[1],succcoord[2],succcoord[3],succcoord[4],succcoord[5],succcoord[6]);
                continue;
            }

            //check if within goal_moe_c cells of the goal
            for(k = 0; k < EnvROBARMCfg.nEndEffGoals; k++)
            {
                if(fabs(endeff[0] - EnvROBARMCfg.EndEffGoals_c[k][0]) < goal_moe_c &&
                   fabs(endeff[1] - EnvROBARMCfg.EndEffGoals_c[k][1]) < goal_moe_c &&
                   fabs(endeff[2] - EnvROBARMCfg.EndEffGoals_c[k][2]) < goal_moe_c)
                {
#if DEBUG_CHECK_GOALRPY
                    printf("Within %.1f cm of goal: ", EnvROBARMCfg.goal_moe_m*100);
                    for(int r=0; r < NUMOFLINKS; r++)
                        printf("%1.3f ",angles[r]);
                    printf("\n");
#endif
                    //6 DoF Goal Search - check if end effector has the correct orientation in the shoulder frame
                    if(EnvROBARMCfg.checkEndEffGoalOrientation)
                    {
                        correct_orientation = 0;

                        //get both RPY solutions
                        getRPY(orientation, &rpy1[0], &rpy1[1], &rpy1[2], 1);
                        getRPY(orientation, &rpy2[0], &rpy2[1], &rpy2[2], 2);

                        for (y = 0; y < 3; y++)
                        {
                            if(rpy1[y] >= PI_CONST)
                                rpy1[y] = -2.0*PI_CONST + rpy1[y];

                            if(rpy2[y] >= PI_CONST)
                                rpy2[y] = -2.0*PI_CONST + rpy2[y];
                        }
#if DEBUG_CHECK_GOALRPY
                        printf("rpy1: (%1.3f %1.3f %1.3f)   goal: (%1.3f %1.3f %1.3f)  ",rpy1[0],rpy1[1],rpy1[2],EnvROBARMCfg.EndEffGoalRPY[k][0],EnvROBARMCfg.EndEffGoalRPY[k][1],EnvROBARMCfg.EndEffGoalRPY[k][2]);
                        printf("rpy1 error: %1.3f %1.3f %1.3f   (margin of error: %1.3f %1.3f %1.3f)\n",angles::shortest_angular_distance(rpy1[0], EnvROBARMCfg.EndEffGoalRPY[k][0]),
                               angles::shortest_angular_distance(rpy1[1], EnvROBARMCfg.EndEffGoalRPY[k][1]), angles::shortest_angular_distance(rpy1[2], EnvROBARMCfg.EndEffGoalRPY[k][2]),
                                       EnvROBARMCfg.GoalRPY_MOE[0],EnvROBARMCfg.GoalRPY_MOE[1],EnvROBARMCfg.GoalRPY_MOE[2]);
                        printf("rpy2: (%1.3f %1.3f %1.3f)   goal: (%1.3f %1.3f %1.3f)  ",rpy2[0],rpy2[1],rpy2[2],EnvROBARMCfg.EndEffGoalRPY[k][0],EnvROBARMCfg.EndEffGoalRPY[k][1],EnvROBARMCfg.EndEffGoalRPY[k][2]);
                        printf("rpy2 error: %1.3f %1.3f %1.3f   (margin of error: %1.3f %1.3f %1.3f)\n",angles::shortest_angular_distance(rpy2[0], EnvROBARMCfg.EndEffGoalRPY[k][0]),
                               angles::shortest_angular_distance(rpy2[1], EnvROBARMCfg.EndEffGoalRPY[k][1]), angles::shortest_angular_distance(rpy2[2], EnvROBARMCfg.EndEffGoalRPY[k][2]),
                                       EnvROBARMCfg.GoalRPY_MOE[0],EnvROBARMCfg.GoalRPY_MOE[1],EnvROBARMCfg.GoalRPY_MOE[2]);
#endif

                        //compare RPY1 to goal RPY and see if it is within the MOE
//                         if(fabs(angles::shortest_angular_distance(rpy1[0], EnvROBARMCfg.EndEffGoalRPY[k][0])) < EnvROBARMCfg.GoalRPY_MOE[0] &&
                        if(fabs(angles::shortest_angular_distance(rpy1[1], EnvROBARMCfg.EndEffGoalRPY[k][1])) < EnvROBARMCfg.GoalRPY_MOE[1] &&
                           fabs(angles::shortest_angular_distance(rpy1[2], EnvROBARMCfg.EndEffGoalRPY[k][2])) < EnvROBARMCfg.GoalRPY_MOE[2])
                        {
                            correct_orientation = 1;
#if DEBUG_CHECK_GOALRPY
                            ComputeEndEffectorPos(angles,endeff_m);
//                             printf("goal reached-> endeff: (%1.3f %1.3f %1.3f)  rpy1: (%1.3f %1.3f %1.3f)\n",endeff_m[0],endeff_m[1],endeff_m[2],rpy1[0],rpy1[1],rpy1[2]);
#endif
                        }

                        //compare RPY2 to goal RPY and see if it is within the MOE
//                         if((fabs(angles::shortest_angular_distance(rpy2[0], EnvROBARMCfg.EndEffGoalRPY[k][0])) < EnvROBARMCfg.GoalRPY_MOE[0] &&
                        if(fabs(angles::shortest_angular_distance(rpy2[1], EnvROBARMCfg.EndEffGoalRPY[k][1])) < EnvROBARMCfg.GoalRPY_MOE[1] &&
                            fabs(angles::shortest_angular_distance(rpy2[2], EnvROBARMCfg.EndEffGoalRPY[k][2])) < EnvROBARMCfg.GoalRPY_MOE[2])
                        {
                            correct_orientation = 1;
#if DEBUG_CHECK_GOALRPY
                            ComputeEndEffectorPos(angles,endeff_m);
//                             printf("goal reached-> endeff: (%1.3f %1.3f %1.3f)  rpy2: (%1.3f %1.3f %1.3f)\n",endeff_m[0],endeff_m[1],endeff_m[2],rpy2[0],rpy2[1],rpy2[2]);
#endif
                        }

                        if(correct_orientation == 1)
                        {
                            bSuccisGoal = true;
                            // printf("goal succ is generated\n");
                            for (int j = 0; j < NUMOFLINKS; j++)
                            {
                                EnvROBARMCfg.goalcoords[j] = succcoord[j];
                                EnvROBARM.goalHashEntry->coord[j] = succcoord[j];
                            }
                            EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[k][0];
                            EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[k][1];
                            EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[k][2];
                            EnvROBARM.goalHashEntry->action = i;
                        }
                    }

                    //3DoF Goal
                    else
                    {
                        bSuccisGoal = true;
                        // printf("goal succ is generated\n");
                        for (int j = 0; j < NUMOFLINKS; j++)
                        {
                            EnvROBARMCfg.goalcoords[j] = succcoord[j];
                            EnvROBARM.goalHashEntry->coord[j] = succcoord[j];
                        }
                        EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[k][0];
                        EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[k][1];
                        EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[k][2];
                        EnvROBARM.goalHashEntry->action = i;

#if DEBUG_CHECK_GOALRPY
                        getRPY(orientation, &rpy1[0], &rpy1[1], &rpy1[2], 1);
                        getRPY(orientation, &rpy2[0], &rpy2[1], &rpy2[2], 2);

                        printf("rpy1: (%1.3f %1.3f %1.3f)\n",rpy1[0],rpy1[1],rpy1[2]);
                        printf("rpy2: (%1.3f %1.3f %1.3f)\n",rpy2[0],rpy2[1],rpy2[2]);

                        printf("goal angles: ");
                        for(int p=0; p<NUMOFLINKS;p++)
                            printf("%1.3f ",angles[p]);
                        printf("\n");
#endif
                    }
                }
            }

            //check if hash entry already exists, if not then create one
            if((OutHashEntry = GetHashEntry(succcoord, NUMOFLINKS, i, bSuccisGoal)) == NULL)
            {
                OutHashEntry = CreateNewHashEntry(succcoord, NUMOFLINKS, endeff, i, orientation);
            }

            //put successor on successor list with cost
            SuccIDV->push_back(OutHashEntry->stateID);

            if(!EnvROBARMCfg.dual_heuristics)
            {
                CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) +
                        GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID) +
                        EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);
            }
            else
            {

                //TODO FIX THIS SO IT CAN HANDLE MULTIPLE GOALS
                goal_rpy[0] = EnvROBARMCfg.EndEffGoalRPY[0][0];
                goal_rpy[1] = EnvROBARMCfg.EndEffGoalRPY[0][1];
                goal_rpy[2] = EnvROBARMCfg.EndEffGoalRPY[0][2];

                //not taking roll into account
                goal_rpy[0] = 0.0;
                rpy1[0] = 0.0;
                rpy2[0] = 0.0;

                //compute cost with minimum angular distance to goal orientation
                if(getAngularEuclDist(rpy2,goal_rpy) <  getAngularEuclDist(rpy1,goal_rpy))
                {
                    CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) +
                            GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID, rpy2, goal_rpy) +
                            EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);
                }
                else
                {
                    CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) +
                            GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID, rpy1, goal_rpy) +
                            EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);
                }
            }
//             printf("%i %i %i --> %i %i %i,  g: %i  h: %i\n",HashEntry->endeff[0],HashEntry->endeff[1],HashEntry->endeff[2],endeff[0],endeff[1],endeff[2],cost(HashEntry,OutHashEntry,bSuccisGoal),GetFromToHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID)); 
        }
    }
}

void EnvironmentROBARM3D::GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV)
{

    printf("ERROR in EnvROBARM... function: GetPreds is undefined\n");
    exit(1);
}

void EnvironmentROBARM3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{


    printf("ERROR in EnvROBARM..function: SetAllActionsandOutcomes is undefined\n");
    exit(1);
}

void EnvironmentROBARM3D::SetAllPreds(CMDPSTATE* state)
{
	//implement this if the planner needs access to predecessors
	
    printf("ERROR in EnvROBARM... function: SetAllPreds is undefined\n");
    exit(1);
}

int EnvironmentROBARM3D::GetEdgeCost(int FromStateID, int ToStateID)
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

bool EnvironmentROBARM3D::AreEquivalent(int State1ID, int State2ID)
{
    EnvROBARMHashEntry_t* HashEntry1 = EnvROBARM.StateID2CoordTable[State1ID];
    EnvROBARMHashEntry_t* HashEntry2 = EnvROBARM.StateID2CoordTable[State2ID];

    for(int i = 0; i < NUMOFLINKS; i++)
    {
        if(HashEntry1->coord[i] != HashEntry2->coord[i])
            return false;
    }
    if(HashEntry1->action != HashEntry2->action)
        return false;

    return true;
}

bool EnvironmentROBARM3D::SetEndEffGoals(double** EndEffGoals, int goal_type, int num_goals, bool bComputeHeuristic)
{
    printf("[SetEndEffGoals] Setting end effector goals...\n");
    //allocate memory
    if(EnvROBARMCfg.nEndEffGoals != num_goals || !EnvROBARMCfg.bGoalIsSet)
    {
        if(EnvROBARMCfg.bGoalIsSet && EnvROBARMCfg.nEndEffGoals != num_goals)
        {
            //delete the old goal array
            for (int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
            {
                delete [] EnvROBARMCfg.EndEffGoals_c[i];
                delete [] EnvROBARMCfg.EndEffGoalRPY[i];
                delete [] EnvROBARMCfg.EndEffGoals_m[i];
            }
            delete [] EnvROBARMCfg.EndEffGoalRPY;
            delete [] EnvROBARMCfg.EndEffGoals_c;
            delete [] EnvROBARMCfg.EndEffGoals_m;

            EnvROBARMCfg.EndEffGoals_c = NULL;
            EnvROBARMCfg.EndEffGoals_m = NULL;
            EnvROBARMCfg.EndEffGoalRPY = NULL;

            EnvROBARMCfg.bGoalIsSet = false;
        }

        EnvROBARMCfg.nEndEffGoals = num_goals;
        EnvROBARMCfg.EndEffGoals_c = new short unsigned int * [num_goals];
        EnvROBARMCfg.EndEffGoals_m = new double * [num_goals];
        EnvROBARMCfg.EndEffGoalRPY = new double * [num_goals];
        for (int i = 0; i < num_goals; i++)
        {
            EnvROBARMCfg.EndEffGoals_c[i] = new short unsigned int [3];
            EnvROBARMCfg.EndEffGoals_m[i] = new double [3];
            EnvROBARMCfg.EndEffGoalRPY[i] = new double [3];
        }
    }

    //Goal is in cartesian coordinates in world frame (meters)
    if(goal_type == 0)
    {
        //loop through all the goal positions
        for(int i = 0; i < num_goals; i++)
        {
            //store destination in meters - just to have (no real reason as of now other than debug output)
            EnvROBARMCfg.EndEffGoals_m[i][0] = EndEffGoals[i][0];
            EnvROBARMCfg.EndEffGoals_m[i][1] = EndEffGoals[i][1];
            EnvROBARMCfg.EndEffGoals_m[i][2] = EndEffGoals[i][2];

            //convert goal position from meters to cells
            ContXYZ2Cell(EndEffGoals[i][0],EndEffGoals[i][1],EndEffGoals[i][2], &(EnvROBARMCfg.EndEffGoals_c[i][0]), &(EnvROBARMCfg.EndEffGoals_c[i][1]), &(EnvROBARMCfg.EndEffGoals_c[i][2]));

            //RPY
            for (int k = 0; k < 3; k++)
                EnvROBARMCfg.EndEffGoalRPY[i][k] = EndEffGoals[i][k+3];
        }
        //set goalangle to invalid number
        EnvROBARMCfg.LinkGoalAngles_d[0] = INVALID_NUMBER;
    }
    //Goal is a joint space vector in radians
    else if(goal_type == 1)
    {
        for(int i = 0; i < 7; i++)
            EnvROBARMCfg.LinkGoalAngles_d[i] = DEG2RAD(EndEffGoals[0][i]);

        //so the goal's location (forward kinematics) can be calculated after the initialization completes
        EnvROBARMCfg.JointSpaceGoal = 1;
    }

    else
    {
        printf("[SetEndEffGoal] Invalid type of goal vector.\n");
    }

    //check if goal positions are valid
    for(int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
    {
        if(EnvROBARMCfg.EndEffGoals_c[i][0] >= EnvROBARMCfg.EnvWidth_c ||
           EnvROBARMCfg.EndEffGoals_c[i][1] >= EnvROBARMCfg.EnvHeight_c ||
           EnvROBARMCfg.EndEffGoals_c[i][2] >= EnvROBARMCfg.EnvDepth_c)
        {
            printf("[SetEndEffGoals] End effector goal position(%u %u %u) is out of bounds.\n",EnvROBARMCfg.EndEffGoals_c[i][0],EnvROBARMCfg.EndEffGoals_c[i][1],EnvROBARMCfg.EndEffGoals_c[i][2]);
            if(!RemoveGoal(i))
            {
                EnvROBARMCfg.bGoalIsSet  = false;
                return false;
            }
            else
            {
                printf("[SetEndEffGoals] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
            }
        }

        if(EnvROBARMCfg.Grid3D[EnvROBARMCfg.EndEffGoals_c[i][0]][EnvROBARMCfg.EndEffGoals_c[i][1]][EnvROBARMCfg.EndEffGoals_c[i][2]] >= EnvROBARMCfg.ObstacleCost)
        {
            printf("[SetEndEffGoals] End effector goal position(%u %u %u) is invalid\n",EnvROBARMCfg.EndEffGoals_c[i][0],EnvROBARMCfg.EndEffGoals_c[i][1],EnvROBARMCfg.EndEffGoals_c[i][2]);
            if(!RemoveGoal(i))
            {
                EnvROBARMCfg.bGoalIsSet  = false;
                return false;
            }
            else
            {
                printf("[SetEndEffGoals] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
            }
        }
    }

    if(EnvROBARMCfg.cost_per_cell == -1)
        ComputeCostPerCell();

    //pre-compute heuristics with new goal
    if(EnvROBARMCfg.dijkstra_heuristic) // && bComputeHeuristic)
        ComputeHeuristicValues();


    EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[0][0];
    EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[0][1];
    EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[0][2];

    EnvROBARMCfg.bGoalIsSet = true;

    //temporary debugging output
    double angle[NUMOFLINKS] = {0};
    ComputeContAngles(EnvROBARM.startHashEntry->coord, angle);
    printf("\n\n[SetEndEffGoals] Ready to Plan:\n");
    printf("start config: ");
    for(int i=0; i < 7; i++)
        printf("%1.3f  ", angle[i]);
    printf("\n");
    double pX,pY,pZ;
    double roll,pitch,yaw;
    getRPY(EnvROBARM.startHashEntry->orientation,&roll,&pitch,&yaw,1);
    Cell2ContXYZ(EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2], &pX, &pY, &pZ);
    printf("start: %i %i %i (cells) --> %.2f %.2f %.2f (meters)   rpy: (%1.2f %1.2f %1.2f)\n", EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2],pX,pY,pZ,roll,pitch,yaw);

    printf("goal coord: ");
    for(int i=0; i < 7; i++)
        printf("%i  ", EnvROBARM.goalHashEntry->coord[i]);
    printf("\n");

    for(int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
    {
        printf("goal %i:  %u %u %u (cells) --> %.2f %.2f %.2f (meters)  rpy: (%1.2f %1.2f %1.2f)\n",i,EnvROBARMCfg.EndEffGoals_c[i][0], EnvROBARMCfg.EndEffGoals_c[i][1],
               EnvROBARMCfg.EndEffGoals_c[i][2],EnvROBARMCfg.EndEffGoals_m[i][0],EnvROBARMCfg.EndEffGoals_m[i][1],EnvROBARMCfg.EndEffGoals_m[i][2],EnvROBARMCfg.EndEffGoalRPY[i][0],EnvROBARMCfg.EndEffGoalRPY[i][1],EnvROBARMCfg.EndEffGoalRPY[i][2]);
    }
    printf("\n\n");

    return true;
}

bool EnvironmentROBARM3D::SetEndEffGoals(double** EndEffGoals, int num_goals)
{
    //allocate memory
    if(EnvROBARMCfg.nEndEffGoals != num_goals || !EnvROBARMCfg.bGoalIsSet)
    {
        if(EnvROBARMCfg.bGoalIsSet && EnvROBARMCfg.nEndEffGoals != num_goals)
        {
            //delete the old goal array
            for (int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
            {
                delete [] EnvROBARMCfg.EndEffGoals_c[i];
                delete [] EnvROBARMCfg.EndEffGoals_m[i];
                delete [] EnvROBARMCfg.EndEffGoalRPY[i];
            }
            delete [] EnvROBARMCfg.EndEffGoals_c;
            delete [] EnvROBARMCfg.EndEffGoals_m;
            delete [] EnvROBARMCfg.EndEffGoalRPY;

            EnvROBARMCfg.EndEffGoals_c = NULL;
            EnvROBARMCfg.EndEffGoals_m = NULL;
            EnvROBARMCfg.EndEffGoalRPY = NULL;

            EnvROBARMCfg.bGoalIsSet = false;
        }

        EnvROBARMCfg.nEndEffGoals = num_goals;
        EnvROBARMCfg.EndEffGoals_c = new short unsigned int * [num_goals];
        EnvROBARMCfg.EndEffGoals_m = new double * [num_goals];
        EnvROBARMCfg.EndEffGoalRPY = new double * [num_goals];
        for (int i = 0; i < num_goals; i++)
        {
            EnvROBARMCfg.EndEffGoals_c[i] = new short unsigned int [3];
            EnvROBARMCfg.EndEffGoals_m[i] = new double [3];
            EnvROBARMCfg.EndEffGoalRPY[i] = new double [3];
        }
    }

    //Goal is in cartesian coordinates in world frame (meters)
    //loop through all the goal positions
    for(int i = 0; i < num_goals; i++)
    {
        //store destination in meters - just to have (no real reason as of now other than debug output)
        EnvROBARMCfg.EndEffGoals_m[i][0] = EndEffGoals[i][0];
        EnvROBARMCfg.EndEffGoals_m[i][1] = EndEffGoals[i][1];
        EnvROBARMCfg.EndEffGoals_m[i][2] = EndEffGoals[i][2];

        //convert goal position from meters to cells
        ContXYZ2Cell(EndEffGoals[i][0],EndEffGoals[i][1],EndEffGoals[i][2], &(EnvROBARMCfg.EndEffGoals_c[i][0]), &(EnvROBARMCfg.EndEffGoals_c[i][1]), &(EnvROBARMCfg.EndEffGoals_c[i][2]));

        //RPY
        for (int k = 0; k < 3; k++)
            EnvROBARMCfg.EndEffGoalRPY[i][k] = EndEffGoals[i][k+3];
    }

    //set goalangle to invalid number
    EnvROBARMCfg.LinkGoalAngles_d[0] = INVALID_NUMBER;

    //check if goal positions are valid
    for(int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
    {
        //check if goals are within arms length
        if(fabs(EnvROBARMCfg.EndEffGoals_c[i][0]) >= EnvROBARMCfg.arm_length ||
           fabs(EnvROBARMCfg.EndEffGoals_c[i][1]) >= EnvROBARMCfg.arm_length ||
           fabs(EnvROBARMCfg.EndEffGoals_c[i][2]) >= EnvROBARMCfg.arm_length)
        {
            printf("End effector goal position(%u %u %u) is out of bounds.\n",EnvROBARMCfg.EndEffGoals_c[i][0],EnvROBARMCfg.EndEffGoals_c[i][1],EnvROBARMCfg.EndEffGoals_c[i][2]);
            if(!RemoveGoal(i))
            {
                EnvROBARMCfg.bGoalIsSet  = false;
                return false;
            }
            else
            {
                printf("[SetEndEffGoals] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                continue;
            }
        }

        if(EnvROBARMCfg.Grid3D[EnvROBARMCfg.EndEffGoals_c[i][0]][EnvROBARMCfg.EndEffGoals_c[i][1]][EnvROBARMCfg.EndEffGoals_c[i][2]] >= EnvROBARMCfg.ObstacleCost)
        {
            printf("End effector goal position(%u %u %u) is invalid\n",EnvROBARMCfg.EndEffGoals_c[i][0],EnvROBARMCfg.EndEffGoals_c[i][1],EnvROBARMCfg.EndEffGoals_c[i][2]);
            if(!RemoveGoal(i))
            {
                EnvROBARMCfg.bGoalIsSet  = false;
                return false;
            }
            else
            {
                printf("[SetEndEffGoals] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
            }
        }
    }

    if(EnvROBARMCfg.cost_per_cell == -1)
        ComputeCostPerCell();

    //pre-compute heuristics with new goal
    if(EnvROBARMCfg.dijkstra_heuristic) // && bComputeHeuristic)
        ComputeHeuristicValues();

    EnvROBARM.goalHashEntry->endeff[0] = EnvROBARMCfg.EndEffGoals_c[0][0];
    EnvROBARM.goalHashEntry->endeff[1] = EnvROBARMCfg.EndEffGoals_c[0][1];
    EnvROBARM.goalHashEntry->endeff[2] = EnvROBARMCfg.EndEffGoals_c[0][2];

    EnvROBARMCfg.bGoalIsSet = true;

    //temporary debugging output
    double angle[NUMOFLINKS] = {0};
    double pX,pY,pZ;
    double roll,pitch,yaw;

    ComputeContAngles(EnvROBARM.startHashEntry->coord, angle);
    printf("\n\n[Environment] Ready to Plan:\n");
    printf("start config: ");
    for(int i=0; i < 7; i++)
        printf("%1.3f  ", angle[i]);
    printf("\n");

    getRPY(EnvROBARM.startHashEntry->orientation,&roll,&pitch,&yaw,1);
    Cell2ContXYZ(EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2], &pX, &pY, &pZ);
    printf("start: %i %i %i (cells) --> %.2f %.2f %.2f (meters)   rpy: (%1.2f %1.2f %1.2f)\n", EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2],pX,pY,pZ,roll,pitch,yaw);

    printf("goal coord: ");
    for(int i=0; i < 7; i++)
        printf("%i  ", EnvROBARM.goalHashEntry->coord[i]);
    printf("\n");

    for(int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
    {
        printf("goal %i:  %u %u %u (cells) --> %.2f %.2f %.2f (meters)  rpy: (%1.2f %1.2f %1.2f)\n",i,EnvROBARMCfg.EndEffGoals_c[i][0], EnvROBARMCfg.EndEffGoals_c[i][1],
               EnvROBARMCfg.EndEffGoals_c[i][2],EnvROBARMCfg.EndEffGoals_m[i][0],EnvROBARMCfg.EndEffGoals_m[i][1],EnvROBARMCfg.EndEffGoals_m[i][2],EnvROBARMCfg.EndEffGoalRPY[i][0],EnvROBARMCfg.EndEffGoalRPY[i][1],EnvROBARMCfg.EndEffGoalRPY[i][2]);
    }
    printf("\n\n");

    return true;
}

bool EnvironmentROBARM3D::SetStartJointConfig(double angles[NUMOFLINKS], bool bRad)
{
    double startangles[NUMOFLINKS];
    short unsigned int elbow[3] = {0}, wrist[3] = {0};

    //set initial joint configuration
    if(bRad) //input is in radians
    {
        for(unsigned int i = 0; i < NUMOFLINKS; i++)
        {
            if(angles[i] < 0)
                EnvROBARMCfg.LinkStartAngles_d[i] = RAD2DEG(angles[i] + PI_CONST*2.0);
            else
                EnvROBARMCfg.LinkStartAngles_d[i] = RAD2DEG(angles[i]);
        }
    }
    else //input is in degrees
    {
        for(unsigned int i = 0; i < NUMOFLINKS; i++)
        {
            if(angles[i] < 0)
                EnvROBARMCfg.LinkStartAngles_d[i] =  angles[i] + 360.0;
            else
                EnvROBARMCfg.LinkStartAngles_d[i] =  angles[i];
        }
    }

    //initialize the angles of the start states
    for(int i = 0; i < NUMOFLINKS; i++)
        startangles[i] = DEG2RAD(EnvROBARMCfg.LinkStartAngles_d[i]);

    //compute arm position in environment
    ComputeCoord(startangles, EnvROBARM.startHashEntry->coord);

    //TODO: REMOVE ME - Don't check start position when running in gazebo.
    //get joint positions of starting configuration
    if(!ComputeEndEffectorPos(startangles, EnvROBARM.startHashEntry->endeff, wrist, elbow, EnvROBARM.startHashEntry->orientation))
    {
        printf("[ComputeEndEffectorPos] Start position is invalid.\n");
//         return false;
    }

//     double rpy1[3],rpy2[3];
//     getRPY(EnvROBARM.startHashEntry->orientation,&rpy1[0],&rpy1[1],&rpy1[2],1);
//     getRPY(EnvROBARM.startHashEntry->orientation,&rpy2[0],&rpy2[1],&rpy2[2],2);
//     printf("rpy1: (%1.3f %1.3f %1.3f)\n",rpy1[0],rpy1[1],rpy1[2]);
//     printf("rpy2: (%1.3f %1.3f %1.3f)\n",rpy2[0],rpy2[1],rpy2[2]);

    //check if starting position is valid - TODO: REMOVE ME - Don't check start position when running in gazebo.
    if(!IsValidCoord(EnvROBARM.startHashEntry->coord,EnvROBARM.startHashEntry->endeff,wrist,elbow,EnvROBARM.startHashEntry->orientation))
    {
        printf("\n[SetStartJointConfig] Start Hash Entry is invalid\n");
        printf("coords: %i %i %i %i %i %i %i\n", EnvROBARM.startHashEntry->coord[0],EnvROBARM.startHashEntry->coord[1],EnvROBARM.startHashEntry->coord[2],EnvROBARM.startHashEntry->coord[3],EnvROBARM.startHashEntry->coord[4],EnvROBARM.startHashEntry->coord[5],EnvROBARM.startHashEntry->coord[6]);
        printf("Start: %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f\n",startangles[0],startangles[1],startangles[2],startangles[3],startangles[4],startangles[5],startangles[6]);
        printf("Start: endeff (%u %u %u)  wrist: (%u %u %u)  elbow: (%u %u %u)\n",EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);
        printf("Start Hash Entry is invalid\n");
//         return false;
    }
    else
    {
        printf("\n[SetStartJointConfig] Start configuration has been set successfully.\n");
        printf("Start: %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f\n",startangles[0],startangles[1],startangles[2],startangles[3],startangles[4],startangles[5],startangles[6]);
        printf("Start: endeff (%u %u %u)  wrist: (%u %u %u)  elbow: (%u %u %u)\n",EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);
    }

    return true;
}

void EnvironmentROBARM3D::StateID2Angles(int stateID, double* angles_r)
{
    bool bGoal = false;
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[stateID];

    if(stateID == EnvROBARM.goalHashEntry->stateID)
        bGoal = true;

    if(bGoal)
        ComputeContAngles(EnvROBARMCfg.goalcoords, angles_r);
    else
        ComputeContAngles(HashEntry->coord, angles_r);

    //convert angles from positive values in radians (from 0->6.28) to centered around 0 (not really needed)
    for (int i = 0; i < NUMOFLINKS; i++)
    {
        if(angles_r[i] >= PI_CONST)
            angles_r[i] = -2.0*PI_CONST + angles_r[i];
    }

    //debugging output
    double pnt[3] = {0};
    Cell2ContXYZ(HashEntry->endeff[0],HashEntry->endeff[1],HashEntry->endeff[2],&(pnt[0]),&(pnt[1]),&(pnt[2]));
    for(int i=0; i < NUMOFLINKS; i++)
        printf("%.3f ",angles_r[i]);
    printf("     endeff: %.2f %.2f %.2f\n",pnt[0],pnt[1],pnt[2]);
}

void EnvironmentROBARM3D::AddObstaclesToEnv(double**obstacles, int numobstacles)
{
    int i,p;
    double obs[6], angles[NUMOFLINKS], orientation[3][3];
    short unsigned int endeff[3],elbow[3],wrist[3], coord[7];

    for(i = 0; i < numobstacles; i++)
    {
        //check if obstacle is within the arm's reach - stupid way of doing this - change it
        if(EnvROBARMCfg.arm_length < fabs(obstacles[i][0]) - obstacles[i][3] - EnvROBARMCfg.padding ||
           EnvROBARMCfg.arm_length < fabs(obstacles[i][1]) - obstacles[i][4] - EnvROBARMCfg.padding ||
           EnvROBARMCfg.arm_length < fabs(obstacles[i][2]) - obstacles[i][5] - EnvROBARMCfg.padding)
        {
            printf("[AddObstaclesToEnv] Obstacle %i, centered at (%1.3f %1.3f %1.3f), is out of the arm's workspace.\n",i,obstacles[i][0],obstacles[i][1],obstacles[i][2]);
            continue;
        }

        for(p = 0; p < 6; p++)
            obs[p] = obstacles[i][p];

        AddObstacleToGrid(obs,0, EnvROBARMCfg.Grid3D, EnvROBARMCfg.GridCellWidth);

        if(EnvROBARMCfg.lowres_collision_checking)
            AddObstacleToGrid(obs,0, EnvROBARMCfg.LowResGrid3D, EnvROBARMCfg.LowResGridCellWidth);

        printf("[AddObstaclesToEnv] Obstacle %i was added to the environment.\n",i);
    }

    //pre-compute heuristics
    if(EnvROBARMCfg.dijkstra_heuristic)
        ComputeHeuristicValues();

    //compute forward kinematics
    ComputeContAngles(EnvROBARM.startHashEntry->coord, angles);
    if(!ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation))
    {
        printf("[AddObstaclesToEnv] Start position is invalid after adding obstacles.\n"); //this should never be true from gazebo
        exit(1);
    }


    //check if the starting position and goal are still valid
    if(!IsValidCoord(EnvROBARM.startHashEntry->coord,EnvROBARM.startHashEntry->endeff,wrist,elbow,EnvROBARM.startHashEntry->orientation))
    {
        printf("Start Hash Entry is invalid after adding obstacles.\n");
        exit(1);
    }

       //check if goals are still valid
    if(EnvROBARMCfg.PlanInJointSpace)
    {
        for (i = 0; i < EnvROBARMCfg.nJointSpaceGoals; i++)
        {
            for(p = 0; p < NUMOFLINKS; p++)
                angles[p] = EnvROBARMCfg.JointSpaceGoals[i][p];

            //get joint positions of starting configuration
            if(!ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation))
            {
                printf("[AddObstaclesToEnv] goal %i: (%.2f %.2f %.2f %.2f %.2f %.2f %.2f) is out of bounds.\n",
                       i,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
                if(!RemoveGoal(i))
                {
                    EnvROBARMCfg.bGoalIsSet  = false;
                    return;
                }
                else
                {
                    printf("[AddObstaclesToEnv] Removed goal %i. %i goals remaining.\n", i, EnvROBARMCfg.nJointSpaceGoals);
                }
            }

            //get coords
            ComputeCoord(angles, coord);
//             printf("[AddObstacles] goal angles %i: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
//                    i,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
//             printf("[AddObstacles] goal coord %i: %i %i %i %i %i %i %i\n",
//                    i,coord[0],coord[1],coord[2],coord[3],coord[4],coord[5],coord[6]);

            //check if starting position is valid
            if(!IsValidCoord(coord,endeff,wrist,elbow,orientation))
            {
                printf("[AddObstaclesToEnv] goal %i: (%.2f %.2f %.2f %.2f %.2f %.2f %.2f) is invalid. Removing it.\n",
                       i,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
                if(!RemoveGoal(i))
                {
                    EnvROBARMCfg.bGoalIsSet  = false;
                    return;
                }
                else
                {
                    printf("[AddObstaclesToEnv] Removed goal %i. %i goals remaining.\n", i, EnvROBARMCfg.nJointSpaceGoals);
                }
            }
        }
    }
    else
    {
        for (i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
        {
            if(EnvROBARMCfg.Grid3D[EnvROBARMCfg.EndEffGoals_c[i][0]][EnvROBARMCfg.EndEffGoals_c[i][1]][EnvROBARMCfg.EndEffGoals_c[i][2]] >= EnvROBARMCfg.ObstacleCost)
            {
                printf("End Effector Goal(%u %u %u) is invalid after adding obstacles.\n",
                    EnvROBARMCfg.EndEffGoals_c[i][0],EnvROBARMCfg.EndEffGoals_c[i][1],EnvROBARMCfg.EndEffGoals_c[i][2]);
                if(!RemoveGoal(i))
                {
                    EnvROBARMCfg.bGoalIsSet  = false;
                    return;
                }
                else
                {
                    printf("[AddObstacles] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                }
            }
        }
    }
}

void EnvironmentROBARM3D::AddObstacles(vector<vector <double> > obstacles)
{
    int i,p, cubes_added=0;
    short unsigned int endeff[3],elbow[3],wrist[3], coord[NUMOFLINKS];
    double obs[6], angles[NUMOFLINKS], orientation[3][3];

    //NOTE insert a mutex around bPlanning
//     if(EnvROBARMCfg.bUpdatePlanningGrid)
//     {
//         printf("[AddObstacles] Copying temporary grid to planning grid...obstacles won't be added to the map.\n");
//         return;
//     }

    for(i = 0; i < (int)obstacles.size(); i++)
    {
        //check if obstacle has 6 parameters --> {X,Y,Z,W,H,D}
        if(obstacles[i].size() < 6)
        {
//             printf("[AddObstacles] Obstacle %i has too few parameters.Skipping it.\n",i);
            continue;
        }

        //throw out obstacles if they are too small
        if(obstacles[i][3] <.001 || obstacles[i][4] <.001 || obstacles[i][5] <.001)
        {
//             printf("[AddObstacles] Obstacle %i is too small.Skipping it.\n",i);
            continue;
        }


        //check if obstacle is within the arm's reach - stupid way of doing this - change it
        if(EnvROBARMCfg.arm_length < fabs(obstacles[i][0]) - obstacles[i][3] - EnvROBARMCfg.padding ||
            EnvROBARMCfg.arm_length < fabs(obstacles[i][1]) - obstacles[i][4] - EnvROBARMCfg.padding ||
            EnvROBARMCfg.arm_length < fabs(obstacles[i][2]) - obstacles[i][5] - EnvROBARMCfg.padding)
        {
//             printf("[AddObstacles] Obstacle %i, centered at (%1.3f %1.3f %1.3f), is out of the arm's workspace.\n",i,obstacles[i][0],obstacles[i][1],obstacles[i][2]);
            continue;
        }
        else
        {
            for(p = 0; p < 6; p++)
                obs[p] = obstacles[i][p];

            //wrap mmutexes around the temp grid
            EnvROBARMCfg.mCopyingGrid.lock();

            //NOTE putting new obstacles in the temporary maps
//             AddObstacleToGrid(obs,0, EnvROBARMCfg.Grid3D, EnvROBARMCfg.GridCellWidth);
            AddObstacleToGrid(obs,0, EnvROBARMCfg.Grid3D_temp, EnvROBARMCfg.GridCellWidth);

            if(EnvROBARMCfg.lowres_collision_checking)
                AddObstacleToGrid(obs,0, EnvROBARMCfg.LowResGrid3D_temp, EnvROBARMCfg.LowResGridCellWidth);
//                 AddObstacleToGrid(obs,0, EnvROBARMCfg.LowResGrid3D, EnvROBARMCfg.LowResGridCellWidth);

            EnvROBARMCfg.mCopyingGrid.unlock();

//             printf("[AddObstacles] Obstacle %i: (%.2f %.2f %.2f) was added to the environment.\n",i,obs[0],obs[1],obs[2]);
            cubes_added++;
        }
    }



    printf("[AddObstacles] %i cubic obstacles were added to the environment.\n",cubes_added);

    //check if start position is still valid
    ComputeContAngles(EnvROBARM.startHashEntry->coord, angles);
    if(!ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation))
    {
        printf("[AddObstacles] Start position is invalid after adding obstacles.\n");
//         exit(1); // TODO shouldnt be here when adding obstacles, before updating start position....perhaps add aflag to know the start isnt set?
    }

    if(!IsValidCoord(EnvROBARM.startHashEntry->coord,EnvROBARM.startHashEntry->endeff,wrist,elbow,EnvROBARM.startHashEntry->orientation))
    {
        printf("[AddObstacles] Start Hash Entry is invalid after adding obstacles.\n");
//         exit(1);
    }

    //check if goals are still valid
    if(EnvROBARMCfg.PlanInJointSpace)
    {
        i = 0;
//         for (i = 0; i < EnvROBARMCfg.nJointSpaceGoals; i++)
        while(i >= 0 && i < EnvROBARMCfg.nJointSpaceGoals)
        {
            for(p = 0; p < NUMOFLINKS; p++)
                angles[p] = EnvROBARMCfg.JointSpaceGoals[i][p];

            //get joint positions of goal configurations
            if(!ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation))
            {
                printf("[AddObstacles] goal %i: (%.2f %.2f %.2f %.2f %.2f %.2f %.2f) is out of bounds.\n",
                       i,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
                if(!RemoveGoal(i))
                {
                    EnvROBARMCfg.bGoalIsSet  = false;
                    return;
                }
                else
                {
                    printf("[AddObstacles] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                }
                continue;
            }

            printf("[AddObstacles] goal %i: elbow (%u %u %u) wrist (%u %u %u) endeff (%u %u %u)\n",
                   i,elbow[0],elbow[1],elbow[2],wrist[0],wrist[1],wrist[2],endeff[0],endeff[1],endeff[2]);

            //get coords
            ComputeCoord(angles, coord);

            //check if goal position is valid
            if(!IsValidCoord(coord,endeff,wrist,elbow,orientation))
            {
                printf("[AddObstacles] goal %i: (%.2f %.2f %.2f %.2f %.2f %.2f %.2f) is invalid. Removing it.\n",
                       i,angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
                if(!RemoveGoal(i))
                {
                    EnvROBARMCfg.bGoalIsSet  = false;
                    return;
                }
                else
                {
                    printf("[AddObstacles] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                }
            }

            i++;
        }
    }
    else
    {
//         for (i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
        while(i >= 0 && i < EnvROBARMCfg.nEndEffGoals)
        {
            if(EnvROBARMCfg.Grid3D[EnvROBARMCfg.EndEffGoals_c[i][0]][EnvROBARMCfg.EndEffGoals_c[i][1]][EnvROBARMCfg.EndEffGoals_c[i][2]] >= EnvROBARMCfg.ObstacleCost)
            {
                printf("[AddObstacles] End Effector Goal(%u %u %u) is invalid after adding obstacles.\n",
                        EnvROBARMCfg.EndEffGoals_c[i][0],EnvROBARMCfg.EndEffGoals_c[i][1],EnvROBARMCfg.EndEffGoals_c[i][2]);
                if(!RemoveGoal(i))
                {
                    EnvROBARMCfg.bGoalIsSet  = false;
                    return;
                }
                else
                {
                    printf("[AddObstacles] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                }
            }
        }
    }

    //pre-compute heuristics
    if(EnvROBARMCfg.dijkstra_heuristic)
        ComputeHeuristicValues();

    printf("[AddObstacles] %i cubes are in the cube vector\n",EnvROBARMCfg.cubes.size());
//     printf("[AddObstacles] %i big cubes are in the cube vector\n",EnvROBARMCfg.bigcubes.size());
}

std::vector<std::vector<double> >* EnvironmentROBARM3D::getCollisionMap()
{
//     for(unsigned int i=0; i < EnvROBARMCfg.cubes.size(); i++)
//     {
//         printf("[sendCollisionMap] cube %i:, ",i);
//         for(unsigned int k=0; k < EnvROBARMCfg.cubes[i].size(); k++)
//             printf("%.3f ",EnvROBARMCfg.cubes[i][k]);
//         printf("\n");
//     }
    return &(EnvROBARMCfg.cubes);
}

void EnvironmentROBARM3D::ClearEnv()
{
    int x, y, z;

    //if the grid is be used for planning then don't clear the obstacle list being sent to gazebo
//     EnvROBARMCfg.mPlanningGrid.lock();

//     EnvROBARMCfg.mPlanningGrid.unlock();

    //clear collision map
//     if(!EnvROBARMCfg.bUpdatePlanningGrid)
//     {

    //clear temporary collision map
    EnvROBARMCfg.mCopyingGrid.lock();
    printf("[ClearEnv] Clearing environment of all obstacles.\n");

    EnvROBARMCfg.cubes.clear();
    
//     if(!EnvROBARMCfg.bPlanning)
//         EnvROBARMCfg.cubes.clear();

    // set all cells to zero
    for (x = 0; x < EnvROBARMCfg.EnvWidth_c; x++)
    {
        for (y = 0; y < EnvROBARMCfg.EnvHeight_c; y++)
        {
            for (z = 0; z < EnvROBARMCfg.EnvDepth_c; z++)
            {
                EnvROBARMCfg.Grid3D_temp[x][y][z] = 0;
            }
        }
    }

    // set all cells to zero in lowres grid
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        for (x = 0; x < EnvROBARMCfg.LowResEnvWidth_c; x++)
        {
            for (y = 0; y < EnvROBARMCfg.LowResEnvHeight_c; y++)
            {
                for (z = 0; z < EnvROBARMCfg.LowResEnvDepth_c; z++)
                {
                    EnvROBARMCfg.LowResGrid3D_temp[x][y][z] = 0;
                }
            }
        }
    }
    EnvROBARMCfg.mCopyingGrid.unlock();

//     for (z = 45; z <= 50; z++)
//     {
//         printf("\n[ClearEnv] z = %i\n",z);
//         for (x = 0; x < EnvROBARMCfg.LowResEnvWidth_c; x++) 
//         {
//             for (y = 0; y < EnvROBARMCfg.LowResEnvHeight_c; y++)
//                 printf("%d ",EnvROBARMCfg.LowResGrid3D[x][y][z]);
//             printf("\n");
//         }
//         printf("\n");
//     }
}

bool EnvironmentROBARM3D::isPathValid(double** path, int num_waypoints)
{
    double angles[NUMOFLINKS], orientation[3][3];
    short unsigned int endeff[3], wrist[3], elbow[3], coord[NUMOFLINKS];

    //loop through each waypoint of the path and check if it's valid
    for(int i = 0; i < num_waypoints; i++)
    {
        //compute FK
        ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation);

        for(int k = 0; k < NUMOFLINKS; k++)
        {
            angles[k] = path[i][k]*(180.0/PI_CONST);

            if (angles[k] < 0)
                angles[k] += 360;
        }

        //compute coords
        ComputeCoord(angles, coord);

        //check if valid
        if(!IsValidCoord(coord,endeff, wrist, elbow, orientation))
        {
            printf("[isPathValid] Path is invalid.\n");
            return false;
        }
    }

    printf("Path is valid.\n");
    return true;
}

//copied from getEulerZYX() in Bullet physics library
void EnvironmentROBARM3D::getRPY(double Rot[3][3], double* roll, double* pitch, double* yaw, int solution_number)
{
    double delta,rpy1[3],rpy2[3];

     // Check that pitch is not at a singularity
    if(fabs(Rot[0][2]) >= 1)
    {
        rpy1[2]  = 0;
        rpy2[2]  = 0;

        // From difference of angles formula
        delta = atan2(Rot[0][0], Rot[2][0]);
        if(Rot[0][2] > 0)   //gimbal locked up
        {
            rpy1[1] = PI_CONST / 2.0;
            rpy2[1] = PI_CONST / 2.0;
            rpy1[0] = rpy1[1] + delta;
            rpy2[0] = rpy2[1] + delta;
        }
        else // gimbal locked down
        {
            rpy1[1] = -PI_CONST / 2.0;
            rpy2[1] = -PI_CONST / 2.0;
            rpy1[0] = -rpy1[1] + delta;
            rpy2[0] = -rpy2[1] + delta;
        }
    }
    else
    {
        rpy1[1] = -asin(Rot[0][2]);
        rpy2[1] = PI_CONST - rpy1[1];


        rpy1[0] = atan2(Rot[1][2]/cos(rpy1[1]),
                        Rot[2][2]/cos(rpy1[1]));

        rpy2[0] = atan2(Rot[1][2]/cos(rpy2[1]),
                        Rot[2][2]/cos(rpy2[1]));

        rpy1[2] = atan2(Rot[0][1]/cos(rpy1[1]),
                        Rot[0][0]/cos(rpy1[1]));

        rpy2[2] = atan2(Rot[0][1]/cos(rpy2[1]),
                        Rot[0][0]/cos(rpy2[1]));
    }

    if (solution_number == 1)
    {
        *yaw = rpy1[2];
        *pitch = rpy1[1];
        *roll = rpy1[0];
    }
    else
    {
        *yaw = rpy2[2];
        *pitch = rpy2[1];
        *roll = rpy2[0];
    }
}

double EnvironmentROBARM3D::getAngularEuclDist(double rpy1[3], double rpy2[3])
{
    double rdiff = fabs(angles::shortest_angular_distance(rpy1[0], rpy2[0]));
    double pdiff = fabs(angles::shortest_angular_distance(rpy1[1], rpy2[1]));
    double ydiff = fabs(angles::shortest_angular_distance(rpy1[2], rpy2[2]));

    return sqrt(rdiff*rdiff + pdiff*pdiff + ydiff*ydiff);
}
//--------------------------------------------------------------


/*------------------------------------------------------------------------*/
                        /* Printing Routines */
/*------------------------------------------------------------------------*/
void EnvironmentROBARM3D::PrintHeurGrid()
{
    int x,y,z;	
    int width = EnvROBARMCfg.EnvWidth_c;
    int height = EnvROBARMCfg.EnvHeight_c;
    int depth = EnvROBARMCfg.EnvDepth_c;
    if(EnvROBARMCfg.lowres_collision_checking)
    {
        width = EnvROBARMCfg.LowResEnvWidth_c;
        height = EnvROBARMCfg.LowResEnvHeight_c;
        depth =  EnvROBARMCfg.LowResEnvDepth_c;
    }

    for (x = 0; x < width; x++)
    {
        printf("\nx = %i\n",x);
        for (y = 0; y < height; y++)
        {
            for(z = 0; z < depth; z++)
            {
                printf("%3.0u ",EnvROBARM.Heur[XYZTO3DIND(x,y,z)]/COSTMULT);
            }
            printf("\n");
        }
        printf("\n");
    }
}

void EnvironmentROBARM3D::printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal)
{
    double angles[NUMOFLINKS];
    int dangles[NUMOFLINKS];
    int i;
    short unsigned int endeff[3];

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


    if(bGoal)
    {
        endeff[0] = EnvROBARM.goalHashEntry->endeff[0];
        endeff[1] = EnvROBARM.goalHashEntry->endeff[1];
        endeff[2] = EnvROBARM.goalHashEntry->endeff[2];
    }
    else
    {
        ComputeEndEffectorPos(angles, endeff);
    }

    if(bVerbose)
        fprintf(fOut, "   endeff: %d %d %d", endeff[0],endeff[1],endeff[2]);
    else
        fprintf(fOut, "%d %d %d", endeff[0],endeff[1],endeff[2]);

    fprintf(fOut, "\n");
}

void EnvironmentROBARM3D::PrintAnglesWithAction(FILE* fOut, EnvROBARMHashEntry_t* HashEntry, bool bGoal, bool bVerbose, bool bLocal)
{
    double angles[NUMOFLINKS];
    int dangles[NUMOFLINKS];
    int i;

    //convert to angles
    ComputeContAngles(HashEntry->coord, angles);

//     if(bGoal)
//         ComputeContAngles(EnvROBARMCfg.goalcoords, angles);
//     else
//         ComputeContAngles(HashEntry->coord, angles);

    //convert to degrees
    if(bVerbose)
    {
        for (i = 0; i < NUMOFLINKS; i++)
        {
            dangles[i] = angles[i]*(180.0/PI_CONST) + 0.999999;
        }
    }

    if(bVerbose)
        fprintf(fOut, "angles: ");

#if OUPUT_DEGREES
    for(i = 0; i < NUMOFLINKS; i++)
    {
        if(!bLocal)
            fprintf(fOut, "%-3i ", dangles[i]);
        else
        {
            if(i > 0)
                fprintf(fOut, "%-3i ", dangles[i]-dangles[i-1]);
            else
                fprintf(fOut, "%-3i ", dangles[i]);
        }
    }
#else
    for(i = 0; i < NUMOFLINKS; i++)
    {
        if(!bLocal)
            fprintf(fOut, "%-.3f ", angles[i]);
        else
        {
            if(i > 0)
                fprintf(fOut, "%-.3f ", angles[i]-angles[i-1]);
            else
                fprintf(fOut, "%-.3f ", angles[i]);
        }
    }
#endif

    if(bVerbose)
        fprintf(fOut, "  endeff: %-2d %-2d %-2d   action: %d", HashEntry->endeff[0],HashEntry->endeff[1],HashEntry->endeff[2], HashEntry->action);
    else
        fprintf(fOut, "%-2d %-2d %-2d %-2d", HashEntry->endeff[0],HashEntry->endeff[1],HashEntry->endeff[2],HashEntry->action);

    fprintf(fOut, "\n");
}

void EnvironmentROBARM3D::PrintConfiguration()
{
    int i;
    double pX, pY, pZ;
    double start_angles[NUMOFLINKS], orientation[3][3];
    short unsigned int wrist[3],elbow[3],endeff[3];

    printf("\nEnvironment/Robot Details:\n");
    printf("Grid Cell Width: %.2f cm\n",EnvROBARMCfg.GridCellWidth*100);

    Cell2ContXYZ(EnvROBARMCfg.BaseX_c,EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c,&pX, &pY, &pZ);
    printf("Shoulder Base: %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",EnvROBARMCfg.BaseX_c,EnvROBARMCfg.BaseY_c, EnvROBARMCfg.BaseZ_c,pX,pY,pZ);

    for(i = 0; i < NUMOFLINKS; i++)
        start_angles[i] = DEG2RAD(EnvROBARMCfg.LinkStartAngles_d[i]);

    ComputeEndEffectorPos(start_angles, endeff, wrist, elbow,orientation);
    Cell2ContXYZ(elbow[0],elbow[1],elbow[2],&pX, &pY, &pZ);
    printf("Elbow Start:   %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",elbow[0],elbow[1],elbow[2],pX,pY,pZ);

    Cell2ContXYZ(wrist[0],wrist[1],wrist[2],&pX, &pY, &pZ);
    printf("Wrist Start:   %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",wrist[0],wrist[1],wrist[2],pX,pY,pZ);

    Cell2ContXYZ(endeff[0],endeff[1],endeff[2], &pX, &pY, &pZ);
    printf("End Effector Start: %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",endeff[0],endeff[1],endeff[2],pX,pY,pZ);

    if(EnvROBARMCfg.bGoalIsSet)
    {
        for(i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
            printf("End Effector Goal:  %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",EnvROBARMCfg.EndEffGoals_c[i][0], EnvROBARMCfg.EndEffGoals_c[i][1],
                   EnvROBARMCfg.EndEffGoals_c[i][2],EnvROBARMCfg.EndEffGoals_m[i][0],EnvROBARMCfg.EndEffGoals_m[i][1],EnvROBARMCfg.EndEffGoals_m[i][2]);
    }

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
            if (EnvROBARMCfg.Grid3D[x][y][z] >= EnvROBARMCfg.ObstacleCost)
                printf("(%i,%i,%i) ",x,y,z);
        }
    }
    printf("\nThe occupancy grid contains %i obstacle cells.\n",sum); 
#endif

    printf("\nDH Parameters:\n");
    printf("LinkTwist: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_alpha[i]);
    printf("\nLinkLength: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_a[i]);
    printf("\nLinkOffset: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_d[i]);
    printf("\nJointAngles: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%.2f  ",EnvROBARMCfg.DH_theta[i]);

    printf("\n\nMotor Limits:\n");
    printf("PosMotorLimits: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%1.2f  ",EnvROBARMCfg.PosMotorLimits[i]);
    printf("\nNegMotorLimits: ");
    for(i=0; i < NUMOFLINKS; i++) 
        printf("%1.2f  ",EnvROBARMCfg.NegMotorLimits[i]);
    printf("\n\n");
}

void EnvironmentROBARM3D::PrintAbridgedConfiguration()
{
    double pX, pY, pZ;
    double start_angles[NUMOFLINKS];
    short unsigned int endeff[3];

    ComputeEndEffectorPos(start_angles, endeff);
    Cell2ContXYZ(endeff[0],endeff[1],endeff[2], &pX, &pY, &pZ);
    printf("End Effector Start: %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",endeff[0],endeff[1],endeff[2],pX,pY,pZ);

    for(int i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
        printf("End Effector Goal:  %i %i %i (cells) --> %.3f %.3f %.3f (meters)\n",EnvROBARMCfg.EndEffGoals_c[i][0], EnvROBARMCfg.EndEffGoals_c[i][1],
               EnvROBARMCfg.EndEffGoals_c[i][2],EnvROBARMCfg.EndEffGoals_m[i][0],EnvROBARMCfg.EndEffGoals_m[i][1],EnvROBARMCfg.EndEffGoals_m[i][2]);

    printf("\n");
}

//display action cost table
void EnvironmentROBARM3D::OutputActionCostTable()
{
    int x,y;
    printf("\nAction Cost Table\n");
    for (x = 0; x < EnvROBARMCfg.nSuccActions; x++)
    {
        printf("%i: ",x); 
        for (y = 0; y < EnvROBARMCfg.nSuccActions; y++)
            printf("%i  ",EnvROBARMCfg.ActiontoActionCosts[x][y]);
        printf("\n");
    }
    printf("\n");
}

//display successor actions
void EnvironmentROBARM3D::OutputActions()
{
    int x,y;
    printf("\nSuccessor Actions\n");
    for (x=0; x < EnvROBARMCfg.nSuccActions; x++)
    {
        printf("%i:  ",x);
        for(y=0; y < NUMOFLINKS; y++)
            printf("%.1f  ",EnvROBARMCfg.SuccActions[x][y]);

        printf("\n");
    }
}

void EnvironmentROBARM3D::OutputPlanningStats()
{
    printf("\n# Possible Actions: %d\n",EnvROBARMCfg.nSuccActions);
//     printf("# GetHashEntry Calls: %i  computed in %3.2f sec\n",num_GetHashEntry,time_gethash/(double)CLOCKS_PER_SEC); 
    printf("# Forward Kinematic Computations: %i  Time per Computation: %.3f (usec)\n",num_forwardkinematics,(DH_time/(double)CLOCKS_PER_SEC)*1000000 / num_forwardkinematics);
    printf("Cost per Cell: %.2f   *sqrt(2): %.2f   *sqrt(3):  %.2f\n",EnvROBARMCfg.cost_per_cell,EnvROBARMCfg.cost_sqrt2_move,EnvROBARMCfg.cost_sqrt3_move);

    if(EnvROBARMCfg.use_DH)
        printf("DH Convention: %3.2f sec\n", DH_time/(double)CLOCKS_PER_SEC);
    else
        printf("Kinematics Library: %3.2f sec\n", KL_time/(double)CLOCKS_PER_SEC);

    printf("Collision Checking: %3.2f sec\n", check_collision_time/(double)CLOCKS_PER_SEC);
    printf("\n");
}
//--------------------------------------------------------------


/*------------------------------------------------------------------------*/
                        /* Forward Kinematics */
/*------------------------------------------------------------------------*/
void EnvironmentROBARM3D::InitializeKinNode() //needed when using kinematic library
{
    clock_t currenttime = clock();

    char *c_filename = getenv("ROS_PACKAGE_PATH");
    std::stringstream filename;
    filename << c_filename << "/robot_descriptions/pr2/pr2_defs/robots/pr2_temp.xml" ;
    EnvROBARMCfg.pr2_kin.loadXML(filename.str());

    EnvROBARMCfg.left_arm = EnvROBARMCfg.pr2_kin.getSerialChain("right_arm");
    assert(EnvROBARMCfg.left_arm);
    EnvROBARMCfg.pr2_config = new JntArray(EnvROBARMCfg.left_arm->num_joints_);

    KL_time += clock() - currenttime;
}

void EnvironmentROBARM3D::CloseKinNode()
{
//     ros::fini();
    sleep(1);
}

//pre-compute DH matrices for all 7 frames with NUM_TRANS_MATRICES degree resolution 
void EnvironmentROBARM3D::ComputeDHTransformations()
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
    for (n = 0; n < NUMOFLINKS; n++)
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

//uses DH convention
void EnvironmentROBARM3D::ComputeForwardKinematics_DH(double angles[NUMOFLINKS])
{
    double sum, theta[NUMOFLINKS];
    int t,x,y,k;
    double temp3[4][4],temp5[4][4], temp7[4][4];
    double R_elbow[4][4] = {{0, 0, -1, 0},
                            {-1, 0, 0, 0},
                            {0, 1, 0, 0},
                            {0, 0, 0, 1}};
    double R_wrist[4][4] = {{0, 0, -1, 0},
                            {-1, 0, 0, 0},
                            {0, 1, 0, 0},
                            {0, 0, 0, 1}};
    double R_endeff[4][4] = {{0, 0, -1, 0},
                            {0, 1, 0, 0},
                            {1, 0, 0, 0},
                            {0, 0, 0, 1}};

    //add any theta offsets to the joint position angles   <--move this into  next loop
    for(x=0; x < NUMOFLINKS; x++)
        theta[x] = angles[x] + EnvROBARMCfg.DH_theta[x];

    // compute transformation matrices and premultiply
    for(int i = 0; i < NUMOFLINKS; i++)
    {
        //convert theta to degrees
        theta[i]=theta[i]*(180.0/PI_CONST);

        //make sure theta is not negative
        if (theta[i] < 0)
            theta[i] = 360.0+theta[i];

        //round to nearest integer
        t = theta[i] + 0.5;

        // bug fix! this is a stupid design....redo this function
        if(t >= 360)
            t = 0;

        //multiply by 4 to get to correct position in T_DH
        t = t*4;

        //multiply by previous transformations to put in shoulder frame
        if(i > 0)
        {
            for (x=0; x<4; x++)
            {
                for (y=0; y<4; y++)
                {
                    sum = 0;
                    for (k=0; k<4; k++)
                        sum += EnvROBARM.Trans[x][k][i-1] * EnvROBARMCfg.T_DH[t+k][y][i];

                    EnvROBARM.Trans[x][y][i] = sum;
                }
            }

            //elbow - rotate axis to be the same as the PR2's coordinate system
            if(i == ELBOW_JOINT)
            {
//             printf("elbow:\n");
                for (x=0; x<4; x++)
                {
                    for (y=0; y<4; y++)
                    {
                        sum = 0;
                        for (k=0; k<4; k++)
                            sum += EnvROBARM.Trans[x][k][i] * R_elbow[k][y];

//                         printf("%.2f ",sum);
                        temp3[x][y] = sum;
                    }
//                 printf("\n");
                }
            }
            else if(i == WRIST_JOINT)
            {
//                 printf("wrist:\n");
                for (x=0; x<4; x++)
                {
                    for (y=0; y<4; y++)
                    {
                        sum = 0;
                        for (k=0; k<4; k++)
                            sum += EnvROBARM.Trans[x][k][i] * R_wrist[k][y];

//                         printf("%.2f ",sum);
                        temp5[x][y] = sum;
                    }
//                     printf("\n");
                }
            }
            else if(i == ENDEFF)
            {
//                 printf("endeff:\n");
                for (x=0; x<4; x++)
                {
                    for (y=0; y<4; y++)
                    {
                        sum = 0;
                        for (k=0; k<4; k++)
                            sum += EnvROBARM.Trans[x][k][i] * R_endeff[k][y];

//                         printf("%.2f ",sum);
                        temp7[x][y] = sum;
                    }
//                     printf("\n");
                }

                //copy it over to the
//                 printf("EnvROBARM.Trans[x][y][7]: \n");
                for (x=0; x<4; x++)
                {
                    for (y=0; y<4; y++)
                    {
                        EnvROBARM.Trans[x][y][ELBOW_JOINT] = temp3[x][y];
                        EnvROBARM.Trans[x][y][WRIST_JOINT] = temp5[x][y];
                        EnvROBARM.Trans[x][y][ENDEFF] = temp7[x][y];
//                         printf("%.2f ",EnvROBARM.Trans[x][y][7]);
                    }
//                     printf("\n");
                }
            }
        }
        else
        {
            for (x=0; x<4; x++)
                for (y=0; y<4; y++)
                    EnvROBARM.Trans[x][y][0] = EnvROBARMCfg.T_DH[t+x][y][0];
        }
    }
}

//uses ros's KDL Library
void EnvironmentROBARM3D::ComputeForwardKinematics_ROS(double *angles, int f_num, double *x, double *y, double*z)
{
    Frame f, f2;
    KDL::Vector gripper(0.0, 0.0, EnvROBARMCfg.DH_d[NUMOFLINKS-1]);

    for(int i = 0; i < NUMOFLINKS; i++)
        (*EnvROBARMCfg.pr2_config)(i) = angles[i];

    EnvROBARMCfg.left_arm->computeFK((*EnvROBARMCfg.pr2_config),f,f_num);

    //add translation from wrist to end of fingers
    if(f_num == 7)
    {
        f.p = f.M*gripper + f.p;

        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                EnvROBARM.Trans[i][j][7] = f.M(i,j);
    }

    //translate xyz coordinates from shoulder frame to base frame
    *x = f.p[0] + EnvROBARMCfg.BaseX_m;
    *y = f.p[1] + EnvROBARMCfg.BaseY_m;
    *z = f.p[2] + EnvROBARMCfg.BaseZ_m;
}

//pre-compute action costs
void EnvironmentROBARM3D::ComputeActionCosts()
{
    int i,x,y;
    double temp = 0.0;
    EnvROBARMCfg.ActiontoActionCosts = new int* [EnvROBARMCfg.nSuccActions];
    for (x = 0; x < EnvROBARMCfg.nSuccActions; x++)
    {
        EnvROBARMCfg.ActiontoActionCosts[x] = new int[EnvROBARMCfg.nSuccActions];
        for (y = 0; y < EnvROBARMCfg.nSuccActions; y++)
        {
            temp = 0.0;
            for (i = 0; i < NUMOFLINKS; i++)
            {
                temp += ((EnvROBARMCfg.SuccActions[x][i]-EnvROBARMCfg.SuccActions[y][i])*(EnvROBARMCfg.SuccActions[x][i]-EnvROBARMCfg.SuccActions[y][i]));
            }
            EnvROBARMCfg.ActiontoActionCosts[x][y] = temp * COSTMULT * EnvROBARMCfg.smoothing_weight * EnvROBARMCfg.use_smooth_actions;
        }
    }
}

void EnvironmentROBARM3D::ComputeCostPerCell()
{
    double angles[NUMOFLINKS],start_angles[NUMOFLINKS]={0};
    double eucl_dist, max_dist = 0;
    int largest_action=0;
    double start_endeff_m[3], endeff_m[3];
    double gridcell_size;

    if(EnvROBARMCfg.lowres_collision_checking)
        gridcell_size = EnvROBARMCfg.GridCellWidth;
    else
        gridcell_size = EnvROBARMCfg.LowResGridCellWidth;

    //starting at zeroed angles, find end effector position after each action
    if(EnvROBARMCfg.use_DH)
        ComputeEndEffectorPos(start_angles,start_endeff_m);
    else
        ComputeForwardKinematics_ROS(start_angles, 7, &(start_endeff_m[0]), &(start_endeff_m[1]), &(start_endeff_m[2]));

    //iterate through all possible actions and find the one with the minimum cost per cell
    for (int i = 0; i < EnvROBARMCfg.nSuccActions; i++)
    {
        for(int j = 0; j < NUMOFLINKS; j++)
            angles[j] = DEG2RAD(EnvROBARMCfg.SuccActions[i][j]);// * (360.0/ANGLEDELTA));

        //starting at zeroed angles, find end effector position after each action
        if(EnvROBARMCfg.use_DH)
            ComputeEndEffectorPos(angles,endeff_m);
        else
            ComputeForwardKinematics_ROS(angles, 7, &(endeff_m[0]), &(endeff_m[1]), &(endeff_m[2]));

        eucl_dist = sqrt((start_endeff_m[0]-endeff_m[0])*(start_endeff_m[0]-endeff_m[0]) +
                (start_endeff_m[1]-endeff_m[1])*(start_endeff_m[1]-endeff_m[1]) +
                (start_endeff_m[2]-endeff_m[2])*(start_endeff_m[2]-endeff_m[2]));

        if (eucl_dist > max_dist)
        {
            max_dist = eucl_dist;
            largest_action = i;
        }
    }

    EnvROBARMCfg.CellsPerAction = max_dist/gridcell_size;
    EnvROBARMCfg.cost_per_cell = COSTMULT/EnvROBARMCfg.CellsPerAction;
    EnvROBARMCfg.cost_sqrt2_move = sqrt(2.0)*EnvROBARMCfg.cost_per_cell;
    EnvROBARMCfg.cost_sqrt3_move = sqrt(3.0)*EnvROBARMCfg.cost_per_cell;
    EnvROBARMCfg.cost_per_mm = EnvROBARMCfg.cost_per_cell / (gridcell_size * 1000);

    double max_angle = 0;
    for(int i=0; i < NUMOFLINKS; i++)
    {
        if(EnvROBARMCfg.SuccActions[largest_action][i] > max_angle)
            max_angle = EnvROBARMCfg.SuccActions[largest_action][i];
    }
    double mm_per_rad = (max_dist*1000.0) / DEG2RAD(max_angle);
    EnvROBARMCfg.cost_per_rad = mm_per_rad * EnvROBARMCfg.cost_per_mm;

    printf("\nCosts:\n");
    printf("Max Distance moved during one Successor Action: %.3f (meters)\n",max_dist);
    printf("Largest Successor Action: ");
    for(int i=0; i < NUMOFLINKS; i++)
        printf("%1.2f ", EnvROBARMCfg.SuccActions[largest_action][i]);
    printf("\n");

//     double rad_per_cell = 2.0*asin((gridcell_size/2.0)/.925);
//     printf("Radians per Cell: %.3f\n",rad_per_cell);
    printf("Cells per Action: %1.3f\n", EnvROBARMCfg.CellsPerAction);
    printf("Cost per Cell: %.2f\n", EnvROBARMCfg.cost_per_cell);
    printf("Cost per sqrt(2) Cells: %.2f\n",EnvROBARMCfg.cost_sqrt2_move);
    printf("Cost per sqrt(3) Cells: %.2f\n",EnvROBARMCfg.cost_sqrt3_move);
    printf("Cost per millimeter: %.2f\n", EnvROBARMCfg.cost_per_mm);
    printf("Meter per Radian Moved: %.3f\n",mm_per_rad/1000.0);
//     printf("Cost per Radian:  %.4f\n",EnvROBARMCfg.cost_per_rad);
//     printf("New Cost per Radian:  %1.3f\n",EnvROBARMCfg.cost_per_cell/rad_per_cell);

    //testing
//     EnvROBARMCfg.cost_per_rad = EnvROBARMCfg.cost_per_cell/rad_per_cell;
}

void EnvironmentROBARM3D::ComputeCostPerRadian()
{
    double angles[NUMOFLINKS],start_angles[NUMOFLINKS]={0};
    double max_ang_dist = 0.0;
    int largest_action=0;
    double ang_dist1, ang_dist2,/*ang_dist3, ang_dist4,*/ rpy1[3], rpy2[3], start_rpy1[3], start_rpy2[3], orientation[3][3];
    short unsigned int endeff[3],wrist[3],elbow[3];

    //starting at zeroed angles, find end effector position after each action
    if(EnvROBARMCfg.use_DH)
        ComputeEndEffectorPos(start_angles,endeff,wrist,elbow,orientation);

    getRPY(orientation,&start_rpy1[0],&start_rpy1[1],&start_rpy1[2],1);
    getRPY(orientation,&start_rpy2[0],&start_rpy2[1],&start_rpy2[2],2);

//     printf("\n\n");
//     printf("[ComputeCostPerRadian] start: roll1: %.4f pitch1: %.4f yaw1: %.4f\n", start_rpy1[0],start_rpy1[1],start_rpy1[2]);
//     printf("[ComputeCostPerRadian] start: roll2: %.4f pitch2: %.4f yaw2: %.4f\n", start_rpy2[0],start_rpy2[1],start_rpy2[2]);
//     printf("\n\n");

    //iterate through all possible actions and find the one with the minimum cost per cell
    for (int i = 0; i < EnvROBARMCfg.nSuccActions; i++)
    {
        for(int j = 0; j < NUMOFLINKS; j++)
            angles[j] = DEG2RAD(EnvROBARMCfg.SuccActions[i][j]);

        if(EnvROBARMCfg.use_DH)
            ComputeEndEffectorPos(angles,endeff,wrist,elbow,orientation);

        getRPY(orientation,&rpy1[0],&rpy1[1],&rpy1[2],1);
        getRPY(orientation,&rpy2[0],&rpy2[1],&rpy2[2],2);

        //set roll to 0 (we'll apply roll manually at the end)
        rpy1[0] = 0.0;
        rpy2[0] = 0.0;
        start_rpy1[0] = 0.0;
        start_rpy2[0] = 0.0;

        ang_dist1 = getAngularEuclDist(rpy1, start_rpy1);
        ang_dist2 = getAngularEuclDist(rpy1, start_rpy2); //CHANGED 2 to 1
//         ang_dist3 = getAngularEuclDist(rpy2, start_rpy1);
//         ang_dist4 = getAngularEuclDist(rpy2, start_rpy2); //CHANGED 2 to 1

        if(max_ang_dist < ang_dist1)
        {
            max_ang_dist = ang_dist1;
            largest_action = i;
        }

        if(max_ang_dist < ang_dist2)
        {
            max_ang_dist = ang_dist2;
            largest_action = i;
        }
//         if(max_ang_dist < ang_dist3)
//         {
//             max_ang_dist = ang_dist3;
//             largest_action = i;
//         }
//         if(max_ang_dist < ang_dist4)
//         {
//             max_ang_dist = ang_dist4;
//             largest_action = i;
//         }
//         printf("[ComputeCostPerRadian] action %i: pitch1: %.4f yaw1: %.4f  pitch2: %.4f yaw2: %.4f\n", i, rpy1[1],rpy2[2],rpy2[1],rpy2[2]);
//         printf("[ComputeCostPerRadian] action %i: ang_dist1: %.4f ang_dist2: %.4f\n", i, ang_dist1,ang_dist2);
    }

    EnvROBARMCfg.cost_per_rad = COSTMULT / max_ang_dist;
    printf("\n[ComputeCostPerRadian] Largest Action: %d  Max Ang Dist: %.4f Cost/Rad: %.4f\n",largest_action,max_ang_dist, EnvROBARMCfg.cost_per_rad);
}

int EnvironmentROBARM3D::GetDistToClosestGoal(short unsigned int* xyz, int* goal_num)
{
    int i,ind = 0;
    int dist, min_dist = 10000000;

    for(i=0; i < EnvROBARMCfg.nEndEffGoals;i++)
    {
        dist = sqrt((EnvROBARMCfg.EndEffGoals_c[i][0] - xyz[0])*(EnvROBARMCfg.EndEffGoals_c[i][0] - xyz[0]) +
                (EnvROBARMCfg.EndEffGoals_c[i][1] - xyz[1])*(EnvROBARMCfg.EndEffGoals_c[i][1] - xyz[1]) +
                (EnvROBARMCfg.EndEffGoals_c[i][2] - xyz[2])*(EnvROBARMCfg.EndEffGoals_c[i][2] - xyz[2]));
        if(dist < min_dist)
        {
            ind = i;
            min_dist = dist;
        }
    }

    (*goal_num) = ind;
    return min_dist;
}

double EnvironmentROBARM3D::GetDistToClosestGoal(double *xyz, int* goal_num)
{
    int i,ind = 0;
    double dist, min_dist = 10000000;

    for(i=0; i < EnvROBARMCfg.nEndEffGoals;i++)
    {
        dist = sqrt((EnvROBARMCfg.EndEffGoals_m[i][0] - xyz[0])*(EnvROBARMCfg.EndEffGoals_m[i][0] - xyz[0]) +
                (EnvROBARMCfg.EndEffGoals_m[i][1] - xyz[1])*(EnvROBARMCfg.EndEffGoals_m[i][1] - xyz[1]) +
                (EnvROBARMCfg.EndEffGoals_m[i][2] - xyz[2])*(EnvROBARMCfg.EndEffGoals_m[i][2] - xyz[2]));
        if(dist < min_dist)
        {
            ind = i;
            min_dist = dist;
        }
    }

    (*goal_num) = ind;
    return min_dist;
}
//--------------------------------------------------------------

/*------------------------------------------------------------------------*/
                        /* Joint Space Goal */
/*------------------------------------------------------------------------*/
void EnvironmentROBARM3D::GetJointSpaceSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV)
{
    short unsigned int succcoord[NUMOFLINKS];
    short unsigned int k, wrist[3], elbow[3], endeff[3];
    double orientation [3][3];
    int i, inc, a, closest_goal;
    double angles[NUMOFLINKS], s_angles[NUMOFLINKS];
    int at_goal_config;

    //to support two sets of succesor actions
    int actions_i_min = 0, actions_i_max = EnvROBARMCfg.nLowResActions;

    //clear the successor array
    SuccIDV->clear();
    CostV->clear();

    //check if there is an actual goal to plan to (should not have to be done here...think of better place)
    if(EnvROBARMCfg.nJointSpaceGoals == 0 || !EnvROBARMCfg.bGoalIsSet)
        return;

    //goal state should be absorbing
    if(SourceStateID == EnvROBARM.goalHashEntry->stateID)
    {
        printf("goal state is absorbing...\n");
        return;
    }

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* HashEntry = EnvROBARM.StateID2CoordTable[SourceStateID];
    ComputeContAngles(HashEntry->coord, s_angles);

    //default coords of successor
    for(i = 0; i < NUMOFLINKS; i++)
        succcoord[i] = HashEntry->coord[i];

    ComputeContAngles(succcoord, angles);

    //check if cell is close to enough to goal to use higher resolution actions
    if(EnvROBARMCfg.multires_succ_actions)
    {
        if (GetDistToClosestGoal(HashEntry->endeff, &closest_goal) <= EnvROBARMCfg.HighResActionsThreshold_c)
        {
            actions_i_min = EnvROBARMCfg.nLowResActions;
            actions_i_max = EnvROBARMCfg.nSuccActions;
        }
    }

    //iterate through successors of s (possible actions)
    for (i = actions_i_min; i < actions_i_max; i++)
    {
        //increase and decrease in ith angle
        for(inc = -1; inc < 2; inc = inc+2)
        {
            if(inc == -1)
            {
                for(a = 0; a < NUMOFLINKS; a++)
                {
                    //if the joint is at 0deg and the next action will decrement it
                    if(HashEntry->coord[a] == 0 && EnvROBARMCfg.SuccActions[i][a] != 0)
                        succcoord[a] =  EnvROBARMCfg.anglevals[a] - EnvROBARMCfg.SuccActions[i][a];
                    //the joint's current position, when decremented by n degrees will go below 0
                    else if(HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a] < 0)
                        succcoord[a] =  EnvROBARMCfg.anglevals[a] + (HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a]);
                    else
                        succcoord[a] = HashEntry->coord[a] - EnvROBARMCfg.SuccActions[i][a];
                }
            }
            else
            {
                for(a = 0; a < NUMOFLINKS; a++)
                    succcoord[a] = (HashEntry->coord[a] + int(EnvROBARMCfg.SuccActions[i][a])) % EnvROBARMCfg.anglevals[a];
            }

            //get the successor
            EnvROBARMHashEntry_t* OutHashEntry;
            bool bSuccisGoal = false;

            //have to create a new entry
            ComputeContAngles(succcoord, angles);

            //get forward kinematics
            if(ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation) == false)
            {
//                 printf("[GetJointSpaceSuccs] FK for endeff (%u %u %u) successor is not valid.\n",endeff[0],endeff[1],endeff[2]);
                continue;
            }

            //do collision checking
            if(!IsValidCoord(succcoord, endeff, wrist, elbow, orientation))
            {
//                 printf("[GetJointSpaceSuccs] endeff (%u %u %u) successor is not valid.\n",endeff[0],endeff[1],endeff[2]);
                continue;
            }

            //check if at goal configuration
            for(k = 0; k < EnvROBARMCfg.nJointSpaceGoals; k++)
            {
                for(a = 0; a < NUMOFLINKS; a++)
                {
                    at_goal_config = 1;

                    if(angles[a] >= PI_CONST)
                        angles[a] = -2.0*PI_CONST + angles[a];

                    if(fabs(angles::shortest_angular_distance(angles[a], EnvROBARMCfg.JointSpaceGoals[k][a])) > EnvROBARMCfg.JointSpaceGoalMOE[a])
                    {
                        at_goal_config = 0;
                        break;
                    }
                }
                if(at_goal_config)
                {
                    bSuccisGoal = true;

                    //NOTE: Allow the temporary grids to be updated
                    EnvROBARMCfg.bPlanning = false;

                    // printf("goal succ is generated\n");
                    for (a = 0; a < NUMOFLINKS; a++)
                    {
                        EnvROBARMCfg.goalcoords[a] = succcoord[a];
                        EnvROBARM.goalHashEntry->coord[a] = succcoord[a];
                    }

                    EnvROBARM.goalHashEntry->endeff[0] = endeff[0];
                    EnvROBARM.goalHashEntry->endeff[1] = endeff[1];
                    EnvROBARM.goalHashEntry->endeff[2] = endeff[2];
                    EnvROBARM.goalHashEntry->action = i;
                }
            }

            //check if hash entry already exists, if not then create one
            if((OutHashEntry = GetHashEntry(succcoord, NUMOFLINKS, i, bSuccisGoal)) == NULL)
            {
                OutHashEntry = CreateNewHashEntry(succcoord, NUMOFLINKS, endeff, i, orientation);
            }

            //put successor on successor list with cost
            SuccIDV->push_back(OutHashEntry->stateID);

            CostV->push_back(cost(HashEntry,OutHashEntry,bSuccisGoal) +
                    GetJointSpaceHeuristic(OutHashEntry->stateID, EnvROBARM.goalHashEntry->stateID) +
                    EnvROBARMCfg.ActiontoActionCosts[HashEntry->action][OutHashEntry->action]);
        }
    }
}

int EnvironmentROBARM3D::GetJointSpaceHeuristic(int FromStateID, int ToStateID)
{
#if USE_HEUR==0
    return 0;
#endif

    int a, h = 0, closest_goal = 0;
    double FromAngles[NUMOFLINKS], ToAngles[NUMOFLINKS];
    double ang_diff, sum = 0, min_dist = 100000000;//, sum_squared;

    //get X, Y, Z for the state
    EnvROBARMHashEntry_t* FromHashEntry = EnvROBARM.StateID2CoordTable[FromStateID];
    EnvROBARMHashEntry_t* ToHashEntry = EnvROBARM.StateID2CoordTable[ToStateID];

    ComputeContAngles(FromHashEntry->coord, FromAngles);
    ComputeContAngles(ToHashEntry->coord, ToAngles);

    //normalize angles
    for(a = 0; a < NUMOFLINKS; a++)
    {
        if(FromAngles[a] >= PI_CONST)
            FromAngles[a] = -2.0*PI_CONST + FromAngles[a];
    }

    //find euclidean distance to closest goal
    for(int k = 0; k < EnvROBARMCfg.nJointSpaceGoals; k++)
    {
        sum = 0;
        for(a = 0; a < NUMOFLINKS; a++)
        {
            if(a == FOREARM_ROLL || a == WRIST_ROLL)
            {
                ang_diff = angles::shortest_angular_distance(FromAngles[a],  EnvROBARMCfg.JointSpaceGoals[k][a]);
            }
            else
            {
                angles::shortest_angular_distance_with_limits(FromAngles[a],  EnvROBARMCfg.JointSpaceGoals[k][a],EnvROBARMCfg.NegMotorLimits[a],EnvROBARMCfg.PosMotorLimits[a],ang_diff);
            }

            sum += (ang_diff*ang_diff);
        }

        if(sqrt(sum) < min_dist)
        {
            min_dist = sqrt(sum);
            closest_goal = k;
        }
    }

    //heuristic = angular euclidean distance (radians)  x  Cost per radian
    h = min_dist * EnvROBARMCfg.cost_per_rad;

    //find closest goal
//     for(int k = 0; k < EnvROBARMCfg.nJointSpaceGoals; k++)
//     {
//         sum = 0;
//         for(a = 0; a < NUMOFLINKS; a++)
//         {
//             //shortest_angular_distance_with_limits!
//             sum += angles::shortest_angular_distance(FromAngles[a], EnvROBARMCfg.JointSpaceGoals[k][a]);
//         }
// 
//         if(sum < min_sum)
//         {
//             min_sum = sum;
//             closest_goal = k; 
//         }
//     }
// 
//     //get euclidean distance
//     sum_squared = 0;
//     for(a=0; a < NUMOFLINKS; a++)
//     {
//         if(a == FOREARM_ROLL || a == WRIST_ROLL)
//         {
//             ang_diff = angles::shortest_angular_distance(FromAngles[a],  EnvROBARMCfg.JointSpaceGoals[closest_goal][a]);
// //             printf("%i: continuous: %.3f, %.3f --> %.3f\n", a, ang_diff, FromAngles[a], EnvROBARMCfg.JointSpaceGoals[closest_goal][a]);
//         }
//         else
//         {
//             angles::shortest_angular_distance_with_limits(FromAngles[a],  EnvROBARMCfg.JointSpaceGoals[closest_goal][a],EnvROBARMCfg.NegMotorLimits[a],EnvROBARMCfg.PosMotorLimits[a],ang_diff);
// //             printf("%i: limited: %.3f, %.3f --> %.3f\n", a, ang_diff, FromAngles[a], EnvROBARMCfg.JointSpaceGoals[closest_goal][a]);
//         }
//         sum_squared += ang_diff * ang_diff;
//     }
//     int h = sqrt(sum_squared) * EnvROBARMCfg.cost_per_rad;

#if DEBUG_JOINTSPACE_PLANNING
    printf("%i:  %.3f %.3f %.3f %.3f %.3f %.3f %.3f  -->  %.3f %.3f %.3f %.3f %.3f %.3f %.3f  ssd(rad): %.3f  h: %i\n",
           FromStateID,FromAngles[0],FromAngles[1],FromAngles[2],FromAngles[3],FromAngles[4],FromAngles[5],FromAngles[6],
           EnvROBARMCfg.JointSpaceGoals[closest_goal][0],EnvROBARMCfg.JointSpaceGoals[closest_goal][1],
           EnvROBARMCfg.JointSpaceGoals[closest_goal][2],EnvROBARMCfg.JointSpaceGoals[closest_goal][3],
           EnvROBARMCfg.JointSpaceGoals[closest_goal][4],EnvROBARMCfg.JointSpaceGoals[closest_goal][5],
           EnvROBARMCfg.JointSpaceGoals[closest_goal][6],sqrt(sum),h);
#endif
    return h;
}

bool EnvironmentROBARM3D::SetJointSpaceGoals(double** JointSpaceGoals, int num_goals)
{
    EnvROBARMCfg.PlanInJointSpace = true;
    int i = 0,k = 0;
    short unsigned  int coord[NUMOFLINKS];
    double angles[NUMOFLINKS], angles_pos[NUMOFLINKS], orientation[3][3];
    short unsigned int elbow[3] = {0}, wrist[3] = {0}, endeff[3] = {0};

    //allocate memory
    if(EnvROBARMCfg.nJointSpaceGoals != num_goals || !EnvROBARMCfg.bGoalIsSet)
    {
        if(EnvROBARMCfg.bGoalIsSet && EnvROBARMCfg.nJointSpaceGoals != num_goals)
        {
            //delete the old goal array
            for (i = 0; i < EnvROBARMCfg.nJointSpaceGoals; i++)
            {
                delete [] EnvROBARMCfg.JointSpaceGoals[i];
            }
            delete [] EnvROBARMCfg.JointSpaceGoals;

            EnvROBARMCfg.JointSpaceGoals = NULL;
            EnvROBARMCfg.bGoalIsSet = false;
        }

        EnvROBARMCfg.nJointSpaceGoals = num_goals;
        EnvROBARMCfg.JointSpaceGoals = new double * [num_goals];
        for (i = 0; i < num_goals; i++)
        {
            EnvROBARMCfg.JointSpaceGoals[i] = new double [NUMOFLINKS];
        }
    }

    //copy obstacles from temp map to real map
    printf("[SetJointSpaceGoals] copying temporary map to real one...\n");
    EnvROBARMCfg.bPlanning = true;
//     EnvROBARMCfg.bUpdatePlanningGrid = true;

    EnvROBARMCfg.mCopyingGrid.lock();
    UpdateEnvironment();
    EnvROBARMCfg.mCopyingGrid.unlock();
//     for (int z = 25; z <= 75; z++)
//     {
//         printf("\n[SetJointSpaceGoals] z = %i\n",z);
//         for (int x = EnvROBARMCfg.EnvWidth_c/2; x < EnvROBARMCfg.EnvWidth_c; x++) 
//         {
//             for (int y = 25; y < 75; y++)
//                 printf("%d ",EnvROBARMCfg.Grid3D[x][y][z]);
//             printf("\n");
//         }
//         printf("\n");
//     }

    //loop through all the goal positions
    for(i = 0; i < EnvROBARMCfg.nJointSpaceGoals; i++)
    {
        for(k = 0; k < NUMOFLINKS; k++)
        {
            EnvROBARMCfg.JointSpaceGoals[i][k] = JointSpaceGoals[i][k];

            if(i==0)
            {
                if(JointSpaceGoals[i][k] < 0)
                    EnvROBARMCfg.LinkStartAngles_d[i] = RAD2DEG(JointSpaceGoals[i][k] + PI_CONST*2.0);
                else
                    EnvROBARMCfg.LinkStartAngles_d[i] = RAD2DEG(JointSpaceGoals[i][k]);
            }

            angles[k] = JointSpaceGoals[i][k];

            angles_pos[k] = JointSpaceGoals[i][k];
            if(angles[k] < 0)
                angles_pos[k] = 2*PI_CONST + angles[k];
        }

        //get joint positions of starting configuration
        if(!ComputeEndEffectorPos(angles, endeff, wrist, elbow, orientation))
        {
            printf("[SetJointSpaceGoals] goal: %.2f %.2f %.2f %.2f %.2f %.2f %.2f is invalid.\n",
                   angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
            printf("[SetJointSpaceGoals] goal: endeff(%u %u %u) wrist(%u %u %u) elbow(%u %u %u)\n",
                   endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);
            if(!RemoveGoal(i))
            {
                EnvROBARMCfg.bGoalIsSet  = false;
                return false;
            }
            else
            {
                printf("[SetEndEffGoals] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                continue;
            }
        }

        //get coords
        ComputeCoord(angles_pos, coord);

        //check if goal position is valid
        if(!IsValidCoord(coord,endeff,wrist,elbow,orientation))
        {
            printf("[SetJointSpaceGoals] goal hash entry: %.2f %.2f %.2f %.2f %.2f %.2f %.2f is invalid.\n",
                   angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);
            printf("[SetJointSpaceGoals] goal: endeff(%u %u %u) wrist(%u %u %u) elbow(%u %u %u)\n",
                   endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);
            if(!RemoveGoal(i))
            {
                EnvROBARMCfg.bGoalIsSet  = false;
                return false;
            }
            else
            {
                printf("[SetEndEffGoals] Removed goal %i. %i goal(s) remaining.\n", i, EnvROBARMCfg.nEndEffGoals);
                continue;
            }
        }

        //set goalHashEntry info just for kicks
        if(i == 0)
        {
            EnvROBARM.goalHashEntry->endeff[0] = endeff[0];
            EnvROBARM.goalHashEntry->endeff[1] = endeff[1];
            EnvROBARM.goalHashEntry->endeff[2] = endeff[2];

            for(k=0; k < NUMOFLINKS; k++)
            {
                EnvROBARM.goalHashEntry->coord[k] = coord[k];
            }
        }

        printf("[SetJointSpaceGoals] goal %i: endeff(%u %u %u) wrist(%u %u %u) elbow(%u %u %u)\n",
               i, endeff[0],endeff[1],endeff[2],wrist[0],wrist[1],wrist[2],elbow[0],elbow[1],elbow[2]);
    }

    EnvROBARMCfg.bGoalIsSet = true;

//     EnvROBARMCfg.bUpdatePlanningGrid = false;

    //temporary debugging output
    double pX,pY,pZ;
    double roll,pitch,yaw;

    ComputeContAngles(EnvROBARM.startHashEntry->coord, angles);
    printf("\n\n[SetJointSpaceGoals] Ready to Plan:\n");
    printf("start: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", angles[0],angles[1],angles[2],angles[3],angles[4],angles[5],angles[6]);

    getRPY(EnvROBARM.startHashEntry->orientation,&roll,&pitch,&yaw,1);
    Cell2ContXYZ(EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2], &pX, &pY, &pZ);
    printf("start: %i %i %i (cells) --> %.2f %.2f %.2f (meters)   rpy: (%1.2f %1.2f %1.2f)\n", EnvROBARM.startHashEntry->endeff[0],EnvROBARM.startHashEntry->endeff[1],EnvROBARM.startHashEntry->endeff[2],pX,pY,pZ,roll,pitch,yaw);

    for(i = 0; i < EnvROBARMCfg.nEndEffGoals; i++)
    {
        printf("goal %i: ",i);
        for(k = 0; k < NUMOFLINKS; k++)
            printf("%.3f  ", EnvROBARMCfg.JointSpaceGoals[i][k]);
        printf("\n");
    }
    printf("\n\n");

    return true;
}
