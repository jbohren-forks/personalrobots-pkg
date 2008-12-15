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
#include <boost/numeric/ublas/matrix.hpp>
#include <robot_kinematics/robot_kinematics.h>

using namespace robot_kinematics;
using namespace KDL;

#ifndef __ENVIRONMENT_ROBARM_H_
#define __ENVIRONMENT_ROBARM_H_

#define NUMOFLINKS 7
#define NUMOFLINKS_DH 8

#define ROBARM_LONGACTIONDIST_CELLS 20   //for PES max. distance in coord to a sample point. It should be exactly so for one of the coordinates and this or smaller for the rest

#define ROBARM_MAXNUMOFLONGACTIONSUCCS ((int)pow((double)2*ROBARM_LONGACTIONDIST_CELLS, NUMOFLINKS))

#define UNIFORM_COST 1	//all the joint actions have the same costs when set

#define INVALID_NUMBER 999

#define NUM_TRANS_MATRICES 360


// coords - used to pass around lists of valid cells
typedef struct
{
    short unsigned int x;
    short unsigned int y;
    short unsigned int z;
    bool bIsObstacle;
}CELLV;

//state structure for Dykstra's Algorithm
typedef struct STATE3D_t
{
    unsigned int g;
    short unsigned int iterationclosed;
    short unsigned int x;
    short unsigned int y;
    short unsigned int z;
} State3D;

// robot arm physical configuration structure
typedef struct ENV_ROBARM_CONFIG
{
    //environment dimensions (meters)
    double EnvWidth_m;
    double EnvHeight_m;
    double EnvDepth_m;

    //environment dimensions (cells)
    int EnvWidth_c;
    int EnvHeight_c;
    int EnvDepth_c;

    //fixed location of shoulder base (cells)
    int BaseX_c;
    int BaseY_c;
    int BaseZ_c;

    //fixed location of shoulder base (meters)
    double BaseX_m;
    double BaseY_m;
    double BaseZ_m;

    //end effector goal position (goal cell)
    short unsigned int EndEffGoalX_c;
    short unsigned int EndEffGoalY_c;
    short unsigned int EndEffGoalZ_c;

    //robot arm dimensions/positions
    double LinkLength_m[NUMOFLINKS];
    double LinkStartAngles_d[NUMOFLINKS];
    double LinkGoalAngles_d[NUMOFLINKS];

    //3d grid of world space 
    char*** Grid3D;     
    double GridCellWidth;   // cells are square

    //DH Parameters
    double DH_alpha[NUMOFLINKS_DH];
    double DH_a[NUMOFLINKS_DH];
    double DH_d[NUMOFLINKS_DH];
    double DH_theta[NUMOFLINKS_DH];

    //Motor Limits
    double PosMotorLimits[NUMOFLINKS];
    double NegMotorLimits[NUMOFLINKS];

    //joint angle discretization
    double angledelta[NUMOFLINKS]; 
    int anglevals[NUMOFLINKS];     

    //for kinematic library use
    RobotKinematics pr2_kin;
    SerialChain *left_arm;
    JntArray *pr2_config;

    short unsigned int goalcoords[NUMOFLINKS];

    //options
    bool use_DH;
    bool enforce_motor_limits;
    bool dijkstra_heuristic;
    bool endeff_check_only;

    //velocities
    int nVelSucc;
    double ** succ_vel;
    int ** ActionCosts;

    //for precomputing cos/sin & DH matrices 
    double cos_r[360];
    double sin_r[360];
    double T_DH[4*NUM_TRANS_MATRICES][4][8];

    //this should be moved to EnvironmentROBARM_t
    double T_array[4][4][8];

} EnvROBARMConfig_t;

// hash entry that contains end effector coordinates
typedef struct ENVROBARMHASHENTRY
{
    int stateID;
    //state coordinates
    short unsigned int coord[NUMOFLINKS];
    short unsigned int wrist[3];
    short unsigned int elbow[3];
    short unsigned int endeff[3];
    short unsigned int endeffx;
    short unsigned int endeffy;
    short unsigned int endeffz;
    short unsigned int action;
} EnvROBARMHashEntry_t;

// main structure that stores environment data used in planning
typedef struct
{
    EnvROBARMHashEntry_t* goalHashEntry;
    EnvROBARMHashEntry_t* startHashEntry;

    //Maps from coords to stateID
    int HashTableSize;
    vector<EnvROBARMHashEntry_t*>* Coord2StateIDHashTable;

    //vector that maps from stateID to coords	
    vector<EnvROBARMHashEntry_t*> StateID2CoordTable;

    //transformation matrices - maybe not needed
    boost::numeric::ublas::matrix<double> T[NUMOFLINKS_DH];

    //any additional variables
    int* Heur;    // euclidean distance
}EnvironmentROBARM_t;

class EnvironmentROBARM : public DiscreteSpaceInformation 
{

public:

    bool InitializeEnv(const char* sEnvFile);

    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    int  GetFromToHeuristic(int FromStateID, int ToStateID);
    int  GetGoalHeuristic(int stateID);
    int  GetStartHeuristic(int stateID);
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    void SetAllPreds(CMDPSTATE* state);
    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);

    int	 SizeofCreatedEnv();
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
    void PrintEnv_Config(FILE* fOut);
    void PrintHeurGrid();
    ~EnvironmentROBARM(){};

    void PrintTimeStat(FILE* fOut);
    void outputangles(double angles[NUMOFLINKS], bool degrees);
    void CloseKinNode();

private:

    //member data
    EnvROBARMConfig_t EnvROBARMCfg;
    EnvironmentROBARM_t EnvROBARM;

    //hash table
    unsigned int GETHASHBIN(short unsigned int* coord, int numofcoord);
    void PrintHashTableHist();
    EnvROBARMHashEntry_t* GetHashEntry(short unsigned int* coord, int numofcoord, bool bIsGoal);
    EnvROBARMHashEntry_t* CreateNewHashEntry(short unsigned int* coord, int numofcoord, short unsigned int endeff[3], short unsigned int wrist[3], short unsigned int elbow[3],short unsigned int action);

    //initialization
    void ReadParams(FILE* fCfg);
    void ReadConfiguration(FILE* fCfg);
    void InitializeEnvConfig();
    void CreateStartandGoalStates();
    bool InitializeEnvironment();
    void ReadSuccActionsFile(FILE* fCfg);

    //coordinate frame/angle functions
    void DiscretizeAngles();
    void Cell2ContXY(int x, int y, int z, double *pX, double *pY, double *pZ);
    void ContXYZ2Cell(double x, double y, double z, short unsigned int* pX, short unsigned int *pY, short unsigned int *pZ);
    void ComputeContAngles(short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS]);
    void ComputeCoord(double angle[NUMOFLINKS], short unsigned int coord[NUMOFLINKS]);
    int ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int*  pX, short unsigned int* pY, short unsigned int*  pZ, short unsigned int wrist[3], short unsigned int elbow[3]);

    //bounds/error checking
    int IsValidCoord(short unsigned int coord[NUMOFLINKS], char*** Grid3D=NULL, vector<CELLV>* pTestedCells=NULL);
    int IsValidCoord(short unsigned int coord[NUMOFLINKS], EnvROBARMHashEntry_t* arm);
    int IsValidLineSegment(double x0, double y0, double z0, double x1, double y1, double z1, char ***Grid3D, vector<CELLV>* pTestedCells);
    int IsValidLineSegment(short unsigned int x0, short unsigned int y0, short unsigned int z0, short unsigned int x1, short unsigned int y1, short unsigned int z1, char ***Grid3D, vector<CELLV>* pTestedCells);
    bool IsValidCell(int X, int Y, int Z);
    bool IsWithinMapCell(int X, int Y, int Z);
    bool AreEquivalent(int State1ID, int State2ID);

    //cost functions
    int cost(short unsigned int state1coord[], short unsigned int state2coord[]); 
    int cost(short unsigned int state1coord[], short unsigned int state2coord[],bool bState2IsGoal);
    int GetEdgeCost(int FromStateID, int ToStateID);
    void ComputeActionCosts();

    //output
    void PrintHeader(FILE* fOut);
    void outputjointpositions(EnvROBARMHashEntry_t* H); // for early debugging - remove this 
    void PrintConfiguration();
    void printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal);
    void PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/, FILE* fOut /*=NULL*/);
    void OutputActionCostTable();
    void OutputActions();

    //compute heuristic
    void InitializeKinNode();
    void ComputeForwardKinematics(double *angles, int f_num, double *x, double *y, double *z);
    void getDistancetoGoal(int* HeurGrid, int goalx, int goaly, int goalz);
    void ComputeHeuristicValues();
    void ReInitializeState3D(State3D* state);
    void InitializeState3D(State3D* state, short unsigned int x, short unsigned int y, short unsigned int z);
    void Create3DStateSpace(State3D**** statespace3D);
    void Delete3DStateSpace(State3D**** statespace3D);
    void Search3DwithQueue(State3D*** statespace, int* HeurGrid, short unsigned  int searchstartx, short unsigned int searchstarty, short unsigned int searchstartz);

    //forward kinematics
    void getDHMatrix(boost::numeric::ublas::matrix<double>*T, double alpha, double a, double d, double theta);
    void getTransformations_robarm7d(boost::numeric::ublas::matrix<double> *T, double angle[NUMOFLINKS]);
    void ValidateDH2KinematicsLibrary();
    void ComputeDHTransformations();
    void GetPrecomputedDHMatrix(boost::numeric::ublas::matrix<double>*T, double theta, int frame);
    void GetDHMatrix( double theta, int frame);
    void GetDHTransformations(double angles[NUMOFLINKS]);
    void Mult4x4(int A, int B, int C);
};

#endif
