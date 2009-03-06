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
 
// #include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
// #include <boost/thread/condition.hpp>

/**

@mainpage

Motion planner for Robotic Arm based on the SBPL. It is capable of finding the shortest path to multiple 6 DoF destinations.


@htmlinclude manifest.html

@b robarm3d will be fully integrated with ros....coming soon.

 **/

 
// #include <boost/numeric/ublas/matrix.hpp>
#include <robot_kinematics/robot_kinematics.h>

using namespace robot_kinematics;
using namespace KDL;

#ifndef __ENVIRONMENT_ROBARM3D_H_
#define __ENVIRONMENT_ROBARM3D_H_

#define NUMOFLINKS 7

#define UNIFORM_COST 1      //all the joint actions have the same costs when set

#define INVALID_NUMBER 999

#define NUM_TRANS_MATRICES 360 //precomputed matrices


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
//     int heapindex;
//     struct listelement* listelem[2];
} State3D;

// robot arm physical configuration structure
typedef struct ENV_ROBARM_CONFIG
{
    //epsilon to be used by planner
    //this is the wrong place to store it, but it allows epsilon to be read in params.cfg
    double epsilon;

    //environment dimensions (meters)
    double EnvWidth_m;
    double EnvHeight_m;
    double EnvDepth_m;

    //environment dimensions (cells)
    int EnvWidth_c;
    int EnvHeight_c;
    int EnvDepth_c;

    //fixed location of shoulder base (cells) 
    short unsigned int BaseX_c;
    short unsigned int BaseY_c;
    short unsigned int BaseZ_c;

    //fixed location of shoulder base (meters)
    double BaseX_m;
    double BaseY_m;
    double BaseZ_m;

    //end effector goal position (cell)
    short unsigned int EndEffGoalX_c;   //get rid of this
    short unsigned int EndEffGoalY_c;
    short unsigned int EndEffGoalZ_c;

    //low resolution environment dimensions (cells)
    int LowResEnvWidth_c;
    int LowResEnvHeight_c;
    int LowResEnvDepth_c;

    //flag determines if the environment has been initialized or not
    bool bInitialized;

    //cost of cells on grid of obstacles and close to obstacles
    char ObstacleCost;
    char medObstacleCost;
    char lowObstacleCost;

    int medCostRadius_c;
    int lowCostRadius_c;

    //DH Parameters
    double DH_alpha[NUMOFLINKS];
    double DH_a[NUMOFLINKS];
    double DH_d[NUMOFLINKS];
    double DH_theta[NUMOFLINKS];

    //Motor Limits
    double PosMotorLimits[NUMOFLINKS];
    double NegMotorLimits[NUMOFLINKS]; //must be negative values

    //joint angle discretization
    double angledelta[NUMOFLINKS];
    int anglevals[NUMOFLINKS];

    //for kinematic library use
    RobotKinematics pr2_kin;
    SerialChain *left_arm;
    JntArray *pr2_config;

    //coords of goal - shouldn't be here
    short unsigned int goalcoords[NUMOFLINKS];

    //robot arm dimensions/positions
    double LinkLength_m[NUMOFLINKS];
    double LinkStartAngles_d[NUMOFLINKS];
    double LinkGoalAngles_d[NUMOFLINKS];

    int arm_length;
    //starting joint configuration
//     double JointStartConfig_d[NUMOFLINKS];

    //end effector goal orientation
    double GoalOrientationMOE[3][3];    //eventually remove this

    short unsigned int ** EndEffGoals_c;
    double ** EndEffGoals_m;
    double ** EndEffGoalRPY;
    double GoalRPY_MOE[3];
    double ** EndEffGoalOrientations;
    int nEndEffGoals;
    bool bGoalIsSet;

    //3D grid of world space 
    char*** Grid3D;
    char*** LowResGrid3D;
    double GridCellWidth;           // cells are square (width=height=depth)
    double LowResGridCellWidth;     // cells are square (width=height=depth)
    char*** Grid3D_temp;
    char*** LowResGrid3D_temp;

    //options
    bool use_DH;
    bool lowres_collision_checking;
    bool multires_succ_actions;
    bool enforce_motor_limits;
    bool dijkstra_heuristic;
    bool endeff_check_only;
    bool use_smooth_actions;
    bool enforce_upright_gripper;
    bool checkEndEffGoalOrientation;
    bool object_grasped;
    bool variable_cell_costs;
    double smoothing_weight;
    double padding;
    double gripper_orientation_moe; //gripper orientation margin of error
    double grasped_object_length_m;
    double goal_moe_m;

    //successor actions
    double ** SuccActions;
    int nSuccActions;
    int nLowResActions;
    int ** ActiontoActionCosts;
    int HighResActionsThreshold_c;

    //for precomputing cos/sin & DH matrices 
    double cos_r[360];
    double sin_r[360];
    double T_DH[4*NUM_TRANS_MATRICES][4][NUMOFLINKS];

    //cell-to-cell costs - TODO change then to integers
    double CellsPerAction;
    double cost_per_cell;
    double cost_sqrt2_move;
    double cost_sqrt3_move;
    double cost_per_mm;
    double cost_per_rad;

    std::vector<std::vector<double> > sbpl_cubes;

    //a bad hack
    bool JointSpaceGoal;
    bool dual_heuristics;
    double uninformative_heur_dist_m;
    double ApplyRPYCost_m;
    double AngularDist_Weight;
    double ExpCoefficient;
    bool angular_dist_cost;

    //joint-space search
    bool PlanInJointSpace;
    double JointSpaceGoalMOE[NUMOFLINKS];
    double ** JointSpaceGoals;
    int nJointSpaceGoals;

    std::vector<std::vector<double> > cubes;

    bool bPlanning;
    bool bUpdatePlanningGrid;

    boost::mutex mCopyingGrid;
    boost::mutex mPlanning;

//     std::vector<std::vector<double> > bigcubes;

//     std::vector < std::vector<double> > EndEffGoals_m;
//     std::vector < std::vector<double> > EndEffGoalsRPY;
//     std::vector < std::vector<short unsigned int> > EndEffGoals_c;

} EnvROBARMConfig_t;

// hash entry that contains end effector coordinates
typedef struct ENVROBARMHASHENTRY
{
    int stateID;                            //hash entry ID number
    short unsigned int coord[NUMOFLINKS];   //state coordinates
    short unsigned int endeff[3];           //end eff pos (xyz)
    short unsigned int action;              //successor action number
    double orientation[3][3];               //orientation of end effector (rotation matrix)
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

    //transformation matrices
    double Trans[4][4][NUMOFLINKS];

    //any additional variables
    int* Heur;
}EnvironmentROBARM3D_t;


/**
 * Environment to be used when planning for a Robotic Arm using the SBPL.
 */
class EnvironmentROBARM3D: public DiscreteSpaceInformation 
{
public:

    EnvironmentROBARM3D();
    ~EnvironmentROBARM3D(){};

    //environment related
    bool InitializeEnv(const char* sEnvFile);
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    /*!
     * @brief SetEnvParameter allows you to change parameters before the environment is initialized.
     * @param parameter name of parameter to change
     * @param value value to set parameter to
     */
    bool SetEnvParameter(char* parameter, double value);

    /*!
     * @brief Set the starting joint configuration of the manipulator.
     * @param angles a list of joint angles
     * @param bRad 0: degrees  1: radians
     */
    bool SetStartJointConfig(double angles[NUMOFLINKS], bool bRad);
    /*!
     * @brief Set the end effector goals.
     * @param EndEffGoals a list of n end effector goals (n x 12: {x, y, z, r11, r12, r13, r21, r22, r23, r31, r32, 33})
     * @param goal_type 0: cartesian  1: joint space (internally converted to cartesian coordinates, does NOT plan to joint space configuration)
     * @param num_goals number of goals in the list
     * @param bComputeHeuristic 1: recompute heuristic (needs to be set to 1, if called from outside the class)
     */
    bool SetEndEffGoals(double** EndEffGoals, int goal_type, int num_goals, bool bComputeHeuristic);
    /*!
     * @brief Add obstacles to the environment
     * @param obstacles a list of cubic obstacles (n x 6: {x_center, y_center, z_center, width, depth, height})
     * @param num_obstacles number of obstacles in the list
     */
    void AddObstaclesToEnv(double**obstacles, int numobstacles);
    /*!
     * @brief Clear the environment of any obstacles
     */
    void ClearEnv();
    /*!
     * @brief Check if path is valid. 
     * @param solution_stateIDs_V vector of stateIDs returned by planner
     */
    bool isPathValid(double** path, int num_waypoints);
    /*!
     * @brief Initialize the Environment & Arm Planner (using File Pointers)
     * @param eCfg pointer to file describing the environment
     * @param pCfg pointer to file with the Arm planner parameters
     */
    bool InitEnvFromFilePtr(FILE* eCfg, FILE* pCfg);
    bool SetEndEffGoals(double** EndEffGoals, int num_goals);

    //this should be removed  - it returns the planner Epsilon
    double GetEpsilon();

    //called by SBPL planner
    int GetFromToHeuristic(int FromStateID, int ToStateID);
    int GetFromToHeuristic(int FromStateID, int ToStateID, double FromRPY[3], double ToRPY[3]);
    int GetJointSpaceHeuristic(int FromStateID, int ToStateID);
    int GetGoalHeuristic(int stateID);
    int GetStartHeuristic(int stateID);
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    void SetAllPreds(CMDPSTATE* state);
    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    void StateID2Angles(int stateID, double* angles_r);
    int	 SizeofCreatedEnv();

    //printing 
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
    void PrintEnv_Config(FILE* fOut);
    void PrintHeurGrid();
    void PrintTimeStat(FILE* fOut);
    void OutputPlanningStats();

    //old function - needed when using KDL for collision detection - will eventually be removed
    void CloseKinNode();

    void AddObstacles(std::vector <std::vector <double> > obstacles);
    void getRPY(double Rot[3][3], double* roll, double* pitch, double* yaw, int solution_number);
//     bool SetEndEffGoals(vector<vector<double> >* EndEffGoals);
    bool SetJointSpaceGoals(double** JointSpaceGoals, int num_goals);
    std::vector<std::vector<double> >* getCollisionMap();

private:

    //member data
    EnvROBARMConfig_t EnvROBARMCfg;          /**< environment configuration struct (stores environment details)> */
    EnvironmentROBARM3D_t EnvROBARM;

    //hash table
    unsigned int GETHASHBIN(short unsigned int* coord, int numofcoord);
    void PrintHashTableHist();
    EnvROBARMHashEntry_t* GetHashEntry(short unsigned int* coord, int numofcoord, short unsigned int action, bool bIsGoal);
    EnvROBARMHashEntry_t* CreateNewHashEntry(short unsigned int* coord, int numofcoord, short unsigned int endeff[3], short unsigned int action, double orientation[3][3]);

    //initialization
    void ReadParamsFile(FILE* fCfg);
    void ReadConfiguration(FILE* fCfg);
    void InitializeEnvConfig();
    void InitializeEnvGrid();
    bool InitializeEnvironment();
    void AddObstacleToGrid(double* obstacle, int type, char*** grid, double gridcell_m);

    //coordinate frame/angle functions
    void DiscretizeAngles();
    void Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ);
    void Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ, double gridcell_m);
    void ComputeContAngles(short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS]);
    void ComputeCoord(double angle[NUMOFLINKS], short unsigned int coord[NUMOFLINKS]);
    void ContXYZ2Cell(double x, double y, double z, short unsigned int* pX, short unsigned int *pY, short unsigned int *pZ);
    void ContXYZ2Cell(double* xyz, double gridcellwidth, int dims_c[3], short unsigned int *pXYZ);
    void HighResGrid2LowResGrid(short unsigned int * XYZ_hr, short unsigned int * XYZ_lr);

    //bounds/error checking
    int IsValidCoord(short unsigned int coord[NUMOFLINKS], short unsigned int endeff_pos[3], short unsigned int wrist_pos[3], short unsigned int elbow_pos[3], double orientation[3][3]);    
    int IsValidLineSegment(double x0, double y0, double z0, double x1, double y1, double z1, char ***Grid3D, vector<CELLV>* pTestedCells);
    int IsValidLineSegment(short unsigned int x0, short unsigned int y0, short unsigned int z0, short unsigned int x1, short unsigned int y1, short unsigned int z1, char ***Grid3D, vector<CELLV>* pTestedCells);
    bool IsValidCell(int X, int Y, int Z);
    bool IsWithinMapCell(int X, int Y, int Z);
    bool AreEquivalent(int State1ID, int State2ID);
    bool RemoveGoal(int goal_index);
    void UpdateEnvironment();

    //cost functions
    int cost(short unsigned int state1coord[], short unsigned int state2coord[]); 
    int cost(EnvROBARMHashEntry_t* HashEntry1, EnvROBARMHashEntry_t* HashEntry2, bool bState2IsGoal);
    int GetEdgeCost(int FromStateID, int ToStateID);
    void ComputeActionCosts();
    void ComputeCostPerCell();
    double getAngularEuclDist(double rpy1[3], double rpy2[3]);

    //output
    void PrintHeader(FILE* fOut);
    void PrintConfiguration();
    void printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal);
    void PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/, FILE* fOut /*=NULL*/);
    void OutputActionCostTable();
    void OutputActions();
    void PrintAnglesWithAction(FILE* fOut, EnvROBARMHashEntry_t* HashEntry, bool bGoal, bool bVerbose, bool bLocal);
    void PrintAbridgedConfiguration();

    //compute heuristic
    void InitializeKinNode();
    void ComputeCostPerRadian();
    void getDistancetoGoal(int* HeurGrid, int goalx, int goaly, int goalz);
    int GetDistToClosestGoal(short unsigned int* xyz, int *goal_num);
    double GetDistToClosestGoal(double *xyz,int *goal_num);
    void ComputeHeuristicValues();
    void ReInitializeState3D(State3D* state);
    void InitializeState3D(State3D* state, short unsigned int x, short unsigned int y, short unsigned int z);
    void Create3DStateSpace(State3D**** statespace3D);
    void Delete3DStateSpace(State3D**** statespace3D);
    int XYZTO3DIND(int x, int y, int z);
//     void Search3DwithQueue(State3D*** statespace, int* HeurGrid, std::vector <std::vector<short unsigned int> >* EndEffGoals_c);
    void Search3DwithQueue(State3D*** statespace, int* HeurGrid, short unsigned int ** EndEffGoals_c);
    void Search3DwithQueue(State3D*** statespace, int* HeurGrid, short unsigned  int searchstartx, short unsigned int searchstarty, short unsigned int searchstartz);
//     void Search3DwithHeap(State3D*** statespace, int* HeurGrid, int searchstartx, int searchstarty, int searchstartz);

    //forward kinematics
    int ComputeEndEffectorPos(double angles[NUMOFLINKS], double endeff_m[3]);
    int ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int endeff[3]);
    int ComputeEndEffectorPos(double angles[NUMOFLINKS], short unsigned int endeff[3], short unsigned int wrist[3], short unsigned int elbow[3], double orientation[3][3]);
    void ComputeDHTransformations();
    void ComputeForwardKinematics_ROS(double *angles, int f_num, double *x, double *y, double *z);
    void ComputeForwardKinematics_DH(double angles[NUMOFLINKS]);

    /* JOINT SPACE PLANNING */
    void GetJointSpaceSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
};

#endif
