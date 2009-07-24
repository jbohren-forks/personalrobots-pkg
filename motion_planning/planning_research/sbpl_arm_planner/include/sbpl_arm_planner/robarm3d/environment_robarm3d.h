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

/** \Author: Benjamin Cohen /bcohen@willowgarage.com **/

#include <boost/thread/mutex.hpp>
#include <kdl_parser/tree_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <voxel3d/voxel3d.h>
#include "boost/shared_ptr.hpp"
// #include <yaml-cpp/yaml.h>
#include <sbpl_pm_wrapper/pm_wrapper.h>

#ifndef __ENVIRONMENT_ROBARM3D_H_
#define __ENVIRONMENT_ROBARM3D_H_

#define NUMOFLINKS 7

#define UNIFORM_COST 1      // all the joint actions have the same costs when set

#define NUM_TRANS_MATRICES 360 //precomputed matrices


/**

@mainpage

A motion planner for a manipulator based on the SBPL. It is capable of finding the shortest path to multiple 6 DoF destinations or a joint configuration.


@htmlinclude manifest.html

@b sbpl_arm_planner will be fully integrated with ros....coming soon.

 **/


/** coords - used to pass around lists of valid cells */
typedef struct
{
    short unsigned int x;
    short unsigned int y;
    short unsigned int z;
    bool bIsObstacle;
}CELLV;

/** state structure for Dykstra's Algorithm */
typedef struct STATE3D_t
{
    unsigned int g;
    short unsigned int iterationclosed;
    short unsigned int x;
    short unsigned int y;
    short unsigned int z;
} State3D;

/** struct that contains the necessary cartesian goal description */
typedef struct GOAL_POSITION
{
  int type;
  double pos[3];
  double rpy[3];
  short unsigned int xyz_tolerance;
  short unsigned int xyz[3];
  short unsigned int xyz_lr[3];
  double rpy_tolerance;
} GoalPos;

/** struct that contains the necessary joint space goal description */
typedef struct GOAL_CONFIGURATION
{
	short unsigned int xyz[3];
  std::vector <double> pos;
  std::vector <double> tolerance_above;
  std::vector <double> tolerance_below;
} GoalConfig;

typedef struct ARM_LINK
{
  short unsigned int joint1;
  short unsigned int joint2;
  double radius;
} arm_link;

typedef struct JOINT
{
  std::string name;
  double positive_limit;
  double negative_limit;
  bool cont;
  double axis[3];
} joint;

/** struct that contains robot arm physical configuration & description of the environment */
typedef struct ENV_ROBARM_CONFIG
{
/* Environment Description */

    //environment dimensions (meters)
    double EnvWidth_m;
    double EnvHeight_m;
    double EnvDepth_m;

    //environment dimensions (cells)
    int EnvWidth_c;
    int EnvHeight_c;
    int EnvDepth_c;

    //low resolution environment dimensions (cells)
    int LowResEnvWidth_c;
    int LowResEnvHeight_c;
    int LowResEnvDepth_c;

    //fixed location of shoulder base (cells) 
    short unsigned int BaseX_c;
    short unsigned int BaseY_c;
    short unsigned int BaseZ_c;

    //fixed location of shoulder base (meters)
    double BaseX_m;
    double BaseY_m;
    double BaseZ_m;

    double origin_m[3];

    //flag determines if the environment has been initialized or not
    bool bInitialized;
    /*------------- Heuristic & Collision Checking -------------*/
    unsigned char*** Grid3D;
    unsigned char*** LowResGrid3D;
    double GridCellWidth;           // cells are square (width=height=depth)
    double LowResGridCellWidth;     // cells are square (width=height=depth)
    unsigned char*** Grid3D_temp;
    unsigned char*** LowResGrid3D_temp;
    short unsigned int Grid3D_dims[3];
    short unsigned int LowResGrid3D_dims[3];

    //obstacles
    std::vector<std::vector<double> > cubes;
    std::vector<std::vector<double> > sbpl_cubes;

    //cost of cells on grid of obstacles and close to obstacles
    char ObstacleCost;
    char medObstacleCost;
    char lowObstacleCost;
    int medCostRadius_c;
    double padding;
    int lowCostRadius_c;

    //mutexes to protect temporary & planning maps
    boost::mutex mCopyingGrid;
    boost::mutex mPlanning;

    /*------------- Manipulator Description -------------*/
    std::string robot_desc;
    double gripper_m[3];

    short unsigned int num_joints;

    //DH Parameters
    double DH_alpha[NUMOFLINKS];
    double DH_a[NUMOFLINKS];
    double DH_d[NUMOFLINKS];
    double DH_theta[NUMOFLINKS];

    //robot arm dimensions/positions
    double LinkStartAngles_d[NUMOFLINKS];

    //Motor Limits
    double PosMotorLimits[NUMOFLINKS];
    double NegMotorLimits[NUMOFLINKS]; //must be negative values

    std::vector <joint> joints;

    //joint angle discretization
    double angledelta[NUMOFLINKS];
    int anglevals[NUMOFLINKS];

    // temporary hack
    int arm_length;

    std::vector <double> link_radius;

    int gripper_radius;
    int forearm_radius;
    int upper_arm_radius;
	
    //for kinematic library use
    KDL::JntArray jnt_pos_in;
    KDL::Frame p_out;
    KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver;
    KDL::Chain arm_chain;

/* Planner Parameters/Options */

    //goals
    bool bGoalIsSet;
    bool PlanInJointSpace;
    std::vector <bool> uselessActions;
    std::vector <GoalPos> EndEffGoals;
    std::vector <GoalConfig> JointSpaceGoals;
    std::vector <std::vector <double> > ParsedGoals;
    std::vector <std::vector <double> >  ParsedGoalTolerance;

    //actual coords of goal - shouldn't be here
    short unsigned int goalcoords[NUMOFLINKS];

    //epsilon to be used by planner
    //this is the wrong place to store it, but it allows epsilon to be read in params.cfg
    double epsilon;

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
    bool sum_heuristics;
    double smoothing_weight;
    double gripper_orientation_moe; //gripper orientation margin of error
    double grasped_object_length_m;
    bool use_voxel3d_occupancy_grid;
    double ApplyRPYCost_m;
    bool use_selective_actions;
    bool exact_gripper_collision_checking;

/* Motion Primitives */
    //successor actions
    double ** SuccActions;
    int nSuccActions;
    int nLowResActions;
    int ** ActiontoActionCosts;
    int HighResActionsThreshold_c;
    double HighResActionsThreshold_r;

/* Costs */
    //action costs - TODO change them to integers
    double CellsPerAction;
    double cost_per_cell;
    double cost_sqrt2_move;
    double cost_sqrt3_move;
    double cost_per_mm;
    double cost_per_rad;
    double cost_per_rad_js;

    //for precomputing cos/sin & DH matrices
    double cos_r[360];
    double sin_r[360];
    double T_DH[4*NUM_TRANS_MATRICES][4][NUMOFLINKS];

} EnvROBARMConfig_t;

/** hash entry that contains end effector coordinates */
typedef struct ENVROBARMHASHENTRY
{
    int stateID;                            //hash entry ID number
    short unsigned int coord[NUMOFLINKS];   //state coordinates
    short unsigned int endeff[3];           //end eff pos (xyz)
    short unsigned int action;              //successor action number
    double orientation[3][3];               //orientation of end effector (rotation matrix)
    double axis_angle;
} EnvROBARMHashEntry_t;

/** main structure that stores environment data used in planning */
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


/** Environment to be used when planning for a Robotic Arm using the SBPL. */
class EnvironmentROBARM3D: public DiscreteSpaceInformation 
{
  public:

		std::vector<int> expanded_states;
		bool save_expanded_states;

    boost::shared_ptr<Voxel3d> voxel_grid;
    boost::shared_ptr<Voxel3d> lowres_voxel_grid;

		
		
    EnvironmentROBARM3D();
    ~EnvironmentROBARM3D(){};

    //environment related
    bool InitializeEnv(const char* sEnvFile);
    bool InitializeMDPCfg(MDPConfig *MDPCfg);
    /*!
     * @brief SetEnvParameter allows you to change parameters before the environment is initialized.
     * @param param name of parameter to change
     * @param value value to set parameter to
     */
    bool SetEnvParameter(std::string param, double value);

    /*!
     * @brief Set the starting joint configuration of the manipulator.
     * @param angles a list of joint angles
     * @param bRad 0: degrees  1: radians
     */
    bool SetStartJointConfig(const double angles[], bool bRad);
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
    bool InitEnvFromFilePtr(FILE* eCfg, FILE* pCfg, const std::string &robot_desc);

    //this should be removed  - it returns the planner Epsilon
    double GetEpsilon();

    /** functions needed by SBPL planner */
    int GetFromToHeuristic(int FromStateID, int ToStateID);
    int GetJointSpaceHeuristic(int FromStateID, int ToStateID);
    int GetGoalHeuristic(int stateID);
    int GetStartHeuristic(int stateID);
    void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    void SetAllPreds(CMDPSTATE* state);
    void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
    void StateID2Angles(int stateID, double* angles_r);
    int	 SizeofCreatedEnv();

    /** output */
    void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
    void PrintEnv_Config(FILE* fOut);
    void PrintHeurGrid();
    void PrintTimeStat(FILE* fOut);
    void OutputPlanningStats();

    /** used when debugging & taking statistics (remove these functions once everything is solidified) */
    void getRPY(double Rot[3][3], double* roll, double* pitch, double* yaw, int solution_number);
    int GetFromToHeuristic(int FromStateID, int ToStateID, FILE* fOut);
    // void getValidPositions(int num_pos, FILE* fOut);
    double gen_rand(double min, double max);
    int ComputeEndEffectorPos(const double angles[], double endeff_m[3], double rpy_r[3]);

    /** used by sbpl_arm_planner_node */
    std::vector<std::vector<double> >* getCollisionMap();
    void AddObstacles(const std::vector <std::vector <double> > &obstacles);
    bool SetGoalPosition(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerances, const std::vector<int> &type);
    void SetGoalPositionTolerance(const std::vector <std::vector<double> > &tolerance);
    void SetGoalPositionType(const std::vector <int> &goal_type);
    bool SetGoalConfiguration(const std::vector <std::vector<double> > &goals, const std::vector<std::vector<double> > &tolerance_above, const std::vector<std::vector<double> > &tolerance_below);
    void SetGoalConfigurationTolerance(const std::vector<std::vector<double> > &tolerance_above, const std::vector<std::vector<double> > &tolerance_below);

    bool updateVoxelGrid(const boost::shared_ptr<Voxel3d> envgrid, const boost::shared_ptr<Voxel3d> lowres_envgrid);
    void initPlanningMonitor(sbpl_arm_planner_node::pm_wrapper * pm);
    bool isValidJointConfiguration(const double  * angles);

    void copyVoxelGrid(const boost::shared_ptr<Voxel3d> &voxel_grid, unsigned char *** Grid3D);

		std::vector<int> debugExpandedStates();

  private:

    /** member data */
    EnvROBARMConfig_t EnvROBARMCfg;          /**< environment configuration struct (stores environment details)> */
    EnvironmentROBARM3D_t EnvROBARM;

    sbpl_arm_planner_node::pm_wrapper *planning_monitor;

    /** hash table */
    unsigned int GETHASHBIN(short unsigned int* coord, int numofcoord);
    bool AreEquivalent(int State1ID, int State2ID);
    EnvROBARMHashEntry_t* GetHashEntry(short unsigned int* coord, int numofcoord, short unsigned int action, bool bIsGoal);
    EnvROBARMHashEntry_t* CreateNewHashEntry(short unsigned int* coord, int numofcoord, short unsigned int endeff[3], short unsigned int action, double orientation[3][3]);

    /** initialization */
    void InitializeEnvConfig();
    void InitializeEnvGrid();
    bool InitializeEnvironment();
    bool InitGeneral();
    void ReadParamsFile(FILE* fCfg);
    void ReadConfiguration(FILE* fCfg);
    void ParseYAMLFile(const char* sParamFile);

    /** coordinate frame/angle functions */
    void DiscretizeAngles();
    inline void Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ);
		inline void Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ, double gridcell_m);
		inline void ComputeContAngles(const short unsigned int coord[NUMOFLINKS], double angle[NUMOFLINKS]);
		inline void ComputeCoord(const double angle[], short unsigned int coord[NUMOFLINKS]);
		inline void ContXYZ2Cell(double x, double y, double z, short unsigned int* pX, short unsigned int *pY, short unsigned int *pZ);
		inline void ContXYZ2Cell(double* xyz, double gridcellwidth, int dims_c[3], short unsigned int *pXYZ);
    void HighResGrid2LowResGrid(short unsigned int * XYZ_hr, short unsigned int * XYZ_lr);

    /** collision checking */
    int IsValidCoord(short unsigned int coord[NUMOFLINKS], short unsigned int endeff_pos[3], short unsigned int wrist_pos[3], short unsigned int elbow_pos[3], double orientation[3][3]);
    int IsValidCoord(const short unsigned int coord[], const std::vector<std::vector<short unsigned int> > &joints, double orientation[3][3], unsigned char ***Grid, const short unsigned int grid_dims[]);
    int IsValidLineSegment(short unsigned int x0, short unsigned int y0, short unsigned int z0, short unsigned int x1, short unsigned int y1, short unsigned int z1, unsigned char ***Grid3D, vector<CELLV>* pTestedCells);
    int IsValidLineSegment(const short unsigned int xyz0[], const short unsigned int xyz1[], const int &radius, unsigned char*** Grid3D, const int grid_dims[], vector<CELLV>* pTestedCells);
    int IsValidLineSegment(const short unsigned int a[], const short unsigned int b[], int radius, vector<CELLV>* pTestedCells, unsigned char *** Grid3D,const  boost::shared_ptr<Voxel3d> vGrid);
    void UpdateEnvironment();
    void AddObstacleToGrid(double* obstacle, int type, unsigned char*** grid, double gridcell_m);
    double distanceBetween3DLineSegments(const short unsigned int l1a[],const short unsigned int l1b[],
					 const short unsigned int l2a[],const short unsigned int l2b[]);
    inline unsigned char getVoxel(const int x, const int y, const int z, const boost::shared_ptr<Voxel3d> grid)
      { return (*grid)(x,y,z);  }

    bool isValidCell(const int xyz[], const int &radius, unsigned char ***Grid, const boost::shared_ptr<Voxel3d> vGrid);
    bool isValidCell(const short unsigned int xyz[], const int &radius, unsigned char ***Grid, const boost::shared_ptr<Voxel3d> vGrid);
    bool isValidCell(const int x, const int y, const int z, const int &radius, unsigned char ***Grid, const boost::shared_ptr<Voxel3d> vGrid);

    /** planning */
    void GetJointSpaceSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
    bool isGoalPosition(const short unsigned int endeff[3], const double orientation[3][3], const GoalPos &goal, const double &axis_angle);
    bool isGoalState(const double angles[], const GoalConfig &goal);

    /** costs */
    int cost(short unsigned int state1coord[], short unsigned int state2coord[]); 
    int cost(EnvROBARMHashEntry_t* HashEntry1, EnvROBARMHashEntry_t* HashEntry2, bool bState2IsGoal);
    int GetEdgeCost(int FromStateID, int ToStateID);
    void ComputeActionCosts();
    void ComputeCostPerCell();
    void ComputeCostPerRadian();

    /** output */
    void PrintHeader(FILE* fOut);
    void PrintHashTableHist();
    void PrintAbridgedConfiguration();
    void PrintConfiguration(FILE* fOut);
    void printangles(FILE* fOut, short unsigned int* coord, bool bGoal, bool bVerbose, bool bLocal);
    void PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal /*=false*/, FILE* fOut /*=NULL*/);
    void OutputActionCostTable(FILE* fOut);
    void OutputActions(FILE* fOut);
    void PrintAnglesWithAction(FILE* fOut, EnvROBARMHashEntry_t* HashEntry, bool bGoal, bool bVerbose, bool bLocal);

    /** heuristic */
    void ComputeHeuristicValues();
    void ReInitializeState3D(State3D* state);
    void InitializeState3D(State3D* state, short unsigned int x, short unsigned int y, short unsigned int z);
    void Create3DStateSpace(State3D**** statespace3D);
    void Delete3DStateSpace(State3D**** statespace3D);
    int XYZTO3DIND(int x, int y, int z);
    void Search3DwithQueue(State3D*** statespace, int* HeurGrid, const vector< GoalPos> &Goals);

    /** distance */
    int GetDistToClosestGoal(short unsigned int* xyz, int *goal_num);
    double GetDistToClosestGoal(double *xyz,int *goal_num);
    double getAngularEuclDist(double rpy1[3], double rpy2[3]);

    /** angles */
    void GetAxisAngle(double R1[3][3], double R2[3][3], double* angle);
    void RPY2Rot(double roll, double pitch, double yaw, double Rot[3][3]);

    /** forward kinematics */
    void ComputeDHTransformations();
    bool InitKDLChain(const string &fKDL);
    int ComputeEndEffectorPos(const double angles[], double endeff_m[3]);
    int ComputeEndEffectorPos(const double angles[], short unsigned int endeff[3]);
    int ComputeEndEffectorPos(const double angles[], short unsigned int endeff[3], short unsigned int wrist[3], short unsigned int elbow[3], double orientation[3][3]);
    void ComputeForwardKinematics_DH(const double angles[]);
    void ComputeForwardKinematics_ROS(const double angles[], int f_num, double *x, double *y, double *z);

    double getDistanceToGoal(const double angles[]);
};

inline bool EnvironmentROBARM3D::isValidCell(const int xyz[], const int &radius, unsigned char ***Grid, const boost::shared_ptr<Voxel3d> vGrid)
{
// 	printf("(%i %i %i) = %u\n",xyz[0], xyz[1], xyz[2], Grid[xyz[0]][xyz[1]][xyz[2]]);
	if(Grid[xyz[0]][xyz[1]][xyz[2]] <= radius)
		return false;

	return true;
}

inline bool EnvironmentROBARM3D::isValidCell(const int x, const int y, const int z, const int &radius, unsigned char ***Grid, const boost::shared_ptr<Voxel3d> vGrid)
{
// 	printf("(%i %i %i) = %u\n",x,y,z, Grid[x][y][z]);
	if(Grid[x][y][z] <= radius)
		return false;
	
	return true;
}

inline bool EnvironmentROBARM3D::isValidCell(const short unsigned int xyz[], const int &radius, unsigned char ***Grid, const boost::shared_ptr<Voxel3d> vGrid)
{
// 	printf("(%i %i %i) = %u\n",xyz[0], xyz[1], xyz[2], Grid[xyz[0]][xyz[1]][xyz[2]]);
	if(Grid[xyz[0]][xyz[1]][xyz[2]] <= radius)
		return false;

	return true;
}

//angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin 0, ...
inline void EnvironmentROBARM3D::ComputeContAngles(const short unsigned int coord[], double angle[])
{
	for(int i = 0; i < EnvROBARMCfg.num_joints; i++)
		angle[i] = coord[i]*EnvROBARMCfg.angledelta[i];
}

inline void EnvironmentROBARM3D::ComputeCoord(const double angle[], short unsigned int coord[])
{
	double pos_angle;

	for(int i = 0; i < EnvROBARMCfg.num_joints; i++)
	{
			//NOTE: Added 3/1/09
		pos_angle = angle[i];
		if(pos_angle < 0.0)
			pos_angle += 2*M_PI;

		coord[i] = (int)((pos_angle + EnvROBARMCfg.angledelta[i]*0.5)/EnvROBARMCfg.angledelta[i]);

		if(coord[i] == EnvROBARMCfg.anglevals[i])
			coord[i] = 0;
	}
}

// convert a cell in the occupancy grid to point in real world 
inline void EnvironmentROBARM3D::Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ)
{
	*pX = x * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.origin_m[0];
	*pY = y * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.origin_m[1];
	*pZ = z * EnvROBARMCfg.GridCellWidth + EnvROBARMCfg.origin_m[2];
}

// convert a cell in the occupancy grid to point in real world
inline void EnvironmentROBARM3D::Cell2ContXYZ(int x, int y, int z, double *pX, double *pY, double *pZ, double gridcell_m)
{
	*pX = x * gridcell_m + EnvROBARMCfg.origin_m[0];
	*pY = y * gridcell_m + EnvROBARMCfg.origin_m[1];
	*pZ = z * gridcell_m + EnvROBARMCfg.origin_m[2];
}

// convert a point in real world to a cell in occupancy grid 
inline void EnvironmentROBARM3D::ContXYZ2Cell(double x, double y, double z, short unsigned int *pX, short unsigned int *pY, short unsigned int *pZ)
{
	*pX = (int)((x - EnvROBARMCfg.origin_m[0]) / EnvROBARMCfg.GridCellWidth);
	if(*pX >= EnvROBARMCfg.EnvWidth_c) *pX = EnvROBARMCfg.EnvWidth_c-1;

	*pY = (int)((y - EnvROBARMCfg.origin_m[1]) / EnvROBARMCfg.GridCellWidth);
	if(*pX >= EnvROBARMCfg.EnvWidth_c) *pX = EnvROBARMCfg.EnvWidth_c-1;

	*pZ = (int)((z - EnvROBARMCfg.origin_m[2]) / EnvROBARMCfg.GridCellWidth);
	if(*pZ >= EnvROBARMCfg.EnvDepth_c) *pZ = EnvROBARMCfg.EnvDepth_c-1;
}

inline void EnvironmentROBARM3D::ContXYZ2Cell(double* xyz, double gridcellwidth, int dims_c[3], short unsigned int *pXYZ)
{
	pXYZ[0] = (int)((xyz[0] - EnvROBARMCfg.origin_m[0]) / gridcellwidth);
	if(pXYZ[0] >= dims_c[0])
		pXYZ[0] = dims_c[0]-1;

	pXYZ[1] = (int)((xyz[1] - EnvROBARMCfg.origin_m[1]) / gridcellwidth);
	if(pXYZ[1] >= dims_c[1])
		pXYZ[1] = dims_c[1]-1;

	pXYZ[2] = (int)((xyz[2] - EnvROBARMCfg.origin_m[2]) / gridcellwidth);
	if(pXYZ[2] >= dims_c[2]) 
		pXYZ[2] = dims_c[2]-1;
}

#endif

