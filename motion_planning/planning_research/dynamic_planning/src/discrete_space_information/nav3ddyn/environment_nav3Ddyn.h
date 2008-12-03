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
#ifndef __ENVIRONMENT_NAV3DDYN_H_
#define __ENVIRONMENT_NAV3DDYN_H_

//shrink footprint
#define NAV3DDYN_SHRINK_FOOTPRINT 0

//eight-connected grid
#define NAV3DDYN_DXYWIDTH 1

//number of theta directions
#define NAV3DDYN_THETADIRS 8

//number of actions per x,y,theta state
#define NAV3DDYN_ACTIONWIDTH 5 //decrease, increase, same angle while moving plus decrease, increase angle while standing.

#define NAV3DDYN_COSTMULT 1000

#define NAV3DDYN_MAXRV 1.57 //max rotational velocity - radians per second
#define NAV3DDYN_NUMRV 5 //number of rotational velocity values to try

//seconds
#define NAV3DDYN_SHORTDUR 0.5
#define NAV3DDYN_LONGDUR 2
#define NAV3DDYN_DURDISC 0.1

#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)

struct EnvNAV3DDYN2Dpt_t{
  double x;
  double y;

  bool operator==(const EnvNAV3DDYN2Dpt_t &p) const{
    return (x == p.x && y==p.y);
  }

  bool operator<(const EnvNAV3DDYN2Dpt_t &p) const{
    return (x < p.x || (x == p.x && y < p.y));
  }
}; 

struct EnvNAV3DDYNContPose_t{
  double X;
  double Y;
  double Theta;

  bool operator==(const EnvNAV3DDYNContPose_t &p) const{
    return (X==p.X && Y==p.Y && Theta==p.Theta);
  }

};

struct EnvNAV3DDYNDiscPose_t
{
  int X;
  int Y;
  int Theta;

  bool operator==(const EnvNAV3DDYNDiscPose_t &p) const{
    return (X==p.X && Y==p.Y && Theta==p.Theta);
  }

};

typedef struct
{
  int dX;  //change in x during action
  int dY;  //change in y
  int dTheta; //change in theta
  unsigned int cost; 

  double rv;  //rotational velocity for action
  double tv;  //translational velocity for action
  vector<EnvNAV3DDYNContPose_t> path_cont; //for debugging purposes
  vector<EnvNAV3DDYNDiscPose_t> path;  //path of center of robot
  vector<EnvNAV3DDYN2Dpt_t> footprint; //footprint swept during action

  int time; //time to execute action

} EnvNAV3DDYNAction_t;



//configuration parameters
typedef struct ENV_NAV3DDYN_CONFIG
{
  
  int EnvWidth_c;
  int EnvHeight_c;
  int StartX_c;
  int StartY_c;
  int StartTheta;
  int EndX_c;
  int EndY_c;
  int EndTheta;
  
  char** Grid2D;

  double nominalvel_mpersecs; //translational velocity
  double timetoturnoneunitinplace_secs;
  double cellsize_m;
  
  int dXY[NAV3DDYN_DXYWIDTH][2];

  //footprint of robot
  vector <EnvNAV3DDYN2Dpt_t> FootprintPolygon; //real world coordinates
  
  EnvNAV3DDYNAction_t** ActionsV; //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
  vector<EnvNAV3DDYNAction_t*>* PredActionsV;
  
} EnvNAV3DDYNConfig_t;



typedef struct 
{
	int stateID;
	int X;
	int Y;
	char Theta;
} EnvNAV3DDYNHashEntry_t;



//variables that dynamically change (e.g., array of states, ...)
typedef struct
{

	int startstateid;
	int goalstateid;

	//hash table of size x_size*y_size. Maps from coords to stateId	
	int HashTableSize;
	vector<EnvNAV3DDYNHashEntry_t*>* Coord2StateIDHashTable;

	//vector that maps from stateID to coords	
	vector<EnvNAV3DDYNHashEntry_t*> StateID2CoordTable;

	//any additional variables

}EnvironmentNAV3DDYN_t;



class EnvironmentNAV3DDYN : public DiscreteSpaceInformation
{

public:
  ~EnvironmentNAV3DDYN(){};

  const EnvNAV3DDYNConfig_t* GetEnvNavConfig();

  //environment initialization functions
  bool InitializeEnv(const char* sEnvFile);
  bool InitializeEnv(int width, int height,
		     const unsigned char* mapdata,
		     double startx, double starty, double starttheta,
		     double goalx, double goaly, double goaltheta,
		     double goaltol_x, double goaltol_y, double goaltol_theta,
		     const vector<sbpl_2Dpt_t> & perimeterptsV,
		     double cellsize_m, double nominalvel_mpersecs, double timetoturnoneunitinplace_secs);

  void PrecomputeActions();
  void CalculateFootprintForPath(vector<EnvNAV3DDYNContPose_t> path, vector<EnvNAV3DDYN2Dpt_t>* footprint);
  void CalculateFootprintForPose(EnvNAV3DDYNContPose_t pose, vector<EnvNAV3DDYN2Dpt_t>* footprint);
  void RemoveDuplicatesFromPath(vector<EnvNAV3DDYNDiscPose_t>* path);
  void RemoveDuplicatesFromFootprint(vector<EnvNAV3DDYN2Dpt_t>* footprint);
  void DiscretizeAndShrinkFootprintBoundary();
  

  //planning initialization functions
  bool InitializeMDPCfg(MDPConfig *MDPCfg);
  int SetStart(double x, double y, double theta);
  int SetGoal(double x, double y, double theta);

  //planning functions
  void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV);
  void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV);
  int  GetFromToHeuristic(int FromStateID, int ToStateID);
  bool UpdateCost(int x, int y, int new_status);
  void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;
  int GetStateFromCoord(int x, int y, int theta);
  bool IsObstacle(int x, int y);

  //debugging functions
  void PrintConfigurationToFile(const char* logFile);
  void PrintActionsToFile(const char* logFile);
  void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
  void PrintTimeStat(FILE* fOut);
  int	 SizeofCreatedEnv();
  void GetEnvParms(int *size_x, int *size_y, double* startx, double* starty, double* starttheta, double* goalx, double* goaly, double* goaltheta,
		   double* cellsize_m);
  void GetContPathFromStateIds(vector<int> stateIdV, vector<EnvNAV3DDYNContPose_t>* path);

  //functions updated but not tested  
  int  GetGoalHeuristic(int stateID);
  int  GetStartHeuristic(int stateID);
  


  //Not Yet Implemented
  void SetAllActionsandAllOutcomes(CMDPSTATE* state);
  void SetAllPreds(CMDPSTATE* state);
  void PrintEnv_Config(FILE* fOut);
  void GetPredsofChangedEdges(vector<nav2dcell_t>* changedcellsV, vector<int> *preds_of_changededgesIDV);
  

 private:
  
	//member data
	EnvNAV3DDYNConfig_t EnvNAV3DDYNCfg;
	EnvironmentNAV3DDYN_t EnvNAV3DDYN;

	//hash table functions
	unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);
	EnvNAV3DDYNHashEntry_t* GetHashEntry(int X, int Y, int Theta);

	//Initialization functions
	void ReadConfiguration(FILE* fCfg);

	void SetConfiguration(int width, int height,
			      const unsigned char* mapdata,
			      int startx, int starty, int starttheta,
			      int goalx, int goaly, int goaltheta,
			      double cellsize_m, double nominalvel_mpersecs, double timetoturnoneunitinplace_secs, const vector<sbpl_2Dpt_t> & robot_perimeterV);
	

	bool InitGeneral();
	void InitializeEnvConfig();
	void InitializeEnvironment();
	void ComputeHeuristicValues();
	int InsideFootprint(EnvNAV3DDYN2Dpt_t pt, vector<EnvNAV3DDYN2Dpt_t>* bounding_polygon);
	void CreateStartandGoalStates();

	//debugging functions
	void PrintHashTableHist();
	bool IsValidCell(int X, int Y);
	bool IsWithinMapCell(int X, int Y);

	
	bool CheckQuant(FILE* fOut);
	EnvNAV3DDYNHashEntry_t* CreateNewHashEntry(int X, int Y, int Theta);
		
};

#endif

