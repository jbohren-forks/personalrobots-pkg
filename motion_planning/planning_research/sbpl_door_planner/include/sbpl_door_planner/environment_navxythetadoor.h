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

#include <sbpl/headers.h>
#include <sbpl_door_planner/door_base_collision_cost.h>
#include <robot_msgs/Door.h>
#include <angles/angles.h>

#ifndef __ENVIRONMENT_NAVXYTHETADOOR_H_
#define __ENVIRONMENT_NAVXYTHETADOOR_H_

#define ENVNAVXYTHETADOOR_DEFAULTDESIREDDOORINTERVALIND 1 //adjacent to door open  
#define ENVNAVXYTHETADOOR_DEFAULTSTARTDOORINTERVALIND 1 //adjacent to door open


typedef struct 
{
	int stateID;
	int X;
	int Y;
	char Theta;
	unsigned char door_intervalind;
} EnvNAVXYTHETADOORHashEntry_t;


class EnvironmentNAVXYTHETADOORLAT : public EnvironmentNAVXYTHETALATTICE
{
  public:

  EnvironmentNAVXYTHETADOORLAT() 
  {desired_door_intervalindex = ENVNAVXYTHETADOOR_DEFAULTDESIREDDOORINTERVALIND;
   start_door_intervalindex = ENVNAVXYTHETADOOR_DEFAULTSTARTDOORINTERVALIND;};

  ~EnvironmentNAVXYTHETADOORLAT() {};

  void setDoorDiscretizationAngle(const double &door_angle_discretization_interval);

  door_base_collision_cost::DoorBaseCollisionCost db_; /*! Class used to compute free (valid) door angles given x, y, theta position of the robot */

  std::vector<int> desired_door_anglesV; 
  //There can only be two door intervals: it is either adjacent to "door closed" (0) or adjacent to "door open" (1)
  unsigned char desired_door_intervalindex;	
  unsigned char start_door_intervalindex;	

  //this function sets the door angles at which a goal configuration is declared EVEN if goalx,goaly,goaltheta are not satisfied.
  //The goal is ALSO declared if goalx,goaly,goaltheta are satisfied but none of the desired door angles are satisfied
  //It is therefore important to keep setting goalx,goaly,goaltheta, otherwise it will be 0s by default and may be satisfied right away. They are
  //also used to compute heuristics.
  //FInally, desired door angles should only be used when the search is done forward (no sense to set it in the backward search)
  void SetDesiredDoorAngles(vector<int> desired_door_anglesV);

  void GetPathMinDoorAngle(const std::vector<EnvNAVXYTHETALAT3Dpt_t> &path, std::vector<double> &angle, std::vector<double> &angle_cost);


  void SetAllActionsandAllOutcomes(CMDPSTATE* state){
    printf("ERROR: SetAllActionsandAllOutcomes not supported in navxythetadoor environment\n");
    exit(1);
  };
  void SetAllPreds(CMDPSTATE* state){
    printf("ERROR: SetAllPreds not supported in navxythetadoor environment\n");
    exit(1);
  };

  void GetPredsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *preds_of_changededgesIDV)
  {
    printf("ERROR: GetPredsofChangedEdges not supported in navxythetadoor environment\n");
    exit(1);
  };

  void GetSuccsofChangedEdges(vector<nav2dcell_t> const * changedcellsV, vector<int> *succs_of_changededgesIDV)
  {
    printf("ERROR: GetSuccsofChangedEdges not supported in navxythetadoor environment\n");
    exit(1); 
  };

  void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV){
    GetSuccs(SourceStateID, SuccIDV, CostV, NULL);
  }

  void GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV, vector<EnvNAVXYTHETALATAction_t*>* actionindV=NULL);

  void GetPreds(int TargetStateID, vector<int>* PredIDV, vector<int>* CostV){
    printf("ERROR: GetPreds not supported in navxythetadoor environment\n");
    exit(1);
  };


  bool GetMinCostDoorAngle(double x, double y, double theta, double &angle, double &door_angle_cost);

  void setDoorProperties(const robot_msgs::Door &door, double door_thickness);

  void setRobotProperties(const double &min_workspace_radius, 
                                                   const double &max_workspace_radius, 
                                                   const double &min_workspace_angle, 
                                                   const double &max_workspace_angle,
                          const robot_msgs::Point32 &robot_shoulder_position);                        


  //overloaded functions to support door_interval  
  virtual void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
  virtual int SetStart(double x, double y, double theta, unsigned char door_interval=ENVNAVXYTHETADOOR_DEFAULTSTARTDOORINTERVALIND);
  virtual int SetGoal(double x, double y, double theta, unsigned char door_interval=ENVNAVXYTHETADOOR_DEFAULTDESIREDDOORINTERVALIND);   
  virtual void SetGoalTolerance(double tol_x, double tol_y, double tol_theta) { /**< not used yet */ };
  virtual void GetCoordFromState(int stateID, int& x, int& y, int& theta, unsigned char& door_interval) const;
  virtual void GetCoordFromState(int stateID, int& x, int& y, int& theta) const;
  virtual int GetStateFromCoord(int x, int y, int theta, unsigned char door_interval=ENVNAVXYTHETADOOR_DEFAULTSTARTDOORINTERVALIND);
  virtual void ConvertStateIDPathintoXYThetaPath(vector<int>* stateIDPath, 
						 vector<EnvNAVXYTHETALAT3Dpt_t>* xythetaPath, vector<unsigned char>* door_intervalindPath);

  virtual void GetActionCost(int SourceX, int SourceY, int SourceTheta, int SourceDoorIntervalIndex,
		     EnvNAVXYTHETALATAction_t* action,
		     int* pCosttoDoorInterval0, int* pCosttoDoorInterval1);

  //returns possible doorangle and associated costs (costs are used as multipliers, cost = 0 will be used as a
  //a multiplication factor of 1 (no penalty). Infinite cost should be indicated by INFINITECOST, or better if not
  //returned at all as a possible doorangle.
  void GetValidDoorAngles(EnvNAVXYTHETALAT3Dpt_t worldrobotpose3D, vector<int>* doorangleV, 
                          vector<int>* dooranglecostV);


  //returns the cost to getting to mincost desired door angles while staying at this pose
  //returns infinity if no desired goal angle is reachable
  int MinCostDesiredDoorAngle(int x, int y, int theta, int door_intervalind);

   //overloaded functions to support door_interval  
  virtual EnvNAVXYTHETADOORHashEntry_t*  GetHashEntry(int X, int Y, int Theta, 
						      unsigned char door_interval);
   virtual EnvNAVXYTHETADOORHashEntry_t* CreateNewHashEntry(int X, int Y, int Theta, unsigned char door_interval);

   virtual int  GetFromToHeuristic(int FromStateID, int ToStateID);
   virtual int  GetGoalHeuristic(int stateID);
   virtual int  GetStartHeuristic(int stateID);
   
   virtual int	 SizeofCreatedEnv();

   virtual void InitializeEnvironment();

  protected:


   //hash table of size x_size*y_size. Maps from coords to stateId	
   int HashTableSize;
   vector<EnvNAVXYTHETADOORHashEntry_t*>* Coord2StateIDHashTable;
   //vector that maps from stateID to coords	
   vector<EnvNAVXYTHETADOORHashEntry_t*> StateID2CoordTable;

   unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Theta);
};

typedef class EnvironmentNAVXYTHETADOORLAT EnvironmentNAVXYTHETADOOR;

#endif
