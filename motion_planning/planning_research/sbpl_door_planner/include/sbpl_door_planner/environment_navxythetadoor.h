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


class EnvironmentNAVXYTHETADOOR : public EnvironmentNAVXYTHETALAT
{
  public:

  EnvironmentNAVXYTHETADOOR() {};

  ~EnvironmentNAVXYTHETADOOR() {};


  door_base_collision_cost::DoorBaseCollisionCost db_; /*! Class used to compute free (valid) door angles given x, y, theta position of the robot */

  std::vector<int> desired_door_anglesV; 

  //this function sets the door angles at which a goal configuration is declared EVEN if goalx,goaly,goaltheta are not satisfied.
  //The goal is ALSO declared if goalx,goaly,goaltheta are satisfied but none of the desired door angles are satisfied
  //It is therefore important to keep setting goalx,goaly,goaltheta, otherwise it will be 0s by default and may be satisfied right away. They are
  //also used to compute heuristics.
  //FInally, desired door angles should only be used when the search is done forward (no sense to set it in the backward search)
  void SetDesiredDoorAngles(vector<int> desired_door_anglesV);


  void SetAllActionsandAllOutcomes(CMDPSTATE* state){
    printf("ERROR: SetAllActionsandAllOutcomes not supported in navxythetadoor environment\n");
    exit(1);
  };
  void SetAllPreds(CMDPSTATE* state){
    printf("ERROR: SetAllPreds not supported in navxythetadoor environment\n");
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
                          const double &robot_shoulder_position_x,
                          const double &robot_shoulder_position_y);

  protected:

  virtual int GetActionCost(int SourceX, int SourceY, int SourceTheta, EnvNAVXYTHETALATAction_t* action);

  //returns possible doorangle and associated costs (costs are used as multipliers, cost = 0 will be used as a
  //a multiplication factor of 1 (no penalty). Infinite cost should be indicated by INFINITECOST, or better if not
  //returned at all as a possible doorangle.
  void GetValidDoorAngles(EnvNAVXYTHETALAT3Dpt_t worldrobotpose3D, vector<int>* doorangleV, 
                          vector<int>* dooranglecostV);


  //returns the cost to getting to mincost desired door angles while staying at this pose
  //returns infinity if no desired goal angle is reachable
  int MinCostDesiredDoorAngle(int x, int y, int theta);

};

#endif
