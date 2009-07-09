/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/** \Author: Benjamin Cohen  **/

/**

@mainpage

A simple wrapper for the planning_monitor class in the planning_environment package. It's intended to be used by someone who wants to use a planning monitor for collision detection on a selective set of joints but doesn't want the other benefits of a planning monitor.

This package is only temporary. I'll eventually replace it with a better solution.

@htmlinclude manifest.html

 **/

#include <ros/ros.h>
#include "ros/node_handle.h"
#include <planning_environment/planning_monitor.h>
#include <planning_environment/collision_models.h>
#include <motion_planning_msgs/KinematicJoint.h>
#include <motion_planning_srvs/MotionPlan.h>
#include <planning_models/kinematic_state_params.h>


namespace sbpl_arm_planner_node
{
  class pm_wrapper
  {
    public:

      pm_wrapper();

      ~pm_wrapper();

      bool initPlanningMonitor(const std::vector<std::string> &links, tf::TransformListener  * tfl);

      bool areLinksValid(const double * angles);

      void updatePM(const motion_planning_srvs::MotionPlan::Request &req);

      void unlockPM();

    private:

      ros::NodeHandle node_;

      ros::Subscriber col_map_subscriber_;

      planning_models::StateParams *start_state_;

      std::string arm_name_;

      std::string robot_description_;

      int groupID_;

      std::string frame_;

      std::vector<std::string> collision_check_links_;

      planning_environment::CollisionModels *collision_model_;

      planning_environment::PlanningMonitor *planning_monitor_;

      planning_models::StateParams* fillStartState(const std::vector<motion_planning_msgs::KinematicJoint> &given);
  };
}

