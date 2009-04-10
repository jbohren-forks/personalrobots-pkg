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
 *   * Neither the name of Willow Garage nor the names of its
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


/* Author: Melonee Wise */

#ifndef ACTION_MOVE_AND_GRASP_PLUG_H
#define ACTION_MOVE_AND_GRASP_PLUG_H

// ROS Stuff
#include <ros/node.h>

// Msgs
#include <robot_mechanism_controllers/JointControllerState.h>
#include <robot_msgs/PlugStow.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

// Srvs 
#include <robot_srvs/MoveToPose.h>

// Robot Action Stuff
#include <robot_actions/action.h>


namespace plugs_core{

class MoveAndGraspPlugAction: public robot_actions::Action<robot_msgs::PlugStow, std_msgs::Empty>
{
public:
  MoveAndGraspPlugAction();
  ~MoveAndGraspPlugAction();

  virtual void handleActivate(const robot_msgs::PlugStow& plug_stow);
  virtual void handlePreempt();


private:
  
  void reset();
  void moveToGrasp();
  void graspPlug();
  void checkGrasp();
  
  std::string action_name_;

  ros::Node* node_;

  std::string gripper_controller_;
  std::string arm_controller_;

  std_msgs::Empty empty_;
  robot_msgs::PlugStow plug_stow_;
 
  
  robot_srvs::MoveToPose::Request req_pose_;
  robot_srvs::MoveToPose::Response res_pose_;
  
  robot_mechanism_controllers::JointControllerState controller_state_msg_;
  
  double last_grasp_value_;
  int grasp_count_;
  std_msgs::Float64 gripper_cmd_;
   
 
};

}
#endif
