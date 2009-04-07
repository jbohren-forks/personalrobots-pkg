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


#include <ros/node.h>
#include <robot_msgs/PlugStow.h>
#include <pr2_mechanism_controllers/SetPeriodicCmd.h>
#include <std_msgs/Empty.h>
#include <robot_actions/action.h>
#include <plug_onbase_detector/plug_onbase_detector.h>

namespace plugs_core{

class MoveAndGraspPlugAction: public robot_actions::Action<robot_msgs::PlugStow, std_msgs::Empty>
{
public:
  MoveAndGraspPlugAction(ros::Node& node);
  ~MoveAndGraspPlugAction();

  virtual void handleActivate(const std_msgs::Empty& empty);
  virtual void handlePreempt();



private:
  // average the last couple plug centroids
  void localizePlug();
  void reset();


  ros::Node& node_;

  

  robot_mechanism_controllers::JointControllerState controller_state_msg_;
  
  bool request_preempt_;
  
  std::string gripper_controller_;
  std::string arm_controller_;
  
  double last_grasp_value_;
  int grasp_count_;
   
 
};

}
#endif
