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

#ifndef ACTION_UNTUCK_ARMS_H
#define ACTION_UNTUCK_ARMS_H


#include <ros/node.h>

#include <std_msgs/Empty.h>
#include <robot_actions/action.h>

#include <robot_msgs/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>

namespace plugs_core{

class UntuckArmsAction: public robot_actions::Action<std_msgs::Empty, std_msgs::Empty>
{
public:
  UntuckArmsAction();
  ~UntuckArmsAction();

  virtual robot_actions::ResultStatus execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback);

private:
  // average the last couple plug centroids
  bool isTrajectoryDone();
  void cancelTrajectory();
  
  std::string action_name_;
  
  ros::Node* node_;

  pr2_mechanism_controllers::TrajectoryStart::Request right_traj_req_;
  pr2_mechanism_controllers::TrajectoryStart::Request left_traj_req_;
  pr2_mechanism_controllers::TrajectoryStart::Response traj_res_;  

  std_msgs::Empty empty_;

  bool traj_error_;

  std::string which_arms_;
  std::string right_arm_controller_;
  std::string left_arm_controller_;
  std::string current_controller_name_;
  
  
  int traj_id_; 
 
};

}
#endif
