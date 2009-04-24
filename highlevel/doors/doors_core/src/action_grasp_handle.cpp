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

/* Author: Wim Meeussen */

#include "doors_core/action_grasp_handle.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;

static const string fixed_frame = "odom_combined";




GraspHandleAction::GraspHandleAction(Node& node) : 
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("grasp_handle"), 
  node_(node),
  tf_(node)
{
  node_.advertise<std_msgs::Float64>("r_gripper_effort_controller/command",10);
};


GraspHandleAction::~GraspHandleAction()
{};



robot_actions::ResultStatus GraspHandleAction::execute(const robot_msgs::Door& goal, robot_msgs::Door& feedback)
{
  ROS_INFO("GraspHandleAction: execute");
 
  // door needs to be in time fixed frame
  if (goal.header.frame_id != fixed_frame){
    ROS_ERROR("GraspHandleAction: goal should be in '%s' frame", fixed_frame.c_str());
    return robot_actions::ABORTED;
  }

  Vector normal(goal.normal.x, goal.normal.y, goal.normal.z);
  Vector handle(goal.handle.x, goal.handle.y, goal.handle.z);
  Stamped<Pose> gripper_pose;
  gripper_pose.frame_id_ = fixed_frame;
  
  // open the gripper while moving in front of the door
  if (isPreemptRequested()) {
    ROS_ERROR("GraspHandleAction: preempted");
    return robot_actions::PREEMPTED;
  }

  std_msgs::Float64 gripper_msg;
  gripper_msg.data = 2.0;
  node_.publish("r_gripper_effort_controller/command", gripper_msg);
  
  // move gripper in front of door
  gripper_pose.setOrigin( Vector3(handle(0) + (normal(0) * -0.15), handle(1) + (normal(1) * -0.15), handle(2) + (normal(2) * -0.15)));
  gripper_pose.setRotation( Quaternion(getDoorAngle(goal), 0, M_PI/2.0) ); 
  gripper_pose.stamp_ = Time::now();
  PoseStampedTFToMsg(gripper_pose, req_moveto.pose);

  ROS_INFO("GraspHandleAction: move in front of handle");
  if (!ros::service::call("r_arm_cartesian_trajectory_controller/move_to", req_moveto, res_moveto)){
    if (isPreemptRequested()){
      ROS_ERROR("GraspHandleAction: preempted");
      return robot_actions::PREEMPTED;
    }
    else{
      ROS_ERROR("GraspHandleAction: move_to command failed");
      return robot_actions::ABORTED;
    }
  }


  // move gripper over door handle
  gripper_pose.frame_id_ = fixed_frame;
  gripper_pose.setOrigin( Vector3(handle(0) + (normal(0) * 0.05 ), handle(1) + (normal(1) * 0.05),  handle(2) + (normal(2) * 0.05)));
  gripper_pose.setRotation( Quaternion(getDoorAngle(goal), 0, M_PI/2.0) ); 
  gripper_pose.stamp_ = Time::now();
  PoseStampedTFToMsg(gripper_pose, req_moveto.pose);

  ROS_INFO("GraspHandleAction: move over handle");
  if (!ros::service::call("r_arm_cartesian_trajectory_controller/move_to", req_moveto, res_moveto)){
    if (isPreemptRequested()){
      ROS_ERROR("GraspHandleAction: preempted");
      return robot_actions::PREEMPTED;
    }
    else{
      ROS_ERROR("GraspHandleAction: move_to command failed");
      return robot_actions::ABORTED;
    }
  }
  
  // close the gripper during 4 seconds
  gripper_msg.data = -2.0;
  node_.publish("r_gripper_effort_controller/command", gripper_msg);
  for (unsigned int i=0; i<100; i++){
    Duration().fromSec(4.0/100.0).sleep();
    if (isPreemptRequested()) {
      gripper_msg.data = 0.0;
      node_.publish("r_gripper_effort_controller/command", gripper_msg);
      ROS_ERROR("GraspHandleAction: preempted");
      return robot_actions::PREEMPTED;
    }
  }
  
  feedback = goal;
  return robot_actions::SUCCESS;
}


double GraspHandleAction::getDoorAngle(const robot_msgs::Door& door)
{
  Vector normal(door.normal.x, door.normal.y, door.normal.z);
  Vector x_axis(1,0,0);
  double dot      = normal(0) * x_axis(0) + normal(1) * x_axis(1);
  double perp_dot = normal(1) * x_axis(0) - normal(0) * x_axis(1);
  return atan2(perp_dot, dot);
}


