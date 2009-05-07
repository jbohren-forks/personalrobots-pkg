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

#include "doors_core/action_release_handle.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;

static const string fixed_frame = "odom_combined";




ReleaseHandleAction::ReleaseHandleAction(Node& node) : 
  robot_actions::Action<door_msgs::Door, door_msgs::Door>("release_handle"), 
  node_(node),
  tf_(node)
{
  node_.advertise<std_msgs::Float64>("r_gripper_effort_controller/command",10);
};


ReleaseHandleAction::~ReleaseHandleAction()
{};



robot_actions::ResultStatus ReleaseHandleAction::execute(const door_msgs::Door& goal, door_msgs::Door& feedback)
{
  ROS_INFO("ReleaseHandleAction: execute");

  // subscribe to the robot pose state message
  node_.subscribe("r_arm_cartesian_pose_controller/state/pose", pose_msg_,  &ReleaseHandleAction::poseCallback, this, 1);
  pose_received_ = false;
 
  // open the gripper during 4 seconds
  ROS_INFO("ReleaseHandleAction: open the gripper");
  std_msgs::Float64 gripper_msg;
  gripper_msg.data = 20.0;
  for (unsigned int i=0; i<100; i++){
    Duration().fromSec(4.0/100.0).sleep();
    node_.publish("r_gripper_effort_controller/command", gripper_msg);
    if (isPreemptRequested()) {
      gripper_msg.data = 0.0;
      node_.publish("r_gripper_effort_controller/command", gripper_msg);
      ROS_ERROR("ReleaseHandleAction: preempted");
      node_.unsubscribe("r_arm_cartesian_pose_controller/state/pose");
      return robot_actions::PREEMPTED;
    }
  }

  // receive robot pose message
  ROS_INFO("ReleaseHandleAction: get current robot pose");
  Duration timeout = Duration().fromSec(2.0);
  Duration poll = Duration().fromSec(0.1);
  Time start_time = ros::Time::now();
  while (!pose_received_){
    if (start_time + timeout < ros::Time::now()){
      ROS_ERROR("ReleaseHandleAction: failed to receive robot pose");
      node_.unsubscribe("r_arm_cartesian_pose_controller/state/pose");
      return robot_actions::ABORTED;
    }
    poll.sleep();
  }
  node_.unsubscribe("r_arm_cartesian_pose_controller/state/pose");
  boost::mutex::scoped_lock lock(pose_mutex_);


  // move gripper away from the door
  Pose offset(Quaternion(0,0,0), Vector3(-0.2,0,0));
  Pose gripper_goal = gripper_pose_ * offset;
  PoseStampedTFToMsg(Stamped<Pose>(gripper_goal, Time::now(), gripper_pose_.frame_id_), req_moveto.pose);

  ROS_INFO("ReleaseHandleAction: move gripper away from door ");
  if (!ros::service::call("r_arm_cartesian_trajectory_controller/move_to", req_moveto, res_moveto)){
    if (isPreemptRequested()){
      ROS_ERROR("ReleaseHandleAction: preempted");
      return robot_actions::PREEMPTED;
    }
    else{
      ROS_ERROR("ReleaseHandleAction: move_to command failed");
      return robot_actions::ABORTED;
    }
  }


  return robot_actions::SUCCESS;
}


void ReleaseHandleAction::poseCallback()
{
  boost::mutex::scoped_lock lock(pose_mutex_);
  PoseStampedMsgToTF(pose_msg_, gripper_pose_);
  pose_received_ = true;
}

