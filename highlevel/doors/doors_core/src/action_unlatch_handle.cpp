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

#include "doors_core/action_unlatch_handle.h"


using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;

static const string fixed_frame = "odom_combined";


UnlatchHandleAction::UnlatchHandleAction(Node& node) :
  robot_actions::Action<door_msgs::Door, door_msgs::Door>("unlatch_handle"),
  node_(node)
{
  node_.advertise<robot_msgs::TaskFrameFormalism>("r_arm_cartesian_tff_controller/command", 10);
};



UnlatchHandleAction::~UnlatchHandleAction()
{
  node_.unadvertise("r_arm_cartesian_tff_controller/command");
};



robot_actions::ResultStatus UnlatchHandleAction::execute(const door_msgs::Door& goal, door_msgs::Door& feedback)
{ 
  ROS_INFO("UnlatchHandleAction: execute");

  // default feedback
  feedback = goal;

  // stop
  tff_stop_.mode.vel.x = tff_stop_.FORCE;
  tff_stop_.mode.vel.y = tff_stop_.FORCE;
  tff_stop_.mode.vel.z = tff_stop_.FORCE;
  tff_stop_.mode.rot.x = tff_stop_.FORCE;
  tff_stop_.mode.rot.y = tff_stop_.FORCE;
  tff_stop_.mode.rot.z = tff_stop_.FORCE;
  
  tff_stop_.value.vel.x = 0.0;
  tff_stop_.value.vel.y = 0.0;
  tff_stop_.value.vel.z = 0.0;
  tff_stop_.value.rot.x = 0.0;
  tff_stop_.value.rot.y = 0.0;
  tff_stop_.value.rot.z = 0.0;
  
  // turn handle
  tff_handle_.mode.vel.x = tff_handle_.FORCE;
  tff_handle_.mode.vel.y = tff_handle_.FORCE;
  tff_handle_.mode.vel.z = tff_handle_.FORCE;
  tff_handle_.mode.rot.x = tff_handle_.FORCE;
  tff_handle_.mode.rot.y = tff_handle_.FORCE;
  tff_handle_.mode.rot.z = tff_handle_.POSITION;
  
  tff_handle_.value.vel.x = 20.0;
  tff_handle_.value.vel.y = 0.0;
  tff_handle_.value.vel.z = 0.0;
  tff_handle_.value.rot.x = 0.0;
  tff_handle_.value.rot.y = 0.0;
  tff_handle_.value.rot.z = 0.0;
  
  // start monitoring tf position
  node_.subscribe("r_arm_cartesian_tff_controller/state/position", tff_msg_,  &UnlatchHandleAction::tffCallback, this, 1);
  Duration timeout = Duration().fromSec(3.0);
  Duration poll = Duration().fromSec(0.1);
  Time start_time = ros::Time::now();
  tff_state_received_ = false;
  while (!tff_state_received_){
    if (start_time + timeout < ros::Time::now()){
      ROS_ERROR("UnlatchHandleAction: failed to receive tff state");
      node_.unsubscribe("r_arm_cartesian_tff_controller/state/position");
      return robot_actions::ABORTED;
    }
    poll.sleep();
  }

  double sleep_time = 0.1;
  ros::Duration wait_after_door_moved = Duration().fromSec(1.0);
  ros::Time time_door_moved;

  // turn handle and push door, until door moves forward
  while (time_door_moved == ros::Time() || ros::Time::now() < time_door_moved + wait_after_door_moved){
    Duration(sleep_time).sleep();
    boost::mutex::scoped_lock lock(tff_mutex_);

    // increase torque to turn handle until door is open
    if (fabs(tff_state_.vel.x) < 0.05)
      tff_handle_.value.rot.x += -0.5 * sleep_time; // add 0.5 Nm per second
    else{
      tff_handle_.value.rot.x = 0;
      if (time_door_moved == Time())
	time_door_moved = ros::Time::now();
    }
    node_.publish("r_arm_cartesian_tff_controller/command", tff_handle_);

    // detect when gripper is not on hanlde
    if (fabs(tff_state_.rot.x) > M_PI/2.0 || fabs(tff_state_.vel.y) > 0.1 || fabs(tff_state_.vel.z) > 0.1 || 
	fabs(tff_state_.rot.y) > M_PI/8.0 || fabs(tff_state_.rot.z) > M_PI/8.0){
      node_.unsubscribe("r_arm_cartesian_tff_controller/state/position");
      node_.publish("r_arm_cartesian_tff_controller/command", tff_stop_);
      ROS_ERROR("UnlatchHandleAction: Gripper was not on door handle");
      return robot_actions::ABORTED;
    }

    // detect when door is locked
    if (fabs(tff_handle_.value.rot.x) > 3.5 && fabs(tff_state_.rot.x < M_PI/6.0)){
      node_.unsubscribe("r_arm_cartesian_tff_controller/state/position");
      node_.publish("r_arm_cartesian_tff_controller/command", tff_stop_);
      ROS_INFO("UnlatchHandleAction: Door is locked");
      feedback.latch_state = door_msgs::Door::LOCKED;
      return robot_actions::SUCCESS;
    }

    // check if preempted
    if (isPreemptRequested()) {
      node_.publish("r_arm_cartesian_tff_controller/command", tff_stop_);
      ROS_ERROR("UnlatchHandleAction: preempted");
      return robot_actions::PREEMPTED;
    }
  }
  
  // finished
  node_.unsubscribe("r_arm_cartesian_tff_controller/state/position");
  feedback.latch_state = door_msgs::Door::UNLATCHED;
  return robot_actions::SUCCESS;
}


void UnlatchHandleAction::tffCallback()
{
  boost::mutex::scoped_lock lock(tff_mutex_);
  tff_state_ = tff_msg_;
  tff_state_received_ = true;
}

