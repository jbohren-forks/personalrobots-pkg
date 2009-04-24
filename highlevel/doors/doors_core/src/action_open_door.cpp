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

#include "doors_core/action_open_door.h"


using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;

static const string fixed_frame = "odom_combined";



// --------------
// UNLATCH HANDLE
// --------------
UnlatchHandleAction::UnlatchHandleAction(Node& node) :
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("unlatch_handle"),
  node_(node)
{
  node_.advertise<robot_msgs::TaskFrameFormalism>("r_arm_cartesian_tff_controller/command", 10);
};



UnlatchHandleAction::~UnlatchHandleAction()
{
  node_.unadvertise("r_arm_cartesian_tff_controller/command");
};



robot_actions::ResultStatus UnlatchHandleAction::execute(const robot_msgs::Door& goal, robot_msgs::Door& feedback)
{ 
  ROS_INFO("UnlatchHandleAction: execute");

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
  
  tff_handle_.value.vel.x = 10.0;
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

  double move_forward = 0;
  double sleep_time = 0.1;
  // turn handle and push door, until door moves forward
  while (move_forward < 0.05){
    // increase torque to turn handle
    tff_handle_.value.rot.x += -0.5 * sleep_time; // add 0.5 Nm per second
    node_.publish("r_arm_cartesian_tff_controller/command", tff_handle_);
    Duration(sleep_time).sleep();

    boost::mutex::scoped_lock lock(tff_mutex_);
    // detect when gripper is not on hanlde
    if (fabs(tff_state_.rot.x) > M_PI/2.0){
      node_.unsubscribe("r_arm_cartesian_tff_controller/state/position");
      node_.publish("r_arm_cartesian_tff_controller/command", tff_stop_);
      ROS_ERROR("UnlatchHandleAction: Gripper was not on door handle");
      return robot_actions::ABORTED;
    }

    // measure distance door moved forward
    move_forward = fabs(tff_state_.vel.x);

    // check if preempted
    if (isPreemptRequested()) {
      node_.publish("r_arm_cartesian_tff_controller/command", tff_stop_);
      ROS_ERROR("UnlatchHandleAction: preempted");
      return robot_actions::PREEMPTED;
    }

  }
  node_.unsubscribe("r_arm_cartesian_tff_controller/state/position");
  
  // keep pushing forward when finished
  tff_handle_.value.rot.x = 0;
  node_.publish("r_arm_cartesian_tff_controller/command", tff_handle_);
  feedback = goal;
  return robot_actions::SUCCESS;
}


void UnlatchHandleAction::tffCallback()
{
  boost::mutex::scoped_lock lock(tff_mutex_);
  tff_state_ = tff_msg_;
  tff_state_received_ = true;
}








// ---------
// OPEN DOOR
// ---------
OpenDoorAction::OpenDoorAction(Node& node) :
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("open_door"),
  node_(node)
{
  node_.advertise<robot_msgs::TaskFrameFormalism>("r_arm_cartesian_tff_controller/command", 10);
};



OpenDoorAction::~OpenDoorAction()
{
  node_.unadvertise("r_arm_cartesian_tff_controller/command");
};



robot_actions::ResultStatus OpenDoorAction::execute(const robot_msgs::Door& goal, robot_msgs::Door& feedback)
{ 
  ROS_INFO("OpenDoorAction: execute");

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
  
  // open door
  tff_door_.mode.vel.x = tff_door_.VELOCITY;
  tff_door_.mode.vel.y = tff_door_.FORCE;
  tff_door_.mode.vel.z = tff_door_.FORCE;
  tff_door_.mode.rot.x = tff_door_.FORCE;
  tff_door_.mode.rot.y = tff_door_.FORCE;
  tff_door_.mode.rot.z = tff_door_.POSITION;
  
  tff_door_.value.vel.x = 0.45;
  tff_door_.value.vel.y = 0.0;
  tff_door_.value.vel.z = 0.0;
  tff_door_.value.rot.x = 0.0;
  tff_door_.value.rot.y = 0.0;
  tff_door_.value.rot.z = 0.0;
  
  // open door
  while (!isPreemptRequested()){
    node_.publish("r_arm_cartesian_tff_controller/command", tff_door_);
    Duration(0.1).sleep();
  }

  // preempted
  node_.publish("r_arm_cartesian_tff_controller/command", tff_stop_);
  return robot_actions::PREEMPTED;
}

