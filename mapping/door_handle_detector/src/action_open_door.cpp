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


#include <door_handle_detector/action_open_door.h>


using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;

static const string fixed_frame = "odom_combined";



OpenDoorAction::OpenDoorAction() : 
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("open_door"), 
  node_(ros::Node::instance()),    
  request_preempt_(false)
{};



OpenDoorAction::~OpenDoorAction()
{};



void OpenDoorAction::handleActivate(const robot_msgs::Door& door)
{
  notifyActivated();
  
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
  
  // open door
  tff_door_.mode.vel.x = tff_door_.VELOCITY;
  tff_door_.mode.vel.y = tff_door_.FORCE;
  tff_door_.mode.vel.z = tff_door_.FORCE;
  tff_door_.mode.rot.x = tff_door_.FORCE;
  tff_door_.mode.rot.y = tff_door_.FORCE;
  tff_door_.mode.rot.z = tff_door_.POSITION;
  
  tff_door_.value.vel.x = 0.25;
  tff_door_.value.vel.y = 0.0;
  tff_door_.value.vel.z = 0.0;
  tff_door_.value.rot.x = 0.0;
  tff_door_.value.rot.y = 0.0;
  tff_door_.value.rot.z = 0.0;
  
  
  
  // turn handle
  for (unsigned int i=0; i<100; i++){
    tff_handle_.value.rot.x += -1.5/100.0;
    node_->publish("cartesian_tff_right/command", tff_handle_);
    Duration().fromSec(4.0/100.0).sleep();
    if (request_preempt_) {
      node_->publish("cartesian_tff_right/command", tff_stop_);
      notifyPreempted(door); 
      return;
    }
  }
  
  // open door
  node_->publish("cartesian_tff_right/command", tff_door_);
  for (unsigned int i=0; i<500; i++){
    Duration().fromSec(10.0/500.0).sleep();
    if (request_preempt_) {
      node_->publish("cartesian_tff_right/command", tff_stop_);
      notifyPreempted(door); 
      return;
    }
  }
  
  // finish
  node_->publish("cartesian_tff_right/command", tff_stop_);
  
  notifySucceeded(door);
}




void OpenDoorAction::handlePreempt()
{
  request_preempt_ = true;
};
