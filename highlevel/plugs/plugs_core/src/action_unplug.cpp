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
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING INeco
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <plugs_core/action_unplug.h>
#include <std_msgs/Float64.h>
#include "mechanism_msgs/SwitchController.h"
#include <ros/ros.h> //For the NodeHandle API 

#define BACKOFF 0.06

namespace plugs_core {

UnplugAction::UnplugAction() :
  robot_actions::Action<std_msgs::Empty, std_msgs::Empty>("unplug"),
  action_name_("unplug"),
  node_(ros::Node::instance()),
  arm_controller_("r_arm_hybrid_controller")
{

  node_->param(action_name_ + "/arm_controller", arm_controller_, arm_controller_);

  if(arm_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, arm controller param was not set.", action_name_.c_str());
      terminate();
      return;
    }

 node_->advertise<manipulation_msgs::TaskFrameFormalism>(arm_controller_ + "/command", 2);
 node_->advertise<std_msgs::Float64>("/r_shoulder_pan_effort/command", 2);

 node_->subscribe(arm_controller_ + "/state", controller_state_msg_, &UnplugAction::checkUnplug, this, 1);


};

UnplugAction::~UnplugAction()
{
 node_->unadvertise(arm_controller_ + "/command");
 node_->unadvertise("/r_shoulder_pan_effort/command");
};

robot_actions::ResultStatus UnplugAction::execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback)
{

  double unplug_x_threshold, unplug_x_target;
  if (!node_->getParam("~x_threshold", unplug_x_threshold, true)) {
    ROS_WARN("unplug x-threshold wasn't set (by plug_in)");
    unplug_x_target = -BACKOFF;
  }
  else{
    unplug_x_target = unplug_x_threshold - 0.02;
  }

  first_state_.header.seq = 0;
  ROS_DEBUG("%s: executing.", action_name_.c_str());
  tff_msg_.header.frame_id = "outlet_pose";
  tff_msg_.header.stamp = ros::Time::now();
  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 2;
  tff_msg_.mode.rot.y = 2;
  tff_msg_.mode.rot.z = 2;
  tff_msg_.value.vel.x = unplug_x_target;
  tff_msg_.value.vel.y = 0.0;
  tff_msg_.value.vel.z = 0.0;
  tff_msg_.value.rot.x = 0.0;
  tff_msg_.value.rot.y = 0.0;
  tff_msg_.value.rot.z = 0.0;

  //Start effort controller
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<mechanism_msgs::SwitchController>("switch_controller");
  mechanism_msgs::SwitchController srv;
  srv.request.start_controllers.push_back("/r_shoulder_pan_effort");
  if(client.call(srv)){
    ROS_INFO("Enabled r_shoulder_pan_effort controller (at least the service call succeeded)");
  }
  else{
    ROS_ERROR("Failed to enable r_shoulder_pan_effort_controller.  I'm continuing, but you might get stuck in a singularity");
  }

  node_->publish(arm_controller_ + "/command", tff_msg_);

  ros::Time started = ros::Time::now();
  while (isActive()) {
    ros::Duration(0.01).sleep();

    double effort = 0.0;
    if (ros::Time::now() - started > ros::Duration(5.0))
      effort = -2.0;
    if (ros::Time::now() - started > ros::Duration(15.0))
      effort = -5.0;
    std_msgs::Float64 msg;  msg.data = effort;
    node_->publish("/r_shoulder_pan_effort/command", msg);
  }
  std_msgs::Float64 msg; msg.data = 0;
  node_->publish("/r_shoulder_pan_effort/command", msg);

  //Stop effort controller
  srv.request.stop_controllers.push_back("/r_shoulder_pan_effort");
  srv.request.start_controllers.clear();
  if(client.call(srv)){
    ROS_INFO("Disabled r_shoulder_pan_effort controller (at least the service call succeeded)");
  }
  else{
    ROS_ERROR("Failed to disable r_shoulder_pan_effort_controller.  I'm continuing, but this could interfere with other controllers");
  }
 
  node_->deleteParam("~x_threshold");

  return waitForDeactivation(feedback);
}

void  UnplugAction::checkUnplug()
{
  if (!isActive())
    return;

  if (first_state_.header.seq == 0) {
    first_state_ = controller_state_msg_;
    return;
  }

  double unplug_x_threshold;
  if (!node_->getParam("~x_threshold", unplug_x_threshold, true)) {
    ROS_WARN("Unplug succeeded because the x-threshold wasn't set (by plug_in)");
    deactivate(robot_actions::SUCCESS, empty_);
    return;
  }

  tff_msg_.header.stamp = ros::Time::now();
  tff_msg_.header.frame_id = "outlet_pose";
  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 3;
  tff_msg_.mode.rot.y = 3;
  tff_msg_.mode.rot.z = 3;
  tff_msg_.value.vel.x = unplug_x_threshold - 0.02;
  tff_msg_.value.vel.y = first_state_.last_pose_meas.vel.y;
  tff_msg_.value.vel.z = first_state_.last_pose_meas.vel.z;
  double time = ros::Time::now().toSec();
  tff_msg_.value.rot.x = first_state_.last_pose_meas.rot.x + 0.06 * sin(time*37.0*M_PI);
  tff_msg_.value.rot.y = first_state_.last_pose_meas.rot.y + 0.15 * sin(time*2.0 *M_PI);
  tff_msg_.value.rot.z = first_state_.last_pose_meas.rot.z + 0.03 * sin(time*55.0 *M_PI);    
  node_->publish(arm_controller_ + "/command", tff_msg_);

  if (controller_state_msg_.last_pose_meas.vel.x  < unplug_x_threshold){
    ROS_DEBUG("%s: succeeded.", action_name_.c_str());
    deactivate(robot_actions::SUCCESS, empty_);
  }
  ROS_DEBUG("Unplug is %f from the threshold", controller_state_msg_.last_pose_meas.vel.x - unplug_x_threshold);

  return;
}

}
