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


#include <plugs_core/action_stow_plug.h>


using namespace plugs_core;

StowPlugAction::StowPlugAction() :
  robot_actions::Action<robot_msgs::PlugStow, std_msgs::Empty>("stow_plug"),
  action_name_("stow_plug"),
  node_(ros::Node::instance()),
  gripper_controller_("r_gripper_position_controller"),  
  arm_controller_("r_arm_cartesian_trajectory_controller")
{

  node_->param(action_name_ + "/gripper_controller", gripper_controller_, gripper_controller_);
  node_->param(action_name_ + "/arm_controller", arm_controller_, arm_controller_);
  node_->advertise<std_msgs::Float64>(gripper_controller_ + "/set_command", 1);

  if(gripper_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, gripper controller param was not set.", action_name_.c_str());
      return;
    }

  if(arm_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, arm controller param was not set.", action_name_.c_str());
      return;
    }

};

StowPlugAction::~StowPlugAction()
{
};

robot_actions::ResultStatus StowPlugAction::execute(const robot_msgs::PlugStow& plug_stow, std_msgs::Empty& feedback)
{
  plug_stow_ = plug_stow;

  reset();
  
  moveToStow();
  releasePlug();
  return waitForDeactivation(feedback);
}

void StowPlugAction::reset()
{
  
  last_grasp_value_ = 0.0;
  grasp_count_ = 0;
  gripper_cmd_.data = 0.04;

  req_pose_.pose.header.frame_id = plug_stow_.header.frame_id; 
  req_pose_.pose.pose.position.x = plug_stow_.plug_centroid.x;
  req_pose_.pose.pose.position.y = plug_stow_.plug_centroid.y;
  req_pose_.pose.pose.position.z = plug_stow_.plug_centroid.z + 0.15;

  req_pose_.pose.pose.orientation.x = -0.19;
  req_pose_.pose.pose.orientation.y = 0.13;
  req_pose_.pose.pose.orientation.z = 0.68;
  req_pose_.pose.pose.orientation.w = 0.68;


}

void StowPlugAction::moveToStow()
{ 
  req_pose_.pose.header.stamp = ros::Time();
  if (!ros::service::call(arm_controller_ + "/move_to", req_pose_, res_pose_))
  {
    ROS_ERROR("%s: Failed to move arm.", action_name_.c_str());
    deactivate(robot_actions::ABORTED, empty_);
    return;
  }
  
  if (isPreemptRequested())
    return;

  req_pose_.pose.pose.position.z = plug_stow_.plug_centroid.z - 0.1;
  req_pose_.pose.header.stamp = ros::Time();
  if (!ros::service::call(arm_controller_ + "/move_to", req_pose_, res_pose_))
  {
    ROS_ERROR("%s: Failed to move arm.", action_name_.c_str());
    deactivate(robot_actions::ABORTED, empty_);
  }

  node_->publish(gripper_controller_ + "/set_command", gripper_cmd_);
  return;
}


void StowPlugAction::releasePlug()
{
  
  if (isPreemptRequested())
    return;

  node_->publish(gripper_controller_ + "/set_command", gripper_cmd_);

  // DO U REALLY WANT TO SUBSCRIBE EACH TIME? WHAT HAPPENS IF U DO NOT?
  node_->subscribe(gripper_controller_ + "/state", controller_state_msg_, &StowPlugAction::checkGrasp, this, 1); 
}

void StowPlugAction::checkGrasp()
{
  if (!isActive())
    return;

  if (isPreemptRequested()){
    deactivate(robot_actions::PREEMPTED, std_msgs::Empty());
    return;
  }
  
  node_->publish(gripper_controller_ + "/set_command", gripper_cmd_);
  
  // Make sure that the gripper has stopped moving
  if(last_grasp_value_ < controller_state_msg_.process_value)
  {
    last_grasp_value_ = controller_state_msg_.process_value;
  }
  else
  {
    grasp_count_++;
  }

  // The gripper is closed and stopped moving
  if(grasp_count_ > 20)
  {
    // Something went wrong... can't open the gripper
    if(controller_state_msg_.error > 0.005)
    {
      ROS_INFO("%s: Error, failed to release plug.", action_name_.c_str());
      node_->unsubscribe(gripper_controller_ + "/state");
      deactivate(robot_actions::ABORTED, std_msgs::Empty());
      return;
    }
    else
    {
      ROS_INFO("%s: succeeded.", action_name_.c_str());
      node_->unsubscribe(gripper_controller_ + "/state");
      deactivate(robot_actions::SUCCESS, std_msgs::Empty());
      return;
    }
  }

  return;
}

