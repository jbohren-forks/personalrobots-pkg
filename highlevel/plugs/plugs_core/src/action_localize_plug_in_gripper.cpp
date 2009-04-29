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


#include <plugs_core/action_localize_plug_in_gripper.h>


namespace plugs_core
{

LocalizePlugInGripperAction::LocalizePlugInGripperAction(ros::Node& node) :
  robot_actions::Action<std_msgs::Empty, std_msgs::Empty>("localize_plug_in_gripper"),
  action_name_("localize_plug_in_gripper"),
  node_(node),
  arm_controller_("r_arm_cartesian_trajectory_controller"),
  servoing_controller_("r_arm_hybrid_controller"),
  TF(*ros::Node::instance(),false, ros::Duration(10))
{
  node_.setParam("~roi_policy", "LastImageLocation");
  node_.setParam("~display", 0);
  node_.setParam("~resize_factor_found", 3.0);
  node_.setParam("~resize_factor_failed", 1.2);
  node_.setParam("~display", "true");
  node_.setParam("~square_size", 0.0042);
  node_.setParam("~board_width", 3);
  node_.setParam("~board_height",4);

  node_.param(action_name_ + "/arm_controller", arm_controller_, arm_controller_);

  if(arm_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, arm controller param was not set.", action_name_.c_str());
      terminate();
      return;
    }

  node_.param(action_name_ + "/servoing_controller", servoing_controller_, servoing_controller_);

  if(servoing_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, servoing controller param was not set.", action_name_.c_str());
      terminate();
      return;
    }

  detector_ = new PlugTracker::PlugTracker(node);
  detector_->deactivate();
  node_.subscribe("~plug_pose", plug_pose_msg_, &LocalizePlugInGripperAction::setToolFrame, this, 1);
};

LocalizePlugInGripperAction::~LocalizePlugInGripperAction()
{
  if(detector_) delete detector_;
};

  robot_actions::ResultStatus LocalizePlugInGripperAction::execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback)
{
  //  outlet_pose_ = outlet_pose;

  reset();

  moveToStage();

  return waitForDeactivation(feedback);
}

void LocalizePlugInGripperAction::reset()
{

  req_pose_.pose.header.frame_id = "base_link";
  req_pose_.pose.pose.position.x = 0.33;
  req_pose_.pose.pose.position.y = -0.09;
  req_pose_.pose.pose.position.z = 0.37;
  req_pose_.pose.pose.orientation.x = -0.04;
  req_pose_.pose.pose.orientation.y = 0.26;
  req_pose_.pose.pose.orientation.z = 0.00;
  req_pose_.pose.pose.orientation.w = 0.96;

}

void LocalizePlugInGripperAction::moveToStage()
{
  req_pose_.pose.header.stamp = ros::Time();
  if (!ros::service::call(arm_controller_ + "/move_to", req_pose_, res_pose_))
  {
    ROS_ERROR("%s: Failed to move arm.", action_name_.c_str());
    deactivate(robot_actions::ABORTED, empty_);
    return;
  }

  if (isPreemptRequested())
  {
    deactivate(robot_actions::PREEMPTED, std_msgs::Empty());
    return;
  }

  req_pose_.pose.header.frame_id = "outlet_pose";
  req_pose_.pose.pose.position.x = -0.12;
  req_pose_.pose.pose.position.y = 0.00;
  req_pose_.pose.pose.position.z = 0.08;
  req_pose_.pose.pose.orientation.x = 0.00;
  req_pose_.pose.pose.orientation.y = 0.14;
  req_pose_.pose.pose.orientation.z = 0.00;
  req_pose_.pose.pose.orientation.w = 0.98;
  req_pose_.pose.header.stamp = ros::Time();
  if (!ros::service::call(arm_controller_ + "/move_to", req_pose_, res_pose_))
  {
    ROS_ERROR("%s: Failed to move arm.", action_name_.c_str());
    deactivate(robot_actions::ABORTED, empty_);
    return;
  }

  if (isPreemptRequested())
  {
    deactivate(robot_actions::PREEMPTED, std_msgs::Empty());
    return;
  }

  req_pose_.pose.pose.position.x = -0.05;
  req_pose_.pose.header.stamp = ros::Time();
  if (!ros::service::call(arm_controller_ + "/move_to", req_pose_, res_pose_))
  {
    ROS_ERROR("%s: Failed to move arm.", action_name_.c_str());
    deactivate(robot_actions::ABORTED, empty_);
    return;
  }

  detector_->activate();
  return;
}



void LocalizePlugInGripperAction::setToolFrame()
{
  if (!isActive())
    return;

  if (isPreemptRequested())
  {
    detector_->deactivate();
    deactivate(robot_actions::PREEMPTED, std_msgs::Empty());
    return;
  }
  robot_srvs::SetPoseStamped::Request req_tool;
  req_tool.p = plug_pose_msg_;
  robot_srvs::SetPoseStamped::Response res_tool;
  if (!ros::service::call(servoing_controller_ + "/set_tool_frame", req_tool, res_tool))
  {
    ROS_ERROR("%s: Failed to set tool frame.", action_name_.c_str());
    deactivate(robot_actions::ABORTED, empty_);
    return;
  }


  detector_->deactivate();
  ROS_INFO("%s: succeeded.", action_name_.c_str());
  deactivate(robot_actions::SUCCESS, empty_);
}
}
