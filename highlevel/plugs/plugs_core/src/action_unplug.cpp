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

 node_->advertise<robot_msgs::TaskFrameFormalism>(arm_controller_ + "/command", 2);

};

UnplugAction::~UnplugAction()
{
};

robot_actions::ResultStatus UnplugAction::execute(const std_msgs::Empty& empty, std_msgs::Empty& feedback)
{
  ROS_DEBUG("%s: executing.", action_name_.c_str());
  tff_msg_.header.frame_id = "outlet_pose";
  tff_msg_.header.stamp = ros::Time::now();
  tff_msg_.mode.vel.x = 3;
  tff_msg_.mode.vel.y = 3;
  tff_msg_.mode.vel.z = 3;
  tff_msg_.mode.rot.x = 2;
  tff_msg_.mode.rot.y = 2;
  tff_msg_.mode.rot.z = 2;
  tff_msg_.value.vel.x = -0.1;  // backs off 10cm
  tff_msg_.value.vel.y = 0.0;
  tff_msg_.value.vel.z = 0.0;
  tff_msg_.value.rot.x = 0.0;
  tff_msg_.value.rot.y = 0.0;
  tff_msg_.value.rot.z = 0.0;

  node_->publish(arm_controller_ + "/command", tff_msg_);
  node_->publish(arm_controller_ + "/command", tff_msg_);

  ROS_DEBUG("%s: succeeded.", action_name_.c_str());
  deactivate(robot_actions::SUCCESS, feedback);
  return waitForDeactivation(feedback);
}

}
