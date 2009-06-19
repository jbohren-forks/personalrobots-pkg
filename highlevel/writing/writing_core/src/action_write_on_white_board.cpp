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


#include <writing_core/action_write_on_white_board.h>


namespace writing_core
{

WriteOnWhiteBoardAction::WriteOnWhiteBoardAction() :
  robot_actions::Action<robot_msgs::Path, std_msgs::Empty>("write_on_white_board"),
  action_name_("write_on_white_board"),
  node_(ros::Node::instance()),
  arm_controller_("r_arm_cartesian_pose_controller")
{

  node_->param(action_name_ + "/arm_controller", arm_controller_, arm_controller_);

  if(arm_controller_ == "" )
    {
      ROS_ERROR("%s: Aborted, arm controller param was not set.", action_name_.c_str());
      terminate();
      return;
    }

};

WriteOnWhiteBoardAction::~WriteOnWhiteBoardAction()
{
};

robot_actions::ResultStatus WriteOnWhiteBoardAction::execute(const robot_msgs::Path& text_trajectory, std_msgs::Empty& feedback)
{
  ROS_DEBUG("%s: executing.", action_name_.c_str());

  text_trajectory_ = text_trajectory;

  //node_->subscribe(arm_controller_+ "/state/error", pose_error_msg_, &WriteOnWhiteBoardAction::writeText, this, 1);
 
  return waitForDeactivation(feedback);
}


void WriteOnWhiteBoardAction::writeText()
{
  if (!isActive())
  {
    //node_->unsubscribe(arm_controller_+ "/state/error");
    return;
  }
  
  if (isPreemptRequested())
  {
    ROS_DEBUG("%s: preempted.", action_name_.c_str());
    //node_->unsubscribe(arm_controller_+ "/state/error");
    deactivate(robot_actions::PREEMPTED, empty_);
    return;
  }

   

  return;
}

}

