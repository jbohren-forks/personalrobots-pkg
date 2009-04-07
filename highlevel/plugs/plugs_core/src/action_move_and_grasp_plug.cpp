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


#include <plugs_core/action_move_to_grasp_plug.h>
#include <math.h>

using namespace plugs_core;

MoveAndGraspPlugAction::MoveAndGraspPlugAction(ros::Node& node) :
  robot_actions::Action<std_msgs::Empty, robot_msgs::PlugStow>("move_to_grasp_plug"),
  node_(node),
  request_preempt_(false),
  gripper_controller_("r_gripper_position_controller"),  
  arm_controller_("right_arm/trajectory_controller")
{
};

MoveAndGraspPlugAction::~MoveAndGraspPlugAction()
{
};

void MoveAndGraspPlugAction::handleActivate(const std_msgs::Empty& empty)
{
  reset();
  
  // Get the controller names from the parameter server
  node_.param("~/gripper_controller", gripper_controller_, gripper_controller_);
  node_.param("~/arm_controller", arm_controller_, arm_controller_); 
  
  if(gripper_controller_ == "" )
  {
    ROS_ERROR("The gripper controller param was not set.");
    notifyAborted();
    return;
  }
  
  if(arm_controller_ == "" )
  {
    ROS_ERROR("The arm controller param was not set.");
    notifyAborted();
    return;
  }


  notifyActivated();

  return;
}

void MoveAndGraspPlugAction::handlePreempt()
{
  request_preempt_ = true;
  return;
}

void MoveAndGraspPlugAction::reset()
{
  
  last_grasp_value_ = 10.0;
  grasp_count_ = 0;

}

void MoveAndGraspPlugAction::graspPlug()
{
  if (request_preempt_)
  {
    notifyPreempted();
    return;
  }

  node_.publish(gripper_controller_ + "/set_command", 0.0)
  node_.subscribe(gripper_controller_ + "/state", controller_state_msg_, &MoveAndGraspPlugAction::checkGrasp, this, 1); 
 

  return;
}

void MoveAndGraspPlugAction::checkGrasp()
{
  if (request_preempt_)
  {
    notifyPreempted();
    return;
  }
  
  // Make sure that the gripper has stopped moving
  if(last_grasp_value > controller_state_msg.process_value)
  {
    last_grasp_value = controller_state_msg.process_value;
  }
  else
  {
    grasp_count_++;
  }

  // The gripper is closed and stopped moving
  if(grasp_count_ > 20)
  {
    // Something went wrong... no plug grasped in gripper
    if(controller_state_msg.error < 0.005)
    {
      node_.unsubscribe(gripper_controller_ + "/state");
      notifyAborted();
      return;
    }
    else
    {
      node_.unsubscribe(gripper_controller_ + "/state");
      notifySucceeded();
      return;
    }
  }

  return;
}

