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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <experimental_controllers/pid_position_velocity_controller.h>

using namespace std;

namespace controller 
{
  ROS_REGISTER_CONTROLLER(PIDPositionVelocityController)

  PIDPositionVelocityController::PIDPositionVelocityController():
    joint_state_(NULL), 
    robot_(NULL), 
    last_time_(0), 
    loop_count_(0), 
    command_(0),
    command_dot_(0)
  {
  }

  PIDPositionVelocityController::~PIDPositionVelocityController()
  {
    if ((void*)sub_command_)
      sub_command_.shutdown();
  }
 
  bool PIDPositionVelocityController::init(mechanism::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
  {
    assert(robot);
    robot_ = robot;
    last_time_ = robot->hw_->current_time_;

    joint_state_ = robot_->getJointState(joint_name);
    if (!joint_state_)
    {
      fprintf(stderr, "PIDPositionVelocityController could not find joint named \"%s\"\n",
              joint_name.c_str());
      return false;
    }

    pid_controller_ = pid;

    return true;
  }

  bool PIDPositionVelocityController::init(mechanism::RobotState *robot, const ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;
    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    control_toolbox::Pid pid;
    if (!pid.init(ros::NodeHandle(node_)))
    {
      ROS_INFO("Could not initialize  pid");
      return false;
    }
    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<robot_mechanism_controllers::JointControllerState>
      (node_, "state", 1));

    sub_command_ = node_.subscribe<std_msgs::Float64>("set_command", 1, &PIDPositionVelocityController::setCommandCB, this);

    return init(robot, joint_name, pid);
  }

  void PIDPositionVelocityController::reset()
  {
    pid_controller_.reset();
  }

  void PIDPositionVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
  {
    pid_controller_.setGains(p,i,d,i_max,i_min);

  }

  void PIDPositionVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_.getGains(p,i,d,i_max,i_min);
  }

  std::string PIDPositionVelocityController::getJointName()
  {
    return joint_state_->joint_->name_;
  }

  // Set the joint velocity command
  void PIDPositionVelocityController::setCommand(double cmd, double cmd_dot)
  {
    command_ = cmd;
    command_dot_ = cmd_dot;
  }

  // Return the current velocity command
  void PIDPositionVelocityController::getCommand(double  & cmd, double & cmd_dot)
  {
    cmd = command_;
    cmd_dot = command_dot_;
  }

  void PIDPositionVelocityController::update()
  {
    assert(robot_ != NULL);
    double error(0),error_dot(0);
    double time = robot_->hw_->current_time_;

    error_dot = joint_state_->velocity_ - command_dot_;

    if(joint_state_->joint_->type_ == mechanism::JOINT_ROTARY)
    {
      angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->joint_limit_min_, joint_state_->joint_->joint_limit_max_,error);
    }
    else if(joint_state_->joint_->type_ == mechanism::JOINT_CONTINUOUS)
    {
      error = angles::shortest_angular_distance(command_, joint_state_->position_);
    }
    else //prismatic
    {
      error = joint_state_->position_ - command_;
    }

    dt_ = time - last_time_;
    joint_state_->commanded_effort_ = pid_controller_.updatePid(error, error_dot, dt_);

    if(loop_count_ % 10 == 0)
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.set_point = command_;
        controller_state_publisher_->msg_.process_value = joint_state_->velocity_;
        controller_state_publisher_->msg_.error = error;
        controller_state_publisher_->msg_.time_step = dt_;

        double dummy;
        getGains(controller_state_publisher_->msg_.p,
                 controller_state_publisher_->msg_.i,
                 controller_state_publisher_->msg_.d,
                 controller_state_publisher_->msg_.i_clamp,
                 dummy);
        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;

    last_time_ = time;
  }

  void PIDPositionVelocityController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    command_ = msg->data;
  }
}
