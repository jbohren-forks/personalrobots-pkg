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
#include <algorithm>

#include <generic_controllers/joint_velocity_controller.h>
#include <math_utils/angles.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointVelocityController)

JointVelocityController::JointVelocityController()
{
  // Initialize PID class
  pid_controller_.initPid(0, 0, 0, 0, 0);
  command_ = 0;
}

JointVelocityController::~JointVelocityController()
{
}

void JointVelocityController::init(double p_gain, double i_gain, double d_gain, double windup, double time, mechanism::Joint *joint)
{
  pid_controller_.initPid(p_gain, i_gain, d_gain, windup, -windup);

  command_= 0;
  last_time_= time;
  joint_ = joint;
}

void JointVelocityController::initXml(mechanism::Robot *robot, TiXmlElement *config)
{
  robot_ = robot;
  TiXmlElement *elt = config->FirstChildElement("joint");
  if (elt) {
    // TODO: error check if xml attributes/elements are missing
    double p_gain = atof(elt->FirstChildElement("pGain")->GetText());
    double i_gain = atof(elt->FirstChildElement("iGain")->GetText());
    double d_gain = atof(elt->FirstChildElement("dGain")->GetText());
    double windup= atof(elt->FirstChildElement("windup")->GetText());
    init(p_gain, i_gain, d_gain, windup, robot->hw_->current_time_, robot->getJoint(elt->Attribute("name")));
  }
}

// Set the joint position command
void JointVelocityController::setCommand(double command)
{
  command_ = command;
}

// Return the current position command
double JointVelocityController::getCommand()
{
  return command_;
}

// Return the measured joint position
double JointVelocityController::getActual()
{
  return joint_->position_;
}

void JointVelocityController::update()
{
  double error(0);
  double torque_cmd(0);
  double time = robot_->hw_->current_time_;

  error = joint_->velocity_ - command_;

  torque_cmd = pid_controller_.updatePid(error, time - last_time_);

  setJointEffort(torque_cmd);
}

void JointVelocityController::setJointEffort(double effort)
{
  joint_->commanded_effort_ = min(max(effort, -joint_->effort_limit_), joint_->effort_limit_);
}




