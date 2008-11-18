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

#include <robot_mechanism_controllers/joint_position_controller.h>
#include <angles/angles.h>

using namespace std;
using namespace controller;


ROS_REGISTER_CONTROLLER(JointPositionController)

JointPositionController::JointPositionController()
: joint_state_(NULL), robot_(NULL), last_time_(0), command_(0)
{
}

JointPositionController::~JointPositionController()
{
}

bool JointPositionController::init(mechanism::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->hw_->current_time_;

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointPositionController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointPositionController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointPositionController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  TiXmlElement *p = j->FirstChildElement("pid");
  control_toolbox::Pid pid;
  if (p)
    pid.initXml(p);
  else
    fprintf(stderr, "JointPositionController's config did not specify the default pid parameters.\n");

  return init(robot, joint_name, pid);
}

void JointPositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointPositionController::getJointName()
{
  return joint_state_->joint_->name_;
}

// Set the joint position command
void JointPositionController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void JointPositionController::getCommand(double & cmd)
{
  cmd = command_;
}

void JointPositionController::update()
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  double time = robot_->hw_->current_time_;

  assert(joint_state_->joint_);

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

  joint_state_->commanded_effort_ = pid_controller_.updatePid(error, time - last_time_);
  last_time_ = time;
}

//------ Joint Position controller node --------
ROS_REGISTER_CONTROLLER(JointPositionControllerNode)

JointPositionControllerNode::JointPositionControllerNode(): node_(ros::node::instance())
{
  c_ = new JointPositionController();
}

JointPositionControllerNode::~JointPositionControllerNode()
{
  delete c_;
}

void JointPositionControllerNode::update()
{
  c_->update();
}

bool JointPositionControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(node_);
  service_prefix_ = config->Attribute("name");

  // Parses subcontroller configuration
  if (!c_->initXml(robot, config))
    return false;
  //subscriptions
  node_->subscribe(service_prefix_ + "/set_command", cmd_, &JointPositionControllerNode::setCommand, this, 1);
  guard_set_command_.set(service_prefix_ + "/set_command");
  //services
  node_->advertise_service(service_prefix_ + "/get_command", &JointPositionControllerNode::getCommand, this);
  guard_get_command_.set(service_prefix_ + "/get_command");

  return true;
}

void JointPositionControllerNode::setCommand()
{
  c_->setCommand(cmd_.data);
}

bool JointPositionControllerNode::getCommand(robot_srvs::GetValue::request &req,
                                             robot_srvs::GetValue::response &resp)
{
  double cmd;
  c_->getCommand(cmd);
  resp.v = cmd;
  return true;
}




