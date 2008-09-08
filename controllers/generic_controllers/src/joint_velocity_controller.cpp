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

#include <generic_controllers/joint_velocity_controller.h>


using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointVelocityController)

JointVelocityController::JointVelocityController()
: joint_state_(NULL), robot_state_(NULL), last_time_(0), command_(0)
{
}

JointVelocityController::~JointVelocityController()
{
}

bool JointVelocityController::init(mechanism::RobotState *robot_state, const std::string &joint_name, const Pid &pid)
{
  assert(robot_state);
  robot_state_ = robot_state;
  last_time_ = robot_state->hw_->current_time_;

  joint_state_ = robot_state_->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointVelocityController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  pid_ = pid;

  return true;
}

bool JointVelocityController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointVelocityController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  Pid pid;
  TiXmlElement *p = j->FirstChildElement("pid");
  if (p)
    pid.initXml(p);
  else
  {
    fprintf(stderr, "JointVelocityController's config did not specify the default pid parameters.\n");
    return false;
  }

  return this->init(robot_state, joint_name, pid);
}

// Set the joint velocity command
void JointVelocityController::setCommand(double command)
{
  command_ = command;
}

// Return the current velocity command
double JointVelocityController::getCommand()
{
  return command_;
}

double JointVelocityController::getTime()
{
  return robot_state_->hw_->current_time_;
}

// Return the measured joint velocity
double JointVelocityController::getMeasuredVelocity()
{
  return joint_state_->velocity_;
}

std::string JointVelocityController::getJointName()
{
  return joint_state_->joint_->name_;
}

void JointVelocityController::update()
{
  double error(0);
  double time = robot_state_->hw_->current_time_;

  error = joint_state_->velocity_ - command_;
  joint_state_->commanded_effort_ = pid_.updatePid(error, time - last_time_);
  last_time_ = time;
}

void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_.setGains(p,i,d,i_max,i_min);
}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_.getGains(p,i,d,i_max,i_min);
}


ROS_REGISTER_CONTROLLER(JointVelocityControllerNode)
JointVelocityControllerNode::JointVelocityControllerNode()
{
  c_ = new JointVelocityController();
}

JointVelocityControllerNode::~JointVelocityControllerNode()
{
  delete c_;
}

void JointVelocityControllerNode::update()
{
  c_->update();
}

bool JointVelocityControllerNode::setCommand(
  generic_controllers::SetCommand::request &req,
  generic_controllers::SetCommand::response &resp)
{
  c_->setCommand(req.command);
  resp.command = c_->getCommand();

  return true;
}

bool JointVelocityControllerNode::getActual(
  generic_controllers::GetActual::request &req,
  generic_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredVelocity();
  resp.time = c_->getTime();

  return true;
}

bool JointVelocityControllerNode::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointVelocityControllerNode\n");
    return false;
  }

  if (!c_->initXml(robot_state, config))
    return false;
  node->advertise_service(topic + "/set_command", &JointVelocityControllerNode::setCommand, this);
  node->advertise_service(topic + "/get_actual", &JointVelocityControllerNode::getActual, this);
  return true;
}
