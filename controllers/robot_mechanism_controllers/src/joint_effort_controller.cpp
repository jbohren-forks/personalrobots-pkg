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
#include <robot_mechanism_controllers/joint_effort_controller.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointEffortController)

JointEffortController::JointEffortController()
{
  robot_ = NULL;
  joint_ = NULL;

  command_ = 0;
}

JointEffortController::~JointEffortController()
{
}

bool JointEffortController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointEffortController was not given a joint\n");
    return false;
  }

  const char *joint_name = j->Attribute("name");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "JointVelocityController could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  return true;
}

// Set the joint position command
void JointEffortController::setCommand(double command)
{
  command_ = command;
}

// Return the current position command
double JointEffortController::getCommand()
{
  return command_;
}

// Return the measured joint position
double JointEffortController::getMeasuredEffort()
{
  return joint_->applied_effort_;
}

double JointEffortController::getTime()
{
  return robot_->hw_->current_time_;
}

void JointEffortController::update()
{

  joint_->commanded_effort_ = command_;
}

ROS_REGISTER_CONTROLLER(JointEffortControllerNode)
JointEffortControllerNode::JointEffortControllerNode()
{
  c_ = new JointEffortController();
}

JointEffortControllerNode::~JointEffortControllerNode()
{
  delete c_;
}

void JointEffortControllerNode::update()
{
  c_->update();
}

bool JointEffortControllerNode::setCommand(
  robot_mechanism_controllers::SetCommand::request &req,
  robot_mechanism_controllers::SetCommand::response &resp)
{
  c_->setCommand(req.command);
  resp.command = c_->getCommand();

  return true;
}

bool JointEffortControllerNode::getActual(
  robot_mechanism_controllers::GetActual::request &req,
  robot_mechanism_controllers::GetActual::response &resp)
{
  resp.command = c_->getMeasuredEffort();
  resp.time = c_->getTime();
  return true;
}

bool JointEffortControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointEffortControllerNode\n");
    return false;
  }

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(topic + "/set_command", &JointEffortControllerNode::setCommand, this);
  guard_set_command_.set(topic + "/set_command");
  node->advertise_service(topic + "/get_actual", &JointEffortControllerNode::getActual, this);
  guard_get_actual_.set(topic + "/get_actual");
  return true;
}

