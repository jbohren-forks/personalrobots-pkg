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
: joint_state_(NULL), robot_(NULL), command_(0)
{
}

JointEffortController::~JointEffortController()
{
}
bool JointEffortController::init(mechanism::RobotState *robot, const std::string &joint_name)
{
  assert(robot);
  robot_ = robot;

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    fprintf(stderr, "JointEffortController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  return true;
}

bool JointEffortController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j)
  {
    fprintf(stderr, "JointEffortController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  return init(robot, joint_name);

}

std::string JointEffortController::getJointName()
{
  return joint_state_->joint_->name_;
}

// Set the joint position command
void JointEffortController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void JointEffortController::getCommand(double & cmd)
{
  cmd = command_;
}

void JointEffortController::update()
{

  joint_state_->commanded_effort_ = command_;
}


//------ Joint Effort controller node --------
ROS_REGISTER_CONTROLLER(JointEffortControllerNode)

JointEffortControllerNode::JointEffortControllerNode(): node_(ros::Node::instance())
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

bool JointEffortControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(node_);
  service_prefix_ = config->Attribute("name");

  // Parses subcontroller configuration
  if (!c_->initXml(robot, config))
    return false;
  //subscriptions
  node_->subscribe(service_prefix_ + "/set_command", cmd_, &JointEffortControllerNode::setCommand, this, 1);
  guard_set_command_.set(service_prefix_ + "/set_command");
  //services
  node_->advertiseService(service_prefix_ + "/get_command", &JointEffortControllerNode::getCommand, this);
  guard_get_command_.set(service_prefix_ + "/get_command");

  return true;
}

void JointEffortControllerNode::setCommand()
{
  c_->setCommand(cmd_.data);
}

bool JointEffortControllerNode::getCommand(robot_srvs::GetValue::request &req,
                                           robot_srvs::GetValue::response &resp)
{
  double cmd;
  c_->getCommand(cmd);
  resp.v = cmd;
  return true;
}

