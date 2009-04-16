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


namespace controller {

//ROS_REGISTER_CONTROLLER(JointEffortController)

JointEffortController::JointEffortController()
: joint_state_(NULL), command_(0), robot_(NULL)
{
}

JointEffortController::~JointEffortController()
{
}
bool JointEffortController::init(mechanism::RobotState *robot, const std::string &joint_name)
{
  if (!robot)
  {
    ROS_ERROR("The given robot was NULL");
    return false;
  }
  robot_ = robot;

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("JointEffortController could not find joint named \"%s\"\n",
            joint_name.c_str());
    return false;
  }

  return true;
}

bool JointEffortController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  if (!robot)
  {
    ROS_ERROR("The given robot was NULL");
    return false;
  }

  TiXmlElement *j = config->FirstChildElement("joint");
  if (!j || !j->Attribute("name"))
  {
    ROS_ERROR("JointEffortController was not given a joint\n");
    return false;
  }

  const char *jn = j->Attribute("name");
  std::string joint_name = jn ? jn : "";

  return init(robot, joint_name);

}

void JointEffortController::update()
{
  joint_state_->commanded_effort_ = command_;
}


//------ Joint Effort controller node --------
ROS_REGISTER_CONTROLLER(JointEffortControllerNode)

JointEffortControllerNode::JointEffortControllerNode(): node_(ros::Node::instance())
{
}

JointEffortControllerNode::~JointEffortControllerNode()
{
  node_->unsubscribe(name_ + "/command");
}

void JointEffortControllerNode::update()
{
  c_.update();
}

bool JointEffortControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(node_);
  name_ = config->Attribute("name");

  if (!c_.initXml(robot, config))
    return false;

  node_->subscribe(name_ + "/command", command_msg_, &JointEffortControllerNode::command, this, 1);

  return true;
}

void JointEffortControllerNode::command()
{
  c_.command_ = command_msg_.data;
}

}
