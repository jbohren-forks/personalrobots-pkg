/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include "robot_mechanism_controllers/cartesian_torque_controller.h"
#include "urdf/parser.h"
#include <algorithm>

namespace controller {

CartesianTorqueController::CartesianTorqueController()
: command_(0,0,0),
  links_(0,(mechanism::LinkState*)NULL),
  joints_(0,(mechanism::JointState*)NULL)
{
}

CartesianTorqueController::~CartesianTorqueController()
{
}

bool CartesianTorqueController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain)
  {
    fprintf(stderr, "Error: CartesianTorqueController was not given a chain\n");
    return false;
  }

  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name)
  {
    fprintf(stderr, "Error: Chain element for CartesianTorqueController must specify the root\n");
    return false;
  }
  if (!tip_name)
  {
    fprintf(stderr, "Error: Chain element for CartesianTorqueController must specify the tip\n");
    return false;
  }

  if (!robot->getLinkState(root_name))
  {
    fprintf(stderr, "Error: link \"%s\" does not exist (CartesianTorqueController)\n", root_name);
    return false;
  }

  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)
  {
    fprintf(stderr, "Error: link \"%s\" does not exist (CartesianTorqueController)\n", tip_name);
    return false;
  }

  // Works up the chain, from the tip to the root.
  while (true)
  {
    links_.push_back(current);

    if (current->link_->name_ == std::string(root_name))
      break;

    joints_.push_back(robot->getJointState(current->link_->joint_name_));
    assert(joints_[joints_.size()-1]);

    current = robot->getLinkState(current->link_->parent_name_);

    if (!current)
    {
      fprintf(stderr, "Error: for CartesianTorqueController, tip is not connected to root\n");
      return false;
    }
  }

  std::reverse(links_.begin(), links_.end());
  std::reverse(joints_.begin(), joints_.end());

  assert(joints_.size() == links_.size() - 1);

  return true;
}

void CartesianTorqueController::update()
{
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
      return;
  }

  tf::Vector3 T = command_;  // torque vector, will be transformed to the current link's frame

  for (unsigned int i = 1; i < links_.size(); ++i)
  {
    T = links_[i]->rel_frame_.getBasis().transpose() * T;
  }
  // At this point, F is the desired torque in the current link's frame

  for (int i = links_.size() - 2; i >= 0; --i)
  {
    // Applies the torque to joint i

    switch (joints_[i]->joint_->type_)
    {
    case mechanism::JOINT_ROTARY:
    case mechanism::JOINT_CONTINUOUS: {
      joints_[i]->commanded_effort_ += T.dot(joints_[i]->joint_->axis_);
      break;
    }
    case mechanism::JOINT_PRISMATIC:
      break;
    default:
      abort();
    }

    // Transforms the torque to the parent link's coordinate frame
    T = links_[i+1]->rel_frame_.getBasis() * T;
  }
}

ROS_REGISTER_CONTROLLER(CartesianTorqueControllerNode)

CartesianTorqueControllerNode::CartesianTorqueControllerNode()
{
}

CartesianTorqueControllerNode::~CartesianTorqueControllerNode()
{
}

bool CartesianTorqueControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to CartesianTorqueControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

//  node->advertise_service(topic + "/set_command",
//                          &CartesianTorqueControllerNode::setCommand, this);
//  guard_set_command_.set(topic + "/set_command");

  node->subscribe(topic + "/set_command", set_command_msg_,
                  &CartesianTorqueControllerNode::setCommand, this, 1);
  guard_set_command_.set(topic + "/set_command");
  return true;
}

void CartesianTorqueControllerNode::update()
{
  c_.update();
}

void CartesianTorqueControllerNode::setCommand()
{
  tf::Vector3MsgToTF(set_command_msg_, c_.command_);
}

}
