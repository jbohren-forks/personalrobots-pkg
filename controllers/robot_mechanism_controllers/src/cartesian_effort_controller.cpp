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

#include "robot_mechanism_controllers/cartesian_effort_controller.h"
#include "urdf/parser.h"
#include <algorithm>

namespace controller {

ROS_REGISTER_CONTROLLER(CartesianEffortController)

CartesianEffortController::CartesianEffortController()
: command_(0,0,0),
  offset_(0,0,0),
  links_(0,(mechanism::LinkState*)NULL),
  joints_(0,(mechanism::JointState*)NULL)
{
}

CartesianEffortController::~CartesianEffortController()
{
}

bool CartesianEffortController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);

  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain)
  {
    fprintf(stderr, "Error: CartesianEffortController was not given a chain\n");
    return false;
  }

  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name)
  {
    fprintf(stderr, "Error: Chain element for CartesianEffortController must specify the root\n");
    return false;
  }
  if (!tip_name)
  {
    fprintf(stderr, "Error: Chain element for CartesianEffortController must specify the tip\n");
    return false;
  }

  if (!robot->getLinkState(root_name))
  {
    fprintf(stderr, "Error: link \"%s\" does not exist (CartesianEffortController)\n", root_name);
    return false;
  }

  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)
  {
    fprintf(stderr, "Error: link \"%s\" does not exist (CartesianEffortController)\n", tip_name);
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
      fprintf(stderr, "Error: for CartesianEffortController, tip is not connected to root\n");
      return false;
    }
  }

  std::reverse(links_.begin(), links_.end());
  std::reverse(joints_.begin(), joints_.end());

  assert(joints_.size() == links_.size() - 1);

  if (chain->Attribute("offset"))
  {
    std::vector<double> offset_pieces;
    urdf::queryVectorAttribute(chain, "offset", &offset_pieces);
    assert(offset_pieces.size() == 3);  // TODO

    offset_[0] = offset_pieces[0];
    offset_[1] = offset_pieces[1];
    offset_[2] = offset_pieces[2];
  }

  return true;
}

void CartesianEffortController::update()
{
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
      return;
  }

  tf::Vector3 F = command_;  // force vector, will be transformed to the current link's frame

  for (unsigned int i = 1; i < links_.size(); ++i)
  {
    F = links_[i]->rel_frame_.getBasis().transpose() * F;
  }
  // At this point, F is the desired force in the current link's frame

  tf::Vector3 r(offset_);  // position of the force in the current frame

  for (int i = links_.size() - 2; i >= 0; --i)
  {
    // Applies the force to joint i

    switch (joints_[i]->joint_->type_)
    {
    case mechanism::JOINT_ROTARY:
    case mechanism::JOINT_CONTINUOUS: {
      tf::Vector3 torque = cross(r, F);
      joints_[i]->commanded_effort_ += torque.dot(joints_[i]->joint_->axis_);

      // Propagate back to link i
      break;
    }
    case mechanism::JOINT_PRISMATIC:
      // TODO: none of this works for prismatic joints yet
      abort();
    default:
      abort();
    }

    // Transforms the effort to the parent link's coordinate frame
    r = links_[i+1]->rel_frame_(r);
    F = links_[i+1]->rel_frame_.getBasis() * F;
  }
}

ROS_REGISTER_CONTROLLER(CartesianEffortControllerNode)

CartesianEffortControllerNode::CartesianEffortControllerNode()
{
}

CartesianEffortControllerNode::~CartesianEffortControllerNode()
{
}

bool CartesianEffortControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to CartesianEffortControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  node->advertise_service(topic + "/set_command",
                          &CartesianEffortControllerNode::setCommand, this);
  guard_set_actual_.set(topic + "/set_command");
  return true;
}

void CartesianEffortControllerNode::update()
{
  c_.update();
}

bool CartesianEffortControllerNode::setCommand(
  robot_srvs::SetVector::request &req,
  robot_srvs::SetVector::response &resp)
{
  tf::Vector3MsgToTF(req.v, c_.command_);
  return true;
}

}
