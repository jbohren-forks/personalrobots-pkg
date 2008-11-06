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

#include "mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"

namespace mechanism {

bool Robot::initXml(TiXmlElement *root)
{
  assert(hw_);
  TiXmlElement *xit = NULL;

  // Constructs the joints.
  for (xit = root->FirstChildElement("joint"); xit;
       xit = xit->NextSiblingElement("joint"))
  {
    Joint *j = new Joint;
    if (j->initXml(xit))
      joints_.push_back(j);
    else
      delete j;
  }

  // Constructs the transmissions.
  for (xit = root->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    const char *type = xit->Attribute("type");
    Transmission *t = type ? TransmissionFactory::instance().create(type) : NULL;
    if (!t)
      fprintf(stderr, "Unknown transmission type: \"%s\"\n", type);
    else if (!t->initXml(xit, this))
      delete t;
    else // Success!
      transmissions_.push_back(t);
  }

  // Constructs the links.
  for (xit = root->FirstChildElement("link"); xit;
       xit = xit->NextSiblingElement("link"))
  {
    Link *link = new Link;
    if (link->initXml(xit, this))
      links_.push_back(link);
    else
      delete link;
  }

  // For now, we only care about the sensors as links.
  for (xit = root->FirstChildElement("sensor"); xit;
       xit = xit->NextSiblingElement("sensor"))
  {
    Link *link = new Link;
    if (link->initXml(xit, this))
      links_.push_back(link);
    else
      delete link;
  }

  /// @todo add checks here to see if MCN is setup correctly, or have a good viewer

  return true;
}

template <class T>
int findIndexByName(std::vector<T*>& v, const std::string &name)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
      return i;
  }
  return -1;
}

int Robot::getActuatorIndex(const std::string &name)
{
  assert(hw_);
  return findIndexByName(hw_->actuators_, name);
}

int Robot::getJointIndex(const std::string &name)
{
  return findIndexByName(joints_, name);
}

int Robot::getLinkIndex(const std::string &name)
{
  return findIndexByName(links_, name);
}

int Robot::getTransmissionIndex(const std::string &name)
{
  return findIndexByName(transmissions_, name);
}

Joint* Robot::getJoint(const std::string &name)
{
  int i = getJointIndex(name);
  return i >= 0 ? joints_[i] : NULL;
}

Actuator* Robot::getActuator(const std::string &name)
{
  int i = getActuatorIndex(name);
  return i >= 0 ? hw_->actuators_[i] : NULL;
}

Link* Robot::getLink(const std::string &name)
{
  int i = getLinkIndex(name);
  return i >= 0 ? links_[i] : NULL;
}

Transmission* Robot::getTransmission(const std::string &name)
{
  int i = getTransmissionIndex(name);
  return i >= 0 ? transmissions_[i] : NULL;
}

RobotState::RobotState(Robot *model, HardwareInterface *hw)
  : model_(model), hw_(hw)
{
  assert(model_);
  assert(hw_);

  joint_states_.resize(model->joints_.size());
  link_states_.resize(model->links_.size());
  transmissions_in_.resize(model->transmissions_.size());
  transmissions_out_.resize(model->transmissions_.size());
  links_joint_.resize(model_->links_.size(), NULL);
  links_parent_.resize(model_->links_.size(), NULL);
  links_children_.resize(model_->links_.size());

  // Points each state object at the corresponding model object
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
    joint_states_[i].joint_ = model->joints_[i];
  for (unsigned int i = 0; i < link_states_.size(); ++i)
    link_states_[i].link_ = model->links_[i];

  // Wires up the transmissions to the state
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    Transmission *t = model_->transmissions_[i];
    for (unsigned int j = 0; j < t->actuator_names_.size(); ++j)
    {
      int index = model_->getActuatorIndex(t->actuator_names_[j]);
      assert(index >= 0);
      transmissions_in_[i].push_back(hw_->actuators_[index]);
    }
    for (unsigned int j = 0; j < t->joint_names_.size(); ++j)
    {
      int index = model_->getJointIndex(t->joint_names_[j]);
      assert(index >= 0);
      transmissions_out_[i].push_back(&joint_states_[index]);
    }
  }

  // Wires up each link to its joint, its parent, and its children.
  for (unsigned int i = 0; i < model_->links_.size(); ++i)
  {
    int joint_index = model_->getJointIndex(model_->links_[i]->joint_name_);
    int parent_index = model_->getLinkIndex(model_->links_[i]->parent_name_);

    if (joint_index >= 0 && parent_index >= 0)
    {
      links_joint_[i] = joint_index;
      links_parent_[i] = parent_index;
      links_children_[parent_index].push_back(i);
    }
    else
    {
      links_joint_[i] = -1;
      links_parent_[i] = -1;
    }
  }
}


JointState *RobotState::getJointState(const std::string &name)
{
  int i = model_->getJointIndex(name);
  return i >= 0 ? &joint_states_[i] : NULL;
}

LinkState *RobotState::getLinkState(const std::string &name)
{
  int i = model_->getLinkIndex(name);
  return i >= 0 ? &link_states_[i] : NULL;
}

void RobotState::propagateState()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePosition(transmissions_in_[i],
                                                 transmissions_out_[i]);
  }

  // Computes the absolute pose of the links using the relative transforms
  for (unsigned int i = 0; i < link_states_.size(); ++i)
  {
    if (links_joint_[i] < 0) // Root link, attached to the world
    {
      propagateAbsolutePose(i);
    }
  }
}

void RobotState::propagateEffort()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagateEffort(transmissions_out_[i],
                                               transmissions_in_[i]);
  }
}

void RobotState::enforceSafety()
{
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
  {
    model_->joints_[i]->enforceLimits(&joint_states_[i]);
  }
}

void RobotState::zeroCommands()
{
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
    joint_states_[i].commanded_effort_ = 0;
}

void RobotState::propagateStateBackwards()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePositionBackwards(transmissions_out_[i],
                                                          transmissions_in_[i]);
  }
}

void RobotState::propagateEffortBackwards()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagateEffortBackwards(transmissions_in_[i],
                                                        transmissions_out_[i]);
  }
}

void RobotState::propagateAbsolutePose(int index)
{
  LinkState *p = links_parent_[index] >= 0 ? &link_states_[links_parent_[index]] : NULL;
  JointState *j = links_joint_[index] >= 0 ? &joint_states_[links_joint_[index]] : NULL;
  link_states_[index].propagateFK(p, j);

  for (unsigned int i = 0; i < links_children_[index].size(); ++i)
    propagateAbsolutePose(links_children_[index][i]);
}

} // namespace mechanism
