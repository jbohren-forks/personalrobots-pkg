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
  for (unsigned int i = 0; i < links_.size(); ++i)
  {
    links_[i]->createTreePointers(this);
  }
  printLinkTree();

  // Constructs the kinematic chains.
  for (xit = root->FirstChildElement("chain"); xit;
       xit = xit->NextSiblingElement("chain"))
  {
    Chain *c = new Chain;
    if (c->initXml(xit, this))
      chains_.push_back(c);
    else
      delete c;
  }

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

void printLinkTreeHelper(Link *link, int depth = 0)
{
  for (int i = 0; i < depth; ++i)
    printf("  ");
  printf("%s (%s)\n", link->name_.c_str(), link->joint_ ? link->joint_->name_.c_str() : "");

  for (unsigned int i = 0; i < link->children_.size(); ++i)
    printLinkTreeHelper(link->children_[i], depth + 1);
}
void Robot::printLinkTree()
{
  Link *root = NULL;
  for (unsigned int i = 0; i < links_.size(); ++i)
  {
    if (links_[i]->parent_name_ == "world")
    {
      root = links_[i];
      break;
    }
  }
  if (!root)
  {
    fprintf(stderr, "Could not print the link tree because there's no link connected to the world\n");
    return;
  }

  printLinkTreeHelper(root, 0);
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

  // TODO: KDL
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

} // namespace mechanism
