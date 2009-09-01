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
 * Author: Stuart Glaser, Wim Meeussen
 */

#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/transmission.h"
#include <tinyxml/tinyxml.h>
#include <kdl_parser/dom_parser.hpp>
#include <urdf/model.h>
#include <pluginlib/class_loader.h>


namespace pr2_mechanism {

bool Robot::initXml(TiXmlElement *root)
{
  // Gets the actuator list from the hardware interface
  if (!hw_){
    ROS_ERROR("Mechanism Model got an invalid pointer to the hardware interface");
    return false;
  }
  actuators_ = hw_->actuators_;

  // Parses the xml into a robot model
  urdf::Model robot_description;
  if (!robot_description.initXml(root)){
    ROS_ERROR("Mechanism Model failed to parse the URDF xml into a robot model");
    return false;
  }

  // Constructs a kdl robot model from the robot description.
  if (!kdl_parser::treeFromRobotModel(robot_description, robot_model_)){
    ROS_ERROR("Mechanism Model failed to construct a kdl tree from the robot model");
    return false;
  }

  // Constructs the joints from the robot description.
  for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator it = robot_description.joints_.begin();
       it != robot_description.joints_.end(); it++){
    Joint* jnt = new Joint;
    if (!jnt->init(it->second)){
      ROS_ERROR("Mechanism Model failed to initialize joint");
      delete jnt;
      return false;
    }
    joints_.push_back(jnt);
  }

  // Constructs the transmissions by parsing custom xml.
  pluginlib::ClassLoader<pr2_mechanism::Transmission> transmission_loader("pr2_mechanism_model", "pr2_mechanism::Transmission");
  TiXmlElement *xit = NULL;
  for (xit = root->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    const char *type = xit->Attribute("type");
    Transmission *t;
    try{
      t = type ? transmission_loader.createClassInstance(type) : NULL;
    }
    catch(...){
      ROS_ERROR("Could not construct transmission of type %s", type);
      return false;
    }
    if (!t)
      ROS_ERROR("Unknown transmission type: %s", type);
    else if (!t->initXml(xit, this)){
      ROS_ERROR("Failed to initialize transmission");
      delete t;
    }
    else // Success!
      transmissions_.push_back(t);
  }


  return true;
}

template <class T>
int findIndexByName(const std::vector<T*>& v, const std::string &name)
{
  for (unsigned int i = 0; i < v.size(); ++i)
  {
    if (v[i]->name_ == name)
      return i;
  }
  return -1;
}

int Robot::getActuatorIndex(const std::string &name) const
{
  return findIndexByName(actuators_, name);
}

int Robot::getJointIndex(const std::string &name) const
{
  return findIndexByName(joints_, name);
}

int Robot::getTransmissionIndex(const std::string &name) const
{
  return findIndexByName(transmissions_, name);
}

Joint* Robot::getJoint(const std::string &name) const
{
  int i = getJointIndex(name);
  return i >= 0 ? joints_[i] : NULL;
}

Actuator* Robot::getActuator(const std::string &name) const
{
  int i = getActuatorIndex(name);
  return i >= 0 ? actuators_[i] : NULL;
}

Transmission* Robot::getTransmission(const std::string &name) const
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
  transmissions_in_.resize(model->transmissions_.size());
  transmissions_out_.resize(model->transmissions_.size());

  // Points each state object at the corresponding model object
  for (unsigned int i = 0; i < joint_states_.size(); ++i)
  {
    joint_states_[i].joint_ = model->joints_[i];
    
    // Only because I'm feeling really nice today.
    if (model->joints_[i]->type_ == urdf::Joint::FIXED)
      joint_states_[i].calibrated_ = true;
  }

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

const JointState *RobotState::getJointState(const std::string &name) const
{
  int i = model_->getJointIndex(name);
  return i >= 0 ? &joint_states_[i] : NULL;
}

void RobotState::propagateState()
{
  for (unsigned int i = 0; i < model_->transmissions_.size(); ++i)
  {
    model_->transmissions_[i]->propagatePosition(transmissions_in_[i],
                                                 transmissions_out_[i]);
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


} // namespace pr2_mechanism
