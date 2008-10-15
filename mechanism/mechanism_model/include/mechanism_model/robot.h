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

/*
 * The robot model tracks the state of the robot.
 *
 * State path:
 *               +---------------+                
 * Actuators --> | Transmissions | --> Joints --> Links
 *               +---------------+                
 *
 * Author: Stuart Glaser
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
#include <map>
#include <string>
#include "stl_utils/stl_utils.h"
#include "mechanism_model/link.h"
#include "mechanism_model/joint.h"
#include "mechanism_model/transmission.h"
#include "hardware_interface/hardware_interface.h"

class TiXmlElement;

namespace mechanism
{

class Robot
{
public:
  Robot() {}

  ~Robot()
  {
    deleteElements(&transmissions_);
    deleteElements(&joints_);
    deleteElements(&links_);
  }

  bool initXml(TiXmlElement *root);

  HardwareInterface *hw_;  // Holds the array of actuators
  std::vector<Transmission*> transmissions_;
  std::vector<Joint*> joints_;
  std::vector<Link*> links_;

  // All return -1 on failure.
  int getJointIndex(const std::string &name);
  int getActuatorIndex(const std::string &name);
  int getLinkIndex(const std::string &name);
  int getTransmissionIndex(const std::string &name);

  // All return NULL on failure
  Joint* getJoint(const std::string &name);
  Actuator* getActuator(const std::string &name);
  Link* getLink(const std::string &name);
  Transmission* getTransmission(const std::string &name);

  // For debugging
  void printLinkTree();
};

class RobotState
{
public:
  RobotState(Robot *model, HardwareInterface *hw);

  Robot *model_;

  HardwareInterface *hw_;  // Actuator states
  std::vector<JointState> joint_states_;
  std::vector<LinkState> link_states_;

  std::vector<std::vector<Actuator*> > transmissions_in_;
  std::vector<std::vector<JointState*> > transmissions_out_;

  std::vector<int> links_joint_;
  std::vector<int> links_parent_;
  std::vector<std::vector<int> > links_children_;

  JointState *getJointState(const std::string &name);
  LinkState *getLinkState(const std::string &name);

  void propagateState();
  void propagateEffort();
  void enforceSafety();
  void zeroCommands();

  void propagateStateBackwards();
  void propagateEffortBackwards();

private:
  void propagateAbsolutePose(int index);
};

}

#endif
