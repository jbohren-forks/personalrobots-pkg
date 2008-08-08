
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Eric Berger
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * MechanismControl mc(hardware_interface);
 * mc.registerActuator("AnActuatorName", 0);
 * mc.registerActuator("AnotherActuatorName", 0);
 * ...
 * mc.init(config);
 *
 * mc.spawnController("JointController", controllerConfig);
 */
#ifndef MECHANISM_CONTROL_H
#define MECHANISM_CONTROL_H

#include <map>
#include <string>
#include <vector>
#include "ros/node.h"
#include <tinyxml/tinyxml.h>
#include <hardware_interface/hardware_interface.h>
#include <mechanism_model/robot.h>
#include <rosthread/mutex.h>
#include <generic_controllers/controller.h>

#include "mechanism_control/ListControllerTypes.h"

typedef controller::Controller* (*ControllerAllocator)();

class MechanismControl {
public:
  MechanismControl(HardwareInterface *hw);
  virtual ~MechanismControl();

  // Real-time functions
  void update();

  // Non real-time functions
  bool init(TiXmlElement* config);
  bool registerActuator(const std::string &name, int index);
  bool addController(controller::Controller *c);
  bool spawnController(const char* type, TiXmlElement* config);

  mechanism::Robot model_;

  // TODO: deprecated.  Replaced by ControllerFactory
  void registerControllerType(const std::string& type, ControllerAllocator f);

private:
  bool initialized_;
  HardwareInterface *hw_;

  const static int MAX_NUM_CONTROLLERS = 100;
  ros::thread::mutex controllers_mutex_;
  controller::Controller* controllers_[MAX_NUM_CONTROLLERS];
};

/*
 * Exposes MechanismControl's interface over ROS
 */
class MechanismControlNode : public MechanismControl, public ros::node
{
public:
  MechanismControlNode(HardwareInterface *hw);
  virtual ~MechanismControlNode() {}

  bool listControllerTypes(mechanism_control::ListControllerTypes::request &req,
                           mechanism_control::ListControllerTypes::response &resp);

private:
};

#endif /* MECHANISM_CONTROL_H */
