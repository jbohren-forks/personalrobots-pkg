
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage, Inc. nor the names of its 
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

#include "mechanism_control/mechanism_control.h"

MechanismControl::MechanismControl(HardwareInterface *hw)
  : initialized_(0), model_((char*)"robot"), hw_(hw)
{
  memset(controllers_, 0, MAX_NUM_CONTROLLERS*sizeof(void*));
}

MechanismControl::~MechanismControl() {
}

bool MechanismControl::registerActuator(const std::string &name, int index) {
  if (initialized_)
    return false;

  actuators_.insert(ActuatorMap::value_type(name, index));

  return true;
}

bool MechanismControl::init() { // TODO: should take in a node of xml?
  // Creates:
  // - joints
  // - transmissions
  return true;
}

// Must be realtime safe.
void MechanismControl::update() {

  // Propogates actuator information into the robot model.
  // Propogates through the robot model.
  // Calls update on all of the plugins.
  // Performs safety checks on the commands.
  // Propogates commands back into the actuators.

#if 0
  //Clear actuator commands

  //Process robot model transmissions
  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagatePosition();
  }

  //update KDL model with new joint position/velocities
  //

  //update all controllers
  for(int i = 0; i < MAX_NUM_CONTROLLERS; i++){
    if(controller[i] != NULL){
      controller[i]->update();
    }
  }

  for(int i = 0; i < r->numJoints; i++){
    r->joint[i].enforceLimits();
  }

  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagateEffort();
  }
#endif
}



void MechanismControl::registerControllerType(const std::string& type, ControllerAllocator f){
  controller_library_.insert(std::pair<std::string,ControllerAllocator>(type, f));
}

bool MechanismControl::spawnController(const char *type, const char *ns){
  controller::Controller *c;
  ControllerAllocator f = controller_library_[type];
  c = f(ns);

  //At this point, the controller is fully initialized and has created the ROS interface, etc.

  //Add controller to list of controllers in realtime-safe manner;
  controllers_mutex_.lock(); //This lock is only to prevent us from other non-realtime threads.  The realtime thread may be spinning through the list of controllers while we are in here, so we need to keep that list always in a valid state.  This is why we fully allocate and set up the controller before adding it into the list of active controllers.
  bool spot_found = false;
  for(int i = 0; i < MAX_NUM_CONTROLLERS; i++){
    if(controllers_[i] == NULL)
    {
      spot_found = true;
      controllers_[i] = c;
      break;
    }
  }
  controllers_mutex_.unlock();

  if (!spot_found)
  {
    delete c;
    return false;
  }

  return true;
}
