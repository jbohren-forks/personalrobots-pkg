
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

#include "mechanism_control.h"

MechanismControl::MechanismControl(HardwareInterface *hw){
  this->hw = hw;
  //Needs to learn pr2.xml filename or get it from param server
  //Create robot modeli
  r = new Robot("robot"); //Should actually be name-space
  //Set up transmissions, joints, and links

  for(int i = 0; i < MAX_NUM_CONTROLLERS; i++){
    controller[i] = NULL;
  }
  //Set up ROS
  //Subscribe to "new controller request" topic
  //(Should be Advertise "new controller" service)
}

void MechanismControl::registerControllerType(const char *type, ControllerAllocationFunc f){
  controllerLibrary[type] = f;
}

//This function will be exported as a ROS service and called externally to set up all the control loops on system startup.
void MechanismControl::requestController(const char *type, const char *ns){
  Controller *c;
  ControllerAllocationFunc f = controllerLibrary[type];
  c = f(ns);
  //At this point, the controller is fully initialized and has created the ROS interface, etc.

  //Add controller to list of controllers in realtime-safe manner;
  controllerListMutex.lock(); //This lock is only to prevent us from other non-realtime threads.  The realtime thread may be spinning through the list of controllers while we are in here, so we need to keep that list always in a valid state.  This is why we fully allocate and set up the controller before adding it into the list of active controllers.
  for(int i = 0; i < MAX_NUM_CONTROLLERS; i++){
    if(controller[i] == NULL){
      controller[i] = c;
    }
    break;
  }
  controllerListMutex.unlock();
}

//This function is called only from the realtime loop.  Everything it calls must also be realtime safe.
void MechanismControl::update(){
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
}
