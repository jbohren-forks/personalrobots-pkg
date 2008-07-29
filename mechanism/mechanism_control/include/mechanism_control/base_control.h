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
#ifndef BASE_CONTROL_H
#define BASE_CONTROL_H

#include <mechanism_model/robot.h>
#include <rosthread/mutex.h>
#include <map>
#include <pr2Controllers/BaseController.h>
#include <hw_interface/hardware_interface.h>
#include <ros/node.h>
#include <joy/Joy.h>

using std::map;

using namespace controller;

//using CONTROLLER::Controller;

//Base requirements for a piece of code that will control a full mechanism
const int MAX_NUM_CONTROLLERS = 1000;

typedef Controller*(*ControllerAllocationFunc)(const char *);

class BaseControl{

  public:

  BaseControl();

  void update(); //Must be realtime safe

  void registerControllerType(const char *type, ControllerAllocationFunc f);

  void requestController(const char *type, const char *ns);

  void init(HardwareInterface *hw);

  BaseController *controller;

  //JointController *controller;

  private:

//map<const char *, ControllerAllocationFunc> controllerLibrary;

//Controller *controller[MAX_NUM_CONTROLLERS];

//ros::thread::mutex controllerListMutex;

  HardwareInterface *hw;

  mechanism::Robot *r;

  void initRobot();

  void initControllers();

};

#endif
