
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage Inc. nor the names of its
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

#include <mechanism_control/base_control.h>
#include <signal.h>
#include <etherdrive_hardware/etherdrive_hardware.h>
#include <sys/time.h>

#define NUM_JOINTS 12

const double maxPositiveTorque = 0.75;

using namespace std;

BaseControl::BaseControl(){
  this->hw = NULL;
}

void BaseControl::init(HardwareInterface *hw){

  this->hw = hw;

  this->initRobot();

  this->initControllers();
}

void BaseControl::initRobot(){
  r = new Robot("robot");

  r->numJoints = NUM_JOINTS;
  r->numLinks = NUM_JOINTS;

  r->joint = new Joint[NUM_JOINTS];
  r->link = new Link[NUM_JOINTS];

  for(int ii=0; ii<NUM_JOINTS; ii++){
    SimpleTransmission *tr = new SimpleTransmission;
    r->transmissions_.push_back(tr);
    tr->actuator = &(hw->actuator[ii]);
    tr->joint = &(r->joint[ii]);
    tr->mechanicalReduction = 1.0;
    tr->motorTorqueConstant = 1.0;
    tr->pulsesPerRevolution = 90000;
    hw->actuator[ii].command.enable = true;
    r->joint[ii].effortLimit = maxPositiveTorque;
  }
}

void BaseControl::initControllers(){
  controller = new BaseController(r,"baseController");
  char *c_filename = getenv("ROS_PACKAGE_PATH");
  std::stringstream filename;
  filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;
  controller->loadXML(filename.str());
  controller->init();
}

//This function is called only from the realtime loop.  Everything it calls must also be realtime safe.
void BaseControl::update(){
  //Clear actuator commands

  //Process robot model transmissions
  for(int i = 0; i < r->transmissions_.size(); i++){
    r->transmissions_[i]->propagatePosition();
  }

  //update KDL model with new joint position/velocities
  controller->update();
#ifdef DEBUG
  for(int i = 0; i < r->numJoints; i++){
    printf("base_control:: cmd:: %d, %f\n",i,r->joint[i].commandedEffort);
  }
#endif
  /*  for(int i = 0; i < r->numJoints; i++){
      r->joint[i].enforceLimits();
      }*/

  for(int i = 0; i < r->transmissions_.size(); i++){
    r->transmissions_[i]->propagateEffort();
  }
}

