
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

#include <mechanism_control/single_control.h>
#include <signal.h>
#include <sys/time.h>

#include <mechanism_model/joint.h>
#include <genericControllers/JointController.h>

using namespace std;

#define NUM_JOINTS 1

const double maxPositiveTorque = 0.75;

SingleControl::SingleControl(){
  this->hw = NULL;
}

void SingleControl::init(HardwareInterface *hw){
  this->hw = hw;
    r = new Robot((char*)"robot");

  r->numJoints = NUM_JOINTS;
  r->numTransmissions = NUM_JOINTS;
  r->numLinks = NUM_JOINTS;

  r->transmission = new SimpleTransmission[NUM_JOINTS];
  r->joint = new Joint[NUM_JOINTS];
  r->link = new Link[NUM_JOINTS];

  for(int ii=0; ii<NUM_JOINTS; ii++){
     r->transmission[ii].actuator = &(hw->actuator[ii]);
     r->transmission[ii].joint = &(r->joint[ii]);
     r->transmission[ii].mechanicalReduction = -1;
     r->transmission[ii].motorTorqueConstant = 1.0;
     r->transmission[ii].pulsesPerRevolution = 1200;
     hw->actuator[ii].command.enable = true;
     r->joint[ii].effortLimit = maxPositiveTorque;
  }

  r->joint->jointLimitMax = 10;
  r->joint->jointLimitMin = -10;
  r->joint->effortLimit = 5;
  controller = new JointController();

#if POSITION
    controller->init(0.3, 0.01, 0.0, 0.75,-0.75,controller::CONTROLLER_POSITION,hw->current_time_,0.75,-0.75, r->joint);
    controller->setPosCmd(1.0);
#elif TORQUE
    controller->init(0.01, 0.1, 0, 0.75,-0.75,controller::CONTROLLER_TORQUE,hw->current_time_,0.75,-0.75, r->joint);
    controller->setTorqueCmd(0.1);
#else
    controller->init(0.01, 0.1, 0, 0.75,-0.75,controller::CONTROLLER_VELOCITY,hw->current_time_,0.75,-0.75, r->joint);
    controller->setVelCmd(100);
#endif
}

void SingleControl::update()//This function is called only from the realtime loop.  Everything it calls must also be realtime safe.
{
  //Clear actuator commands

  //Process robot model transmissions
  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagatePosition();
  }

  //update KDL model with new joint position/velocities
  controller->update(hw->current_time_);
#ifdef DEBUG
  for(int i = 0; i < r->numJoints; i++){
    printf("base_control:: cmd:: %d, %f\n",i,r->joint[i].commandedEffort);
  }
#endif
  /*  for(int i = 0; i < r->numJoints; i++){
    r->joint[i].enforceLimits();
    }*/

  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagateEffort();
  }
}
