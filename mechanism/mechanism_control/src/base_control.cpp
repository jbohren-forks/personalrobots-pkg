
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

#include "mechanism_control/base_control.h"

MechanismControl::MechanismControl(HardwareInterface *hw){
  this->hw = hw;
}

void MechanismControl::Init()
{
  r = new Robot("robot"); 

  r->numJoints = BASE_NUM_JOINTS;
  r->numTransmissions = BASE_NUM_JOINTS;
  r->numLinks = BASE_NUM_JOINTS;

  r->transmission = new SimpleTransmission[BASE_NUM_JOINTS];
  r->joint = new Joint[BASE_NUM_JOINTS];
  r->link = new Link[BASE_NUM_JOINTS];

  for(int ii=0; ii<BASE_NUM_JOINTS; ii++){
     r->transmission[ii].actuator = &(hw->actuator[ii]);
     r->transmission[ii].joint = &(r->joint[ii]);
     r->transmission[ii].mechanicalReduction = 1;
     r->transmission[ii].motorTorqueConstant = 0.074;
  }

  controller = new BaseController(r);
  controller->Init();
}

void MechanismControl::update()//This function is called only from the realtime loop.  Everything it calls must also be realtime safe.
{
  //Clear actuator commands

  //Process robot model transmissions
  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagatePosition();
  }

  //update KDL model with new joint position/velocities
  //
  controller->Update();

  for(int i = 0; i < r->numJoints; i++){
    r->joint[i].enforceLimits();
  }

  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagateEffort();
  }
}

