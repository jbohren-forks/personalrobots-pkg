
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

//Null hardware implementation
//Creates empty robot and runs it with null controller

#include "null_hardware.h"
#include "mechanism_control.h"

NullHardware::NullHardware(){
  //Read actuators.xml and initialize hardware
  int numActuators = 10;
  HardwareInterface *hw = new HardwareInterface(numActuators);

  controller = new MechanismControl(hw);
  for(int i = 0; i < 100; i++){
    //Read in data from hardware
    for(int i = 0; i < hw->numActuators; i++){
      updateState(&hw->actuator[i].state);
    }
    //Fill in HardwareInterface with all new status
    controller->update();
    //Send HardwareInterface commands out to motors
    for(int i = 0; i < hw->numActuators; i++){
      readCommand(&hw->actuator[i].command);
    }
    //Sleep(1)
  }
}

void NullHardware::updateState(ActuatorState *state){
  state->timestamp++;
  state->encoderCount = 100;
}

void NullHardware::readCommand(ActuatorCommand *command){
  //double current = command->current;
  //writeMotor(current);
}

int main(int argc, char *argv[]){
  NullHardware *h = new NullHardware();
  delete(h);
}
