
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Sachin Chitta, Jimmy Sastra
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

//Etherdrive hardware implementation
//Creates etherdrive interface to the robot actuators

#include "etherdrive/etherdrive_hardware.h"

EtherdriveHardware::EtherdriveHardware(int numBoards, int numActuators, int boardLookUp[], int portLookUp[], int jointId[], string etherIP[], string hostIP[]){
   this->numBoards = numBoards;
   this->numActuators = numActuators;
   for(int ii = 0; ii < numActuators; ii++)
   {
      this->boardLookUp[ii] = boardLookUp[ii];
      this->portLookUp[ii] = portLookUp[ii];
      this->jointId[ii] = jointId[ii];
      this->etherIP[ii] = etherIP[ii];
      this->hostIP[ii] = hostIP[ii];
   }
 
   edBoard = new EtherDrive[numBoards];
};

void EtherdriveHardware::init(){
   for(int ii=0; ii<numBoards; ii++)
      edBoard[ii].init(etherIP[ii],hostIP[ii]);
   setControlMode(ETHERDRIVE_CURRENT_MODE);
   setGains(0,10,0,100,1004,1); // hard-coded for all the boards and all the motors for now
};

void EtherdriveHardware::updateState(HardwareInterface *hw){
   for(int ii = 0; ii < numActuators; ii++)
   {
      hw->actuator[jointId[ii]].state.timestamp++;
      hw->actuator[jointId[ii]].state.encoderCount = edBoard[boardLookUp[ii]].get_enc(portLookUp[ii]);
   }
};

void EtherdriveHardware::sendCommand(HardwareInterface *hw){
   int command = 0;
   for(int ii = 0; ii < numActuators; ii++)
   {
      if( hw->actuator[ii].command.enable){
         command = ETHERDRIVE_CURRENT_TO_CMD*(int)hw->actuator[ii].command.current;
         edBoard[boardLookUp[ii]].set_drv(portLookUp[ii], command);
      }
   }
}



void EtherdriveHardware::setGains(int P, int I, int D, int W, int M, int Z)
{
   for(int ii = 0; ii < numActuators; ii++)
   {
      edBoard[boardLookUp[ii]].set_gain(portLookUp[ii],'P',P);
      edBoard[boardLookUp[ii]].set_gain(portLookUp[ii],'I',I);
      edBoard[boardLookUp[ii]].set_gain(portLookUp[ii],'D',D);
      edBoard[boardLookUp[ii]].set_gain(portLookUp[ii],'W',W);
      edBoard[boardLookUp[ii]].set_gain(portLookUp[ii],'M',M);
      edBoard[boardLookUp[ii]].set_gain(portLookUp[ii],'Z',Z);
   }
}

void EtherdriveHardware::setControlMode(int controlMode)
{
   for(int ii=0; ii < numBoards; ii++)
      edBoard[ii].set_control_mode(controlMode);
}

int main(int argc, char *argv[]) {
  int numBoards = 1;
  int numActuators = 1;
  int boardLookUp[] ={0}; 
  int portLookUp[] = {0};
  int jointId[]={0};
  string etherIP[] = "10.11.0.3";
  string hostIP[] = "10.11.0.102";

  EtherdriveHardware* EDtest = new EtherdriveHardware(numBoards, numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP);

  EDtest->init();
  return 0;
}

/*int main(int argc, char *argv[]){
   EtherdriveHardware *h = new NullHardware();
   delete(h);
   }*/
