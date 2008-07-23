
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

#include "etherdrive_hardware/etherdrive_hardware.h"
#include <sys/time.h>

using namespace std;

double EtherdriveHardware::GetTimeHardware()
{
  struct timeval t;
  gettimeofday( &t, 0);
  //printf("GetTime: %f,  %f \n",(double) t.tv_usec,(double) t.tv_sec);
  return ((double) t.tv_usec *1e-6 + (double) t.tv_sec);
}

EtherdriveHardware::EtherdriveHardware(int numBoards, int numActuators, int boardLookUp[], int portLookUp[], int jointId[], string etherIP[], string hostIP[]){
  int ii;
  this->numBoards = numBoards;
  this->numActuators = numActuators;
  this->hw = new HardwareInterface(numActuators);

  for(ii = 0; ii < numActuators; ii++){
      this->boardLookUp[ii] = boardLookUp[ii];
      this->portLookUp[ii] = portLookUp[ii];
      this->jointId[ii] = jointId[ii];
    }

  for(ii = 0; ii < numBoards; ii++){
      this->etherIP[ii] = etherIP[ii];
      this->hostIP[ii] = hostIP[ii];
  }
  edBoard = new EtherDrive[numBoards];
};

void EtherdriveHardware::init(){
  for(int ii=0; ii<numBoards; ii++)
    edBoard[ii].init(etherIP[ii],hostIP[ii]);
   setGains(0,10,0,100,1004,1); // hard-coded for all the boards and all the motors for now
   setControlMode(ETHERDRIVE_VOLTAGE_MODE);
   setMotorsOn(true);
};

void EtherdriveHardware::updateState(){
  double newTime;
  double newCount, dT, dE;
  for(int ii = 0; ii < numActuators; ii++)
    {
      newTime = this->GetTimeHardware();
      newCount = edBoard[boardLookUp[ii]].get_enc(portLookUp[ii]);
      dT = (double) (newTime - hw->actuator[ii].state.timestamp);
     if(dT < 1e-6){
	//printf("Changing dT\n");
	dT = 1e-3;
	}
      dE = (double) (newCount - hw->actuator[ii].state.encoderCount);
      double vel = dE / dT;
      hw->actuator[ii].state.encoderVelocity = vel;
  
      hw->actuator[ii].state.timestamp = newTime;
      hw->actuator[ii].state.encoderCount = (int) newCount;
#ifdef DEBUG
      printf("etherdrive_hardware.cpp:: %d, enc: %d\n",ii,hw->actuator[ii].state.encoderCount);
#endif
    }
};

void EtherdriveHardware::sendCommand(){
  int command = 0;
  for(int ii = 0; ii < numActuators; ii++)
    {
      if( hw->actuator[ii].command.enable){
	command = (int)(ETHERDRIVE_CURRENT_TO_CMD*hw->actuator[ii].command.current);
#ifdef DEBUG
	printf("command: %d, %d\n", ii, command);
#endif
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

void EtherdriveHardware::setMotorsOn(bool motorsOn)
{
  for(int ii = 0; ii < numBoards; ii++)
  { 
    if(motorsOn)
      edBoard[ii].motors_on();
    else
      edBoard[ii].motors_off();
  }
}

void EtherdriveHardware::update() {
  sendCommand();
  for(int ii = 0; ii < numBoards; ii++)
    edBoard[ii].tick();
  updateState();
}

EtherdriveHardware::~EtherdriveHardware()
{
  //  printf("Switching off motors \n");
  setMotorsOn(false);
};

/*void EtherdriveHardware::LoadXML(std::string filename)
{
   robot_desc::URDF model;
   int exists = 0;

   if(!model.loadFile(filename.c_str()))
      return CONTROLLER_MODE_ERROR;

   const robot_desc::URDF::Data &data = model.getData();

   std::vector<std::string> types;
   std::vector<std::string> names;
   std::vector<std::string>::iterator iter;

   data.getDataTagTypes(types);

   for(iter = types.begin(); iter != types.end(); iter++){
      if(*iter == "etherdrive"){
         exists = 1;
         break;
      }
   }

   if(!exists)
      return CONTROLLER_MODE_ERROR;

   data.getDataTagNames("etherdrive",names);


   
   param_map = model.getDataTagValues("mcb","etherdrive");   

   return CONTROLLER_ALL_OK;
}

void EtherdriveHardware::LoadParam(std::string label, double &param)
{
   if(param_map.find(label) != param_map.end()) // if parameter value has been initialized in the xml file, set internal parameter value
      param = atof(param_map[label].c_str());
}

void EtherdriveHardware::LoadParam(std::string label, int &param)
{
   if(param_map.find(label) != param_map.end())
      param = atoi(param_map[label].c_str());
}

void EtherdriveHardware::LoadParam(std::string label, std::string &param)
{
   if(param_map.find(label) != param_map.end()) // if parameter value has been initialized in the xml file, set internal parameter value
      param = param_map[label];
}
*/
#ifdef _ETHERDRIVE_MAIN_TEST
int main(int argc, char *argv[]){

  int numBoards = 2;
  int numActuators = 2;
  int boardLookUp[] ={0, 1}; 
  int portLookUp[] = {0, 0};
  int jointId[]={0, 1};
  string etherIP[] = {"10.12.0.103", "10.11.0.102"};
  string hostIP[] = {"10.12.0.2", "10.11.0.3"};

  EtherdriveHardware *h = new EtherdriveHardware(numBoards, numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP);
  HardwareInterface *hi = new HardwareInterface(1);
  h->init();
  hi->actuator[0].command.enable = true;
  hi->actuator[0].command.current = 0.5;
  
  hi->actuator[1].command.enable = true;
  hi->actuator[1].command.current = 0.5;

  h->sendCommand(hi);
  for(;;) {
    h->update();
    usleep(1000);
  }  

  delete(h);
  delete(hi);
  }
#endif
