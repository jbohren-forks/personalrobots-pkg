
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

//Simulated Iface-hardware implementation for gazebo

#include <gazebo_hardware/gazebo_hardware.h>

using namespace std;

GazeboHardware::GazeboHardware(int numBoards, int numActuators, int boardLookUp[], int portLookUp[], int jointId[], string etherIP[], string hostIP[],HardwareInterface *hw){
  this->numBoards    = numBoards;
  this->numActuators = numActuators;
  this->hw = hw;
  for(int ii = 0; ii < numActuators; ii++)
    {
      this->boardLookUp[ii] = boardLookUp[ii];
      this->portLookUp[ii] = portLookUp[ii];
      this->jointId[ii] = jointId[ii];
      this->etherIP[ii] = etherIP[ii];
      this->hostIP[ii] = hostIP[ii];
    }
  //edBoard = new EtherDrive[numBoards];
  client                  = new gazebo::Client();
  edBoard                 = new gazebo::PR2ArrayIface[numBoards];
  pr2GripperLeftIface     = new gazebo::PR2GripperIface();
  pr2GripperRightIface    = new gazebo::PR2GripperIface();

};

void GazeboHardware::init(){

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
  }


  /// Open the actuator arrays for casters spine arm interface
  try
  {
    edBoard[1].Open(client, "pr2_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 interface\n"
    << e << "\n";
  }

  /// Open the actuator arrays for head interface
  try
  {
    edBoard[0].Open(client, "pr2_head_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 head interface\n"
    << e << "\n";
  }

  /// Open the interface for gripper left
  try
  {
    pr2GripperLeftIface->Open(client, "pr2_gripper_left_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 gripper left interface\n"
    << e << "\n";
  }

  /// Open the interface for gripper right
  try
  {
    pr2GripperRightIface->Open(client, "pr2_gripper_right_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the pr2 gripper right interface\n"
    << e << "\n";
  }

  setGains(1,0,0,0,0,0);               // hard-coded for all the boards and all the motors for now
  setControlMode(PR2::TORQUE_CONTROL); // for all motors
  setMotorsOn(true);


};

void GazeboHardware::updateState(){
  //
  //  assuming hw has entire list of actuators
  //
  for(int ii = 0; ii < numActuators; ii++)
    {
      hw->actuator[jointId[ii]].state.timestamp++;
      if (boardLookUp[ii] == 2)
      {
        pr2GripperLeftIface->Lock(1);
        hw->actuator[jointId[ii]].state.encoderCount = GAZEBO_POS_TO_ENCODER*(
          ( pr2GripperLeftIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
         +(-pr2GripperLeftIface->data->actualFingerPosition[1] + 0.015)     );
        pr2GripperLeftIface->Unlock();
      }
      else if (boardLookUp[ii] == 3)
      {
        pr2GripperRightIface->Lock(1);
        hw->actuator[jointId[ii]].state.encoderCount = GAZEBO_POS_TO_ENCODER*(
          ( pr2GripperRightIface->data->actualFingerPosition[0] - 0.015)  // FIXME: not debugged
         +(-pr2GripperRightIface->data->actualFingerPosition[1] + 0.015)    );
        pr2GripperRightIface->Unlock();
      }
      else
      {
        edBoard[boardLookUp[ii]].Lock(1);
        hw->actuator[jointId[ii]].state.encoderCount = GAZEBO_POS_TO_ENCODER*edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].actualPosition;
        edBoard[boardLookUp[ii]].Unlock();
      }
      //fprintf(stderr,"edh:: %d\n",hw->actuator[jointId[ii]].state.encoderCount);
    }

};

void GazeboHardware::sendCommand(){
  double command = 0;
  for(int ii = 0; ii < numActuators; ii++)
    {
      if( hw->actuator[ii].command.enable){
        command = (GAZEBO_CURRENT_TO_CMD*hw->actuator[ii].command.current);
        //fprintf(stderr,"command: %f\n", command);
        if (boardLookUp[ii] == 2)
        {
          pr2GripperLeftIface->Lock(1);
          pr2GripperLeftIface->data->cmdForce             = command;
          pr2GripperLeftIface->Unlock();
        }
        else if (boardLookUp[ii] == 3)
        {
          pr2GripperRightIface->Lock(1);
          pr2GripperRightIface->data->cmdForce            = command;
          pr2GripperRightIface->Unlock();
        }
        else
        {
          edBoard[boardLookUp[ii]].Lock(1);
          edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].cmdEffectorForce = command;
          edBoard[boardLookUp[ii]].Unlock();
        }
      }
    }
}

void GazeboHardware::setGains(int P, int I, int D, int W, int M, int Z)
{
  for(int ii = 0; ii < numActuators; ii++)
  {
    if (boardLookUp[ii] == 2)
    {
      pr2GripperLeftIface->data->pGain     = P;
      pr2GripperLeftIface->data->iGain     = I;
      pr2GripperLeftIface->data->dGain     = D;
    }
    else if (boardLookUp[ii] == 3)
    {
      pr2GripperRightIface->data->pGain    = P;
      pr2GripperRightIface->data->iGain    = I;
      pr2GripperRightIface->data->dGain    = D;
    }
    else
    {
      edBoard[boardLookUp[ii]].Lock(1);
      edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].pGain = P;
      edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].iGain = I;
      edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].dGain = D;
      edBoard[boardLookUp[ii]].Unlock();
    }
  }
}

void GazeboHardware::setControlMode(int controlMode)
{
  for(int ii = 0; ii < numActuators; ii++)
  {
    if (boardLookUp[ii] == 2)
    {
      pr2GripperLeftIface->Lock(1);
      pr2GripperLeftIface->data->controlMode     = controlMode;
      pr2GripperLeftIface->Unlock();
    }
    else if (boardLookUp[ii] == 3)
    {
      pr2GripperRightIface->Lock(1);
      pr2GripperRightIface->data->controlMode    = controlMode;
      pr2GripperRightIface->Unlock();
    }
    else
    {
      edBoard[boardLookUp[ii]].Lock(1);
      edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].controlMode = controlMode;
      edBoard[boardLookUp[ii]].Unlock();
    }
  }
}

void GazeboHardware::setMotorsOn(bool motorsOn)
{
  for(int ii = 0; ii < numActuators; ii++)
  { 
    hw->actuator[ii].command.enable = true;
    if (boardLookUp[ii] == 2)
    {
      pr2GripperLeftIface->Lock(1);
      pr2GripperLeftIface->data->cmdEnableMotor     = motorsOn;
      pr2GripperLeftIface->Unlock();
    }
    else if (boardLookUp[ii] == 3)
    {
      pr2GripperRightIface->Lock(1);
      pr2GripperRightIface->data->cmdEnableMotor    = motorsOn;
      pr2GripperRightIface->Unlock();
    }
    else
    {
      edBoard[boardLookUp[ii]].Lock(1);
      edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].cmdEnableMotor = motorsOn;
      edBoard[boardLookUp[ii]].Unlock();
    }
  }
}

void GazeboHardware::tick() {
  for(int ii = 0; ii < numActuators; ii++)
  {
    if (boardLookUp[ii] == 2)
    {
      //pr2GripperLeftIface->Lock(1);
      //pr2GripperLeftIface->data->cmdEnableMotor     = motorsOn;
      //pr2GripperLeftIface->Unlock();
    }
    else if (boardLookUp[ii] == 3)
    {
      //pr2GripperRightIface->Lock(1);
      //pr2GripperRightIface->data->cmdEnableMotor    = motorsOn;
      //pr2GripperRightIface->Unlock();
    }
    else
    {
      //edBoard[boardLookUp[ii]].Lock(1);
      //edBoard[boardLookUp[ii]].data->actuators[portLookUp[ii]].cmdEnableMotor = motorsOn;
      //edBoard[boardLookUp[ii]].Unlock();
    }
  }
}

GazeboHardware::~GazeboHardware()
{
  //  printf("Switching off motors \n");
  setMotorsOn(false);
};

