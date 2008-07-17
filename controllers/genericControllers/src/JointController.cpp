/////////////////////////////////////////////////////////////////////////////////////
//Software License Agreement (BSD License)
//
//Copyright (c) 2008, David Li, Melonee Wise, John Hsu
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////////////

#include <genericControllers/JointController.h>
#include <iostream>
#include <sys/time.h>
//Todo: 
//1. Get and set params via server
//2. Integrate Joint and robot objects
//3. Integrate Motor controller time

using namespace CONTROLLER;
 
//---------------------------------------------------------------------------------//
//CONSTRUCTION/DESTRUCTION CALLS
//---------------------------------------------------------------------------------//


JointController::JointController( )
{
  
}


JointController::~JointController( )
{
  
}

void JointController::Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint, double dt) {
  //Instantiate PID class
  pidController.InitPid(PGain,IGain,DGain,IMax,IMin); //Constructor for pid controller  

  //Set commands to zero
  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;
  
  //Init time
  lastTime = time;

  //Temporary: will transition to use param server
  this->PGain = PGain;
  this->IGain = IGain;
  this->DGain = DGain;
  this->IMax = IMax;
  this->IMin = IMin;

  this->maxPositiveTorque = maxPositiveTorque;
  this->maxNegativeTorque = maxNegativeTorque;
  this->maxEffort = maxEffort;
  
  this->joint = joint;

  controlMode = mode;

  //set dt if included
  this->dt = dt;

  //Turn on controller
  EnableController();

}

void JointController::Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint) {
  //Instantiate PID class
  pidController.InitPid(PGain,IGain,DGain,IMax,IMin); //Constructor for pid controller  

  //Set commands to zero
  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;
  
  //Init time
  lastTime = time;

  //Temporary: will transition to use param server
  this->PGain = PGain;
  this->IGain = IGain;
  this->DGain = DGain;
  this->IMax = IMax;
  this->IMin = IMin;

  this->maxPositiveTorque = maxPositiveTorque;
  this->maxNegativeTorque = maxNegativeTorque;
  this->maxEffort = maxEffort;
  
  this->joint = joint;

  controlMode = mode;

  
  dt=0.001; //TODO: make sure this goes away

  //Turn on controller
  EnableController();

}


//---------------------------------------------------------------------------------//
//TIME CALLS
//---------------------------------------------------------------------------------//


//Returns the current time. Will eventually mode to use motor board controller time TODO
void JointController::GetTime(double *time){

  struct timeval t;
  gettimeofday( &t, 0);
  *time =  (double) (t.tv_usec *1e-6 + t.tv_sec);

//  return PR2::PR2_ALL_OK;
//  return(myPR2->hw.GetSimTime(time));
}

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//


//Set the controller control mode
CONTROLLER_CONTROL_MODE JointController::SetMode(CONTROLLER_CONTROL_MODE mode){
  controlMode = mode;
  return CONTROLLER_MODE_SET;
}

//Getter for control mode
CONTROLLER_CONTROL_MODE JointController::GetMode(){
  return controlMode;
}

//Allow controller to function
CONTROLLER_CONTROL_MODE JointController::EnableController(){
  enabled = true;
  return CONTROLLER_ENABLED;
}

//Disable functioning. Set joint torque to zero.
CONTROLLER_CONTROL_MODE JointController::DisableController(){
  enabled = false;
  joint->commandedEffort = 0; //Immediately set commanded Effort to 0
  return CONTROLLER_DISABLED;
}

bool JointController::CheckForSaturation(void){ 
  return SaturationFlag;
}



//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE JointController::SetTorqueCmd(double torque){
// double maxEffort = joint->effortLimit;
  
  if(controlMode != CONTROLLER_TORQUE)  //Make sure we're in torque command mode
  return CONTROLLER_MODE_ERROR;
  
  cmdTorque = torque;  
  
  if(cmdTorque > maxEffort){ //Truncate to positive limit
    cmdTorque = maxEffort;
    return CONTROLLER_TORQUE_LIMIT;
  }
  else if (cmdPos < -maxEffort){ //Truncate to negative limit
    cmdTorque = -maxEffort;
    return CONTROLLER_TORQUE_LIMIT;
  }
  
  return CONTROLLER_CMD_SET;
}

//Return current torque command
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetTorqueCmd(double *torque)
{
  *torque = cmdTorque;
}

//Query motor for actual torque 
CONTROLLER::CONTROLLER_ERROR_CODE
JointController::GetTorqueAct(double *torque)
{
  *torque = joint->appliedEffort; //Read torque from joint
  return CONTROLLER::CONTROLLER_ALL_OK; 
}

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//


//Query mode, then set desired position 
CONTROLLER_ERROR_CODE JointController::SetPosCmd(double pos)
{
  if(controlMode != CONTROLLER_POSITION)  //Make sure we're in position command mode
  return CONTROLLER_MODE_ERROR;
  
  cmdPos = pos;  
  if(cmdPos > joint->jointLimitMax){ //Truncate to positive limit
    cmdPos = joint->jointLimitMax;
    return CONTROLLER_JOINT_LIMIT;
  }
  else if (cmdPos < joint->jointLimitMin){ //Truncate to negative limit
    cmdPos = joint->jointLimitMin;
    return CONTROLLER_JOINT_LIMIT;
  }
  return CONTROLLER_CMD_SET;
  
}

//Return the current position command
CONTROLLER::CONTROLLER_ERROR_CODE JointController::GetPosCmd(double *pos)
{
  *pos = cmdPos;
  return CONTROLLER::CONTROLLER_ALL_OK; 
}

//Query the joint for the actual position
CONTROLLER::CONTROLLER_ERROR_CODE JointController::GetPosAct(double *pos)
{
  *pos = joint->position;
  return CONTROLLER::CONTROLLER_ALL_OK; 
}

//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//
//Check mode, then set the commanded velocity
CONTROLLER_ERROR_CODE JointController::SetVelCmd(double vel)
{
  if(controlMode == CONTROLLER_VELOCITY){ //Make sure we're in velocity command mode
    cmdVel = vel;  
    return CONTROLLER_CMD_SET;
  }
  else return CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
CONTROLLER::CONTROLLER_ERROR_CODE JointController::GetVelCmd(double *vel)
{
  *vel = cmdVel;
  return CONTROLLER::CONTROLLER_ALL_OK; 
}

//Query our joint for velocity
CONTROLLER::CONTROLLER_ERROR_CODE JointController::GetVelAct(double *vel)
{
  *vel = joint->velocity;
  return CONTROLLER::CONTROLLER_ALL_OK; 
}
//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//
void JointController::Update(void)
{
  double error(0),time(0),currentTorqueCmd(0);
  GetTime(&time); //TODO: Replace time with joint->timeStep
  switch (controlMode)
  {
    case CONTROLLER_TORQUE: //Pass through torque command
      currentTorqueCmd = cmdTorque;
      break;
    case CONTROLLER_POSITION: //Close the loop around position
      //ASSUME ROTARY JOINT FOR NOW
      error = shortest_angular_distance(joint->position, cmdPos); 
      currentTorqueCmd = pidController.UpdatePid(error,dt);
#ifdef DEBUG
     std::cout << "JC:: " << joint->position << " cmd:: " << cmdPos << "error:: " << error << "cTC:: " << currentTorqueCmd << std::endl; 
#endif
      break;
    case CONTROLLER_VELOCITY: //Close the loop around velocity
      printf("vel. control \n");
      error = cmdVel-joint->velocity; 
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
      //idea how to limit the velocity near limit
      //disToMin = shortest_angular_distance(joint->position, joint->jointLimitMin);
      //disToMax = shortest_angular_distance(joint->position, joint->jointLimitMax);
      //closestLimit =  (disToMin<disToMax)?disToMin:disToMax //min
      //if(joint->velocity^2/(2*maxAcc)-closestLimit<0.1 && accLimit) 
      //{
      //  cmdVel=2*maxAcc*closestLimit;
      //}      
      break;
    default: //On error (no mode), set torque to zero
      currentTorqueCmd = 0; 
      //TODO:put somekind of error here for no mode
    }
  lastTime = time;
  //Make sure we're enabled. If not, send a 0 torque command
  if(enabled) SafelySetTorqueInternal(currentTorqueCmd);   
  else SafelySetTorqueInternal(0); //Send a zero command if disabled
}

//---------------------------------------------------------------------------------//
//PARAM SERVER CALLS
//---------------------------------------------------------------------------------//

//TODO: Case statement to effect changes when parameters are set here
CONTROLLER_ERROR_CODE JointController::SetParam(std::string label,double value)
{   
  return CONTROLLER_ALL_OK;
}

/*
CONTROLLER_ERROR_CODE
JointController::SetParam(std::string label,std::string value)
{
  return CONTROLLER_ALL_OK;
}
*/

CONTROLLER_ERROR_CODE JointController::GetParam(std::string label, double* value)
{
return CONTROLLER_ALL_OK;
}

/*
CONTROLLER_ERROR_CODE JointController::GetParam(std::string label, std::string value)
{
}
*/


//---------------------------------------------------------------------------------//
//SAFETY CALLS
//---------------------------------------------------------------------------------//

//Truncates (if needed), then sets the torque
double JointController::SafelySetTorqueInternal(double torque)
{
  double newTorque;
    
//  std::cout<<"Effort:"<<torque<<std::endl; 
  //Read the max positive and max negative torque once
//  maxPositiveTorque = joint->effortLimit;
 // maxNegativeTorque = -joint->effortLimit; 

  if(torque>maxPositiveTorque){
    newTorque = maxPositiveTorque;
    SaturationFlag = true;
  }
  else if (torque< maxNegativeTorque) {
    newTorque = maxNegativeTorque;
    SaturationFlag = true;
  }
  else {
    newTorque = torque;
    SaturationFlag = false;
  }

  
  //Set torque command 
  //  printf("JC::cE:: %f,%f\n",torque,newTorque);
  joint->commandedEffort = newTorque; 
  return newTorque;
}

