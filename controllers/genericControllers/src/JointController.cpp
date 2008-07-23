/////////////////////////////////////////////////////////////////////////////////////
//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Willow Garage, Inc.
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
// * Neither the name of Willow Garage nor the names of its
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
#define DEFAULTMAXACCEL 5
//#define DEBUG 1
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
//Instantiate PID class
  pidController.InitPid(0,0,0,0,0); //Constructor for pid controller  

  //Set commands to zero
  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;
 
  controlMode = CONTROLLER::CONTROLLER_DISABLED;
}


JointController::~JointController( )
{
  
}

void JointController::Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint, double dt) {
  //Instantiate PID class
  pidController.InitPid(PGain,IGain,DGain,IMax,IMin); //Constructor for pid controller  
  //  printf("JointController:: %f, %f\n",IMax,IMin); 
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
  *time = ((double)t.tv_usec*1e-6 + (double) t.tv_sec);

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
  controlMode = CONTROLLER_DISABLED;
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
  return CONTROLLER::CONTROLLER_ALL_OK;
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
  if(controlMode == CONTROLLER_VELOCITY || controlMode == ETHERDRIVE_SPEED){ //Make sure we're in velocity command mode
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


double JointController::GetMaxVelocity(){
      double disToMin,disToMax,closestLimit;
      disToMin = fabs(shortest_angular_distance(joint->position, joint->jointLimitMin));
      disToMax = fabs(shortest_angular_distance(joint->position, joint->jointLimitMax));
      closestLimit =  (disToMin<disToMax)?disToMin:disToMax; //min
      //std::cout<<"Dis to min"<<disToMin<<" Dist to Max"<<disToMax<<" Closest limit"<<closestLimit<<std::endl;

      return sqrt(fabs(closestLimit*maxAccel));

}

//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//


void JointController::Update(void)
{
  double error(0),time(0),currentTorqueCmd(0);
  double maxVelocity = cmdVel;
  if(controlMode==CONTROLLER::CONTROLLER_DISABLED)return; //If we're not initialized, don't try to interact

  GetTime(&time); //TODO: Replace time with joint->timeStep


  double  currentVoltageCmd,v_backemf, v_clamp_min,v_clamp_max,k;      


  switch (controlMode)
    {
    case CONTROLLER_TORQUE: //Pass through torque command
      currentTorqueCmd = cmdTorque;
      break;
    case CONTROLLER_POSITION: //Close the loop around position
      //ASSUME ROTARY JOINT FOR NOW
      error = shortest_angular_distance(cmdPos, joint->position); 
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
#ifdef DEBUG
//      std::cout << "JC:: " << joint->position << ", cmdPos:: " << cmdPos << ", error:: " << error << ", cTC:: " << currentTorqueCmd << std::endl; 
#endif
      break;
    case CONTROLLER_VELOCITY: //Close the loop around velocity
      if(capAccel){
        maxVelocity = GetMaxVelocity(); //Check max velocity coming into wall
        if(fabs(cmdVel)>maxVelocity){
           cmdVel = -maxVelocity; //Truncate velocity smoothly
           //  std::cout<<"*******************"<<cmdVel<<std::endl;
        }
      }
      error = joint->velocity - cmdVel;
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
           // currentTorqueCmd = 0.5;
      //      printf("JointController.cpp:: error:: %f, dT:: %f \n", error, time-lastTime);
      //idea how to limit the velocity near limit
      //disToMin = shortest_angular_distance(joint->position, joint->jointLimitMin);
      //disToMax = shortest_angular_distance(joint->position, joint->jointLimitMax);
      //closestLimit =  (disToMin<disToMax)?disToMin:disToMax //min
      //if(joint->velocity^2/(2*maxAcc)-closestLimit<0.1 && accLimit) 
      //{
      //  cmdVel=2*maxAcc*closestLimit;
      //}      
      break;
    case ETHERDRIVE_SPEED: // Use hack to contol speed in voltage control mode for the etherdrive
      printf("JC:: %f\n",cmdVel);
      currentVoltageCmd = cmdVel*20*60/(136*2*M_PI); 
      v_backemf = joint->velocity*20*60/(136*2*M_PI);

      v_clamp_min = v_backemf - 3;// 0.655*16.7;      
      v_clamp_max = v_backemf + 3;//0.655*16.7;

      k = 1.0/ 36.0;

      printf("JC::%f\t%f\t%f\n", v_clamp_min, currentVoltageCmd, v_clamp_max);

      if (currentVoltageCmd > v_clamp_max)
	currentVoltageCmd = v_clamp_max;

      if (currentVoltageCmd < v_clamp_min)
	currentVoltageCmd = v_clamp_min;

      currentTorqueCmd = currentVoltageCmd * k; //Convert to match PWM conversion inside boards
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
  #ifdef DEBUG
  printf("JC:: torque:: %f,%f\n",torque,newTorque);
  #endif
  joint->commandedEffort = newTorque; 
  return newTorque;
}

CONTROLLER_ERROR_CODE JointController::LoadXML(std::string filename)
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
      if(*iter == "controller"){
         exists = 1;
         break;
      }
   }

   if(!exists)
      return CONTROLLER_MODE_ERROR;

   exists = 0;
   data.getDataTagNames("controller",names);

   for(iter = names.begin(); iter != names.end(); iter++){
      if(*iter == this->jointName){
         exists = 1;
         break;
      }
   }

   if(!exists)
      return CONTROLLER_MODE_ERROR;

   param_map = data.getDataTagValues("controller",this->jointName);   

   LoadParam("PGain",PGain);
   LoadParam("DGain",DGain);
   LoadParam("IGain",IGain);
   LoadParam("IMax",IMax);
   LoadParam("IMin",IMin);
   LoadParam("maxEffort",maxEffort);
   LoadParam("maxPositiveTorque",maxPositiveTorque);
   LoadParam("maxNegativeTorque",maxPositiveTorque);
   LoadParam("dt",dt);

   return  CONTROLLER_ALL_OK;
}

void JointController::LoadParam(std::string label, double &param)
{
   if(param_map.find(label) != param_map.end()) // if parameter value has been initialized in the xml file, set internal parameter value
      param = atof(param_map[label].c_str());
}

void JointController::LoadParam(std::string label, int &param)
{
   if(param_map.find(label) != param_map.end())
      param = atoi(param_map[label].c_str());
}
