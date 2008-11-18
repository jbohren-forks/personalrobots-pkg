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

using namespace controller;

//---------------------------------------------------------------------------------//
//CONSTRUCTION/DESTRUCTION CALLS
//---------------------------------------------------------------------------------//


JointController::JointController()
{
//Instantiate PID class
  pidController.InitPid(0,0,0,0,0); //Constructor for pid controller

  //Set commands to zero
  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;

  controlMode = controller::CONTROLLER_DISABLED;
}


JointController::~JointController( )
{

}

void JointController::init(double pGain, double iGain, double dGain, double windupMax, double windupMin, controllerControlMode mode, double time, double maxEffort, double minEffort, mechanism::Joint *joint) {

  pidController.InitPid(pGain,iGain,dGain,windupMax,windupMin); //Constructor for pid controller

  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;

  //Init time
  lastTime = time;

  //Temporary: will transition to use param server
  this->pGain = pGain;
  this->iGain = iGain;
  this->dGain = dGain;
  this->windupMax  = windupMax;
  this->windupMin  = windupMin;

  this->minEffort = minEffort;
  this->maxEffort = maxEffort;
  this->joint = joint;

  controlMode = mode;
  enableController();
}

void JointController::init(pidControlParam pcp, controllerControlMode mode, double time, double maxEffort, double minEffort, Joint *joint) {

  pidController.InitPid(pcp.pGain,pcp.iGain,pcp.dGain,pcp.windupMax,pcp.windupMin); //Constructor for pid controller

  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;

  //Init time
  lastTime = time;

  //Temporary: will transition to use param server
  this->pGain = pcp.pGain;
  this->iGain = pcp.iGain;
  this->dGain = pcp.dGain;
  this->windupMax  = pcp.windupMax;
  this->windupMin  = pcp.windupMin;

  this->minEffort = minEffort;
  this->maxEffort = maxEffort;
  this->joint = joint;


  controlMode = mode;
  enableController();
}


void JointController::init(double time, Joint *joint) {
  pidController.InitPid(pGain,iGain,dGain,windupMax,windupMin); //Constructor for pid controller

  cmdTorque = 0;
  cmdPos = 0;
  cmdVel = 0;

  lastTime = time;
  this->joint = joint;
  enableController();
}


//---------------------------------------------------------------------------------//
//TIME CALLS
//---------------------------------------------------------------------------------//


//Returns the current time. Will eventually mode to use motor board controller time TODO
void JointController::getTime(double *time){
  struct timeval t;
  gettimeofday( &t, 0);
  *time = ((double)t.tv_usec*1e-6 + (double) t.tv_sec);
}

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//

//Set the controller control mode
controllerControlMode JointController::setMode(controllerControlMode mode){
  controlMode = mode;
  return CONTROLLER_MODE_SET;
}

//Getter for control mode
controllerControlMode JointController::getMode(){
  return controlMode;
}

//Allow controller to function
controllerControlMode JointController::enableController(){
  enabled = true;
  return CONTROLLER_ENABLED;
}

//Disable functioning. Set joint torque to zero.
controllerControlMode JointController::disableController(){
  enabled = false;
  joint->commandedEffort = 0; //Immediately set commanded Effort to 0
  controlMode = CONTROLLER_DISABLED;
  return CONTROLLER_DISABLED;
}

bool JointController::checkForSaturation(void){
  return saturationFlag;
}



//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode JointController::setTorqueCmd(double torque){
// double maxEffort = joint->effortLimit;

  if(controlMode != CONTROLLER_TORQUE)  //Make sure we're in torque command mode
  return CONTROLLER_MODE_ERROR;

  cmdTorque = torque;

  if(cmdTorque >= maxEffort){ //Truncate to positive limit
    cmdTorque = maxEffort;
    return CONTROLLER_TORQUE_LIMIT;
  }
  else if (cmdTorque <= minEffort){ //Truncate to negative limit
    cmdTorque = minEffort;
    return CONTROLLER_TORQUE_LIMIT;
  }

  return CONTROLLER_CMD_SET;
}

controllerErrorCode JointController::getTorqueCmd(double *torque)
{
  *torque = cmdTorque;
  return CONTROLLER_ALL_OK;
}

controllerErrorCode JointController::getTorqueAct(double *torque)
{
  *torque = joint->appliedEffort; //Read torque from joint
  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//


//Query mode, then set desired position
controllerErrorCode JointController::setPosCmd(double pos)
{
  if(controlMode != CONTROLLER_POSITION)  //Make sure we're in position command mode
  return CONTROLLER_MODE_ERROR;

  cmdPos = pos;
  if(cmdPos >= joint->jointLimitMax && joint->type != mechanism::JOINT_CONTINUOUS){ //Truncate to positive limit
    cmdPos = joint->jointLimitMax;
    return CONTROLLER_JOINT_LIMIT;
  }
  else if (cmdPos <= joint->jointLimitMin && joint->type != mechanism::JOINT_CONTINUOUS){ //Truncate to negative limit
    cmdPos = joint->jointLimitMin;
    return CONTROLLER_JOINT_LIMIT;
  }
  return CONTROLLER_CMD_SET;
}

//Return the current position command
controller::controllerErrorCode JointController::getPosCmd(double *pos)
{
  *pos = cmdPos;
  return controller::CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
controller::controllerErrorCode JointController::getPosAct(double *pos)
{
  *pos = joint->position;
  return controller::CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//
//Check mode, then set the commanded velocity
controllerErrorCode JointController::setVelCmd(double vel)
{
  if(controlMode == CONTROLLER_VELOCITY || controlMode == ETHERDRIVE_SPEED){ //Make sure we're in velocity command mode
    cmdVel = vel;
    return CONTROLLER_CMD_SET;
  }
  else return CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
controller::controllerErrorCode JointController::getVelCmd(double *vel)
{
  *vel = cmdVel;
  return controller::CONTROLLER_ALL_OK;
}

//Query our joint for velocity
controller::controllerErrorCode JointController::getVelAct(double *vel)
{
  *vel = joint->velocity;
  return controller::CONTROLLER_ALL_OK;
}


double JointController::getMaxVelocity(){
      double disToMin,disToMax,closestLimit;
      disToMin = fabs(angles::shortest_angular_distance(joint->position, joint->jointLimitMin));
      disToMax = fabs(angles::shortest_angular_distance(joint->position, joint->jointLimitMax));
      closestLimit =  (disToMin<disToMax)?disToMin:disToMax; //min
      //std::cout<<"Dis to min"<<disToMin<<" Dist to Max"<<disToMax<<" Closest limit"<<closestLimit<<std::endl;
      return sqrt(fabs(closestLimit*maxAccel));
}

//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//


void JointController::update(void)
{
  double error(0),time(0),currentTorqueCmd(0);
  double maxVelocity = cmdVel;
  double currentVoltageCmd,v_backemf,v_clamp_min,v_clamp_max,k;


   if(controlMode == controller::CONTROLLER_DISABLED){
    printf("JointController.cpp: Error:: controller disabled\n");
    return; //If we're not initialized, don't try to interact
  }
  getTime(&time); //TODO: Replace time with joint->timeStep
  switch(controlMode)
  {
    case CONTROLLER_TORQUE: //Pass through torque command
      currentTorqueCmd = cmdTorque;
      break;

    case CONTROLLER_POSITION: //Close the loop around position
      if(joint->type == mechanism::JOINT_ROTARY || joint->type == mechanism::JOINT_CONTINUOUS)
	error = angles::shortest_angular_distance(cmdPos, joint->position);
      else
        error = joint->position - cmdPos;
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
      break;

    case CONTROLLER_VELOCITY: //Close the loop around velocity
      if(capAccel){
        maxVelocity = getMaxVelocity(); //Check max velocity coming into wall
        if(fabs(cmdVel)>maxVelocity){
          cmdVel = -maxVelocity; //Truncate velocity smoothly
          //  std::cout<<"*******************"<<cmdVel<<std::endl;
        }
      }
      error = joint->velocity - cmdVel;

      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
      break;
    case ETHERDRIVE_SPEED: // Use hack to contol speed in voltage control mode for the etherdrive
#ifdef DEBUG
      printf("JC:: %f\n",cmdVel);
#endif
      currentVoltageCmd = cmdVel*20*60/(136*2*M_PI);
      v_backemf = joint->velocity*20*60/(136*2*M_PI);

      v_clamp_min = v_backemf - 3;// 0.655*16.7;
      v_clamp_max = v_backemf + 3;//0.655*16.7;

      k = 1.0/ 36.0;
#ifdef DEBUG
      printf("JC::%f\t%f\t%f\n", v_clamp_min, currentVoltageCmd, v_clamp_max);
#endif
      if (currentVoltageCmd > v_clamp_max)
	currentVoltageCmd = v_clamp_max;

      if (currentVoltageCmd < v_clamp_min)
	currentVoltageCmd = v_clamp_min;
      currentTorqueCmd = currentVoltageCmd * k; //Convert to match PWM conversion inside boards
      break;

    default:
      printf("JointController.cpp: Error:: invalid controlMode\n");
      currentTorqueCmd = 0;
  }
  lastTime = time;

  setJointEffort(currentTorqueCmd);
}
void JointController::update(double time)
{
  double error(0),currentTorqueCmd(0);
  double maxVelocity = cmdVel;
  double currentVoltageCmd,v_backemf,v_clamp_min,v_clamp_max,k;


   if(controlMode == controller::CONTROLLER_DISABLED){
    printf("JointController.cpp: Error:: controller disabled\n");
    return; //If we're not initialized, don't try to interact
  }

  switch(controlMode)
  {
    case CONTROLLER_TORQUE: //Pass through torque command
      currentTorqueCmd = cmdTorque;
      break;

    case CONTROLLER_POSITION: //Close the loop around position
      if(joint->type == mechanism::JOINT_ROTARY || joint->type == mechanism::JOINT_CONTINUOUS)
	error = angles::shortest_angular_distance(cmdPos, joint->position);
      else
        error = joint->position - cmdPos;
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
      break;

    case CONTROLLER_VELOCITY: //Close the loop around velocity
      if(capAccel){
        maxVelocity = getMaxVelocity(); //Check max velocity coming into wall
        if(fabs(cmdVel)>maxVelocity){
          cmdVel = -maxVelocity; //Truncate velocity smoothly
          //  std::cout<<"*******************"<<cmdVel<<std::endl;
        }
      }
      error = joint->velocity - cmdVel;
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime);
      break;
    case ETHERDRIVE_SPEED: // Use hack to contol speed in voltage control mode for the etherdrive
#ifdef DEBUG
      printf("JC:: %f\n",cmdVel);
#endif
      currentVoltageCmd = cmdVel*20*60/(136*2*M_PI);
      v_backemf = joint->velocity*20*60/(136*2*M_PI);

      v_clamp_min = v_backemf - 3;// 0.655*16.7;
      v_clamp_max = v_backemf + 3;//0.655*16.7;

      k = 1.0/ 36.0;
#ifdef DEBUG
      printf("JC::%f\t%f\t%f\n", v_clamp_min, currentVoltageCmd, v_clamp_max);
#endif
      if (currentVoltageCmd > v_clamp_max)
        currentVoltageCmd = v_clamp_max;

      if (currentVoltageCmd < v_clamp_min)
        currentVoltageCmd = v_clamp_min;
      currentTorqueCmd = currentVoltageCmd * k; //Convert to match PWM conversion inside boards
      break;

    default:
      printf("JointController.cpp: Error:: invalid controlMode\n");
      currentTorqueCmd = 0;
  }
  lastTime = time;

  setJointEffort(currentTorqueCmd);
}


controllerErrorCode JointController::setParam(std::string label,double value)
{
  return CONTROLLER_ALL_OK;
}

controllerErrorCode JointController::getParam(std::string label, double* value)
{
return CONTROLLER_ALL_OK;
}


//Truncates (if needed), then sets the effort
void JointController::setJointEffort(double effort)
{
  double newEffort;

  newEffort = effort;
  saturationFlag = false;

  if(effort >= maxEffort){
    newEffort = maxEffort;
    saturationFlag = true;
  }
  else if (effort <= minEffort) {
    newEffort = minEffort;
    saturationFlag = true;
  }
  joint->commandedEffort = newEffort;
}

controllerErrorCode JointController::loadXML(std::string filename)
{
   robot_desc::URDF model;
   int exists = 0;

   if(!model.loadFile(filename.c_str()))
      return CONTROLLER_MODE_ERROR;

   const robot_desc::URDF::Map &data = model.getMap();

   std::vector<std::string> types;
   std::vector<std::string> names;
   std::vector<std::string>::iterator iter;

   data.getMapTagFlags(types);

   for(iter = types.begin(); iter != types.end(); iter++){
      if(*iter == "controller"){
         exists = 1;
         break;
      }
   }

   if(!exists)
      return CONTROLLER_MODE_ERROR;

   exists = 0;
   data.getMapTagNames("controller",names);

   for(iter = names.begin(); iter != names.end(); iter++){
      if(*iter == this->name){
         exists = 1;
         break;
      }
   }

   if(!exists)
      return CONTROLLER_MODE_ERROR;

   paramMap = data.getMapTagValues("controller",this->name);

   loadParam("pGain",pGain);
   loadParam("dGain",dGain);
   loadParam("iGain",iGain);
   loadParam("windupMax",windupMax);
   loadParam("windupMin",windupMin);
   loadParam("maxEffort",maxEffort);
   loadParam("minEffort",minEffort);

   return  CONTROLLER_ALL_OK;
}

void JointController::loadParam(std::string label, double &param)
{
   if(paramMap.find(label) != paramMap.end()) // if parameter value has been initialized in the xml file, set internal parameter value
      param = atof(paramMap[label].c_str());
}

void JointController::loadParam(std::string label, int &param)
{
   if(paramMap.find(label) != paramMap.end())
      param = atoi(paramMap[label].c_str());
}

std::string JointController::getName()
{
  return this->name;
}

void JointController::setName(const std::string & name)
{
    this->name = name;
}

