#include <genericControllers/JointController.h>

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

void JointController::Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort) {
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
  
  controlMode = mode;

  //Turn on controller
  EnableController();

}

//---------------------------------------------------------------------------------//
//TIME CALLS
//---------------------------------------------------------------------------------//


//Returns the current time. Will eventually mode to use motor board controller time TODO
PR2::PR2_ERROR_CODE JointController::GetTime(double *time){
  return PR2::PR2_ALL_OK;
//  return(myPR2->hw.GetSimTime(time));
}

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//


//Set the controller control mode
void JointController::SetMode(CONTROLLER_CONTROL_MODE mode){
  controlMode = mode;
}

//Getter for control mode
CONTROLLER_CONTROL_MODE JointController::GetMode(){
  return controlMode;
}

//Allow controller to function
void JointController::EnableController(){
  enabled = true;
}

//Disable functioning. Set joint torque to zero.
void JointController::DisableController(){
  enabled = false;
  //thisJoint->commandedEffort = 0; //Immediately set commanded Effort to 0
}

bool JointController::CheckForSaturation(void){ 
  return SaturationFlag;
}



//---------------------------------------------------------------------------------//
//TORQUE CALLS
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE JointController::SetTorqueCmd(double torque){
// double maxEffort = thisJoint->effortLimit;
  
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
  
  return CONTROLLER_ALL_OK;

}

//Return current torque command
CONTROLLER_ERROR_CODE JointController::GetTorqueCmd(double *torque)
{
  *torque = cmdTorque;
  return CONTROLLER_ALL_OK;
}

//Query motor for actual torque 
CONTROLLER_ERROR_CODE JointController::GetTorqueAct(double *torque)
{
//  *torque = thisJoint->appliedEffort; //Read torque from joint
  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//POSITION CALLS
//---------------------------------------------------------------------------------//


//Query mode, then set desired position 
CONTROLLER_ERROR_CODE
JointController::SetPosCmd(double pos)
{
  if(controlMode != CONTROLLER_POSITION)  //Make sure we're in position command mode
  return CONTROLLER_MODE_ERROR;
  
  cmdPos = pos;  
//  if(cmdPos > thisJoint->jointLimitMax){ //Truncate to positive limit
//    cmdPos = thisJoint->jointLimitMax;
//    return CONTROLLER_JOINT_LIMIT;
//  }
//  else if (cmdPos < thisJoint->jointLimitMin){ //Truncate to negative limit
//    cmdPos = thisJoint->jointLimitMin;
//    return CONTROLLER_JOINT_LIMIT;
//  }
  return CONTROLLER_ALL_OK;
  
}

//Return the current position command
CONTROLLER_ERROR_CODE JointController::GetPosCmd(double *pos)
{
  *pos = cmdPos;
  return CONTROLLER_ALL_OK;
}

//Query the joint for the actual position
CONTROLLER_ERROR_CODE JointController::GetPosAct(double *pos)
{
//  *pos = thisJoint->position;
  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//VELOCITY CALLS
//---------------------------------------------------------------------------------//
//Check mode, then set the commanded velocity
CONTROLLER_ERROR_CODE JointController::SetVelCmd(double vel)
{
  if(controlMode == CONTROLLER_VELOCITY){ //Make sure we're in velocity command mode
    cmdVel = vel;  
    return CONTROLLER_ALL_OK;
  }
  else return CONTROLLER_MODE_ERROR;
}

//Return the internally stored commanded velocity
CONTROLLER_ERROR_CODE JointController::GetVelCmd(double *vel)
{
  *vel = cmdVel;
  return CONTROLLER_ALL_OK;
}

//Query our joint for velocity
CONTROLLER_ERROR_CODE JointController::GetVelAct(double *vel)
{
//  *vel = thisJoint->velocity;
  return CONTROLLER_ALL_OK;
}
//---------------------------------------------------------------------------------//
//UPDATE CALLS
//---------------------------------------------------------------------------------//
void JointController::Update(void)
{
  double error,time,currentTorqueCmd;
  GetTime(&time); //TODO: Replace time with thisJoint->timeStep

  switch (controlMode)
  {
    case CONTROLLER_TORQUE: //Pass through torque command
      currentTorqueCmd = cmdTorque;
      break;
    case CONTROLLER_POSITION: //Close the loop around position
      //ASSUME ROTARY JOINT FOR NOW
  //    error = shortest_angular_distance(thisJoint->position, cmdPos); 
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime); 
      break;
    case CONTROLLER_VELOCITY: //Close the loop around velocity
   //   error = thisJoint->velocity - cmdVel; 
      currentTorqueCmd = pidController.UpdatePid(error,time-lastTime); 
      break;
    default: //On error (no mode), set torque to zero
      currentTorqueCmd = 0; 
      //TODO:put somekind of error here for no mode
    }

  //Make sure we're enabled. If not, send a 0 torque command
  if(enabled) SafelySetTorqueInternal(currentTorqueCmd);   
  else SafelySetTorqueInternal(0); //Send a zero command if disabled
}

//---------------------------------------------------------------------------------//
//PARAM SERVER CALLS
//---------------------------------------------------------------------------------//

//TODO: Case statement to effect changes when parameters are set here
CONTROLLER_ERROR_CODE JointController::SetParam(string label,double value)
{   
  return CONTROLLER_ALL_OK;
}

/*
CONTROLLER_ERROR_CODE
JointController::SetParam(string label,string value)
{
  return CONTROLLER_ALL_OK;
}
*/

CONTROLLER_ERROR_CODE JointController::GetParam(string label, double* value)
{
}

/*
CONTROLLER_ERROR_CODE JointController::GetParam(string label, string value)
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
  CONTROLLER_ERROR_CODE status = CONTROLLER_ALL_OK;
    
  //Read the max positive and max negative torque once
  //maxPositiveTorque = thisJoint->effortLimit;
  //maxNegativeTorque = -thisJoint->effortLimit; 

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
//  thisJoint->commandedEffort = newTorque; 
  
  return newTorque;
}

