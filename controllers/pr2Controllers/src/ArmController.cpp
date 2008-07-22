#include <pr2Controllers/ArmController.h>

using namespace CONTROLLER;
/***************************************************/
/*! \class CONTROLLER::ArmController
    \brief An overall arm controller
    
    This class implements controller loops for
    the PR2 Arm

    ASSUMES:
     7 rotary joints

    NOTES:
      Need to init all joints separately
      Modes: Position, Velocity, Torque (same as joint controllers)
      

    Parameters to be set by SetParam
    PGain
    IGain
    DGain
    IMax
    IMin

    Parameters fetched from joint
    Time
    SaturationEffort
    MaxEffort

    Steps to bring an ArmController online
      1. Initialize all joints
      2. InitArm
    
*/
/***************************************************/

//---------------------------------------------------------------------------------//
//CONSTRUCTION/DESTRUCTION CALLS
//---------------------------------------------------------------------------------//

ArmController::ArmController()
{
  //Default construction of JointControllers should init them to blank controllers
}
    
ArmController::~ArmController( )
{
}

void ArmController::initJoint(int jointNum, double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint)
{
  lowerControl[jointNum].Init(PGain,  IGain,  DGain,  IMax,  IMin,  CONTROLLER::CONTROLLER_DISABLED,  time,  maxPositiveTorque,  maxNegativeTorque,  maxEffort,  joint); //Initialize joint, but keep in disabled state 
}

CONTROLLER_ERROR_CODE ArmController::initArm(CONTROLLER_CONTROL_MODE mode)
{

  //Reset commanded positions
  for(int i =0 ;i<6;i++){
    cmdPos[i] = 0.0;
    cmdVel[i] = 0.0;
  }

  //Set mode
  controlMode = mode;

    for(int i =0;i<ARM_MAX_JOINTS;i++){
      lowerControl[i].SetMode(mode); //Set underlying jointControllers to position control
    }
  
  //Turn on the controller
  enableController();

  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//
CONTROLLER::CONTROLLER_CONTROL_MODE ArmController::setMode(CONTROLLER_CONTROL_MODE mode)
{
  for(int i = 0;i<ARM_MAX_JOINTS;i++){
    lowerControl[i].SetMode(mode);
  }
  return CONTROLLER_MODE_SET;
}

CONTROLLER::CONTROLLER_CONTROL_MODE ArmController::enableController(void)
{
  enabled = true;
 for(int i = 0;i<ARM_MAX_JOINTS;i++){
    lowerControl[i].EnableController();
  }
  return CONTROLLER_ENABLED;
}

CONTROLLER::CONTROLLER_CONTROL_MODE  ArmController::disableController(void)
{ 
  enabled = false;
 for(int i = 0;i<ARM_MAX_JOINTS;i++){
    lowerControl[i].DisableController();
  }
  return CONTROLLER_DISABLED;

}
      
    
CONTROLLER::CONTROLLER_CONTROL_MODE  ArmController::getMode(void)
{
  return controlMode;
}

void  ArmController::checkForSaturation(bool* status[])
{
  for(int i= 0;i<ARM_MAX_JOINTS;i++){
    *status[i] = lowerControl[i].CheckForSaturation();
  }
}


//---------------------------------------------------------------------------------//
//HAND CALLS
//Require forward and inverse kinematics
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE
ArmController::setHandCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandCartesianPositionCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandCartesianPositionAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER_ERROR_CODE ArmController::setHandOrientation(double roll, double pitch, double yaw)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandOrientationCmd(double *roll, double *pitch, double *yaw)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandOrientationAct(double *roll, double *pitch, double *yaw)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER_ERROR_CODE ArmController::setHandParam(std::string label, double value)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandParam(std::string label, double *value)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER_ERROR_CODE ArmController::setHandParam(std::string label, std::string value)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandParam(std::string label, std::string *value)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//ARM JOINT POSITION CALLS
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE
ArmController::setArmJointPosition(int numJoints, double angles[],double speed[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;
  if(controlMode!=CONTROLLER::CONTROLLER_POSITION) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].SetPosCmd(angles[i]);
    std::cout<<i<<":"<<current<<std::endl;
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
CONTROLLER_ERROR_CODE ArmController::getArmJointPositionCmd(int *numJoints, double *angles[],double *speed[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

   for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].GetPosCmd(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
CONTROLLER_ERROR_CODE
ArmController::getArmJointPositionAct(int *numJoints, double *angles[],double *speed[])
{
 CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
 CONTROLLER_ERROR_CODE current;

  for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].GetPosAct(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;
}

//---------------------------------------------------------------------------------//
//ARM JOINT TORQUE CALLS
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE
ArmController::setArmJointTorque(int numJoints, double torque[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;
  if(controlMode!=CONTROLLER::CONTROLLER_TORQUE) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].SetTorqueCmd(torque[i]); //If we find even a single error, complete the set position and return error for overall
     if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
CONTROLLER_ERROR_CODE ArmController::getArmJointTorqueCmd(int *numJoints, double *torque[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

   for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].GetTorqueCmd(torque[i]); //If we find even a single error,return error for overall
   if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
CONTROLLER_ERROR_CODE ArmController::getArmJointTorqueAct(int *numJoints, double *torque[])
{
 CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
 CONTROLLER_ERROR_CODE current;

  for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].GetTorqueAct(torque[i]); //If we find even a single error,return error for overall 
   if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

//---------------------------------------------------------------------------------//
//ARM JOINT VELOCITY CALLS
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE
ArmController::setArmJointSpeed(int numJoints, double speed[])
{
   CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;
  if(controlMode!=CONTROLLER::CONTROLLER_VELOCITY) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].SetVelCmd(speed[i]); //If we find even a single error, complete the set position and return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}
CONTROLLER_ERROR_CODE ArmController::getArmJointSpeedCmd(int *numJoints, double *speed[])
{ 
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

   for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].GetVelCmd(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

 }
CONTROLLER_ERROR_CODE ArmController::getArmJointSpeedAct(int *numJoints, double *speed[])
{ 
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

  for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].GetVelAct(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

//---------------------------------------------------------------------------------//
// GAZE POINT CALLS
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE
ArmController::setArmCamGazePoint(double x, double y, double z)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getArmCamGazePointCmd(double *x, double *y, double *z)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getArmCamGazePointAct(double *x, double *y, double *z)
{

  return CONTROLLER::CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
// UPDATE CALLS
//---------------------------------------------------------------------------------//

void
ArmController::Update( )
{
  if(!enabled){
    for(int i = 0;i<ARM_MAX_JOINTS;i++){    
      lowerControl[i].DisableController();
    }
  } //Make sure controller is enabled. Otherwise, tell all jointcontrollers to shut down

  for(int i = 0;i<ARM_MAX_JOINTS;i++){   
    lowerControl[i].Update();
  }
}

/*
PR2::PR2_ERROR_CODE
ArmController::setArmJointMaxTorque(int numJoints, double maxTorque[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointMaxTorqueCmd(int *numJoints, double *maxTorque[])
{

  return PR2::PR2_ALL_OK;
}
PR2::PR2_ERROR_CODE
ArmController::getArmJointMaxTorqueAct(int *numJoints, double *maxTorque[])
{

  return PR2::PR2_ALL_OK;
}
*/

