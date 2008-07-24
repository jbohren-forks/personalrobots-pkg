#include <pr2Controllers/ArmController.h>

using namespace controller;
/***************************************************/
/*! \class controller::ArmController
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

void ArmController::init()
{
  // to be filled in here
  // initJointControllers();

}


void ArmController::initJoint(int jointNum, double PGain, double IGain, double DGain, double IMax, double IMin, controllerControlMode mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint)
{
  lowerControl[jointNum].init(PGain,  IGain,  DGain,  IMax,  IMin,  CONTROLLER_DISABLED,  time,  maxEffort,  -maxEffort, joint); //Initialize joint, but keep in disabled state 
}

controllerErrorCode ArmController::initArm(controllerControlMode mode)
{

  //Reset commanded positions
  for(int i =0 ;i<6;i++){
    cmdPos[i] = 0.0;
    cmdVel[i] = 0.0;
  }

  //Set mode
  controlMode = mode;

    for(int i =0;i<ARM_MAX_JOINTS;i++){
      lowerControl[i].setMode(mode); //Set underlying jointControllers to position control
    }
  
  //Turn on the controller
  enableController();

  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//MODE/ENABLE CALLS
//---------------------------------------------------------------------------------//
controllerControlMode ArmController::setMode(controllerControlMode mode)
{
  for(int i = 0;i<ARM_MAX_JOINTS;i++){
    lowerControl[i].setMode(mode);
  }
  return CONTROLLER_MODE_SET;
}

controllerControlMode ArmController::enableController(void)
{
  enabled = true;
 for(int i = 0;i<ARM_MAX_JOINTS;i++){
    lowerControl[i].enableController();
  }
  return CONTROLLER_ENABLED;
}

controllerControlMode  ArmController::disableController(void)
{ 
  enabled = false;
 for(int i = 0;i<ARM_MAX_JOINTS;i++){
    lowerControl[i].disableController();
  }
  return CONTROLLER_DISABLED;

}
      
    
controllerControlMode  ArmController::getMode(void)
{
  return controlMode;
}

void  ArmController::checkForSaturation(bool* status[])
{
  for(int i= 0;i<ARM_MAX_JOINTS;i++){
    *status[i] = lowerControl[i].checkForSaturation();
  }
}


//---------------------------------------------------------------------------------//
//HAND CALLS
//Require forward and inverse kinematics
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setHandCartesianPosition(double x, double y, double z, double roll, double pitch, double yaw)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandCartesianPositionCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandCartesianPositionAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{

  return CONTROLLER_ALL_OK;
}

controllerErrorCode ArmController::setHandOrientation(double roll, double pitch, double yaw)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandOrientationCmd(double *roll, double *pitch, double *yaw)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandOrientationAct(double *roll, double *pitch, double *yaw)
{

  return CONTROLLER_ALL_OK;
}

controllerErrorCode ArmController::setHandParam(std::string label, double value)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandParam(std::string label, double *value)
{

  return CONTROLLER_ALL_OK;
}

controllerErrorCode ArmController::setHandParam(std::string label, std::string value)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandParam(std::string label, std::string *value)
{

  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//ARM JOINT POSITION CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmJointPosition(int numJoints, double angles[],double speed[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;
  if(controlMode!=CONTROLLER_POSITION) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].setPosCmd(angles[i]);
    std::cout<<i<<":"<<current<<std::endl;
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
controllerErrorCode ArmController::getArmJointPositionCmd(int *numJoints, double *angles[],double *speed[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].getPosCmd(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
controllerErrorCode
ArmController::getArmJointPositionAct(int *numJoints, double *angles[],double *speed[])
{
 controllerErrorCode error = CONTROLLER_ALL_OK;
 controllerErrorCode current;

  for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].getPosAct(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;
}

//---------------------------------------------------------------------------------//
//ARM JOINT TORQUE CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmJointTorque(int numJoints, double torque[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;
  if(controlMode!=CONTROLLER_TORQUE) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].setTorqueCmd(torque[i]); //If we find even a single error, complete the set position and return error for overall
     if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
controllerErrorCode ArmController::getArmJointTorqueCmd(int *numJoints, double *torque[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].getTorqueCmd(torque[i]); //If we find even a single error,return error for overall
   if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
controllerErrorCode ArmController::getArmJointTorqueAct(int *numJoints, double *torque[])
{
 controllerErrorCode error = CONTROLLER_ALL_OK;
 controllerErrorCode current;

  for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].getTorqueAct(torque[i]); //If we find even a single error,return error for overall 
   if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

//---------------------------------------------------------------------------------//
//ARM JOINT VELOCITY CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmJointSpeed(int numJoints, double speed[])
{
   controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;
  if(controlMode!=CONTROLLER_VELOCITY) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].setVelCmd(speed[i]); //If we find even a single error, complete the set position and return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}
controllerErrorCode ArmController::getArmJointSpeedCmd(int *numJoints, double *speed[])
{ 
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].getVelCmd(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

 }
controllerErrorCode ArmController::getArmJointSpeedAct(int *numJoints, double *speed[])
{ 
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

  for (int i = 0;i<*numJoints;i++){
    current = lowerControl[i].getVelAct(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

//---------------------------------------------------------------------------------//
// GAZE POINT CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmCamGazePoint(double x, double y, double z)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getArmCamGazePointCmd(double *x, double *y, double *z)
{

  return CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getArmCamGazePointAct(double *x, double *y, double *z)
{

  return CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
// UPDATE CALLS
//---------------------------------------------------------------------------------//

void
ArmController::Update( )
{
  if(!enabled){
    for(int i = 0;i<ARM_MAX_JOINTS;i++){    
      lowerControl[i].disableController();
    }
  } //Make sure controller is enabled. Otherwise, tell all jointcontrollers to shut down

  for(int i = 0;i<ARM_MAX_JOINTS;i++){   
    lowerControl[i].update();
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

