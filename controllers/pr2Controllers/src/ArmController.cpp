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


void ArmController::initJoint(int jointNum, double PGain, double IGain, double DGain, double IMax, double IMin, controllerControlMode mode, double time, double maxEffort,double minEffort, mechanism::Joint *joint)
{
  lowerControl[jointNum].init(PGain,  IGain,  DGain,  IMax,  IMin,   CONTROLLER_DISABLED,  time,   maxEffort, minEffort, joint); //Initialize joint, but keep in disabled state 
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
ArmController::setHandCartesianPos(double x, double y, double z, double roll, double pitch, double yaw)
{	
  //Define position and rotation
  KDL::Vector position(x,y,z);
  KDL::Rotation rotation;
  rotation = rotation.RPY(roll,pitch,yaw);
  
  cmdPos[0] = x;
  cmdPos[1] = y;
  cmdPos[2] = z;
  cmdPos[3] = roll;
  cmdPos[4] = pitch;
  cmdPos[5] = yaw;

  //Create frame based on position and rotation
  KDL::Frame f;
  f.p = position;
  f.M = rotation;
  
  //Create joint arrays
	KDL::JntArray q_init(ARM_MAX_JOINTS);
	KDL::JntArray q_out(ARM_MAX_JOINTS);
  
  //Use zero values for initialization
  for(int i = 0;i<ARM_MAX_JOINTS;i++){
    q_init(i)= 0.0;
  }

  //Perform inverse kinematics
  if (pr2_kin.IK(q_init, f, q_out)){
   // cout<<"IK result:"<<q_out<<endl;
 }else{ 
    //cout<<"Could not compute Inv Kin."<<endl;
    return  CONTROLLER_JOINT_ERROR; 
  }

   //------ checking that IK returned a valid soln -----
  KDL::Frame f_ik;
  if (pr2_kin.FK(q_out,f_ik))
  {
    //    cout<<"End effector after IK:"<<f_ik<<endl;
  }
  else{
   // cout<<"Could not compute Fwd Kin. (After IK)"<<endl;
    return  CONTROLLER_JOINT_ERROR;
  }

  //Record commands and shove to arm
  for(int ii = 0; ii < ARM_MAX_JOINTS; ii++){
    lowerControl[ii].setPosCmd(q_out(ii));
    //std::cout<<"*"<<q_out(ii);
  }
  //std::cout<<std::endl;
  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandCartesianPosCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
  *x = cmdPos[0];
  *y = cmdPos[1];
  *z = cmdPos[2];
  *roll = cmdPos[3];
  *pitch = cmdPos[4];
  *yaw = cmdPos[5];
  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandCartesianPosAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
  double pos;
  KDL::JntArray q_jts(ARM_MAX_JOINTS);
  //Read current joint locations
  for(int i = 0;i<ARM_MAX_JOINTS;i++){
   lowerControl[i].getPosAct(&pos);
    q_jts(i) = pos;
  }

  //Perform forward kinematics to get cartesian location
  KDL::Frame fk;
  if (pr2_kin.FK(q_jts,fk))
  {
    //    cout<<"End effector:"<<fk<<endl;
  }
  else{
   // cout<<"Could not compute Fwd Kin. "<<endl;
    return  CONTROLLER_JOINT_ERROR;
  }

  *x = fk.p.x();
  *y = fk.p.y();
  *z = fk.p.z();
  fk.M.GetRPY(*roll,*pitch,*yaw);

  return  CONTROLLER_ALL_OK;
}

controllerErrorCode ArmController::setHandOrientation(double roll, double pitch, double yaw)
{
  //Get the current commanded hand orientation
  double x,y,z;
  this->getHandCartesianPosCmd(&x,&y,&z,&roll,&pitch,&yaw);
  return this->setHandCartesianPos(x,y,z,roll,pitch,yaw);
}

controllerErrorCode ArmController::getHandOrientationCmd(double *roll, double *pitch, double *yaw)
{
  *roll = cmdPos[3];
  *pitch = cmdPos[4];
  *yaw = cmdPos[5];

  return  CONTROLLER_ALL_OK;
}

controllerErrorCode ArmController::getHandOrientationAct(double *roll, double *pitch, double *yaw)
{ 
  double x,y,z;
  return this->getHandCartesianPosAct(&x,&y,&z,roll,pitch,yaw);
}

controllerErrorCode ArmController::setHandParam(std::string label, double value)
{

  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandParam(std::string label, double *value)
{

  return  CONTROLLER_ALL_OK;
}

controllerErrorCode ArmController::setHandParam(std::string label, std::string value)
{

  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandParam(std::string label, std::string *value)
{

  return  CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
//ARM JOINT POSITION CALLS
//
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmJointPos( double angles[], double speeds[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;
  if(controlMode!= CONTROLLER_POSITION) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].setPosCmd(angles[i]);
//    std::cout<<i<<":"<<current<<std::endl;
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
controllerErrorCode ArmController::getArmJointPosCmd( double *angles[], double *speeds[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].getPosCmd(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
controllerErrorCode
ArmController::getArmJointPosAct( double *angles[], double *speeds[])
{
 controllerErrorCode error = CONTROLLER_ALL_OK;
 controllerErrorCode current;

  for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].getPosAct(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;
}

controllerErrorCode ArmController::setOneArmJointPos(int numJoint, double angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].setPosCmd(angle);
}
 
controllerErrorCode ArmController::getOneArmJointPosCmd(int numJoint, double *angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].getPosCmd(angle);
}

controllerErrorCode ArmController::getOneArmJointPosAct(int numJoint, double *angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].getPosAct(angle);
}

//---------------------------------------------------------------------------------//
//ARM JOINT TORQUE CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmJointTorque( double torque[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;
  if(controlMode!= CONTROLLER_TORQUE) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].setTorqueCmd(torque[i]); //If we find even a single error, complete the set position and return error for overall
     if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
controllerErrorCode ArmController::getArmJointTorqueCmd( double *torque[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].getTorqueCmd(torque[i]); //If we find even a single error,return error for overall
   if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
controllerErrorCode ArmController::getArmJointTorqueAct( double *torque[])
{
 controllerErrorCode error = CONTROLLER_ALL_OK;
 controllerErrorCode current;

  for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].getTorqueAct(torque[i]); //If we find even a single error,return error for overall 
   if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

controllerErrorCode ArmController::setOneArmJointTorque(int numJoint,double torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].setTorqueCmd(torque);
}


controllerErrorCode ArmController::getArmJointTorqueCmd(int numJoint, double *torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].getTorqueCmd(torque);

}

controllerErrorCode ArmController::getArmJointTorqueAct(int numJoint, double *torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].getTorqueAct(torque);

}


//---------------------------------------------------------------------------------//
//ARM JOINT VELOCITY CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmJointSpeed( double speed[])
{
   controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;
  if(controlMode!= CONTROLLER_VELOCITY) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].setVelCmd(speed[i]); //If we find even a single error, complete the set position and return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}
controllerErrorCode ArmController::getArmJointSpeedCmd( double *speed[])
{ 
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].getVelCmd(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

 }
controllerErrorCode ArmController::getArmJointSpeedAct( double *speed[])
{ 
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

  for (int i = 0;i<getNumJoints();i++){
    current = lowerControl[i].getVelAct(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

controllerErrorCode ArmController::setOneArmJointSpeed(int numJoint, double speed){
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].setVelCmd(speed);
}

controllerErrorCode ArmController::getOneArmJointSpeedCmd(int numJoint, double *speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].getVelCmd(speed);
}
controllerErrorCode ArmController::getOneArmJointSpeedAct(int numJoint, double *speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].getVelAct(speed);

}

//---------------------------------------------------------------------------------//
// GAZE POINT CALLS
//---------------------------------------------------------------------------------//
controllerErrorCode
ArmController::setArmCamGazePoint(double x, double y, double z)
{

  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getArmCamGazePointCmd(double *x, double *y, double *z)
{

  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getArmCamGazePointAct(double *x, double *y, double *z)
{

  return  CONTROLLER_ALL_OK;
}

//---------------------------------------------------------------------------------//
// UPDATE CALLS
//---------------------------------------------------------------------------------//

void
ArmController::update( )
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


//---------------------------------------------------------------------------------//
// MISC CALLS
//---------------------------------------------------------------------------------//
int ArmController::getNumJoints(){
    return ARM_MAX_JOINTS;
}

