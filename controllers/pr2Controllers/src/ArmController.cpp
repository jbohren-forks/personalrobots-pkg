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

CONTROLLER_ERROR_CODE ArmController::initArm(CONTROLLER_CONTROL_MODE mode, PR2::PR2Robot* robot)
{
  //Record the robot pointer for kinematics
  this->robot = robot;

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
  if (robot->pr2_kin.IK(q_init, f, q_out)){
   // cout<<"IK result:"<<q_out<<endl;
 }else{ 
    //cout<<"Could not compute Inv Kin."<<endl;
    return CONTROLLER::CONTROLLER_JOINT_ERROR; 
  }

   //------ checking that IK returned a valid soln -----
  KDL::Frame f_ik;
  if (robot->pr2_kin.FK(q_out,f_ik))
  {
    //    cout<<"End effector after IK:"<<f_ik<<endl;
  }
  else{
   // cout<<"Could not compute Fwd Kin. (After IK)"<<endl;
    return CONTROLLER::CONTROLLER_JOINT_ERROR;
  }

  //Record commands and shove to arm
  for(int ii = 0; ii < ARM_MAX_JOINTS; ii++){
    lowerControl[ii].SetPosCmd(q_out(ii));
    //std::cout<<"*"<<q_out(ii);
  }
  //std::cout<<std::endl;
  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandCartesianPosCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
  *x = cmdPos[0];
  *y = cmdPos[1];
  *z = cmdPos[2];
  *roll = cmdPos[3];
  *pitch = cmdPos[4];
  *yaw = cmdPos[5];
  return CONTROLLER::CONTROLLER_ALL_OK;
}
CONTROLLER_ERROR_CODE ArmController::getHandCartesianPosAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
  double pos;
  KDL::JntArray q_jts(ARM_MAX_JOINTS);
  //Read current joint locations
  for(int i = 0;i<ARM_MAX_JOINTS;i++){
   lowerControl[i].GetPosAct(&pos);
    q_jts(i) = pos;
  }

  //Perform forward kinematics to get cartesian location
  KDL::Frame fk;
  if (robot->pr2_kin.FK(q_jts,fk))
  {
    //    cout<<"End effector:"<<fk<<endl;
  }
  else{
   // cout<<"Could not compute Fwd Kin. "<<endl;
    return CONTROLLER::CONTROLLER_JOINT_ERROR;
  }

  *x = fk.p.x();
  *y = fk.p.y();
  *z = fk.p.z();
  fk.M.GetRPY(*roll,*pitch,*yaw);

  return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER_ERROR_CODE ArmController::setHandOrientation(double roll, double pitch, double yaw)
{
  //Get the current commanded hand orientation
  double x,y,z,currentroll,currentpitch,currentyaw;
  this->getHandCartesianPosCmd(&x,&y,&z,&roll,&pitch,&yaw);
  return this->setHandCartesianPos(x,y,z,roll,pitch,yaw);
}

CONTROLLER_ERROR_CODE ArmController::getHandOrientationCmd(double *roll, double *pitch, double *yaw)
{
  *roll = cmdPos[3];
  *pitch = cmdPos[4];
  *yaw = cmdPos[5];

  return CONTROLLER::CONTROLLER_ALL_OK;
}

CONTROLLER_ERROR_CODE ArmController::getHandOrientationAct(double *roll, double *pitch, double *yaw)
{ 
  double x,y,z;
  return this->getHandCartesianPosAct(&x,&y,&z,roll,pitch,yaw);
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
//
//---------------------------------------------------------------------------------//
CONTROLLER_ERROR_CODE
ArmController::setArmJointPos(int numJoints, double angles[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;
  if(controlMode!=CONTROLLER::CONTROLLER_POSITION) return CONTROLLER_MODE_ERROR; //TODO implement errors?
  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].SetPosCmd(angles[i]);
//    std::cout<<i<<":"<<current<<std::endl;
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
CONTROLLER_ERROR_CODE ArmController::getArmJointPosCmd(int numJoints, double *angles[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

   for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].GetPosCmd(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;
}
CONTROLLER_ERROR_CODE
ArmController::getArmJointPosAct(int numJoints, double *angles[])
{
 CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
 CONTROLLER_ERROR_CODE current;

  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].GetPosAct(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;
}

CONTROLLER_ERROR_CODE ArmController::setOneArmJointPos(int numJoint, double angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].SetPosCmd(angle);
}
 
CONTROLLER_ERROR_CODE ArmController::getOneArmJointPosCmd(int numJoint, double *angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].GetPosCmd(angle);
}

CONTROLLER_ERROR_CODE ArmController::getOneArmJointPosAct(int numJoint, double *angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].GetPosAct(angle);
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
CONTROLLER_ERROR_CODE ArmController::getArmJointTorqueCmd(int numJoints, double *torque[])
{
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

   for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].GetTorqueCmd(torque[i]); //If we find even a single error,return error for overall
   if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
CONTROLLER_ERROR_CODE ArmController::getArmJointTorqueAct(int numJoints, double *torque[])
{
 CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
 CONTROLLER_ERROR_CODE current;

  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].GetTorqueAct(torque[i]); //If we find even a single error,return error for overall 
   if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

CONTROLLER_ERROR_CODE ArmController::setOneArmJointTorque(int numJoint,double torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].SetTorqueCmd(torque);
}


CONTROLLER_ERROR_CODE ArmController::getArmJointTorqueCmd(int numJoint, double *torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].GetTorqueCmd(torque);

}

CONTROLLER_ERROR_CODE ArmController::getArmJointTorqueAct(int numJoint, double *torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].GetTorqueAct(torque);

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
CONTROLLER_ERROR_CODE ArmController::getArmJointSpeedCmd(int numJoints, double *speed[])
{ 
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

   for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].GetVelCmd(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

 }
CONTROLLER_ERROR_CODE ArmController::getArmJointSpeedAct(int numJoints, double *speed[])
{ 
  CONTROLLER_ERROR_CODE error = CONTROLLER_ALL_OK;
  CONTROLLER_ERROR_CODE current;

  for (int i = 0;i<numJoints;i++){
    current = lowerControl[i].GetVelAct(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

CONTROLLER_ERROR_CODE ArmController::setOneArmJointSpeed(int numJoint, double speed){
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].SetVelCmd(speed);
}

CONTROLLER_ERROR_CODE ArmController::getOneArmJointSpeedCmd(int numJoint, double *speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].GetVelCmd(speed);
}
CONTROLLER_ERROR_CODE ArmController::getOneArmJointSpeedAct(int numJoint, double *speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return lowerControl[numJoint].GetVelAct(speed);

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


