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

#include <pr2Controllers/ArmController.h> 
#define NDOF 6 // accounts for x,y,z,roll,pitch,yaw
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
  this->robot = NULL;
  this->name  = "armController";

  this->armJointControllers = new JointController[ARM_MAX_JOINTS];
}
ArmController::ArmController(mechanism::Robot *robot)
{
  this->robot = robot;
  this->name  = "armController";

  this->armJointControllers = new JointController[ARM_MAX_JOINTS];
}
ArmController::ArmController(mechanism::Robot *robot, std::string name)
{
  this->robot = robot;
  this->name  = name;

  this->armJointControllers = new JointController[ARM_MAX_JOINTS];
}
     
ArmController::ArmController(mechanism::Robot *robot, std::string name,int armNumJoints, int jcToRobotJointMap[], int jcToHIActuatorMap[])
{
  this->robot = robot;
  this->name  = "armController";
 
  this->armJointControllers = new JointController[ARM_MAX_JOINTS];
  init(jcToRobotJointMap);
}

ArmController::~ArmController()
{
}


controllerErrorCode ArmController::loadXML(std::string filename)
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

  for(iter = names.begin(); iter != names.end(); iter++){
    if(*iter == this->name){
      exists = 1;
      break;
    }
  }
  paramMap = data.getDataTagValues("controller",this->name);   

  loadParam("pGain",pGain);
  printf("BC:: %f\n",pGain);
  loadParam("dGain",dGain);
  loadParam("iGain",iGain);

  loadParam("maxXDot",maxXDot);
  loadParam("maxYDot",maxYDot);
  loadParam("maxYawDot",maxYawDot);

  return CONTROLLER_ALL_OK;
}


void ArmController::init(int jcToRobotJointMap[])
{
  // to be filled in here
  static int defaultMap[ARM_MAX_JOINTS] = {0,1,2,3,4,5,6}; //Specify default mapping
  if(jcToRobotJointMap!=NULL)  initJointControllers(jcToRobotJointMap);
  else initJointControllers(defaultMap);

}


void ArmController::initJointControllers( int jcToRobotJointMap[])
{

  /**************************************************************************************/
  /*                                                                                    */
  /* MAPPING BETWEEN: armController->jointController array and robot->joint array       */
  /*                                                                                    */
  /* note: this is how the arm's joint controller arrays knows when robot->joint        */
  /*       to use                                                                       */
  /*                                                                                    */
  /**************************************************************************************/

  for (int ii=0; ii < ARM_MAX_JOINTS; ii++) {
    // FIXME: hardcoded values for gazebo for now
    //armJointControllers[ii].init(pGain, iGain, dGain, iMax, iMin,                ETHERDRIVE_SPEED, getTime(), maxEffort, minEffort, &(robot->joint[ii]));
  //  armJointControllers[ii].init( 1000,     0,     0,  500, -500, controller::CONTROLLER_TORQUE, getTime(),      1000,     -1000, &(robot->joint[jcToRobotJointMap[ii]]));
//    printf("Index: %u\n",jcToRobotJointMap[ii]);
//    armJointControllers[ii].enableController();
    }
  double time = getTime();
  //Explicitly initialize the controllers we wish to use. Don't forget to set the controllers to torque mode in the world file! 
  initJoint(0,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100, &(robot->joint[jcToRobotJointMap[0]]));
  initJoint(1,1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,1000,-1000, &(robot->joint[jcToRobotJointMap[1]]));

  initJoint(2,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100, &(robot->joint[jcToRobotJointMap[2]]));

  initJoint(3,300,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100, &(robot->joint[jcToRobotJointMap[3]]));

  initJoint(4,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100, &(robot->joint[jcToRobotJointMap[4]]));

  initJoint(5,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100, &(robot->joint[jcToRobotJointMap[5]]));
  initJoint(6,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100, &(robot->joint[jcToRobotJointMap[6]]));


  

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize controllers                                   */
  /*           hardcoded values, should be replaced by reading controllers.xml file      */
  /*         TODO: assigned by xml fils and param servers                                */
  /*                                                                                     */
  /***************************************************************************************/
#if 0
  // initialize each jointController jc[i], associate with a joint joint[i]
  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    jc[i]  = new controller::JointController();
    jc[i]->init(joint[i]);
  }

  //Explicitly initialize the controllers we wish to use. Don't forget to set the controllers to torque mode in the world file! 
  armJointController[PR2::ARM_L_PAN]->initJoint(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_PAN]);
  armJointController[PR2::ARM_L_SHOULDER_PITCH]->init(1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,-1000,1000,joint[PR2::ARM_L_SHOULDER_PITCH]);
  armJointController[PR2::ARM_L_SHOULDER_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_SHOULDER_ROLL]);
  armJointController[PR2::ARM_L_ELBOW_PITCH]->init(300,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_ELBOW_PITCH]);
  armJointController[PR2::ARM_L_ELBOW_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_ELBOW_ROLL]);
  armJointController[PR2::ARM_L_WRIST_PITCH]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_WRIST_PITCH]);
  armJointController[PR2::ARM_L_WRIST_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_WRIST_ROLL]);

  armJointController[PR2::ARM_R_PAN]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_PAN]);
  armJointController[PR2::ARM_R_SHOULDER_PITCH]->init(1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,-1000,1000,joint[PR2::ARM_R_SHOULDER_PITCH]);
  armJointController[PR2::ARM_R_SHOULDER_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_SHOULDER_ROLL]);
  armJointController[PR2::ARM_R_ELBOW_PITCH]->init(300,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_ELBOW_PITCH]);
  armJointController[PR2::ARM_R_ELBOW_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_ELBOW_ROLL]);
  armJointController[PR2::ARM_R_WRIST_PITCH]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_WRIST_PITCH]);
  armJointController[PR2::ARM_R_WRIST_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_WRIST_ROLL]);
#endif
}

double ArmController::getTime()
{
  struct timeval t;
  gettimeofday( &t, 0);
  return (double) (t.tv_usec *1e-6 + t.tv_sec);
}

void ArmController::initJoint(int jointNum, double PGain, double IGain, double DGain, double IMax, double IMin, controllerControlMode mode, double time, double maxEffort,double minEffort, mechanism::Joint *joint)
{
  armJointControllers[jointNum].init(PGain,  IGain,  DGain,  IMax,  IMin,   CONTROLLER_DISABLED,  time,   maxEffort, minEffort, joint); //Initialize joint, but keep in disabled state 
}

controllerErrorCode ArmController::initArm(controllerControlMode mode)
{

  //Reset commanded positions
  for(int i =0 ;i< NDOF ;i++){
    cmdPos[i] = 0.0;
    cmdVel[i] = 0.0;
  }

  //Set mode
  controlMode = mode;
    for(int i =0;i<ARM_MAX_JOINTS;i++){
      armJointControllers[i].setMode(mode); //Set underlying jointControllers to position control
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
    armJointControllers[i].setMode(mode);
  }
  return CONTROLLER_MODE_SET;
}

 controllerControlMode ArmController::enableController(void)
{
  enabled = true;
 for(int i = 0;i<ARM_MAX_JOINTS;i++){
    armJointControllers[i].enableController();
  }
  return CONTROLLER_ENABLED;
}

 controllerControlMode  ArmController::disableController(void)
{ 
  enabled = false;
 for(int i = 0;i<ARM_MAX_JOINTS;i++){
    armJointControllers[i].disableController();
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
    *status[i] = armJointControllers[i].checkForSaturation();
  }
}


//---------------------------------------------------------------------------------//
//HAND CALLS
//Require forward and inverse kinematics
//---------------------------------------------------------------------------------//

controllerErrorCode ArmController::setHandCartesianPosLinear(double x, double y, double z, double roll, double pitch, double yaw, double timestep, double stepSize)
{

  double pos;
  //Set up controller
  setMode(CONTROLLER_POSITION);
  this->timeStep = timestep;
  this->stepSize = stepSize;

  //Calculate end position
  KDL::Vector endPos(x,y,z);
  KDL::Rotation endRot;
  endRot = endRot.RPY(roll,pitch,yaw);
  endFrame.p = endPos;
  endFrame.M = endRot;  

  //Get current location
  KDL::JntArray q = KDL::JntArray(pr2_kin.nJnts);

  //Get joint positions
  for(int i=0;i<ARM_MAX_JOINTS;i++){   
    armJointControllers[i].getPosAct(&pos);
    q(i) = pos;
  }

  KDL::Frame f;
  pr2_kin.FK(q,f);

  startPosition = f.p; //Record Starting location
  KDL::Vector move = endPos-startPosition; //Vector to ending position

  double dist = move.Norm(); //Distance to move
  moveDirection = move/dist; //Normalize movement vector
    
  rotInterpolator.SetStartEnd(f.M, endRot);
  double total_angle = rotInterpolator.Angle();

  nSteps = (int)(dist/stepSize);
  if(nSteps==0){
    std::cout<<"ArmController.cpp: Error:: number of steps calculated to be 0"<<std::endl;
    return CONTROLLER_COMPUTATION_ERROR;
  } 
  angleStep = total_angle/nSteps;
  lastTime= getTime(); //Record first time marker
  stepIndex = 0; //Reset step index

  linearInterpolation = true;
  return CONTROLLER_ALL_OK;
}

controllerErrorCode
ArmController::setHandCartesianPos(double x, double y, double z, double roll, double pitch, double yaw)
{	
  linearInterpolation = false;
  
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
    
  return commandCartesianPos(f);
 }

controllerErrorCode ArmController::commandCartesianPos(KDL::Frame f)
{
   //Create joint arrays
	KDL::JntArray q_init(ARM_MAX_JOINTS);
	KDL::JntArray q_out(ARM_MAX_JOINTS);
  
  //Use zero values for initialization
  for(int i = 0;i<getNumJoints();i++){
    q_init(i)= 0.0;
  }

  //Perform inverse kinematics
  if (this->pr2_kin.IK(q_init, f, q_out)){
   // cout<<"IK result:"<<q_out<<endl;
  }else{ 
    //cout<<"Could not compute Inv Kin."<<endl;
    return  CONTROLLER_JOINT_ERROR; 
  }

   //------ checking that IK returned a valid soln -----
  KDL::Frame f_ik;
  if (this->pr2_kin.FK(q_out,f_ik))
  {
    //    cout<<"End effector after IK:"<<f_ik<<endl;
  }
  else{
   // cout<<"Could not compute Fwd Kin. (After IK)"<<endl;
    return  CONTROLLER_JOINT_ERROR;
  }

  //Record commands and shove to arm
  for(int ii = 0; ii < ARM_MAX_JOINTS; ii++){
    armJointControllers[ii].setPosCmd(q_out(ii));
    //std::cout<<"*"<<q_out(ii);
  }
  //std::cout<<std::endl;
  return  CONTROLLER_ALL_OK;


}

controllerErrorCode ArmController::getHandCartesianPosCmd(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
  *x     = cmdPos[0];
  *y     = cmdPos[1];
  *z     = cmdPos[2];
  *roll  = cmdPos[3];
  *pitch = cmdPos[4];
  *yaw   = cmdPos[5];
  return  CONTROLLER_ALL_OK;
}
controllerErrorCode ArmController::getHandCartesianPosAct(double *x, double *y, double *z, double *roll, double *pitch, double *yaw)
{
  double pos;
  KDL::JntArray q_jts(ARM_MAX_JOINTS);
  //Read current joint locations
  for(int i = 0;i<ARM_MAX_JOINTS;i++){
   armJointControllers[i].getPosAct(&pos);
    q_jts(i) = pos;
  }

  //Perform forward kinematics to get cartesian location
  KDL::Frame fk;
  if (this->pr2_kin.FK(q_jts,fk))
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
  *roll  = cmdPos[3];
  *pitch = cmdPos[4];
  *yaw   = cmdPos[5];

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
    current = armJointControllers[i].setPosCmd(angles[i]);
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
    current = armJointControllers[i].getPosCmd(angles[i]); //If we find even a single error,return error for overall
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
    current = armJointControllers[i].getPosAct(angles[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;
}

controllerErrorCode ArmController::setOneArmJointPos(int numJoint, double angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].setPosCmd(angle);
}
 
controllerErrorCode ArmController::getOneArmJointPosCmd(int numJoint, double *angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].getPosCmd(angle);
}

controllerErrorCode ArmController::getOneArmJointPosAct(int numJoint, double *angle)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].getPosAct(angle);
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
    current = armJointControllers[i].setTorqueCmd(torque[i]); //If we find even a single error, complete the set position and return error for overall
     if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
controllerErrorCode ArmController::getArmJointTorqueCmd( double *torque[])
{
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<getNumJoints();i++){
    current = armJointControllers[i].getTorqueCmd(torque[i]); //If we find even a single error,return error for overall
   if(current!=CONTROLLER_ALL_OK) error = current;

  }  
  return error;

}
controllerErrorCode ArmController::getArmJointTorqueAct( double *torque[])
{
 controllerErrorCode error = CONTROLLER_ALL_OK;
 controllerErrorCode current;

  for (int i = 0;i<getNumJoints();i++){
    current = armJointControllers[i].getTorqueAct(torque[i]); //If we find even a single error,return error for overall 
   if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

controllerErrorCode ArmController::setOneArmJointTorque(int numJoint,double torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].setTorqueCmd(torque);
}


controllerErrorCode ArmController::getArmJointTorqueCmd(int numJoint, double *torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].getTorqueCmd(torque);

}

controllerErrorCode ArmController::getArmJointTorqueAct(int numJoint, double *torque)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].getTorqueAct(torque);

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
    current = armJointControllers[i].setVelCmd(speed[i]); //If we find even a single error, complete the set position and return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}
controllerErrorCode ArmController::getArmJointSpeedCmd( double *speed[])
{ 
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

   for (int i = 0;i<getNumJoints();i++){
    current = armJointControllers[i].getVelCmd(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

 }
controllerErrorCode ArmController::getArmJointSpeedAct( double *speed[])
{ 
  controllerErrorCode error = CONTROLLER_ALL_OK;
  controllerErrorCode current;

  for (int i = 0;i<getNumJoints();i++){
    current = armJointControllers[i].getVelAct(speed[i]); //If we find even a single error,return error for overall
    if(current!=CONTROLLER_ALL_OK) error = current;
  }  
  return error;

}

controllerErrorCode ArmController::setOneArmJointSpeed(int numJoint, double speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].setVelCmd(speed);
}

controllerErrorCode ArmController::getOneArmJointSpeedCmd(int numJoint, double *speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].getVelCmd(speed);
}
controllerErrorCode ArmController::getOneArmJointSpeedAct(int numJoint, double *speed)
{
  if(numJoint<0 || numJoint > ARM_MAX_JOINTS) return CONTROLLER_JOINT_ERROR; //Index out of bounds
  return armJointControllers[numJoint].getVelAct(speed);

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
  KDL::Frame f;
  double time;

  if(!enabled){
    for(int i = 0;i<ARM_MAX_JOINTS;i++){    
      armJointControllers[i].disableController();
    }
  } //Make sure controller is enabled. Otherwise, tell all jointcontrollers to shut down


  if(linearInterpolation)
  {
    //Perform linearly interpolated motion 
      time = getTime();
    double deltaT = time-lastTime; 
    if(deltaT>= timeStep) { //If enough time has elapsed for next set point, increment motion
      if(stepIndex >= nSteps){ //We're done. Reset motion
        //std::cout<<"Finished motion"<<std::endl;
        linearInterpolation = false;
        commandCartesianPos(endFrame);
        stepIndex = 0;
        stepSize=0;
        nSteps=0;
        timeStep=0;
        lastTime = 0;
      } else {        
        //std::cout<<"Anglestep:"<<angleStep<<" timeStep:"<<timeStep<<" nSteps:"<<nSteps<<" stepSize: "<<stepSize<<" stepIndex:"<<stepIndex<<std::endl;
        lastTime = time;
        f.p = startPosition+(stepIndex+1)*moveDirection*stepSize;
        f.M = rotInterpolator.Pos(angleStep*(stepIndex+1));
        commandCartesianPos(f); //Command new location
        stepIndex++; 
      }
    }
  }
  
  //Servo to commanded location
  for(int i = 0;i<ARM_MAX_JOINTS;i++){   
    armJointControllers[i].update();
  }
  
}

void ArmController::loadParam(std::string label, double &param)
{
  if(paramMap.find(label) != paramMap.end()) // if parameter value has been initialized in the xml file, set internal parameter value
    param = atof(paramMap[label].c_str());
}

void ArmController::loadParam(std::string label, int &param)
{
  if(paramMap.find(label) != paramMap.end())
    param = atoi(paramMap[label].c_str());
}

//---------------------------------------------------------------------------------//
// MISC CALLS
//---------------------------------------------------------------------------------//
int ArmController::getNumJoints()
{
  return pr2_kin.nJnts;
}

