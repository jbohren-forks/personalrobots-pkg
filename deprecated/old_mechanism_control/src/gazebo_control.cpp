
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Willow Garage Inc. nor the names of its
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

#include <mechanism_control/gazebo_control.h>
#include <signal.h>
#include <sys/time.h>

#include <pr2Core/pr2Core.h> // this should be replaced by xmls

const double maxPositiveTorque = 10000;

using namespace std;

GazeboMechanismControl::GazeboMechanismControl(){
  this->hardwareInterface = NULL;
}

GazeboMechanismControl::~GazeboMechanismControl(){
  this->hardwareInterface = NULL;
}


void GazeboMechanismControl::init(HardwareInterface *hardwareInterface){

  this->hardwareInterface = hardwareInterface;

  this->initRobot();

  this->initControllers();
}

void GazeboMechanismControl::initRobot(){
  r = new mechanism::Robot((char*)"robot");

  r->numJoints        = PR2::MAX_JOINTS;
  r->numLinks         = PR2::MAX_JOINTS;

  r->joint = new Joint[PR2::MAX_JOINTS];
  r->link = new Link[PR2::MAX_JOINTS];

  // MAPPING BETWEEN hwInterface->actuators and robot->joints
  // assign transmissions for all joints
  for(int ii=0; ii<PR2::MAX_JOINTS; ii++){
    SimpleTransmission *tr = new SimpleTransmission;
    r->transmissions_.push_back(tr);
    tr->actuator = &(hardwareInterface->actuator[ii]);
    tr->joint = &(r->joint[ii]);
    tr->mechanicalReduction = 1.0;
    tr->motorTorqueConstant = 1.0;
    tr->pulsesPerRevolution = 1.0;
    r->joint[ii].effortLimit = maxPositiveTorque;
  }
  // enable all actuators, just so happens num. joints == num. actuators for now
  for(int ii=0; ii<PR2::MAX_JOINTS; ii++){
    hardwareInterface->actuator[ii].command.enable = true;
  }
}

void GazeboMechanismControl::initControllers(){

  // assuming that: robot joint array == hardware interface actuator array
  // hard coded mapping between jointcontroller array and robot joint array
  int baseMapToRobotJointIndex[] = {
      PR2::CASTER_FL_STEER   , PR2::CASTER_FL_DRIVE_L , PR2::CASTER_FL_DRIVE_R ,
      PR2::CASTER_FR_STEER   , PR2::CASTER_FR_DRIVE_L , PR2::CASTER_FR_DRIVE_R ,
      PR2::CASTER_RL_STEER   , PR2::CASTER_RL_DRIVE_L , PR2::CASTER_RL_DRIVE_R ,
      PR2::CASTER_RR_STEER   , PR2::CASTER_RR_DRIVE_L , PR2::CASTER_RR_DRIVE_R };
  int leftArmMapToRobotJointIndex[] = {
      PR2::ARM_L_PAN           , PR2::ARM_L_SHOULDER_PITCH, PR2::ARM_L_SHOULDER_ROLL ,
      PR2::ARM_L_ELBOW_PITCH   , PR2::ARM_L_ELBOW_ROLL    , PR2::ARM_L_WRIST_PITCH   ,
      PR2::ARM_L_WRIST_ROLL    };
  int rightArmMapToRobotJointIndex[] = {
      PR2::ARM_R_PAN           , PR2::ARM_R_SHOULDER_PITCH, PR2::ARM_R_SHOULDER_ROLL ,
      PR2::ARM_R_ELBOW_PITCH   , PR2::ARM_R_ELBOW_ROLL    , PR2::ARM_R_WRIST_PITCH   ,
      PR2::ARM_R_WRIST_ROLL    };
  int headMapToRobotJointIndex[] = {
      PR2::HEAD_YAW            , PR2::HEAD_PITCH };
  int laserScannerMapToRobotJointIndex[] = {
      PR2::HEAD_LASER_PITCH  };
  int spineMapToRobotJointIndex[] = {
      PR2::SPINE_ELEVATOR  };
  // hard coded mapping between jointcontroller array and hardware interface actuator array
  int baseMapToHIActuatorIndex[] = {
      PR2::CASTER_FL_STEER   , PR2::CASTER_FL_DRIVE_L , PR2::CASTER_FL_DRIVE_R ,
      PR2::CASTER_FR_STEER   , PR2::CASTER_FR_DRIVE_L , PR2::CASTER_FR_DRIVE_R ,
      PR2::CASTER_RL_STEER   , PR2::CASTER_RL_DRIVE_L , PR2::CASTER_RL_DRIVE_R ,
      PR2::CASTER_RR_STEER   , PR2::CASTER_RR_DRIVE_L , PR2::CASTER_RR_DRIVE_R };
  int leftArmMapToHIActuatorIndex[] = {
      PR2::ARM_L_PAN           , PR2::ARM_L_SHOULDER_PITCH, PR2::ARM_L_SHOULDER_ROLL ,
      PR2::ARM_L_ELBOW_PITCH   , PR2::ARM_L_ELBOW_ROLL    , PR2::ARM_L_WRIST_PITCH   ,
      PR2::ARM_L_WRIST_ROLL    };
  int rightArmMapToHIActuatorIndex[] = {
      PR2::ARM_R_PAN           , PR2::ARM_R_SHOULDER_PITCH, PR2::ARM_R_SHOULDER_ROLL ,
      PR2::ARM_R_ELBOW_PITCH   , PR2::ARM_R_ELBOW_ROLL    , PR2::ARM_R_WRIST_PITCH   ,
      PR2::ARM_R_WRIST_ROLL    };
  int headMapToHIActuatorIndex[] = {
      PR2::HEAD_YAW                 , PR2::HEAD_PITCH };
  int laserScannerMapToHIActuatorIndex[] = {
      PR2::HEAD_LASER_PITCH  };
  int spineMapToHIActuatorIndex[] = {
      PR2::SPINE_ELEVATOR  };

  int leftArmNumJoints  = 7;
  int rightArmNumJoints = 7;

  baseController         = new BaseController(r,"baseController"); // controller got lucky, starts at 0 in pr2Core, if pr2Core changes, update.
  leftArmController      = new ArmController(r,"leftArmController",leftArmNumJoints,leftArmMapToRobotJointIndex,leftArmMapToHIActuatorIndex);
  rightArmController     = new ArmController(r,"rightArmController",rightArmNumJoints,rightArmMapToRobotJointIndex,rightArmMapToHIActuatorIndex);
  //headController         = new HeadController(r,"headController",headNumJoints,headMapToRobotJointIndex[],headMapToHIActuatorIndex[]);
  //laserScannerController = new LaserScannerController(r,"headController",laserScannerNumJoints,laserScannerMapToRobotJointIndex[],laserScannerMapToHIActuatorIndex[]);
  //spineController        = new SpineController(r,"headController",spienNumJoints,spineMapToRobotJointIndex[],spineMapToHIActuatorIndex[]);

  // xml information, hardcoded
  char *c_filename = getenv("ROS_PACKAGE_PATH");
  std::stringstream filename;
  filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;

  baseController->loadXML(filename.str());
  baseController->init();

  filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;
  leftArmController->loadXML(filename.str());
  leftArmController->init();

  //rightArmController->loadXML(filename.str());
  rightArmController->init();

  //headController->loadXML(filename.str());
  //headController->init();

  //LaserScannerController->loadXML(filename.str());
  //LaserScannerController->init();

  //SpineController->loadXML(filename.str());
  //SpineController->init();
}

//This function is called only from the realtime loop.  Everything it calls must also be realtime safe.
void GazeboMechanismControl::update(){
  //Clear actuator commands

  //Process robot model transmissions
  for(int i = 0; i < r->transmissions_.size(); i++){
    r->transmissions_[i]->propagatePosition();
  }

  //update KDL model with new joint position/velocities
  baseController->update();
  leftArmController->update();
  rightArmController->update();
  //headController->update();
#ifdef DEBUG
  for(int i = 0; i < r->numJoints; i++){
    printf("base_control:: cmd:: %d, %f\n",i,r->joint[i].commandedEffort);
  }
#endif
  /*  for(int i = 0; i < r->numJoints; i++){
      r->joint[i].enforceLimits();
      }*/

  for(int i = 0; i < r->transmissions_.size(); i++){
    r->transmissions_[i]->propagateEffort();
  }
}

