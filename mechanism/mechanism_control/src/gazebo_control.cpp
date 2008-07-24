
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
  r = new Robot("robot"); 

  r->numJoints        = PR2::MAX_JOINTS;
  r->numTransmissions = PR2::MAX_JOINTS;
  r->numLinks         = PR2::MAX_JOINTS;

  r->transmission = new SimpleTransmission[PR2::MAX_JOINTS];
  r->joint = new Joint[PR2::MAX_JOINTS];
  r->link = new Link[PR2::MAX_JOINTS];

  // assign transmissions for all joints
  for(int ii=0; ii<PR2::MAX_JOINTS; ii++){
    r->transmission[ii].actuator = &(hardwareInterface->actuator[ii]);
    r->transmission[ii].joint = &(r->joint[ii]);
    r->transmission[ii].mechanicalReduction = 1.0;
    r->transmission[ii].motorTorqueConstant = 1.0;
    r->transmission[ii].pulsesPerRevolution = 1.0;
    r->joint[ii].effortLimit = maxPositiveTorque;
  }
  // enable all actuators, just so happens num. joints == num. actuators for now
  for(int ii=0; ii<PR2::MAX_JOINTS; ii++){
    hardwareInterface->actuator[ii].command.enable = true;
  }
}

void GazeboMechanismControl::initControllers(){
  baseController     = new BaseController(r,"baseController");
  leftArmController  = new ArmController(); //r,"leftArmController");
  rightArmController = new ArmController(); //r,"rightArmController");
  //headController     = new HeadController(r,"headController");
  //laserScannerController     = new LaserScannerController(r,"headController");
  //spineController     = new SpineController(r,"headController");

  // xml information, hardcoded
  char *c_filename = getenv("ROS_PACKAGE_PATH");
  std::stringstream filename;
  filename << c_filename << "/robot_descriptions/wg_robot_description/pr2/pr2.xml" ;

  baseController->loadXML(filename.str());
  baseController->init();

  //leftArmController->loadXML(filename.str());
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
  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagatePosition();
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

  for(int i = 0; i < r->numTransmissions; i++){
    r->transmission[i].propagateEffort();
  }
}

