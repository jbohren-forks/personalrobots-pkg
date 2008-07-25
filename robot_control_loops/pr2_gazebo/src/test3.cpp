/*
 *  pr2_gazebo
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <pthread.h>
#include <pr2Core/pr2Core.h>
#include <mechanism_model/joint.h>

// gazebo
#include <libpr2API/pr2API.h>


// controller
#include <pr2Controllers/ArmController.h>
#include <pr2Controllers/HeadController.h>
#include <pr2Controllers/SpineController.h>
#include <pr2Controllers/BaseController.h>
#include <pr2Controllers/LaserScannerController.h>
#include <pr2Controllers/GripperController.h>

// roscpp
#include <ros/node.h>

#include <time.h>
#include <signal.h>

#include <RosGazeboControlsNode/RosGazeboControlsNode.h>

//Joint defaults
#define DEFAULTJOINTMAX 1
#define DEFAULTJOINTMIN -1

 pthread_mutex_t simMutex; //Mutex for sim R/W
mechanism::Joint* JointArray[PR2::MAX_JOINTS]; //Joint pointer array

void finalize(int)
{
  fprintf(stderr,"Caught sig, clean-up and exit\n");
  sleep(1);
  exit(-1);
}


void *nonRealtimeLoop(void *rgn)
{
  std::cout << "Started nonRT loop" << std::endl;
  while (1)
  {
    pthread_mutex_lock(&simMutex); //Lock for r/w
    ((RosGazeboControlsNode*)rgn)->Update();
    pthread_mutex_unlock(&simMutex); //Unlock when done
    // some time out for publishing ros info
    usleep(10000);
  }

}

int 
main(int argc, char** argv)
{ double time;
  double torque;
  /***************************************************************************************/
  /*                                                                                     */
  /*                           init Threads                                              */
  /*                                                                                     */
  /***************************************************************************************/

  // we need 2 threads, one for RT and one for nonRT
  pthread_t threads[2];

  pthread_mutex_init(&simMutex, NULL);

  /***************************************************************************************/
  /*                                                                                     */
  /*                           init Joints                                               */
  /*                                                                                     */
  /***************************************************************************************/

  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    JointArray[i] = new mechanism::Joint();
    JointArray[i]->jointLimitMax = DEFAULTJOINTMAX;
    JointArray[i]->jointLimitMin = DEFAULTJOINTMIN;
    JointArray[i]->type = mechanism::JOINT_ROTARY;
  }


  /***************************************************************************************/
  /*                                                                                     */
  /*                           The main simulator object                                 */
  /*                                                                                     */
  /***************************************************************************************/
  ros::init(argc,argv);
  printf("Creating robot\n");
  PR2::PR2Robot* myPR2;
  // initialize robot object
  myPR2 = new PR2::PR2Robot();
  // initialize connections
  myPR2->InitializeRobot();
  // Set control mode for the base
  myPR2->SetBaseControlMode(PR2::PR2_CARTESIAN_CONTROL);
  // myPR2->SetJointControlMode(PR2::CASTER_FL_STEER, PR2::TORQUE_CONTROL);
  // myPR2->SetJointControlMode(PR2::CASTER_FR_STEER, PR2::TORQUE_CONTROL);
  // myPR2->SetJointControlMode(PR2::CASTER_RL_STEER, PR2::TORQUE_CONTROL);
  // myPR2->SetJointControlMode(PR2::CASTER_RR_STEER, PR2::TORQUE_CONTROL);

  myPR2->EnableGripperLeft();
  myPR2->EnableGripperRight();

  // Set control mode for the arms
  // FIXME: right now this just sets default to pd control
  //myPR2->SetArmControlMode(PR2::PR2_RIGHT_ARM, PR2::PR2_JOINT_CONTROL);
  //myPR2->SetArmControlMode(PR2::PR2_LEFT_ARM, PR2::PR2_JOINT_CONTROL);
  //------------------------------------------------------------

  // set torques for driving the robot wheels
  // myPR2->hw.SetJointTorque(PR2::CASTER_FL_DRIVE_L, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_FR_DRIVE_L, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_RL_DRIVE_L, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_RR_DRIVE_L, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_FL_DRIVE_R, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_FR_DRIVE_R, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_RL_DRIVE_R, 1000.0 );
  // myPR2->hw.SetJointTorque(PR2::CASTER_RR_DRIVE_R, 1000.0 );

  /***************************************************************************************/
  /*                                                                                     */
  /*                        build actuators from pr2Actuators.xml                        */
  /*                                                                                     */
  /***************************************************************************************/
  //Actuators myActuators(myPR2);
  
  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize controllers                                   */
  /*                                                                                     */
  /***************************************************************************************/
  controller::ArmController          myArm;
  controller::HeadController         myHead;
  controller::SpineController        mySpine;
  controller::BaseController         myBase;
  controller::LaserScannerController myLaserScanner;
  controller::GripperController      myGripper;
  
  /***************************************************************************************/
  /*                                                                                     */
  /*                            Joint Controllers                                        */
  /*                                                                                     */
  /***************************************************************************************/
/*
  controller::JointController* ControllerArray[PR2::MAX_JOINTS]; //Create array of pointers to controllers
  for(int i = 0;i<PR2::MAX_JOINTS;i++){
      ControllerArray[i] = new controller::JointController(); //initialize blank controller
  }
 
  //Explicitly initialize the controllers we wish to use. Don't forget to set the controllers to torque mode in the world file! 
 
 ControllerArray[PR2::ARM_L_PAN]->init(1000,0,0,500,-500, controller::controller_POSITION,time,1000,-1000,JointArray[PR2::ARM_L_PAN]);
 ControllerArray[PR2::ARM_L_SHOULDER_PITCH]->init(1000,0,0,500,-500, controller::controller_POSITION,time,10000,-10000,JointArray[PR2::ARM_L_SHOULDER_PITCH]);
   
 ControllerArray[PR2::ARM_L_SHOULDER_ROLL]->init(100,0,0,500,-500, controller::controller_POSITION,time,100,-100,JointArray[PR2::ARM_L_SHOULDER_ROLL]);
 ControllerArray[PR2::ARM_L_ELBOW_PITCH]->init(300,0,0,500,-500, controller::controller_POSITION,time,100,-100,JointArray[PR2::ARM_L_ELBOW_PITCH]);
 ControllerArray[PR2::ARM_L_ELBOW_ROLL]->init(100,0,0,500,-500, controller::controller_POSITION,time,100,-100,JointArray[PR2::ARM_L_ELBOW_ROLL]);
 ControllerArray[PR2::ARM_L_WRIST_PITCH]->init(100,0,0,500,-500, controller::controller_POSITION,time,100,-100,JointArray[PR2::ARM_L_WRIST_PITCH]);
*/
 //Switch control to arm controller
  
  myArm.initJoint(0,1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,1000,-1000,JointArray[PR2::ARM_L_PAN]);
  myArm.initJoint(1,1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,1000,-1000,JointArray[PR2::ARM_L_SHOULDER_PITCH]);
  myArm.initJoint(2,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,JointArray[PR2::ARM_L_SHOULDER_ROLL]);
  myArm.initJoint(3,300,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,JointArray[PR2::ARM_L_ELBOW_PITCH]);
  myArm.initJoint(4,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,JointArray[PR2::ARM_L_ELBOW_ROLL]);
  myArm.initJoint(5,100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,JointArray[PR2::ARM_L_WRIST_PITCH]);
  myArm.initJoint(6,0.5,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,JointArray[PR2::ARM_L_WRIST_ROLL]);

myArm.initArm(controller::CONTROLLER_POSITION);

myArm.setHandCartesianPos(0.3,-0.2,0.56,0,0.0,0.0);


//  ControllerArray[PR2::ARM_L_WRIST_ROLL]->init(0.5,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,JointArray[PR2::ARM_L_WRIST_ROLL]);

  
//  ControllerArray[PR2::ARM_L_WRIST_ROLL]->capAccel = true;
//  ControllerArray[PR2::ARM_L_WRIST_ROLL]->maxAccel = 0.5;
//  std::cout<<"Arm check:"<< ControllerArray[PR2::ARM_L_SHOULDER_PITCH]->GetMode()<<std::endl;
//  std::cout<<"Wrist check:"<< ControllerArray[PR2::ARM_L_WRIST_ROLL]->GetMode()<<std::endl;

  //Set initial Commands
//  std::cout<<"Velocity:"<<  ControllerArray[PR2::ARM_L_WRIST_ROLL]->SetVelCmd(-0.5)<<std::endl;
// ControllerArray[PR2::ARM_L_WRIST_ROLL]->SetPosCmd(2.0);
//  std::cout<<"Torque:"<<ControllerArray[PR2::ARM_L_PAN]->setTorqueCmd(1000)<<std::endl;

 /***************************************************************************************/
  /*                                                                                     */
  /*                           Laser  Controllers                                        */
  /*                                                                                     */
  /***************************************************************************************/

 controller::LaserScannerController*  laser = new controller::LaserScannerController();
  laser->init(0.01,0,0,100,-100,controller::CONTROLLER_VELOCITY,time,100,-100, JointArray[PR2::HEAD_LASER_PITCH]);
 // laser->SetVelCmd(0.5);
  

      //Set profiles
    laser->setSinewaveProfile(1,0.5,0.01,0);
//      laser->SetSawtoothProfile(1,0.5,0.01,0);
  laser->setMode(controller::CONTROLLER_AUTOMATIC);


/*
  controller::JointController        leftShoulderPitch; //Test jointController
  controller::JointController        leftWristRoll; //Test jointController
  myPR2->hw.GetSimTime(&time);
  leftShoulderPitch.init(1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,1000,-1000,1000,JointArray[PR2::ARM_L_SHOULDER_PITCH]);
  leftWristRoll.init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,100,-100,100,JointArray[PR2::ARM_L_WRIST_ROLL]);
 // std::cout<<"****************"<<test.SetTorqueCmd(-500.0)<<std::endl;
  std::cout<<"****************"<<leftShoulderPitch.SetPosCmd(-1.0)<<std::endl;
  std::cout<<"****************"<<leftWristRoll.SetPosCmd(2.0)<<std::endl;  
//std::cout<<"****************"<<leftWristRoll.SetTorqueCmd(50.0)<<std::endl;
*/
  //void JointController::init(double PGain, double IGain, double DGain, double IMax, double IMin, controller_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint) {

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize ROS Gazebo Nodes                              */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Creating node \n");
  RosGazeboControlsNode rgn(argc,argv,argv[1],myPR2,&myArm,&myHead,&mySpine,&myBase,&myLaserScanner,&myGripper);
 // rgn.useControllerArray = true;
  /***************************************************************************************/
  /*                                                                                     */
  /*                                on termination...                                    */
  /*                                                                                     */
  /***************************************************************************************/
  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

  // see if we can subscribe models needed
  if (rgn.AdvertiseSubscribeMessages() != 0)
    exit(-1);

  /***************************************************************************************/
  /*                                                                                     */
  /* Update ROS Gazebo Node                                                              */
  /*   contains controller pointers for the non-RT setpoints                             */
  /*                                                                                     */
  /***************************************************************************************/
  int rgnt = pthread_create(&threads[0],NULL, nonRealtimeLoop, (void *) (&rgn));
  if (rgnt)
  {
    printf("Could not start a separate thread for ROS Gazebo Node (code=%d)\n",rgnt);
    exit(-1);
  }

  /***************************************************************************************/
  /*                                                                                     */
  /*      RealTime loop using Gazebo ClientWait function call                            */
  /*        this is updated once every gazebo timestep (world time step size)            */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Entering realtime loop\n");
  double pos,effort;
  while(1)
  { 
    myPR2->hw.GetSimTime(&time);

    if(pthread_mutex_trylock(&simMutex) != 0) continue; //Try to lock here. But continue on if fails to enforce real time
    // Update Controllers
    //   each controller will try to read new commands from shared memory with nonRT hooks,
    //   and skip update if locked by nonRT loop.

    /*
    myArm.Update();
    myHead.Update();
    mySpine.Update();
    myBase.Update();
    myLaserScanner.Update();
    myGripper.Update();
    */
  //Update all the joint controllers

/*
 for(int i = PR2::ARM_L_PAN;i<=PR2::ARM_L_WRIST_ROLL;i++){
 
      ControllerArray[i]->update();
  }
*/
    myArm.update();     
    laser->update();

/*    double cmdvel;
      ControllerArray[PR2::ARM_L_WRIST_ROLL]->getVelAct(&cmdvel);
      ControllerArray[PR2::ARM_L_WRIST_ROLL]->getPosAct(&pos);
      std::cout<<pos<<" "<<cmdvel<<std::endl;
*/
//      std::cout<<"Mode:"<<ControllerArray[PR2::ARM_L_PAN]->GetMode()<<std::endl;

//   std::cout<<"*"<<torque<<std::endl;
//    std::cout<<JointArray[PR2::ARM_L_SHOULDER_PITCH]->velocity<<std::endl;
//    ControllerArray[PR2::ARM_L_SHOULDER_PITCH]->GetPosCmd(&pos);
//    std::cout<<JointArray[PR2::ARM_L_ELBOW_PITCH]->commandedEffort<<std::endl;
//  std::cout<<JointArray[PR2::ARM_L_ELBOW_PITCH]->commandedEffort<<std::endl;
//    std::cout<<JointArray[PR2::ARM_L_SHOULDER_PITCH]->position<<std::endl;
//    std::cout<<JointArray[PR2::ARM_L_SHOULDER_PITCH]->type<<std::endl;
//    std::cout<<pos<<std::endl;

    // TODO: Safety codes should go here...

    // Send updated controller commands to hardware
    myPR2->hw.UpdateJointArray(JointArray);
//     myPR2->hw.UpdateHW();

    pthread_mutex_unlock(&simMutex); //Unlock after we're done with r/w
    // wait for Gazebo time step
  //  myPR2->hw.ClientWait();
  }
  
  /***************************************************************************************/
  /*                                                                                     */
  /* have to call this explicitly for some reason.  probably interference                */
  /* from signal handling in Stage / FLTK?                                               */
  /*                                                                                     */
  /***************************************************************************************/
  ros::msg_destruct();

  exit(0);

}
