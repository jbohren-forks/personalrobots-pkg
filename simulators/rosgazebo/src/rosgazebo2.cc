/*
 *  rosgazebo
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

// robot model
#include <pr2Core/pr2Core.h>
#include <robot_model/joint.h>

// gazebo hardware, sensors
#include <gazebo_hardware/gazebo_hardware.h>
#include <gazebo_sensors/gazebo_sensors.h>


// controller objects
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

//#include <RosGazeboNode/RosGazeboNode.h>

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
    //((RosGazeboNode*)rgn)->Update();
    pthread_mutex_unlock(&simMutex); //Unlock when done
    // some time out for publishing ros info
    usleep(10000);
  }

}

int 
main(int argc, char** argv)
{

  double time;
  double torque;

  // we need 2 threads, one for RT and one for nonRT
  pthread_t threads[2];

  pthread_mutex_init(&simMutex, NULL);
  
  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    JointArray[i] = new mechanism::Joint();
  }

  ros::init(argc,argv);

  /***************************************************************************************/
  /*                                                                                     */
  /*                           The main simulator object                                 */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Creating robot\n");
  //PR2::PR2Robot* myPR2;
  // Initialize robot object
  //myPR2 = new PR2::PR2Robot();
  // Initialize connections
  //myPR2->InitializeRobot();
  // Set control mode for the base
  //myPR2->SetBaseControlMode(PR2::PR2_CARTESIAN_CONTROL);
  //myPR2->EnableGripperLeft();
  //myPR2->EnableGripperRight();

  /***************************************************************************************/
  /*                                                                                     */
  /*   hardware info, eventually retrieve from param server (from xml files)             */
  /*                                                                                     */
  /***************************************************************************************/
  int iB, iP;

  int numBoards    = 4;                 // 0 for head and 1 for arms, see pr2.model:Pr2_Actarray for now
  int numActuators = PR2::MAX_JOINTS-2; // total number of actuators for both boards, last 2 in pr2Core (BASE_6DOF and PR2_WORLD don't count)

  int boardLookUp[] ={1,1,1,1,1,1,1,1,1,1,1,1, // casters
                      1,                       // spine elevator
                      1,1,1,1,1,1,1,           // left arm
                      1,1,1,1,1,1,1,           // right arm
                      2,                       // left gripper
                      3,                       // right gripper
                      0,0,0,0,0,0,0            // head y/p, laser p, left ptz p/t, right ptz p/t
                      };  // 0 for head, 1 for caster+arm
  //int numJointsLookUp[]   ={7, 28};  // joints per actuator array
  int portLookUp[] = {    // the way joints are listed in pr2.model
      PR2::CASTER_FL_STEER     ,
      PR2::CASTER_FL_DRIVE_L   ,
      PR2::CASTER_FL_DRIVE_R   ,
      PR2::CASTER_FR_STEER     ,
      PR2::CASTER_FR_DRIVE_L   ,
      PR2::CASTER_FR_DRIVE_R   ,
      PR2::CASTER_RL_STEER     ,
      PR2::CASTER_RL_DRIVE_L   ,
      PR2::CASTER_RL_DRIVE_R   ,
      PR2::CASTER_RR_STEER     ,
      PR2::CASTER_RR_DRIVE_L   ,
      PR2::CASTER_RR_DRIVE_R   ,
      PR2::SPINE_ELEVATOR      ,
      PR2::ARM_L_PAN           ,
      PR2::ARM_L_SHOULDER_PITCH,
      PR2::ARM_L_SHOULDER_ROLL ,
      PR2::ARM_L_ELBOW_PITCH   ,
      PR2::ARM_L_ELBOW_ROLL    ,
      PR2::ARM_L_WRIST_PITCH   ,
      PR2::ARM_L_WRIST_ROLL    ,
      PR2::ARM_R_PAN           ,
      PR2::ARM_R_SHOULDER_PITCH,
      PR2::ARM_R_SHOULDER_ROLL ,
      PR2::ARM_R_ELBOW_PITCH   ,
      PR2::ARM_R_ELBOW_ROLL    ,
      PR2::ARM_R_WRIST_PITCH   ,
      PR2::ARM_R_WRIST_ROLL    ,
      PR2::ARM_L_GRIPPER      - PR2::ARM_L_GRIPPER ,
      PR2::ARM_R_GRIPPER      - PR2::ARM_R_GRIPPER ,
      PR2::HEAD_YAW           - PR2::HEAD_YAW ,
      PR2::HEAD_PITCH         - PR2::HEAD_YAW ,
      PR2::HEAD_LASER_PITCH   - PR2::HEAD_YAW ,
      PR2::HEAD_PTZ_L_PAN     - PR2::HEAD_YAW ,
      PR2::HEAD_PTZ_L_TILT    - PR2::HEAD_YAW ,
      PR2::HEAD_PTZ_R_PAN     - PR2::HEAD_YAW ,
      PR2::HEAD_PTZ_R_TILT    - PR2::HEAD_YAW };

  
  int jointId[]    = {    // numbering system
      PR2::CASTER_FL_STEER     ,
      PR2::CASTER_FL_DRIVE_L   ,
      PR2::CASTER_FL_DRIVE_R   ,
      PR2::CASTER_FR_STEER     ,
      PR2::CASTER_FR_DRIVE_L   ,
      PR2::CASTER_FR_DRIVE_R   ,
      PR2::CASTER_RL_STEER     ,
      PR2::CASTER_RL_DRIVE_L   ,
      PR2::CASTER_RL_DRIVE_R   ,
      PR2::CASTER_RR_STEER     ,
      PR2::CASTER_RR_DRIVE_L   ,
      PR2::CASTER_RR_DRIVE_R   ,
      PR2::SPINE_ELEVATOR      ,
      PR2::ARM_L_PAN           ,
      PR2::ARM_L_SHOULDER_PITCH,
      PR2::ARM_L_SHOULDER_ROLL ,
      PR2::ARM_L_ELBOW_PITCH   ,
      PR2::ARM_L_ELBOW_ROLL    ,
      PR2::ARM_L_WRIST_PITCH   ,
      PR2::ARM_L_WRIST_ROLL    ,
      PR2::ARM_R_PAN           ,
      PR2::ARM_R_SHOULDER_PITCH,
      PR2::ARM_R_SHOULDER_ROLL ,
      PR2::ARM_R_ELBOW_PITCH   ,
      PR2::ARM_R_ELBOW_ROLL    ,
      PR2::ARM_R_WRIST_PITCH   ,
      PR2::ARM_R_WRIST_ROLL    ,
      PR2::ARM_L_GRIPPER       ,
      PR2::ARM_R_GRIPPER       ,
      PR2::HEAD_YAW            ,
      PR2::HEAD_PITCH          ,
      PR2::HEAD_LASER_PITCH    ,
      PR2::HEAD_PTZ_L_PAN      ,
      PR2::HEAD_PTZ_L_TILT     ,
      PR2::HEAD_PTZ_R_PAN      ,
      PR2::HEAD_PTZ_R_TILT     };

  string etherIP[] = {"10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103",
                      "10.12.0.103"
                     };
  string hostIP[]  = {"10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  ",
                      "10.12.0.2  "
                     };
  /***************************************************************************************/
  /*                                                                                     */
  /*                        initialize hardware                                          */
  /*                                                                                     */
  /***************************************************************************************/
  HardwareInterface *hi = new HardwareInterface(numActuators);
  GazeboHardware    *h  = new GazeboHardware(numBoards,numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP, hi);
  // connect to hardware
  h->init();

  // access to all the gazebo sensors
  GazeboSensors    *s  = new GazeboSensors();

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize controllers                                   */
  /*                                                                                     */
  /***************************************************************************************/
  CONTROLLER::ArmController          myArm;
  CONTROLLER::HeadController         myHead;
  CONTROLLER::SpineController        mySpine;
  CONTROLLER::BaseController         myBase;
  CONTROLLER::LaserScannerController myLaserScanner;
  CONTROLLER::GripperController      myGripper;

  CONTROLLER::JointController        *jc =new CONTROLLER::JointController(); //Test jointController
  jc->Init(1000,0,0,500,-500, CONTROLLER::CONTROLLER_POSITION,time,1000,-1000,1000,JointArray[PR2::ARM_L_SHOULDER_PITCH]);
  //void JointController::Init(double PGain, double IGain, double DGain, double IMax, double IMin, CONTROLLER_CONTROL_MODE mode, double time, double maxPositiveTorque, double maxNegativeTorque, double maxEffort, mechanism::Joint *joint) {
  // std::cout<<"****************"<<jc.SetTorqueCmd(-500.0)<<std::endl;
  // std::cout<<"****************"<<jc.SetPosCmd(0.0)<<std::endl;

  /***************************************************************************************/
  /*                                                                                     */
  /*                        initialize robot model                                       */
  /*                                                                                     */
  /***************************************************************************************/
  mechanism::Joint *joint = new mechanism::Joint();

  /***************************************************************************************/
  /*                                                                                     */
  /*                        initialize transmission                                      */
  /*                                                                                     */
  /***************************************************************************************/
  SimpleTransmission *st = new SimpleTransmission(joint,&hi->actuator[0],1,1,14000);
  
  //myPR2->hw.GetSimTime(&time);

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize ROS Gazebo Nodes                              */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Creating node \n");
  //RosGazeboNode rgn(argc,argv,argv[1],myPR2,&myArm,&myHead,&mySpine,&myBase,&myLaserScanner,&myGripper,JointArray);

  /***************************************************************************************/
  /*                                                                                     */
  /*                                on termination...                                    */
  /*                                                                                     */
  /***************************************************************************************/
  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

  // see if we can subscribe models needed
  //if (rgn.AdvertiseSubscribeMessages() != 0)
  //  exit(-1);

  /***************************************************************************************/
  /*                                                                                     */
  /* Update ROS Gazebo Node                                                              */
  /*   contains controller pointers for the non-RT setpoints                             */
  /*                                                                                     */
  /***************************************************************************************/
  //int rgnt = pthread_create(&threads[0],NULL, nonRealtimeLoop, (void *) (&rgn));
  //if (rgnt)
  //{
  //  printf("Could not start a separate thread for ROS Gazebo Node (code=%d)\n",rgnt);
  //  exit(-1);
  //}

  /***************************************************************************************/
  /*                                                                                     */
  /*      RealTime loop using Gazebo ClientWait function call                            */
  /*        this is updated once every gazebo timestep (world time step size)            */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Entering realtime loop\n");
  double pos;
  while(1)
  { 
    //myPR2->hw.GetSimTime(&time);
    //std::cout<<"Time:"<<time<<std::endl;

    //pthread_mutex_trylock(&simMutex); //Try to lock here. But continue on if fails to enforce real time
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


    // jc.GetTorqueCmd(&torque);
    //   std::cout<<"*"<<torque<<std::endl;
    // std::cout<<JointArray[PR2::ARM_L_SHOULDER_PITCH]->position<<std::endl;
    // TODO: Safety codes should go here...

    // old:  Send updated controller commands to hardware
    // myPR2->hw.UpdateJointArray(JointArray);




    // get encoder counts from hardware
    h->updateState();

    // setup joint state from encoder counts
    st->propagatePosition();

    //update controller
    jc->Update();

    //cout << "pos:: " << joint->position << ", eff:: " << joint->commandedEffort << endl;

    // update command current from joint command
    st->propagateEffort();

    // send command to hardware
    h->sendCommand();

    // refresh hardware
    h->tick();

    //pthread_mutex_unlock(&simMutex); //Unlock after we're done with r/w

    // wait for Gazebo time step
    //myPR2->hw.ClientWait();
    usleep(1000);

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
