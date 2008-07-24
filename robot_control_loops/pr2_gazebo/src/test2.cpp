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

// robot model
#include <pr2Core/pr2Core.h>

// gazebo hardware, sensors
#include <gazebo_hardware/gazebo_hardware.h>
#include <gazebo_sensors/gazebo_sensors.h>

// controller objects
#include <mechanism_control/gazebo_control.h>

// ros node for controllers
#include <rosControllers/RosJointController.h>

// roscpp
#include <ros/node.h>

#include <time.h>
#include <signal.h>


void finalize(int)
{
  fprintf(stderr,"Caught sig, clean-up and exit\n");
  sleep(1);
  exit(-1);
}

#define TEST_ROS_CONTROLLER 0
#if TEST_ROS_CONTROLLER
#include <pthread.h>
pthread_mutex_t simMutex[PR2::MAX_JOINTS]; //Mutex for sim R/W
void *nonRealtimeLoop(void *rjc)
{
  std::cout << "Started nonRT loop" << std::endl;
  while (1)
  {
    for (int i = 0;i<PR2::MAX_JOINTS;i++){
      pthread_mutex_lock(&simMutex[i]); //Lock for r/w
      ((RosJointController*)rjc)->update();
      pthread_mutex_unlock(&simMutex[i]); //Unlock when done
    }
    // some time out for publishing ros info
    usleep(10000);
  }

}
#endif

int 
main(int argc, char** argv)
{

  // ros node stuff
  ros::node *rosnode;
  ros::init(argc,argv);
  rosnode = new ros::node("gazebo_mechanism_node");

  double time;
  double torque;

#if TEST_ROS_CONTROLLER
  // we need 2 threads, one for RT and one for nonRT
  pthread_t threads[2];

  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    pthread_mutex_init(&(simMutex[i]), NULL);
  }
#endif

  ros::init(argc,argv);

  /***************************************************************************************/
  /*                                                                                     */
  /*                           The main simulator object                                 */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Creating robot\n");

  /***************************************************************************************/
  /*                                                                                     */
  /*   hardware info, eventually retrieve from param server (from xml files)             */
  /*                                                                                     */
  /***************************************************************************************/
  int numBoards    = 4;                 // 0 for head and 1 for arms, see pr2.model:Pr2_Actarray for now
  int numActuators = PR2::MAX_JOINTS-2; // total number of actuators for both boards, last 2 in pr2Core (BASE_6DOF and PR2_WORLD don't count)

  int boardLookUp[] ={0,0,0,                   // head pitch, yaw, hokuyo pitch
                      0,0,0,0,0,0,0,0,0,0,0,0, // casters
                      0,                       // spine elevator
                      0,0,0,0,0,0,0,           // left arm
                      0,0,0,0,0,0,0,           // right arm
                      1,                       // left gripper
                      2,                       // right gripper
                      3,3,4,4                  // left ptz p/t, right ptz p/t
                      };  // 0=actarray, 1=left gripper, 2=right gripper, 3=left ptz, 4=right ptz
  //int numJointsLookUp[]   ={7, 28};  // joints per actuator array
  int portLookUp[] = {    // the way joints are listed in pr2.model
      PR2::HEAD_YAW            , PR2::HEAD_PITCH          , PR2::HEAD_LASER_PITCH    ,
      PR2::CASTER_FL_STEER     , PR2::CASTER_FL_DRIVE_L   , PR2::CASTER_FL_DRIVE_R   ,
      PR2::CASTER_FR_STEER     , PR2::CASTER_FR_DRIVE_L   , PR2::CASTER_FR_DRIVE_R   ,
      PR2::CASTER_RL_STEER     , PR2::CASTER_RL_DRIVE_L   , PR2::CASTER_RL_DRIVE_R   ,
      PR2::CASTER_RR_STEER     , PR2::CASTER_RR_DRIVE_L   , PR2::CASTER_RR_DRIVE_R   ,
      PR2::SPINE_ELEVATOR      ,
      PR2::ARM_L_PAN           , PR2::ARM_L_SHOULDER_PITCH, PR2::ARM_L_SHOULDER_ROLL ,
      PR2::ARM_L_ELBOW_PITCH   , PR2::ARM_L_ELBOW_ROLL    , PR2::ARM_L_WRIST_PITCH   ,
      PR2::ARM_L_WRIST_ROLL    ,
      PR2::ARM_R_PAN           , PR2::ARM_R_SHOULDER_PITCH, PR2::ARM_R_SHOULDER_ROLL ,
      PR2::ARM_R_ELBOW_PITCH   , PR2::ARM_R_ELBOW_ROLL    , PR2::ARM_R_WRIST_PITCH   ,
      PR2::ARM_R_WRIST_ROLL    ,
      PR2::ARM_L_GRIPPER      - PR2::ARM_L_GRIPPER        ,
      PR2::ARM_R_GRIPPER      - PR2::ARM_R_GRIPPER        ,
      PR2::HEAD_PTZ_L_PAN     - PR2::HEAD_PTZ_L_PAN       ,
      PR2::HEAD_PTZ_L_TILT    - PR2::HEAD_PTZ_L_PAN       ,
      PR2::HEAD_PTZ_R_PAN     - PR2::HEAD_PTZ_L_PAN       ,
      PR2::HEAD_PTZ_R_TILT    - PR2::HEAD_PTZ_L_PAN       };

  
  int jointId[]    = {    // numbering system
      PR2::HEAD_YAW            , PR2::HEAD_PITCH          , PR2::HEAD_LASER_PITCH    ,
      PR2::CASTER_FL_STEER     , PR2::CASTER_FL_DRIVE_L   , PR2::CASTER_FL_DRIVE_R   ,
      PR2::CASTER_FR_STEER     , PR2::CASTER_FR_DRIVE_L   , PR2::CASTER_FR_DRIVE_R   ,
      PR2::CASTER_RL_STEER     , PR2::CASTER_RL_DRIVE_L   , PR2::CASTER_RL_DRIVE_R   ,
      PR2::CASTER_RR_STEER     , PR2::CASTER_RR_DRIVE_L   , PR2::CASTER_RR_DRIVE_R   ,
      PR2::SPINE_ELEVATOR      ,
      PR2::ARM_L_PAN           , PR2::ARM_L_SHOULDER_PITCH, PR2::ARM_L_SHOULDER_ROLL ,
      PR2::ARM_L_ELBOW_PITCH   , PR2::ARM_L_ELBOW_ROLL    , PR2::ARM_L_WRIST_PITCH   ,
      PR2::ARM_L_WRIST_ROLL    ,
      PR2::ARM_R_PAN           , PR2::ARM_R_SHOULDER_PITCH, PR2::ARM_R_SHOULDER_ROLL ,
      PR2::ARM_R_ELBOW_PITCH   , PR2::ARM_R_ELBOW_ROLL    , PR2::ARM_R_WRIST_PITCH   ,
      PR2::ARM_R_WRIST_ROLL    ,
      PR2::ARM_L_GRIPPER       ,
      PR2::ARM_R_GRIPPER       ,
      PR2::HEAD_PTZ_L_PAN      ,
      PR2::HEAD_PTZ_L_TILT     ,
      PR2::HEAD_PTZ_R_PAN      ,
      PR2::HEAD_PTZ_R_TILT     };

  string etherIP[] = {"10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103",
                      "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103",
                      "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103",
                      "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103",
                      "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103",
                      "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103", "10.12.0.103"
                     };
  string hostIP[]  = {"10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ",
                      "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ",
                      "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ",
                      "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ",
                      "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ",
                      "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  ", "10.12.0.2  "
                     };
  /***************************************************************************************/
  /*                                                                                     */
  /*                        initialize hardware                                          */
  /*                                                                                     */
  /*   User initialized                                                                  */
  /*                                                                                     */
  /*  single hardware interface object, containing array of actuators                    */
  /*  single gazebo hardware object                                                      */
  /*                                                                                     */
  /***************************************************************************************/
  // mapping between actuators in h and actuators in hi defined by boardLookUp, portLookUp and jointId
  GazeboHardware         hardware(numBoards,numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP);
  // MechanismControl
  GazeboMechanismControl mechanismControl;
  mechanismControl.init(hardware.hardwareInterface); // pass hardware interface to mechanism control
  mechanismControl.baseController->setVelocity(0,0,0);

  // access to all the gazebo sensors
  GazeboSensors         gazeboSensor;


  //myPR2->hw.GetSimTime(&time);
#if TEST_ROS_CONTROLLER
  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize ROS Gazebo Nodes                              */
  /*        this should probably live inside each pr2Controller                          */
  /*                                                                                     */
  /***************************************************************************************/
  RosJointController*               rjc[PR2::MAX_JOINTS];  // One ros node per controller
  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    // name for controller
    char tmp[21];
    sprintf(tmp,"joint_controller_%04d", i);
    // instantiate
    rjc[i] = new RosJointController(tmp);
    rjc[i]->init(jc[i]);
    // see if we can subscribe models needed
    if (rjc[i]->advertiseSubscribeMessages() != 0)
      exit(-1);
  }
  /***************************************************************************************/
  /*                                                                                     */
  /* Update ROS Gazebo Node                                                              */
  /*   contains controller pointers for the non-RT setpoints                             */
  /*                                                                                     */
  /***************************************************************************************/
  int rjct[PR2::MAX_JOINTS];
  for (int i=0;i<PR2::MAX_JOINTS;i++) {
    rjct[i] = pthread_create(&threads[0],NULL, nonRealtimeLoop, (void *) (&(rjc[i])));
    if (rjct[i])
    {
      printf("Could not start a separate thread for %d-th ROS Gazebo Node (code=%d)\n",i,rjct[i]);
      exit(-1);
    }
  }
#endif

  /***************************************************************************************/
  /*                                                                                     */
  /*                                on termination...                                    */
  /*                                                                                     */
  /***************************************************************************************/
  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

  /***************************************************************************************/
  /*                                                                                     */
  /*      RealTime loop using Gazebo ClientWait function call                            */
  /*        this is updated once every gazebo timestep (world time step size)            */
  /*                                                                                     */
  /***************************************************************************************/
  printf("Entering realtime loop\n");
  while(1)
  { 
    //myPR2->hw.GetSimTime(&time);
    //std::cout<<"Time:"<<time<<std::endl;

    // Update Controllers
    //   each controller will try to read new commands from shared memory with nonRT hooks,
    //   and skip update if locked by nonRT loop.

    // get encoder counts from hardware
    hardware.updateState();

    // update mechanism control
    mechanismControl.update();

    // wait for Gazebo time step, fix Wait() call or else...
    //myPR2->hw.ClientWait();
    usleep(800);

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
