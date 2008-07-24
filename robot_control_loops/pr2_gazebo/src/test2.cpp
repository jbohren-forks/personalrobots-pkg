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
#include <mechanism_model/joint.h>

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

// ros node for controllers
#include <rosControllers/RosJointController.h>

// roscpp
//#include <ros/node.h>

#include <time.h>
#include <signal.h>

pthread_mutex_t simMutex[PR2::MAX_JOINTS]; //Mutex for sim R/W

void finalize(int)
{
  fprintf(stderr,"Caught sig, clean-up and exit\n");
  sleep(1);
  exit(-1);
}


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

int 
main(int argc, char** argv)
{

  double time;
  double torque;

  // we need 2 threads, one for RT and one for nonRT
  pthread_t threads[2];

  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    pthread_mutex_init(&(simMutex[i]), NULL);
  }

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
      PR2::HEAD_YAW            ,
      PR2::HEAD_PITCH          ,
      PR2::HEAD_LASER_PITCH    ,
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
  /*                        initialize robot model                                       */
  /*                                                                                     */
  /*    in Robot                                                                         */
  /*                                                                                     */
  /***************************************************************************************/
  mechanism::Joint* joint[PR2::MAX_JOINTS]; //Joint pointer array
  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    joint[i] = new mechanism::Joint();
  }

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
  // a single HardwareInterface objects with numActuators actuators
  HardwareInterface *hi = new HardwareInterface(numActuators);
  // hi is passed to h during construction of h,
  // mapping between actuators in h and actuators in hi defined by boardLookUp, portLookUp and jointId
  GazeboHardware    *h  = new GazeboHardware(numBoards,numActuators, boardLookUp, portLookUp, jointId, etherIP, hostIP);
  // connect to hardware (gazebo in this case)
  h->init();

  // access to all the gazebo sensors
  GazeboSensors    *s  = new GazeboSensors();

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize controllers                                   */
  /*                                                                                     */
  /*         TODO: move to mechanism_control                                             */
  /*                                                                                     */
  /***************************************************************************************/
  // stubs
  controller::ArmController          myArm;
  controller::HeadController         myHead;
  controller::SpineController        mySpine;
  controller::BaseController         myBase;
  controller::LaserScannerController myLaserScanner;
  controller::GripperController      myGripper;

  // used here in this test
  controller::JointController*       jc[PR2::MAX_JOINTS];  // One controller per joint

  // initialize each jointController jc[i], associate with a joint joint[i]
  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    jc[i]  = new controller::JointController();
    jc[i]->init(1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,-1000,1000,joint[i]);
  }

  //Explicitly initialize the controllers we wish to use. Don't forget to set the controllers to torque mode in the world file! 
  jc[PR2::ARM_L_PAN]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_PAN]);
  jc[PR2::ARM_L_SHOULDER_PITCH]->init(1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,-1000,1000,joint[PR2::ARM_L_SHOULDER_PITCH]);
  jc[PR2::ARM_L_SHOULDER_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_SHOULDER_ROLL]);
  jc[PR2::ARM_L_ELBOW_PITCH]->init(300,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_ELBOW_PITCH]);
  jc[PR2::ARM_L_ELBOW_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_ELBOW_ROLL]);
  jc[PR2::ARM_L_WRIST_PITCH]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_WRIST_PITCH]);
  jc[PR2::ARM_L_WRIST_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_L_WRIST_ROLL]);

  jc[PR2::ARM_R_PAN]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_PAN]);
  jc[PR2::ARM_R_SHOULDER_PITCH]->init(1000,0,0,500,-500, controller::CONTROLLER_POSITION,time,-1000,1000,joint[PR2::ARM_R_SHOULDER_PITCH]);
  jc[PR2::ARM_R_SHOULDER_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_SHOULDER_ROLL]);
  jc[PR2::ARM_R_ELBOW_PITCH]->init(300,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_ELBOW_PITCH]);
  jc[PR2::ARM_R_ELBOW_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_ELBOW_ROLL]);
  jc[PR2::ARM_R_WRIST_PITCH]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_WRIST_PITCH]);
  jc[PR2::ARM_R_WRIST_ROLL]->init(100,0,0,500,-500, controller::CONTROLLER_POSITION,time,-100,100,joint[PR2::ARM_R_WRIST_ROLL]);

  //JointController::init(double PGain, double IGain, double DGain, double IMax,
  //                      double IMin, CONTROLLER_CONTROL_MODE mode, double time,
  //                      double minEffore,
  //                      double maxEffort, mechanism::Joint *joint)

  /***************************************************************************************/
  /*                                                                                     */
  /*                        initialize transmission                                      */
  /*                                                                                     */
  /*       TODO: in Robot                                                                */
  /*                                                                                     */
  /***************************************************************************************/
  SimpleTransmission* st[PR2::MAX_JOINTS];
  for (int i = 0;i<PR2::MAX_JOINTS;i++){
    st[i] = new SimpleTransmission(joint[i],&hi->actuator[0],1,1,1);
  }
  
  //myPR2->hw.GetSimTime(&time);

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize ROS Gazebo Nodes                              */
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
  /*                                on termination...                                    */
  /*                                                                                     */
  /***************************************************************************************/
  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

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

    // get encoder counts from hardware
    h->updateState();


    for (int i = 0;i<PR2::MAX_JOINTS;i++){
      // setup joint state from encoder counts
      st[i]->propagatePosition();
      //update controller
      if (pthread_mutex_trylock(&simMutex[i])==0)
      {
        jc[i]->update();
        pthread_mutex_unlock(&simMutex[i]); //Unlock after we're done with r/w
      }
      // update command current from joint command
      st[i]->propagateEffort();
    }

    //cout << "pos:: " << joint->position << ", eff:: " << joint->commandedEffort << endl;

    // send command to hardware
    h->sendCommand();

    // refresh hardware
    h->tick();


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
