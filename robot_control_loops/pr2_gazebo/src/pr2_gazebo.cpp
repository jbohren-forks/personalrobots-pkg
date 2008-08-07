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

// gazebo
#include <libpr2API/pr2API.h>

// roscpp
#include <ros/node.h>

#include <time.h>
#include <signal.h>


#include <RosGazeboNode/RosGazeboNode.h>

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
    ((RosGazeboNode*)rgn)->Update();
    // some time out for publishing ros info
    usleep(10000);
  }

}

int 
main(int argc, char** argv)
{ 
  // we need 2 threads, one for RT and one for nonRT
  pthread_t threads[2];

  ros::init(argc,argv);

  /***************************************************************************************/
  /*                                                                                     */
  /*                           The main simulator object                                 */
  /*                                                                                     */
  /***************************************************************************************/
  PR2::PR2Robot* myPR2;
  // Initialize robot object
  myPR2 = new PR2::PR2Robot();
  // Initialize connections
  myPR2->InitializeRobot();
  // Set control mode for the base
  myPR2->SetBaseControlMode(PR2::PR2_CARTESIAN_CONTROL);

  /***************************************************************************************/
  /*                                                                                     */
  /*                        build actuators from pr2Actuators.xml                        */
  /*                                                                                     */
  /***************************************************************************************/
  
  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize controllers                                   */
  /*                                                                                     */
  /***************************************************************************************/

  /***************************************************************************************/
  /*                                                                                     */
  /*                            initialize ROS Gazebo Nodes                              */
  /*                                                                                     */
  /***************************************************************************************/
  RosGazeboNode rgn(argc,argv,argv[1],myPR2);

  /***************************************************************************************/
  /*                                                                                     */
  /*                                on termination...                                    */
  /*                                                                                     */
  /***************************************************************************************/
  signal(SIGINT,  (&finalize));
  signal(SIGQUIT, (&finalize));
  signal(SIGTERM, (&finalize));

  // let rosgazebonode read the xml data from pr2.xml in ros
  rgn.LoadRobotModel();

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
  while(1)
  {

    // Update Controllers
    //   each controller will try to read new commands from shared memory with nonRT hooks,
    //   and skip update if locked by nonRT loop.

    // TODO: Safety codes should go here...

    // Send updated controller commands to hardware
    myPR2->hw.UpdateHW();

    // wait for Gazebo time step
    myPR2->hw.ClientWait();
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
