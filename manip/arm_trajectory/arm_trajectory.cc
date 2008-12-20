/*
 *  arm trajectory node
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
 *
 *  Author: John Hsu, David Li
 *  Description: A node used to send arm trajectory messages for path following
 *  Note: This class is for testing purposes only and will be obsoleted by a node that sends kinematic states
 */

#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <iostream>

#include "ringbuffer.h"

// roscpp
#include <ros/node.h>
#include <rosthread/mutex.h>
// roscpp - arm trajectory
#include <std_msgs/ArmTrajectory.h>

#include <time.h>
#include <signal.h>

using namespace std;

// Our node
class ArmTrajectoryNode : public ros::node
{
  private:
    // Messages that we'll send or receive
    std_msgs::ArmTrajectory armTrajectoryMsg;

    // A mutex to lock access to fields that are used in message callbacks
    ros::thread::mutex lock;

   double currentVelocity[6]; //Encode x,y,z,y,p,r for one time stamp



  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    ArmTrajectoryNode(int argc, char** argv, const char* fname);
    ~ArmTrajectoryNode();

    // advertise / subscribe models
    int AdvertiseMessages();

    // Do one update of the simulator.  May pause if the next update time
    // has not yet arrived.
    void SendTrajectory();

    // for the point cloud data
    ringBuffer<std_msgs::Point32> *trajectory_p;
    ringBuffer<std_msgs::Float32>        *trajectory_v;

    // keep count for full cloud
    int trajectory_pts;

    // clean up on interrupt
    static void finalize(int);

    //Early test via keyboard
   void SendOneVelocity();
};


ArmTrajectoryNode::ArmTrajectoryNode(int argc, char** argv, const char* fname) :
        ros::node("arm_trajectory_node")
{
  // initialize random seed
  srand(time(NULL));

  // Initialize ring buffer for point cloud data
  this->trajectory_p = new ringBuffer<std_msgs::Point32>();
  this->trajectory_v = new ringBuffer<std_msgs::Float32       >();

  // FIXME:  move this to Advertise/Subscribe Models
  param("trajectory_pts",trajectory_pts, 10000);
  this->trajectory_p->allocate(this->trajectory_pts);
  this->trajectory_v->allocate(this->trajectory_pts);
}

void ArmTrajectoryNode::finalize(int)
{
  fprintf(stderr,"Caught sig, clean-up and exit\n");
  sleep(1);
  exit(-1);
}


int
ArmTrajectoryNode::AdvertiseMessages()
{
  advertise<std_msgs::ArmTrajectory>("arm_trajectory");
  return(0);
}

ArmTrajectoryNode::~ArmTrajectoryNode()
{
}

void ArmTrajectoryNode::SendOneVelocity(){
    char xdot[100];
    char ydot[100];
    char zdot[100];
   std_msgs::ArmTrajectory currentMsg;
	std_msgs::Float32        tmp_trajectory_v;
  this->lock.lock();

    cout<<"X dot?:";
    gets(xdot);
	cout<<"Y dot?:";
    gets(ydot);
	cout<<"Z dot?:";
    gets(zdot);



    currentVelocity[0] = atof(xdot);
    currentVelocity[1] = atof(ydot);
    currentVelocity[2] = atof(zdot);

    currentVelocity[3] = 0.0;
    currentVelocity[4] = 0.0;
    currentVelocity[5] = 0.0;

	currentMsg.set_pts_size(0);
 	currentMsg.set_vel_size(6);

    for(int ii=0;ii<6;ii++){
	tmp_trajectory_v.data = currentVelocity[ii];
	currentMsg.vel[ii] = tmp_trajectory_v;
	}
	  publish("arm_trajectory",currentMsg);
	cout<<"Send!\n"<<endl;
	this->lock.unlock();

}

void
ArmTrajectoryNode::SendTrajectory()
{
  this->lock.lock();

  std_msgs::Point32 tmp_trajectory_p;
  std_msgs::Float32        tmp_trajectory_v;

  /***************************************************************/
  /*                                                             */
  /*  publish some test trajectory                               */
  /*                                                             */
  /***************************************************************/
  for(int i=0;i < this->trajectory_pts;i++)
  {
    // specify some points
    tmp_trajectory_p.x                = (double)0;
    tmp_trajectory_p.y                = (double)0;
    tmp_trajectory_p.z                = (double)i / (double)this->trajectory_pts;
    tmp_trajectory_v.data             = 1.0;

    // push pcd point into structure
    this->trajectory_p->add((std_msgs::Point32)tmp_trajectory_p);
    this->trajectory_v->add(                          tmp_trajectory_v);
  }

  this->armTrajectoryMsg.set_pts_size(this->trajectory_pts);
  this->armTrajectoryMsg.set_vel_size(this->trajectory_pts);

  for(int i=0;i< this->trajectory_pts ;i++)
  {
    this->armTrajectoryMsg.pts[i].x    = this->trajectory_p->buffer[i].x;
    this->armTrajectoryMsg.pts[i].y    = this->trajectory_p->buffer[i].y;
    this->armTrajectoryMsg.pts[i].z    = this->trajectory_p->buffer[i].z;
    this->armTrajectoryMsg.vel[i]      = this->trajectory_v->buffer[i];
  }
  publish("arm_trajectory",this->armTrajectoryMsg);

  this->lock.unlock();
}




int
main(int argc, char** argv)
{
  ros::init(argc,argv);

  ArmTrajectoryNode gn(argc,argv,argv[1]);

  signal(SIGINT,  (&gn.finalize));
  signal(SIGQUIT, (&gn.finalize));
  signal(SIGTERM, (&gn.finalize));

  if (gn.AdvertiseMessages() != 0)
    exit(-1);

  //gn.SendTrajectory();

for(;;)
  gn.SendOneVelocity();

  std::cout << "Trajectory Sent on topic arm_trajectory" << std::endl;

  exit(0);

}
