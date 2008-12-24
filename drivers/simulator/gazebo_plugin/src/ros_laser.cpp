/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
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
 */
/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: ros_laser.cpp 6683 2008-06-25 19:12:30Z natepak $
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugin/ros_laser.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include "RaySensor.hh"

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_laser", RosLaser);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosLaser::RosLaser(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<RaySensor*>(this->parent);

  if (!this->myParent)
    gzthrow("RosLaser controller requires a Ray Sensor as its parent");

  // set parent sensor to active automatically
  this->myParent->SetActive(true);

  rosnode = ros::g_node; // comes from where?  common.h exports as global variable
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    // start a ros node if none exist
    ros::init(argc,argv);
    rosnode = new ros::node("ros_gazebo",ros::node::DONT_HANDLE_SIGINT);
    printf("-------------------- starting node in laser \n");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosLaser::~RosLaser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosLaser::LoadChild(XMLConfigNode *node)
{
  this->topicName = node->GetString("topicName","default_ros_laser",0); //read from xml file
  std::cout << "================= " << this->topicName <<  std::endl;
  rosnode->advertise<std_msgs::LaserScan>(this->topicName,10);
  this->frameName = node->GetString("frameName","default_ros_laser",0); //read from xml file
  this->gaussianNoise = node->GetDouble("gaussianNoise",0.0,0); //read from xml file
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosLaser::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosLaser::UpdateChild()
{
    this->PutLaserData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosLaser::FiniChild()
{
  rosnode->unadvertise(this->topicName);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void RosLaser::PutLaserData()
{
  int i, ja, jb;
  double ra, rb, r, b;
  int v;

  Angle maxAngle = this->myParent->GetMaxAngle();
  Angle minAngle = this->myParent->GetMinAngle();

  double maxRange = this->myParent->GetMaxRange();
  double minRange = this->myParent->GetMinRange();
  int rayCount = this->myParent->GetRayCount();
  int rangeCount = this->myParent->GetRangeCount();

  /***************************************************************/
  /*                                                             */
  /*  point scan from laser                                      */
  /*                                                             */
  /***************************************************************/
  this->lock.lock();
  // Add Frame Name
  this->laserMsg.header.frame_id = this->frameName;
  this->laserMsg.header.stamp.sec = (unsigned long)floor(Simulator::Instance()->GetSimTime());
  this->laserMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  Simulator::Instance()->GetSimTime() - this->laserMsg.header.stamp.sec) );


  double tmp_res_angle = (maxAngle.GetAsRadian() - minAngle.GetAsRadian())/((double)(rangeCount -1)); // for computing yaw
  int    num_channels = 1;
  this->laserMsg.angle_min = minAngle.GetAsRadian();
  this->laserMsg.angle_max = maxAngle.GetAsRadian();
  this->laserMsg.angle_increment = tmp_res_angle;
  this->laserMsg.time_increment  = 0; // instantaneous simulator scan
  this->laserMsg.scan_time       = 0; // FIXME: what's this?
  this->laserMsg.range_min = minRange;
  this->laserMsg.range_max = maxRange;
  this->laserMsg.set_ranges_size(rangeCount);
  this->laserMsg.set_intensities_size(rangeCount);

  // Interpolate the range readings from the rays
  for (i = 0; i<rangeCount; i++)
  {
    b = (double) i * (rayCount - 1) / (rangeCount - 1);
    ja = (int) floor(b);
    jb = std::min(ja + 1, rayCount - 1);
    b = b - floor(b);

    assert(ja >= 0 && ja < rayCount);
    assert(jb >= 0 && jb < rayCount);

    ra = std::min(this->myParent->GetRange(ja) , maxRange);
    rb = std::min(this->myParent->GetRange(jb) , maxRange);

    // Range is linear interpolation if values are close,
    // and min if they are very different
    //if (fabs(ra - rb) < 0.10)
      r = (1 - b) * ra + b * rb;
    //else r = std::min(ra, rb);

    // Intensity is either-or
    v = (int) this->myParent->GetRetro(ja) || (int) this->myParent->GetRetro(jb);

    /***************************************************************/
    /*                                                             */
    /*  point scan from laser                                      */
    /*                                                             */
    /***************************************************************/
    if (r == maxRange)
      this->laserMsg.ranges[i]        = r + minRange; // no noise if at max range
    else
      this->laserMsg.ranges[i]        = r + minRange + this->GaussianKernel(0,this->gaussianNoise) ;
    this->laserMsg.intensities[i]   = v + this->GaussianKernel(0,this->gaussianNoise) ;
  }

  // send data out via ros message
  rosnode->publish(this->topicName,this->laserMsg);
  this->lock.unlock();

}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double RosLaser::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}



