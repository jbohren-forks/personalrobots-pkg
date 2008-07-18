#pragma once
/*
 *  ROS WRAPPER FOR JOINT CONTROLLER
 *
 *  stubbed, to be filled in
 *
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

#include <genericControllers/JointController.h>

// roscpp
#include <ros/node.h>
// roscpp - laser
#include <std_msgs/LaserScan.h>
// roscpp - laser image (point cloud)
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Point3DFloat32.h>
#include <std_msgs/ChannelFloat32.h>
// roscpp - used for shutter message right now
#include <std_msgs/Empty.h>
// roscpp - used for broadcasting time over ros
#include <rostools/Time.h>
// roscpp - base
#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/BaseVel.h>
// roscpp - arm
#include <std_msgs/PR2Arm.h>
// roscpp - camera
#include <std_msgs/Image.h>

// for frame transforms
#include <rosTF/rosTF.h>

#include <time.h>
#include <iostream>

// Our node
class RosJointController : public ros::node
{
  private:
    // Messages that we'll send or receive
    std_msgs::BaseVel velMsg;
    std_msgs::LaserScan laserMsg;
    std_msgs::PointCloudFloat32 cloudMsg;
    std_msgs::PointCloudFloat32 full_cloudMsg;
    std_msgs::Empty shutterMsg;  // marks end of a cloud message
    std_msgs::RobotBase2DOdom odomMsg;
    rostools::Time timeMsg;

    // A mutex to lock access to fields that are used in message callbacks
    ros::thread::mutex lock;

    // for frame transforms, publish frame transforms
    rosTFServer tf;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    RosJointController(std::string jointName);
    ~RosJointController();

    // advertise / subscribe models
    int advertiseSubscribeMessages();

    void init(CONTROLLER::JointController *jc);
    // Do one update of the simulator.  May pause if the next update time
    // has not yet arrived.
    void update();

    // Message callback for a std_msgs::BaseVel message, which set velocities.
    void cmdReceived();

    //Keep track of controllers
    CONTROLLER::JointController* jc;
};




