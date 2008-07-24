#pragma once
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

// gazebo
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <libpr2API/pr2API.h>
#include <libpr2HW/pr2HW.h>

#include <pr2Controllers/ArmController.h>
#include <pr2Controllers/HeadController.h>
#include <pr2Controllers/SpineController.h>
#include <pr2Controllers/BaseController.h>
#include <pr2Controllers/LaserScannerController.h>
#include <pr2Controllers/GripperController.h>

#include "ringbuffer.h"
#include "mechanism_model/joint.h"
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

#include <std_msgs/EndEffectorState.h>

// roscpp - camera
#include <std_msgs/Image.h>

// for frame transforms
#include <rosTF/rosTF.h>

#include <time.h>

// Our node
class RosGazeboNode : public ros::node
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

    // time step calculation
    double lastTime, simTime;

    // smooth vx, vw commands
    double vxSmooth, vwSmooth;

    // used to generate Gaussian noise (for PCD)
    double GaussianKernel(double mu,double sigma);

    // used to generate Gaussian noise (for PCD)
    PR2::PR2Robot *PR2Copy;
    controller::ArmController          *armCopy;
    controller::HeadController         *headCopy;
    controller::SpineController        *spineCopy;
    controller::BaseController         *baseCopy;
    controller::LaserScannerController *laserScannerCopy;
    controller::GripperController      *gripperCopy;

    //Copy data from message to Joint Array. Required to be in update for thread locking.
    void UpdateLeftArm();
    void UpdateRightArm();

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    RosGazeboNode(int argc, char** argv, const char* fname,
         PR2::PR2Robot          *myPR2,
         controller::ArmController          *myArm,
         controller::HeadController         *myHead,
         controller::SpineController        *mySpine,
         controller::BaseController         *myBase,
         controller::LaserScannerController *myLaserScanner,
         controller::GripperController      *myGripper
         );
    ~RosGazeboNode();
   // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    RosGazeboNode(int argc, char** argv, const char* fname,
         PR2::PR2Robot          *myPR2,
         controller::ArmController          *myArm,
         controller::HeadController         *myHead,
         controller::SpineController        *mySpine,
         controller::BaseController         *myBase,
         controller::LaserScannerController *myLaserScanner,
         controller::GripperController      *myGripper,
         controller::JointController** ControllerArray
         );

    // advertise / subscribe models
    int AdvertiseSubscribeMessages();

    // Do one update of the simulator.  May pause if the next update time
    // has not yet arrived.
    void Update();

    // Message callback for a std_msgs::BaseVel message, which set velocities.
    void cmdvelReceived();

    // Message callback for a std_msgs::PR2Arm message, which sets arm configuration.
    void cmd_leftarmconfigReceived();
    void cmd_rightarmconfigReceived();

    void cmd_leftarmcartesianReceived();
    void cmd_rightarmcartesianReceived();

    // laser range data
    float    ranges[GZ_LASER_MAX_RANGES];
    uint8_t  intensities[GZ_LASER_MAX_RANGES];

    // camera data
    std_msgs::Image img;
    std_msgs::Image img_ptz_right;
    std_msgs::Image img_ptz_left;
    std_msgs::Image img_wrist_right;
    std_msgs::Image img_wrist_left;
    std_msgs::Image img_forearm_right;
    std_msgs::Image img_forearm_left;
    
    // arm joint data
    std_msgs::PR2Arm leftarm;
    std_msgs::PR2Arm rightarm;

    // end effector cmds
    std_msgs::EndEffectorState cmd_leftarmcartesian;
    std_msgs::EndEffectorState cmd_rightarmcartesian;

    //Flags to indicate that a new message has arrived
    bool newRightArmPos;
    bool newLeftArmPos;

    //Flag set to indicate that we should use new controls architecture
    bool useControllerArray; 

    // for the point cloud data
    ringBuffer<std_msgs::Point3DFloat32> *cloud_pts;
    ringBuffer<float>                    *cloud_ch1;

    vector<std_msgs::Point3DFloat32> *full_cloud_pts;
    vector<float>                    *full_cloud_ch1;

    // keep count for full cloud
    int max_cloud_pts;
    int max_full_cloud_pts;

    //Keep track of controllers
    controller::JointController** ControllerArray;
};




