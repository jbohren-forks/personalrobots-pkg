/*
 * hokuyourg_player
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@mainpage

@htmlinclude manifest.html

@b The imu_node is designed to make use of the microstrain inertialink
or 3dmgx2 IMUs and makes use of the 3dmgx2_driver.

<hr>

@section information Information

The IMU provides a single message PoseWithRatesStamped messaged at 100Hz
which is taken from the 3DMGX2 ACCEL_ANGRATE_ORIENTATION message.

<hr>

@section usage Usage

@par Example

@verbatim
$ imu_node
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "imu_data"/<a href="../../std_msgs/html/classstd__msgs_1_1PoseWithRatesStamped.html">std_msgs/PoseWithRatesStamped</a> : the imu data
- @b "/diagnostics"/<a href="../../robot_msgs/html/classrobot__msgs_1_1DiagnosticMessage.html">robot_msgs/DiagnosticMessage</a> : diagnostic status information.

<hr>

@section services
 - @b "~self_test"    :  SelfTest service provided by SelfTest helper class

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

 - @b "~port"      : @b [string] the port the imu is running on
 - @b "~frame_id"  : @b [string] the frame in which imu readings will be returned (Default: "imu")
 - @b "~autostart" : @b [bool] whether the imu starts on its own (this is only useful for bringing up an imu in test mode)

 **/

#include <assert.h>
#include <math.h>
#include <iostream>

#include "3dmgx2.h"

#include "ros/node.h"
#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include "std_msgs/PoseWithRatesStamped.h"

#include "tf/transform_datatypes.h"
#include "imu_node/AddOffset.h"

using namespace std;

class ImuNode: public ros::Node
{
public:
  MS_3DMGX2::IMU imu;
  std_msgs::PoseWithRatesStamped reading;

  string port;

  MS_3DMGX2::IMU::cmd cmd;

  int count_;

  SelfTest<ImuNode> self_test_;
  DiagnosticUpdater<ImuNode> diagnostic_;

  bool running;

  bool autostart;

  string frameid_;
  
  double offset_;

  ImuNode() : ros::Node("imu"), count_(0), self_test_(this), diagnostic_(this)
  {
    advertise<std_msgs::PoseWithRatesStamped>("imu_data", 100);

    advertise_service("imu/add_offset", &ImuNode::addOffset, this);

    param("~port", port, string("/dev/ttyUSB0"));

    param("~autostart", autostart, true);

    cmd = MS_3DMGX2::IMU::CMD_ACCEL_ANGRATE_ORIENT;
    
    running = false;

    param("~frameid", frameid_, string("imu"));

    param("~time_offset", offset_, 0.0);

    self_test_.setPretest(&ImuNode::pretest);
    self_test_.addTest(&ImuNode::InterruptionTest);
    self_test_.addTest(&ImuNode::ConnectTest);
    self_test_.addTest(&ImuNode::GyroBiasTest);
    self_test_.addTest(&ImuNode::StreamedDataTest);
    self_test_.addTest(&ImuNode::GravityTest);
    self_test_.addTest(&ImuNode::DisconnectTest);
    self_test_.addTest(&ImuNode::ResumeTest);

    diagnostic_.addUpdater( &ImuNode::freqStatus );
  }

  ~ImuNode()
  {
    stop();
  }

  int start()
  {
    stop();

    try
    {
      imu.open_port(port.c_str());

      ROS_INFO("Initializing IMU sensor.");

      imu.init_gyros();

      ROS_INFO("Initializing IMU time with offset %f.", offset_);

      imu.init_time(offset_);

      ROS_INFO("IMU sensor initialized.");

      imu.set_continuous(cmd);

      running = true;

    } catch (MS_3DMGX2::exception& e) {
      ROS_INFO("Exception thrown while starting IMU.\n %s", e.what());
      return -1;
    }

    return(0);
  }
  
  int stop()
  {
    if(running)
    {
      try
      {
        imu.close_port();
      } catch (MS_3DMGX2::exception& e) {
        ROS_INFO("Exception thrown while stopping IMU.\n %s", e.what());
      }
      running = false;
    }

    return(0);
  }

  int publish_datum()
  {
    try
    {
      uint64_t time;

      double accel[3];
      double angrate[3];
      double orientation[9];

      imu.receive_accel_angrate_orientation(&time, accel, angrate, orientation);

      reading.acc.acc.ax = accel[0];
      reading.acc.acc.ay = accel[1];
      reading.acc.acc.az = accel[2];
 
      reading.vel.ang_vel.vx = angrate[0];
      reading.vel.ang_vel.vy = angrate[1];
      reading.vel.ang_vel.vz = angrate[2];
      
      btTransform pose(btMatrix3x3(orientation[0], orientation[1], orientation[2],
                                   orientation[3], orientation[4], orientation[5],
                                   orientation[6], orientation[7], orientation[8]), 
                       btVector3(0,0,0));

      tf::PoseTFToMsg(pose, reading.pos);
      
      
      reading.header.stamp = ros::Time().fromNSec(time);
      reading.header.frame_id = frameid_;

      publish("imu_data", reading);
        
    } catch (MS_3DMGX2::exception& e) {
      ROS_INFO("Exception thrown while trying to get the IMU reading.\n%s", e.what());
      return -1;
    }

    count_++;

    return(0);
  }

  bool spin()
  {
    // Start up the laser
    while (ok())
    {
      if (autostart && start() == 0)
      {
        while(ok()) {
          if(publish_datum() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
        }
      } else {
        usleep(1000000);
        self_test_.checkTest();
        diagnostic_.update();
      }
    }

    stop();

    return true;
  }


  void pretest()
  {
    try
    {
      imu.close_port();
    } catch (MS_3DMGX2::exception& e) {
    }
  }

  void InterruptionTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Interruption Test";

    if (num_subscribers("imu_data") == 0 )
    {
      status.level = 0;
      status.message = "No operation interrupted.";
    }
    else
    {
      status.level = 1;
      status.message = "There were active subscribers.  Running of self test interrupted operations.";
    }
  }

  void ConnectTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Connection Test";

    imu.open_port(port.c_str());

    status.level = 0;
    status.message = "Connected successfully.";
  }

  void GyroBiasTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Gyro Bias Test";

    double bias_x;
    double bias_y;
    double bias_z;
    
    imu.init_gyros(&bias_x, &bias_y, &bias_z);

    status.level = 0;
    status.message = "Successfully calculated gyro biases.";

    status.set_values_size(3);
    status.values[0].label = "Bias_X";
    status.values[0].value = bias_x;
    status.values[1].label = "Bias_Y";
    status.values[1].value = bias_y;
    status.values[2].label = "Bias_Z";
    status.values[2].value = bias_z;

  }


  void StreamedDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Streamed Data Test";

    uint64_t time;
    double accel[3];
    double angrate[3];

    if (!imu.set_continuous(MS_3DMGX2::IMU::CMD_ACCEL_ANGRATE))
    {
      status.level = 2;
      status.message = "Could not start streaming data.";
    } else {

      for (int i = 0; i < 100; i++)
      {
        imu.receive_accel_angrate(&time, accel, angrate);
      }
      
      imu.stop_continuous();

      status.level = 0;
      status.message = "Data streamed successfully.";
    }
  }

  void GravityTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Streamed Data Test";

    uint64_t time;
    double accel[3];
    double angrate[3];

    double grav = 0.0;

    double grav_x = 0.0;
    double grav_y = 0.0;
    double grav_z = 0.0;

    if (!imu.set_continuous(MS_3DMGX2::IMU::CMD_ACCEL_ANGRATE))
    {
      status.level = 2;
      status.message = "Could not start streaming data.";
    } else {

      int num = 200;

      for (int i = 0; i < num; i++)
      {
        imu.receive_accel_angrate(&time, accel, angrate);
        
        grav_x += accel[0];
        grav_y += accel[1];
        grav_z += accel[2];

      }
      
      imu.stop_continuous();

      grav += sqrt( pow(grav_x / (double)(num), 2.0) + 
                    pow(grav_y / (double)(num), 2.0) + 
                    pow(grav_z / (double)(num), 2.0));
      
      //      double err = (grav - MS_3DMGX2::G);
      double err = (grav - 9.796);
      
      if (fabs(err) < .05)
      {
        status.level = 0;
        status.message = "Gravity detected correctly.";
      } else {
        status.level = 2;
        ostringstream oss;
        oss << "Measured gravity deviates by " << err;
        status.message = oss.str();
      }

      status.set_values_size(2);
      status.values[0].label = "Measured gravity";
      status.values[0].value = grav;
      status.values[1].label = "Gravity error";
      status.values[1].value = err;
    }
  }


  void DisconnectTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Disconnect Test";

    imu.close_port();

    status.level = 0;
    status.message = "Disconnected successfully.";
  }

  void ResumeTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Resume Test";

    if (running)
    {

      imu.open_port(port.c_str());

      if (imu.set_continuous(cmd) != true)
      {
        status.level = 2;
        status.message = "Failed to resume previous mode of operation.";
        return;
      }
    }

    status.level = 0;
    status.message = "Previous operation resumed successfully.";    
  }

  void freqStatus(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Frequency Status";

    double desired_freq = 100.0;
    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq))
    {
      status.level = 2;
      status.message = "Desired frequency not met";
    }
    else
    {
      status.level = 0;
      status.message = "Desired frequency met";
    }

    status.set_values_size(3);
    status.values[0].label = "Scans in interval";
    status.values[0].value = count_;
    status.values[1].label = "Desired frequency";
    status.values[1].value = desired_freq;
    status.values[2].label = "Actual frequency";
    status.values[2].value = freq;

    count_ = 0;
  }



  bool addOffset(imu_node::AddOffset::request &req, imu_node::AddOffset::response &resp)
  {
    double offset = req.add_offset;
    offset_ += offset;

    ROS_INFO("Adding %f to existing IMU time offset.", offset);
    ROS_INFO("Total IMU time offset is now %f.", offset_);

    // send changes to inu driver
    imu.set_fixed_offset(offset_);

    // write changes to param server
    set_param("~time_offset", offset_);

    // set response
    resp.total_offset = offset_;

    return true;
  }



};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  ImuNode in;

  in.spin();

  ros::fini();

  return(0);
}
