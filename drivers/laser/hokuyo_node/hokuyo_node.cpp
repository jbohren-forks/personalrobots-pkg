/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**

@mainpage

@htmlinclude manifest.html

@b hokuyo_node is a driver for SCIP2.0 compliant Hokuyo laser range-finders.
This driver has been designed, primarliy with the Hokuyo UTM-30LX in mind, also 
known as the Hokuyo Top-URG.

<hr>

@section information Information

Hokuyo scans are taken in a counter-clockwise direction.  Angles are measured
counter clockwise with 0 pointing directly forward.

<hr>

@section usage Usage
@verbatim
$ hokuyo_node [standard ROS args]
@endverbatim

@par Example

@verbatim
$ hokuyo_node
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "scan"/<a href="../../std_msgs/html/classstd__msgs_1_1LaserScan.html">LaserScan</a> : scan data from the laser.

<hr>

@section services
 - @b "~self_test"    :  <a href="../../std_msgs/html/classrobot__msgs_1_1SelfTest.html">SelfTest</a>

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "~min_ang"       : @b [double] the angle of the first range measurement in degrees (Default: -90.0)
- @b "~max_ang"       : @b [double] the angle of the last range measurement in degrees (Default: 90.0)
- @b "~cluster"       : @b [int]    the number of adjascent range measurements to cluster into a single reading (Default: 1)
- @b "~skip"          : @b [int]    the number of scans to skip between each measured scan (Default: 1)
- @b "~port"          : @b [string] the port where the hokuyo device can be found (Default: "/dev/ttyACM0")
- @b "~autostart      : @b [bool]   whether the node should automatically start the hokuyo (Default: true)
- @b "~calibrate_time : @b [bool]   whether the node should calibrate the hokuyo's time offset (Default: true)
- @b "~frame_id       : @b [string] the frame in which laser scans will be returned (Default: "FRAMEID_LASER")

 **/

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ros/node.h"
#include "ros/time.h"
#include "ros/common.h"

#include "std_msgs/LaserScan.h"

#include "self_test/self_test.h"

#include "hokuyo.h"



using namespace std;

class HokuyoNode: public ros::node
{
private:
  hokuyo::LaserScan  scan_;
  hokuyo::LaserConfig cfg_;

  bool running_;
  
  int count_;
  ros::Time next_time_;

  SelfTest<HokuyoNode> self_test_;

public:
  hokuyo::Laser laser_;
  std_msgs::LaserScan scan_msg_;

  double min_ang_;
  double max_ang_;
  int cluster_;
  int skip_;
  string port_;
  bool autostart_;
  bool calibrate_time_;
  string frameid_;

  HokuyoNode() : ros::node("hokuyo"), running_(false), count_(0), self_test_(this)
  {
    advertise<std_msgs::LaserScan>("scan", 100);

    param("~min_ang", min_ang_, -90.0);
    min_ang_ *= M_PI/180;
    param("~max_ang", max_ang_, 90.0);
    max_ang_ *= M_PI/180;
    param("~cluster", cluster_, 1);
    param("~skip", skip_, 1);
    param("~port", port_, string("/dev/ttyACM0"));
    param("~autostart", autostart_, true);
    param("~calibrate_time", calibrate_time_, true);
    param("~frameid", frameid_, string("FRAMEID_LASER"));

    self_test_.setPretest( &HokuyoNode::pretest );

    self_test_.addTest( &HokuyoNode::interruptionTest );
    self_test_.addTest( &HokuyoNode::connectTest );
    self_test_.addTest( &HokuyoNode::IDTest );
    self_test_.addTest( &HokuyoNode::statusTest );
    self_test_.addTest( &HokuyoNode::laserTest );
    self_test_.addTest( &HokuyoNode::polledDataTest );
    self_test_.addTest( &HokuyoNode::streamedDataTest );
    self_test_.addTest( &HokuyoNode::streamedIntensityDataTest );
    self_test_.addTest( &HokuyoNode::laserOffTest );
    self_test_.addTest( &HokuyoNode::disconnectTest );
    self_test_.addTest( &HokuyoNode::resumeTest );

  }

  ~HokuyoNode()
  {
    stop();
  }

  int start()
  {
    stop();

    try
    {
      laser_.open(port_.c_str());

      string id = laser_.getID();
      log(ros::INFO, "Connected to device with ID: %s", id.c_str());

      laser_.laserOn();

      if (calibrate_time_)
        laser_.calcLatency(true, min_ang_, max_ang_, cluster_, skip_);
        
      int status = laser_.requestScans(true, min_ang_, max_ang_, cluster_, skip_);

      if (status != 0) {
        log(ros::WARNING,"Failed to request scans from device.  Status: %d.", status);
        return -1;
      }

      running_ = true;

    } catch (hokuyo::Exception& e) {
      log(ros::WARNING,"Exception thrown while starting urg.\n%s", e.what());
      return -1;
    }

    next_time_ = ros::Time::now();

    return(0);
  }

  int stop()
  {
    if(running_)
    {
      try
      {
        laser_.close();
      } catch (hokuyo::Exception& e) {
        log(ros::WARNING,"Exception thrown while trying to close:\n%s",e.what());
      }
      running_ = false;
    }

    return 0;
  }

  int publishScan()
  {
    try
    {
      int status = laser_.serviceScan(&scan_);
      
      if(status != 0)
      {
        log(ros::WARNING,"Error getting scan: %d", status);
        return 0;
      }
    } catch (hokuyo::CorruptedDataException &e) {
      log(ros::WARNING,"Skipping corrupted data");
      return 0;
    } catch (hokuyo::Exception& e) {
      log(ros::WARNING,"Exception thrown while trying to get scan.\n%s", e.what());
      running_ = false; //If we're here, we are no longer running
      return -1;
    }
    
    scan_msg_.angle_min = scan_.config.min_angle;
    scan_msg_.angle_max = scan_.config.max_angle;
    scan_msg_.angle_increment = scan_.config.ang_increment;
    scan_msg_.time_increment = scan_.config.time_increment;
    scan_msg_.scan_time = scan_.config.scan_time;
    scan_msg_.range_min = scan_.config.min_range;
    scan_msg_.range_max = scan_.config.max_range;
    scan_msg_.set_ranges_size(scan_.num_readings);
    scan_msg_.set_intensities_size(scan_.num_readings);
    scan_msg_.header.stamp = ros::Time(scan_.system_time_stamp);
    scan_msg_.header.frame_id = frameid_;
      
    for(int i = 0; i < scan_.num_readings; ++i)
    {
      scan_msg_.ranges[i]  = scan_.ranges[i];
      scan_msg_.intensities[i] = scan_.intensities[i];
    }

    publish("scan", scan_msg_);

    count_++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time_) {
      std::cout << count_ << " scans/sec at " << now_time << std::endl;
      count_ = 0;
      next_time_ = next_time_ + ros::Duration(1,0);
    }

    return(0);
  }

  bool spin()
  {
    // Start up the laser
    while (ok())
    {
      if (autostart_ && start() == 0)
      {
        while(ok()) {
          if(publishScan() < 0)
            break;
          self_test_.checkTest();
        }
      } else {
        usleep(1000000);
        self_test_.checkTest();
      }
    }

    //stopping should be fine even if not running
    stop();

    return true;
  }


  void pretest()
  {
    // Stop for good measure.
    try
    {
      laser_.close();
    } catch (hokuyo::Exception& e) {
      // Ignore exception here.
    }
  }

  void interruptionTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Interruption Test";

    if (num_subscribers("scan") == 0)
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

  void connectTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Connection Test";

    laser_.open(port_.c_str());

    status.level = 0;
    status.message = "Connected successfully.";
  }

  void IDTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "ID Test";

    string id = laser_.getID();

    if (id == std::string("H0000000"))
    {
      status.level = 1;
      status.message = id + std::string(" is indication of failure.");
    }
    else
    {
      status.level = 0;
      status.message = id;
    }

    self_test_.setID(id);
  }

  void statusTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Status Test";

    std::string stat = laser_.getStatus();

    if (stat != std::string("Sensor works well."))
    {
      status.level = 2;
    } else {
      status.level = 0;
    }

    status.message = stat;
  }

  void laserTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Laser Test";

    laser_.laserOn();

    status.level = 0;
    status.message = "Laser turned on successfully.";
  }

  void polledDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Polled Data Test";

    hokuyo::LaserScan  scan;

    int res = laser_.pollScan(&scan, min_ang_, max_ang_, cluster_, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {
      status.level = 0;
      status.message = "Polled Hokuyo for data successfully.";
    }
  }

  void streamedDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Streamed Data Test";

    hokuyo::LaserScan  scan;

    int res = laser_.requestScans(false, min_ang_, max_ang_, cluster_, skip_, 99, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {

      for (int i = 0; i < 99; i++)
      {
        laser_.serviceScan(&scan, 1000);
      }

      status.level = 0;
      status.message = "Streamed data from Hokuyo successfully.";

    }
  }

  void streamedIntensityDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Streamed Intensity Data Test";

    hokuyo::LaserScan  scan;

    int res = laser_.requestScans(false, min_ang_, max_ang_, cluster_, skip_, 99, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {

      int corrupted_data = 0;

      for (int i = 0; i < 99; i++)
      {
        try {
          laser_.serviceScan(&scan, 1000);
        } catch (hokuyo::CorruptedDataException &e) {
          corrupted_data++;
        }
      }
      if (corrupted_data == 1)
      {
        status.level = 1;
        status.message = "Single corrupted message.  This is acceptable and unavoidable";
      } else if (corrupted_data > 1)
      {
        status.level = 2;
        ostringstream oss;
        oss << corrupted_data << " corrupted messages.";
        status.message = oss.str();
      } else
      {
        status.level = 0;
        status.message = "Stramed data with intensity from Hokuyo successfully.";
      }
    }
  }

  void laserOffTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Laser Off Test";

    laser_.laserOff();

    status.level = 0;
    status.message = "Laser turned off successfully.";
  }

  void disconnectTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Disconnect Test";

    laser_.close();

    status.level = 0;
    status.message = "Disconnected successfully.";
  }

  void resumeTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Resume Test";

    if (running_)
    {
      laser_.open(port_.c_str());
      laser_.laserOn();

      int res = laser_.requestScans(true, min_ang_, max_ang_, cluster_, skip_);

      if (res != 0)
      {
        status.level = 2;
        status.message = "Failed to resume previous mode of operation.";
        return;
      }
    }

    status.level = 0;
    status.message = "Previous operation resumed successfully.";    
  }

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  HokuyoNode h;

  h.spin();

  ros::fini();

  return(0);
}
