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

@mainpage hokuyo_node

@htmlinclude manifest.html

@b hokuyo_node is a driver for SCIP2.0 compliant Hokuyo laser range-finders.
This driver has been designed, primarliy with the Hokuyo UTM-30LX in mind, also
known as the Hokuyo Top-URG. The driver has been extended to support some SCIP1.0 compliant 
range-finders such as the URG-04LX.

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
- @b "scan"/<a href="../../sensor_msgs/html/classstd__msgs_1_1LaserScan.html">sensor_msgs/LaserScan</a> : scan data from the laser.
- @b "/diagnostics"/<a href="../../robot_msgs/html/classrobot__msgs_1_1DiagnosticMessage.html">robot_msgs/DiagnosticMessage</a> : diagnostic status information.

<hr>

@section services
 - @b "~self_test"    :  SelfTest service provided by SelfTest helper class

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "~min_ang_degrees" : @b [double] the angle of the first range measurement in degrees (Default: -90.0)
- @b "~max_ang_degrees" : @b [double] the angle of the last range measurement in degrees (Default: 90.0)
- @b "~min_ang"         : @b [double] the angle of the first range measurement in radians (Default: -pi/2)
- @b "~max_ang"         : @b [double] the angle of the last range measurement in radians (Default: pi/2)
- @b "~intensity"       : @b [bool]   whether or not the hokuyo returns intensity values (Default: true)
- @b "~cluster"         : @b [int]    the number of adjascent range measurements to cluster into a single reading (Default: 1)
- @b "~skip"            : @b [int]    the number of scans to skip between each measured scan (Default: 1)
- @b "~port"            : @b [string] the port where the hokuyo device can be found (Default: "/dev/ttyACM0")
- @b "~autostart"       : @b [bool]   whether the node should automatically start the hokuyo (Default: true)
- @b "~calibrate_time"  : @b [bool]   whether the node should calibrate the hokuyo's time offset (Default: true)
- @b "~hokuyoLaserModel04LX" : @b [bool]	whether the laser is a hokuyo mode 04LX by setting boolean LaserIsHokuyoModel04LX (Default: false)
- @b "~frameid"        : @b [string] the frame in which laser scans will be returned (Default: "FRAMEID_LASER")
- @b "~reconfigure"    : @b [bool] set to true to force the node to reread its configuration, the node will reset it to false when it is reconfigured (Default: false)
 **/

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "ros/node.h"
#include "ros/time.h"
#include "ros/common.h"

#include "sensor_msgs/LaserScan.h"

#include "self_test/self_test.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

#include "hokuyo.h"

using namespace std;

class HokuyoNode: public ros::Node
{
private:
  hokuyo::LaserScan  scan_;
  hokuyo::LaserConfig cfg_;

  bool running_;

  int count_;

  SelfTest<HokuyoNode> self_test_;
  DiagnosticUpdater<HokuyoNode> diagnostic_;

public:
  hokuyo::Laser laser_;
  sensor_msgs::LaserScan scan_msg_;

  double min_ang_;
  double max_ang_;
  bool intensity_;
  int cluster_;
  int skip_;
  string port_;
  bool autostart_;
  bool calibrate_time_;
  bool LaserIsHokuyoModel04LX;
  string frameid_;
  string device_id_;
  string device_status_;
  string connect_fail_;

  HokuyoNode() : ros::Node("hokuyo"), running_(false), count_(0), self_test_(this), diagnostic_(this)
  {
    advertise<sensor_msgs::LaserScan>("scan", 100);
    
    read_config();

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
    
    diagnostic_.add( "Connection Status", this, &HokuyoNode::connectionStatus );
    diagnostic_.add( "Frequency Status", this, &HokuyoNode::freqStatus );
  }

  void read_config()
  {
    if (hasParam("~min_ang_degrees") && hasParam("~min_ang"))
    {
      ROS_FATAL("Minimum angle is specified in both radians and degrees");
      shutdown();
    }

    if (hasParam("~max_ang_degrees") && hasParam("~max_ang"))
    {
      ROS_FATAL("Maximum angle is specified in both radians and degrees");
      shutdown();
    }

    if (hasParam("~min_ang_degrees"))
    {
      getParam("~min_ang_degrees", min_ang_);
      min_ang_ *= M_PI/180;
    }
    else if (hasParam("~min_ang"))
    {
      getParam("~min_ang", min_ang_);
    }
    else
    {
      min_ang_ = -M_PI/2.0;
    }

    if (hasParam("~max_ang_degrees"))
    {
      getParam("~max_ang_degrees", max_ang_);
      max_ang_ *= M_PI/180;
    }
    else if (hasParam("~max_ang"))
    {
      getParam("~max_ang", max_ang_);
    }
    else
    {
      max_ang_ = M_PI/2.0;
    }

    param("~intensity", intensity_, true);
    param("~cluster", cluster_, 1);
    param("~skip", skip_, 1);
    param("~port", port_, string("/dev/ttyACM0"));
    param("~autostart", autostart_, true);
    param("~calibrate_time", calibrate_time_, true);
    param("~hokuyoLaserModel04LX", LaserIsHokuyoModel04LX, false);  // LaserIsHokuyoModel04LX must be set to true via this parameter for a model 04LX rangefinder
    param("~frameid", frameid_, string("FRAMEID_LASER"));
  }

  void check_reconfigure()
  {
    bool need_reconfigure;
    param("~reconfigure", need_reconfigure, false);
    if (need_reconfigure)
    {
      ROS_INFO("Reconfigured the hokuyo.");
      diagnostic_.broadcast(2,"Reconfiguring");
      bool was_running = running_;
      if (was_running)
        stop();

      read_config();

      if (was_running)
        start();

      setParam("~reconfigure", false);
      diagnostic_.force_update();
    }
  }

  ~HokuyoNode()
  {
    stop();
  }

  int start()
  {
    stop();

    ROS_DEBUG("Calling start");

    try
    {
      device_id_ = std::string("unknown");
      device_status_ = std::string("unknown");

      laser_.open(port_.c_str(), LaserIsHokuyoModel04LX);

      device_id_ = laser_.getID();
      device_status_ = laser_.getStatus();
      ROS_INFO("Connected to device with ID: %s", device_id_.c_str());

      laser_.laserOn();

      if (calibrate_time_)
      {
       // first parameter false when 04LX laser used because 04LX sensor only accepts MD commands, not ME commands
        ROS_INFO("Starting calibration");
        laser_.calcLatency(!LaserIsHokuyoModel04LX, min_ang_, max_ang_, cluster_, skip_);
        calibrate_time_ = false; // @todo Hack so that if there is a transmission the slow calibration process does not happen again.
        ROS_INFO("Calibration finished");
      }

      hokuyo::LaserConfig config;
     
      laser_.getConfig(config);

      setParam("~min_ang_limit", (double)(config.min_angle));
      setParam("~max_ang_limit", (double)(config.max_angle));
      setParam("~min_range", (double)(config.min_range));
      setParam("~max_range", (double)(config.max_range));

      // first parameter false when 04LX laser used because 04LX sensor only accepts MD commands, not ME commands
      int status = laser_.requestScans(!LaserIsHokuyoModel04LX && intensity_, min_ang_, max_ang_, cluster_, skip_);

      if (status != 0) {
        ROS_WARN("Failed to request scans from device.  Status: %d.", status);
        ROS_DEBUG("Start failed");
        return -1;
      }

      running_ = true;

    } catch (hokuyo::Exception& e) {
      ROS_WARN("Exception thrown while starting urg.\n%s", e.what());
      connect_fail_ = e.what();
      ROS_DEBUG("Start excepted");
      return -1;
    }
    
    ROS_DEBUG("Start completed successfully");

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
        ROS_WARN("Exception thrown while trying to close:\n%s",e.what());
      }
      running_ = false;
    }

    return 0;
  }

  int publishScan()
  {
    //ROS_DEBUG("publishScan");
    try
    {
      int status = laser_.serviceScan(scan_);

      if(status != 0)
      {
        ROS_WARN("Error getting scan: %d", status);
        return 0;
      }
    } catch (hokuyo::CorruptedDataException &e) {
      ROS_WARN("Skipping corrupted data");
      return 0;
    } catch (hokuyo::Exception& e) {
      ROS_WARN("Exception thrown while trying to get scan.\n%s", e.what());
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
    scan_msg_.ranges = scan_.ranges;
    scan_msg_.intensities = scan_.intensities;
    scan_msg_.header.stamp = ros::Time().fromNSec((uint64_t)scan_.system_time_stamp);
    scan_msg_.header.frame_id = frameid_;

    publish("scan", scan_msg_);

    count_++;
    
    //ROS_DEBUG("publishScan done");

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
          diagnostic_.update();
          check_reconfigure();
        }
      } else {
        usleep(100000);
        self_test_.checkTest();
        diagnostic_.update();
        check_reconfigure();
      }
    }

    //stopping should be fine even if not running
    stop();

    return true;
  }

  void connectionStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Connection Status";

    if (!running_)
    {
      status.summary(2, "Not connected. " + connect_fail_);
    }
    else if (device_status_ != std::string("Sensor works well."))
    {
      status.summary(2, "Sensor not operational");
    }
    else {
      status.summary(0, "Sensor connected");
    }

    status.add("Port", port_);
    status.add("Device ID", device_id_);
    status.add("Device Status", device_status_);
  }

  void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.name = "Frequency Status";

    double desired_freq = (1. / scan_.config.scan_time) / ((double) (skip_) + 1.0);
    double freq = (double)(count_)/diagnostic_.getPeriod();

    if (freq < (.9*desired_freq))
    {
      status.summary(2, "Desired frequency not met");
    }
    else
    {
      status.summary(0, "Desired frequency met");
    }

    status.add("Scans in interval", count_);
    status.add("Desired frequency", desired_freq);
    status.add("Actual frequency", freq);
    
    count_ = 0;
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

  void interruptionTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Interruption Test";

    if (numSubscribers("scan") == 0)
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

  void connectTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Connection Test";

    laser_.open(port_.c_str(), LaserIsHokuyoModel04LX);

    status.level = 0;
    status.message = "Connected successfully.";
  }

  void IDTest(diagnostic_msgs::DiagnosticStatus& status)
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

  void statusTest(diagnostic_msgs::DiagnosticStatus& status)
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

  void laserTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Laser Test";

    laser_.laserOn();

    status.level = 0;
    status.message = "Laser turned on successfully.";
  }

  void polledDataTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Polled Data Test";

    hokuyo::LaserScan  scan;

    int res = laser_.pollScan(scan, min_ang_, max_ang_, cluster_, 1000);

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

  void streamedDataTest(diagnostic_msgs::DiagnosticStatus& status)
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
        laser_.serviceScan(scan, 1000);
      }

      status.level = 0;
      status.message = "Streamed data from Hokuyo successfully.";

    }
  }

  void streamedIntensityDataTest(diagnostic_msgs::DiagnosticStatus& status)
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
          laser_.serviceScan(scan, 1000);
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

  void laserOffTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Laser Off Test";

    laser_.laserOff();

    status.level = 0;
    status.message = "Laser turned off successfully.";
  }

  void disconnectTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Disconnect Test";

    laser_.close();

    status.level = 0;
    status.message = "Disconnected successfully.";
  }

  void resumeTest(diagnostic_msgs::DiagnosticStatus& status)
  {
    status.name = "Resume Test";

    if (running_)
    {
      laser_.open(port_.c_str(), LaserIsHokuyoModel04LX);
      laser_.laserOn();

      int res = laser_.requestScans(!LaserIsHokuyoModel04LX, min_ang_, max_ang_, cluster_, skip_);

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

  return(0);
}
