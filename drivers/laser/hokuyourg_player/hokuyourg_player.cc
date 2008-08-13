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

@b hokuyourg_player is a driver for the Hokuyo URG line of laser range-finders.

This node should be capable of controlling any of Hokuyo's URG laser range-finders.
However, to date is has only been tested on the TOP-URG.

<hr>

@section information Information

Hokuyo URG devices take scans in a counter-clockwise direction.  Angles are measured
counter clockwise with 0 pointing directly forward.

<hr>

@section usage Usage
@verbatim
$ hokuyourg_player [standard ROS args]
@endverbatim

@par Example

@verbatim
$ hokuyourg_player
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- @b "scan"/<a href="../../std_msgs/html/classstd__msgs_1_1LaserScan.html">LaserScan</a> : scan data from the laser.

<hr>

@section services



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
#include <pthread.h>

#include "ros/node.h"
#include "rosthread/mutex.h"
#include "ros/time.h"

#include "std_msgs/LaserScan.h"
#include "robot_srvs/SelfTest.h"

#include "namelookup/nameLookupClient.hh"

#include "urg_laser.h"

using namespace std;

class HokuyoNode: public ros::node
{
private:
  URG::laser_scan_t  scan;
  URG::laser_config_t cfg;
  bool running;
  ros::thread::mutex testing_mutex;
  
  int count;
  ros::Time next_time;
  
  nameLookupClient lookup_client;

public:
  URG::laser urg;
  std_msgs::LaserScan scan_msg;
  double min_ang;
  double max_ang;
  
  int cluster;
  int skip;
  
  string port;
  
  bool autostart;

  bool calibrate_time;

  string frameid;

  string id;

  HokuyoNode() : ros::node("urglaser"), running(false), count(0), lookup_client(*this)
  {
    advertise<std_msgs::LaserScan>("scan");
    advertise_service("~self_test", &HokuyoNode::SelfTest);

    param("~min_ang", min_ang, -90.0);
    min_ang *= M_PI/180;

    param("~max_ang", max_ang, 90.0);
    max_ang *= M_PI/180;

    param("~cluster", cluster, 1);
    param("~skip", skip, 1);

    param("~port", port, string("/dev/ttyACM0"));

    param("~autostart", autostart, true);

    param("~calibrate_time", calibrate_time, true);

    param("~frameid", frameid, string("FRAMEID_LASER"));
  }

  ~HokuyoNode()
  {
    stop();
  }

  int start()
  {
    stop();

    testing_mutex.lock();
    try
    {
      urg.open(port.c_str());

      string id = urg.get_ID();
      printf("Connected to URG with ID: %s\n", id.c_str());

      urg.laser_on();

      if (calibrate_time)
        urg.calc_latency(true, min_ang, max_ang, cluster, skip);
        
      int status = urg.request_scans(true, min_ang, max_ang, cluster, skip);

      if (status != 0) {
        printf("Failed to request scans from URG.  Status: %d.\n", status);
        testing_mutex.unlock();
        return -1;
      }

      running = true;

    } catch (URG::exception& e) {
      printf("Exception thrown while starting urg.\n%s\n", e.what());
      testing_mutex.unlock();
      return -1;
    }

    next_time = ros::Time::now();

    testing_mutex.unlock();
    return(0);
  }

  int stop()
  {
    testing_mutex.lock();
    if(running)
    {
      try
      {
        urg.close();
      } catch (URG::exception& e) {
        printf("Exception thrown while trying to close:\n%s\n",e.what());
      }
      running = false;
    }

    testing_mutex.unlock();
    return 0;
  }

  int publish_scan()
  {
    testing_mutex.lock();
    try
    {
      int status = urg.service_scan(&scan);
      
      if(status != 0)
      {
        printf("error getting scan: %d\n", status);
        return 0;
      }
    } catch (URG::corrupted_data_exception &e) {
      printf("CORRUPTED DATA\n");
      testing_mutex.unlock();
      return 0;
    } catch (URG::exception& e) {
      printf("Exception thrown while trying to get scan.\n%s\n", e.what());
      running = false; //If we're here, we are no longer running
      testing_mutex.unlock();
      return -1;
    }
    
    scan_msg.angle_min = scan.config.min_angle;
    scan_msg.angle_max = scan.config.max_angle;
    scan_msg.angle_increment = scan.config.ang_increment;
    scan_msg.time_increment = scan.config.time_increment;
    scan_msg.scan_time = scan.config.scan_time;
    scan_msg.range_min = scan.config.min_range;
    scan_msg.range_max = scan.config.max_range;
    scan_msg.set_ranges_size(scan.num_readings);
    scan_msg.set_intensities_size(scan.num_readings);
    scan_msg.header.stamp = ros::Time(scan.system_time_stamp);
    scan_msg.header.frame_id = lookup_client.lookup(frameid);
      
    for(int i = 0; i < scan.num_readings; ++i)
    {
      scan_msg.ranges[i]  = scan.ranges[i];
      scan_msg.intensities[i] = scan.intensities[i];
    }

    publish("scan", scan_msg);

    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " scans/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    testing_mutex.unlock();
    sched_yield();
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
          if(publish_scan() < 0)
            break;
        }
      } else {
        usleep(1000000);
      }
    }

    //stopping should be fine even if not running
    stop();

    return true;
  }


  bool SelfTest(robot_srvs::SelfTest::request &req,
                robot_srvs::SelfTest::response &res)
  {
    testing_mutex.lock();

    printf("Entering self test.  Other operation suspended\n");

    // Stop for good measure.
    try
    {
      urg.close();
    } catch (URG::exception& e) {
    }

    std::vector<robot_msgs::DiagnosticStatus> status_vec;
    std::vector<void (HokuyoNode::*)(robot_msgs::DiagnosticStatus&)> test_fncs;

    test_fncs.push_back( &HokuyoNode::InterruptionTest );
    test_fncs.push_back( &HokuyoNode::ConnectTest );
    test_fncs.push_back( &HokuyoNode::IDTest );
    test_fncs.push_back( &HokuyoNode::StatusTest );
    test_fncs.push_back( &HokuyoNode::LaserTest );
    test_fncs.push_back( &HokuyoNode::PolledDataTest );
    test_fncs.push_back( &HokuyoNode::StreamedDataTest );
    test_fncs.push_back( &HokuyoNode::StreamedIntensityDataTest );
    test_fncs.push_back( &HokuyoNode::LaserOffTest );
    test_fncs.push_back( &HokuyoNode::DisconnectTest );
    test_fncs.push_back( &HokuyoNode::ResumeTest );

    for (std::vector<void (HokuyoNode::*)(robot_msgs::DiagnosticStatus&)>::iterator test_fncs_iter = test_fncs.begin();
           test_fncs_iter != test_fncs.end();
           test_fncs_iter++)
    {
      robot_msgs::DiagnosticStatus status;

      status.name = "None";
      status.level = 2;
      status.message = "No message was set";

      try {

        (*this.*(*test_fncs_iter))(status);

      } catch (URG::exception& e)
      {
        status.level = 2;
        status.message = std::string("Caught exception: ") + e.what();
      }

      status_vec.push_back(status);
    }

    res.id = id;

    res.passed = true;
    for (std::vector<robot_msgs::DiagnosticStatus>::iterator status_iter = status_vec.begin();
         status_iter != status_vec.end();
         status_iter++)
    {
      if (status_iter->level >= 2)
      {
        res.passed = false;
      }
    }

    res.set_status_vec(status_vec);

    printf("Self test completed\n");

    testing_mutex.unlock();
    return true;
  }

  void InterruptionTest(robot_msgs::DiagnosticStatus& status)
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

  void ConnectTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Connection Test";

    urg.open(port.c_str());

    status.level = 0;
    status.message = "Connected successfully.";
  }

  void IDTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "ID Test";

    id = urg.get_ID();

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
  }

  void StatusTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "ID Test";

    std::string stat = urg.get_status();

    if (stat != std::string("Sensor works well."))
    {
      status.level = 2;
    } else {
      status.level = 0;
    }

    status.message = stat;
  }

  void LaserTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Laser Test";

    urg.laser_on();

    status.level = 0;
    status.message = "Laser turned on successfully.";
  }

  void PolledDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Polled Data Test";

    URG::laser_scan_t  scan;

    int res = urg.poll_scan(&scan, min_ang, max_ang, cluster, 1000);

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

  void StreamedDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Streamed Data Test";

    URG::laser_scan_t  scan;

    int res = urg.request_scans(false, min_ang, max_ang, cluster, skip, 99, 1000);

    if (res != 0)
    {
      status.level = 2;
      ostringstream oss;
      oss << "Hokuyo error code: " << res << ". Consult manual for meaning.";
      status.message = oss.str();

    } else {

      for (int i = 0; i < 99; i++)
      {
        urg.service_scan(&scan, 1000);
      }

      status.level = 0;
      status.message = "Streamed data from Hokuyo successfully.";

    }
  }

  void StreamedIntensityDataTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Streamed Intensity Data Test";

    URG::laser_scan_t  scan;

    int res = urg.request_scans(false, min_ang, max_ang, cluster, skip, 99, 1000);

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
          urg.service_scan(&scan, 1000);
        } catch (URG::corrupted_data_exception &e) {
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

  void LaserOffTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Laser Off Test";

    urg.laser_off();

    status.level = 0;
    status.message = "Laser turned off successfully.";
  }

  void DisconnectTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Disconnect Test";

    urg.close();

    status.level = 0;
    status.message = "Disconnected successfully.";
  }

  void ResumeTest(robot_msgs::DiagnosticStatus& status)
  {
    status.name = "Resume Test";

    if (running)
    {
      urg.open(port.c_str());
      urg.laser_on();

      int res = urg.request_scans(true, min_ang, max_ang, cluster, skip);

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

  HokuyoNode hn;

  hn.spin();

  ros::fini();

  return(0);
}
