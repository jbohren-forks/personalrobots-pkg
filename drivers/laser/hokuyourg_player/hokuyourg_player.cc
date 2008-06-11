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

@section parameters ROS parameters

Reads the following parameters from the parameter server

- @b "urglaser/min_ang" : @b [double] the angle of the first range measurement in degrees (Default: -90.0)
- @b "urglaser/max_ang" : @b [double] the angle of the last range measurement in degrees (Default: 90.0)
- @b "urglaser/cluster" : @b [int]    the number of adjascent range measurements to cluster into a single reading (Default: 1)
- @b "urglaser/skip"    : @b [int]    the number of scans to skip between each measured scan (Default: 1)
- @b "urglaser/port"    : @b [string] the port where the hokuyo device can be found (Default: "/dev/ttyACM0")

 **/

#include <assert.h>
#include <math.h>
#include <iostream>

//#include <libstandalone_drivers/urg_laser.h>
#include "urg_laser.h"

#include <ros/node.h>
#include <std_msgs/LaserScan.h>
#include "ros/time.h"

using namespace std;

class HokuyoNode: public ros::node
{
private:
  URG::laser_scan_t* scan;
  URG::laser_config_t cfg;
  bool running;
  unsigned int scanid;
  
  int count;
  ros::Time next_time;
  
public:
  URG::laser urg;
  std_msgs::LaserScan scan_msg;
  double min_ang;
  double max_ang;
  
  int cluster;
  int skip;
  
  string port;
  
  HokuyoNode() : ros::node("urglaser"), count(0)
    {
      advertise<std_msgs::LaserScan>("scan");

      param("urglaser/min_ang", min_ang, -90.0);
      min_ang *= M_PI/180;

      param("urglaser/max_ang", max_ang, 90.0);
      max_ang *= M_PI/180;

      param("urglaser/cluster", cluster, 1);
      param("urglaser/skip", skip, 1);

      param("urglaser/port", port, string("/dev/ttyACM0"));
       
      scan = new URG::laser_scan_t;
      assert(scan);
      running = false;
      scanid = 0;
    }

    ~HokuyoNode()
    {
      stop();
      delete scan;
    }

    int start()
    {
      stop();

      try
      {
        urg.open(port.c_str());

        running = true;

        printf("Connected to URG with ID: %d\n", urg.get_ID());

        urg.laser_on();
        urg.calc_latency(true, min_ang, max_ang, cluster, skip);
        
        int status = urg.request_scans(true, min_ang, max_ang, cluster, skip);

        if (status != 0) {
          printf("Failed to request scans from URG.  Status: %d.\n", status);
          return -1;
        }

      } catch (URG::exception& e) {
        printf("Exception thrown while starting urg.\n%s\n", e.what());
        return -1;
      }

      next_time = ros::Time::now();

      return(0);
    }

    int stop()
    {
      if(running)
      {
        try
        {
          urg.close();
        } catch (URG::exception& e) {
          printf("%s\n",e.what());
        }
        running = false;
      }
      return(0);
    }

    int publish_scan()
    {
      
      try
      {
        int status = urg.service_scan(scan);
        
        if(status != 0)
        {
          printf("error getting scan: %d\n", status);
          return 0;
        }
      } catch (URG::exception& e) {
        printf("Exception thrown while trying to get scan.\n%s\n", e.what());
        return -1;
      }

      count++;
      ros::Time now_time = ros::Time::now();
      if (now_time > next_time) {
        std::cout << count << " scans/sec at " << now_time << std::endl;
        count = 0;
        next_time = next_time + ros::Duration(1,0);
      }

      scan_msg.angle_min = scan->config.min_angle;
      scan_msg.angle_max = scan->config.max_angle;
      scan_msg.angle_increment = scan->config.ang_increment;
      scan_msg.time_increment = scan->config.time_increment;
      scan_msg.scan_time = scan->config.scan_time;
      scan_msg.range_min = scan->config.min_range;
      scan_msg.range_max = scan->config.max_range;
      scan_msg.set_ranges_size(scan->num_readings);
      scan_msg.set_intensities_size(scan->num_readings);
      scan_msg.header.stamp = ros::Time(scan->system_time_stamp);
      
      for(int i = 0; i < scan->num_readings; ++i)
      {
        scan_msg.ranges[i]  = scan->ranges[i];
        scan_msg.intensities[i] = scan->intensities[i];
      }

      publish("scan", scan_msg);
      return(0);
    }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  HokuyoNode hn;

  // Start up the laser
  while (hn.ok())
  {
    do {
      usleep(1000000);
    } while(hn.ok() && hn.start() != 0);

    while(hn.ok()) {
      if(hn.publish_scan() < 0)
        break;
    }
  }

  //stopping should be fine even if not running
  hn.stop();

  ros::fini();

  return(0);
}
