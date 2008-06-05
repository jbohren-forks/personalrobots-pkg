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

This node uses part of the the Player @b urglaser driver.  For detailed documentation,
consult <a href="http://playerstage.sourceforge.net/doc/Player-cvs/player/group__driver__urglaser.html">Player urglaser documentation</a>.
Note that this node does not actually wrap the @b
urglaser driver, but rather calls into the underlying library, @b
liburglaser_standalone.

This node should be capable to controlling any of Hokuyo's URG laser.
However, to date is has only been tested on the TOP-URG.

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
- @b "scan"/LaserScan : scan data from the laser.

<hr>

@section parameters ROS parameters

- None

@todo Expose the various urglaser parameters via ROS.

 **/

#include <assert.h>
#include <math.h>
#include <iostream>

//#include <libstandalone_drivers/urg_laser.h>
#include "urg_laser.h"

#include <ros/node.h>
#include <std_msgs/MsgLaserScan.h>
#include "ros/time.h"

using namespace std;

class HokuyoNode: public ros::node
{
  private:
    urg_laser_scan_t* scan;
    urg_laser_config_t cfg;
    bool running;
    unsigned int scanid;

    int count;
    ros::Time next_time;

  public:
    urg_laser urg;
    MsgLaserScan scan_msg;
    double min_ang;
    double max_ang;

    int cluster;
    int skip;

    string port;

    HokuyoNode() : ros::node("urglaser"), count(0)
    {
      advertise<MsgLaserScan>("scan");

      param("urglaser/min_ang", min_ang, -90.0);
      min_ang *= M_PI/180;

      param("urglaser/max_ang", max_ang, 90.0);
      max_ang *= M_PI/180;

      param("urglaser/cluster", cluster, 1);
      param("urglaser/skip", skip, 1);

      param("urglaser/port", port, string("/dev/ttyACM0"));
       
      scan = new urg_laser_scan_t;
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
      if((urg.open(port.c_str()) < 0))
      {
        puts("error connecting to laser");
        return(-1);
      }
      running = true;

      printf("Connected to URG with ID: %d\n", urg.get_ID());

      urg.urg_cmd("BM");

      int status = urg.request_scans(true, min_ang, max_ang, cluster, skip);

      if (status != 0) {
        printf("Failed to request scans %d.\n", status);
        return -1;
      }
      next_time = ros::Time::now();

      return(0);
    }

    int stop()
    {
      if(running)
      {
        urg.close();
        running = false;
      }
      return(0);
    }

    int publish_scan()
    {


      //      int status = urg.poll_scan(scan, -M_PI/2.0, M_PI/2.0);
      int status = urg.service_scan(scan);
      if(status != 0)
      {
        printf("error getting scan: %d\n", status);
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
      scan_msg.angle_increment = scan->config.resolution;
      scan_msg.range_max = cfg.max_range;
      scan_msg.set_ranges_size(scan->num_readings);
      scan_msg.set_intensities_size(scan->num_readings);

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
    } while(hn.start() != 0);

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
