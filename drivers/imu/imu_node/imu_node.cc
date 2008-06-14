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

@b 3dmgx-node

<hr>

@section information Information

<hr>

@section usage Usage

@par Example

@verbatim
$ 3dmgx2_node
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Publishes to (name / type):
- None

<hr>

@section parameters ROS parameters

Reads the following parameters from the parameter server

 **/

#include <assert.h>
#include <math.h>
#include <iostream>

#include "3dmgx2.h"

#include <ros/node.h>
#include <std_msgs/ImuData.h>
#include "ros/time.h"

using namespace std;

class IMU_node: public ros::node
{
public:
  MS_3DMGX2::IMU imu;
  std_msgs::ImuData reading;

  string port;

  int count;
  ros::Time next_time;

  bool running;
  
  IMU_node() : ros::node("imu"), count(0)
  {
    advertise<std_msgs::ImuData>("imu_data");

    param("imu/port", port, string("/dev/ttyUSB0"));
       
    running = false;
  }

  ~IMU_node()
  {
    stop();
  }

  int start()
  {
    stop();

    try
    {
      imu.open_port(port.c_str());

      running = true;

      imu.init_time();

      imu.set_continuous(MS_3DMGX2::IMU::CMD_ACCEL_ANGRATE);
    } catch (MS_3DMGX2::exception& e) {
      printf("Exception thrown while starting imu.\n %s\n", e.what());
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
        imu.close_port();
      } catch (MS_3DMGX2::exception& e) {
        printf("Exception thrown while stopping imu.\n %s\n", e.what());
      }
      running = false;
    }
    return(0);
  }

  int publish_datum()
  {
     
    uint64_t time;
    double accel[3];
    double angrate[3];

    try
    {

      imu.receive_accel_angrate(&time, accel, angrate);
        
    } catch (MS_3DMGX2::exception& e) {
      printf("Exception thrown while trying to get the reading.\n%s\n", e.what());
      return -1;
    }

    count++;
    ros::Time now_time = ros::Time::now();
    if (now_time > next_time) {
      std::cout << count << " scans/sec at " << now_time << std::endl;
      count = 0;
      next_time = next_time + ros::Duration(1,0);
    }

    reading.accel.x = accel[0];
    reading.accel.y = accel[1];
    reading.accel.z = accel[2];
 
    reading.angrate.x = angrate[0];
    reading.angrate.y = angrate[1];
    reading.angrate.z = angrate[2];

    reading.header.stamp = ros::Time(time);

    publish("imu_data", reading);
    return(0);
  }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  IMU_node in;

  // Start up the laser
  while (in.ok())
  {
    do {
      usleep(1000000);
    } while(in.ok() && in.start() != 0);

    while(in.ok()) {
      if(in.publish_datum() < 0)
        break;
    }
  }

  //stopping should be fine even if not running
  in.stop();

  ros::fini();

  return(0);
}
