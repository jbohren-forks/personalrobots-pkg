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

#ifndef __ODOM_CALIB__
#define __ODOM_CALIB__

// ros stuff
#include <ros/node.h>
#include <tf/tf.h>

// messages
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/PoseWithRatesStamped.h"

namespace calibration
{

class odom_calib: public ros::node
{
public:
  /// constructor
  odom_calib();

  /// destructor
  virtual ~odom_calib();

  /// callback function for odo data
  void odom_callback();

  /// callback function for imu data
  void imu_callback();

  // spin
  void start();
  void spin();
  void stop();

private:

  // correct for angle overflow
  void AngleOverflowCorrect(double& a, double ref);

  // messages to receive
  std_msgs::RobotBase2DOdom       _odom;  
  std_msgs::PoseWithRatesStamped  _imu;  

  // estimated robot pose message to send
  std_msgs::BaseVel               _vel; 

  // active sensors
  bool _odom_active, _imu_active, _completed;

  // angles
  double _odom_begin, _odom_end, _imu_begin, _imu_end;
  double _rot_vel, _rot_angle;

  // mutex
  ros::thread::mutex _odom_mutex, _imu_mutex;  

}; // class

}; // namespace

#endif
