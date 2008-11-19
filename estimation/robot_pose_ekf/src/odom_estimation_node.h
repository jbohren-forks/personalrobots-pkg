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

#ifndef __ODOM_ESTIMATION_NODE__
#define __ODOM_ESTIMATION_NODE__

// ros stuff
#include <ros/node.h>
#include <tf/tf.h>
#include "odom_estimation.h"

// messages
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/PoseWithRatesStamped.h"
#include "std_msgs/PoseStamped.h"
//#include "visual_odometry/Pose.h"

// log files
#include <fstream>

namespace estimation
{

class odom_estimation_node: public ros::node
{
public:
  /// constructor
  odom_estimation_node();

  /// destructor
  virtual ~odom_estimation_node();

  /// callback function for vel data
  void vel_callback();

  /// callback function for odo data
  void odom_callback();

  /// callback function for imu data
  void imu_callback();

  /// callback function for vo data
  void vo_callback();

  /// filter loop
  void spin();


private:

  /// update filter
  void Update(const ros::Time& time);

  /// ekf filter
  odom_estimation _my_filter;

  // messages to receive
  std_msgs::BaseVel               _vel;  
  std_msgs::RobotBase2DOdom       _odom;  
  std_msgs::PoseWithRatesStamped  _imu;  
  std_msgs::PoseWithRatesStamped  _vo;  
  //visual_odometry::Pose           _vo;  

  // estimated robot pose message to send
  std_msgs::PoseStamped _output; 

  // vectors
  MatrixWrapper::ColumnVector _vel_desi;
  tf::Transform _odom_meas, _imu_meas,_vo_meas;
  ros::Time _odom_time, _imu_time, _vo_time, _filter_time;
  bool _vel_active, _odom_active, _imu_active, _vo_active;
  double _freq, _timeout;

  // mutex
  ros::thread::mutex _filter_mutex;

  // log files for debugging
  std::ofstream _odom_file, _imu_file, _vo_file, _corr_file, _time_file, _extra_file;


}; // class

}; // namespace

#endif
