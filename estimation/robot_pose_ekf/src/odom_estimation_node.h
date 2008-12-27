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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_notifier.h>
#include "odom_estimation.h"

// messages
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/PoseWithRatesStamped.h"
#include "std_msgs/PoseStamped.h"
#include "robot_msgs/VOPose.h"
#include "robot_msgs/PoseWithCovariance.h"

#include <rosthread/mutex.h>

// log files
#include <fstream>

namespace estimation
{

class OdomEstimationNode: public ros::node
{
public:
  /// constructor
  OdomEstimationNode(const string& node_name);

  /// destructor
  virtual ~OdomEstimationNode();

  /// callback function for vel data
  void velCallback();

  /// callback function for odo data
  void odomCallback();

  /// callback function for imu data
  void imuCallback();

  /// callback function for vo data
  void voCallback(const tf::MessageNotifier<robot_msgs::VOPose>::MessagePtr& vo);

  /// filter loop
  void spin();


private:
  std::string node_name_;

  /// ekf filter
  OdomEstimation my_filter_;

  // messages to receive
  std_msgs::BaseVel               vel_;  
  std_msgs::RobotBase2DOdom       odom_;  
  std_msgs::PoseWithRatesStamped  imu_;  
  robot_msgs::VOPose              vo_;  

  // estimated robot pose message to send
  robot_msgs::PoseWithCovariance  output_; 

  // robot state
  tf::TransformListener    robot_state_;
  tf::TransformBroadcaster odom_broadcaster_;

  // message notifier for vo
  tf::MessageNotifier<robot_msgs::VOPose>  vo_notifier_;

  // vectors
  MatrixWrapper::ColumnVector vel_desi_;
  tf::Transform odom_meas_, imu_meas_, vo_meas_;
  tf::Transform base_vo_init_, vo_camera_;
  tf::Stamped<tf::Transform> camera_base_;
  ros::Time odom_time_, imu_time_, vo_time_;
  ros::Time odom_stamp_, imu_stamp_, vo_stamp_, filter_stamp_;
  ros::Time odom_init_stamp_, imu_init_stamp_, vo_init_stamp_;
  bool vel_active_, odom_active_, imu_active_, vo_active_;
  bool odom_used_, imu_used_, vo_used_;
  bool odom_initializing_, imu_initializing_, vo_initializing_;
  double freq_, timeout_, odom_multiplier_;

  // mutex
  ros::thread::mutex odom_mutex_, imu_mutex_, vo_mutex_, vel_mutex_;

  // log files for debugging
  std::ofstream odom_file_, imu_file_, vo_file_, corr_file_, time_file_, extra_file_;


}; // class

}; // namespace

#endif
