
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

#include "odom_estimation_node.h"


using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


#define __EKF_DEBUG_FILE__

namespace estimation
{
  // constructor
  OdomEstimationNode::OdomEstimationNode()
    : ros::node("odom_estimation"),
      robot_state_(*this, true),
      odom_broadcaster_(*this),
      vo_notifier_(&robot_state_, this,  boost::bind(&OdomEstimationNode::voCallback, this, _1), "vo", "base_link", 10),
      vel_desi_(2),
      vel_active_(false),
      odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false)
  {
    // advertise our estimation
    advertise<robot_msgs::PoseWithCovariance>("odom_estimation",10);

    // subscribe to messages
    subscribe("cmd_vel",      vel_,  &OdomEstimationNode::velCallback,  10);
    subscribe("odom",         odom_, &OdomEstimationNode::odomCallback, 10);
    subscribe("imu_data",     imu_,  &OdomEstimationNode::imuCallback,  10);

    // paramters
    param("odom_estimation/freq", freq_, 30.0);
    param("odom_estimation/sensor_timeout", timeout_, 1.0);
    param("odom_estimation/sensor_init_time", sensor_init_time_, 1.0);

    // fiexed transform between camera frame and vo frame
    vo_camera_ = Transform(Quaternion(M_PI/2.0, -M_PI/2,0), Vector3(0,0,0));
    
#ifdef __EKF_DEBUG_FILE__
    // open files for debugging
    odom_file_.open("odom_file.txt");
    imu_file_.open("imu_file.txt");
    vo_file_.open("vo_file.txt");
    corr_file_.open("corr_file.txt");
    time_file_.open("time_file.txt");
    extra_file_.open("extra_file.txt");
#endif
  };




  // destructor
  OdomEstimationNode::~OdomEstimationNode(){
#ifdef __EKF_DEBUG_FILE__
    // close files for debugging
    odom_file_.close();
    imu_file_.close();
    vo_file_.close();
    corr_file_.close();
    time_file_.close();
    extra_file_.close();
#endif
  };





  // callback function for odom data
  void OdomEstimationNode::odomCallback()
  {
    // receive data
    odom_mutex_.lock();
    odom_time_ = Time::now();
    odom_meas_ = Transform(Quaternion(odom_.pos.th,0,0), Vector3(odom_.pos.x, odom_.pos.y, 0));
    my_filter_.addMeasurement(Stamped<Transform>(odom_meas_, odom_.header.stamp,"wheelodom", "base_footprint"));
    odom_mutex_.unlock();

#ifdef __EKF_DEBUG_FILE__
    // write to file
    double tmp, yaw;
    odom_meas_.getBasis().getEulerZYX(yaw, tmp, tmp);
    odom_file_ << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
#endif

    // activate odom
   if (!odom_active_) {
     if (!odom_initializing_){
       odom_initializing_ = true;
       odom_init_time_ = odom_time_;
       ROS_INFO("Initializing Odom sensor");      
     }
     else if ( (odom_time_ - odom_init_time_).toSec() > sensor_init_time_){
       odom_active_ = true;
       odom_initializing_ = false;
       ROS_INFO("Odom sensor activated");      
     }
   }
  };




  // callback function for imu data
  void OdomEstimationNode::imuCallback()
  {
    // receive data
    imu_mutex_.lock();
    imu_time_ = Time::now();
    PoseMsgToTF(imu_.pos, imu_meas_);
    my_filter_.addMeasurement(Stamped<Transform>(imu_meas_, imu_.header.stamp, "imu", "base_footprint"));
    imu_mutex_.unlock();

#ifdef __EKF_DEBUG_FILE__
    // write to file
    double tmp, yaw;
    imu_meas_.getBasis().getEulerZYX(yaw, tmp, tmp); 
    imu_file_ << yaw << endl;
#endif

    // activate imu
   if (!imu_active_) {
     if (!imu_initializing_){
       imu_initializing_ = true;
       imu_init_time_ = imu_time_;
       ROS_INFO("Initializing Imu sensor");      
     }
     else if ( (imu_time_ - imu_init_time_).toSec() > sensor_init_time_){
       imu_active_ = true;
       imu_initializing_ = false;
       ROS_INFO("Imu sensor activated");      
     }
   }
  };




  // callback function for VO data
  void OdomEstimationNode::voCallback(const MessageNotifier<robot_msgs::VOPose>::MessagePtr& vo)
  {
    vo_mutex_.lock();
    
    // get data
    vo_ = *vo;
    vo_time_ = Time::now();
    robot_state_.lookupTransform("stereo_link","base_link", vo_.header.stamp, camera_base_);
    PoseMsgToTF(vo_.pose, vo_meas_);

    // initialize
    if (!vo_active_){
      base_vo_init_ = camera_base_.inverse() * vo_camera_.inverse() * vo_meas_.inverse();
    }
    // vo measurement as base transform
    Transform vo_meas_base = base_vo_init_ * vo_meas_ * vo_camera_ * camera_base_;
    my_filter_.addMeasurement(Stamped<Transform>(vo_meas_base, vo_.header.stamp, "vo", "base_footprint"),
			      pow(21.0-(min(200.0,(double)vo_.inliers)/10),2));
    vo_mutex_.unlock();

#ifdef __EKF_DEBUG_FILE__
      // write to file
    double Rx, Ry, Rz;
    vo_meas_base.getBasis().getEulerZYX(Rz, Ry, Rx);
    vo_file_ << vo_meas_base.getOrigin().x() << " " << vo_meas_base.getOrigin().y() << " " << vo_meas_base.getOrigin().z() << " "
	     << Rx << " " << Ry << " " << Rz << endl;
#endif

    // activate vo
   if (!vo_active_) {
     if (!vo_initializing_){
       vo_initializing_ = true;
       vo_init_time_ = vo_time_;
       ROS_INFO("Initializing Vo sensor");      
     }
     else if ( (vo_time_ - vo_init_time_).toSec() > sensor_init_time_){
       vo_active_ = true;
       vo_initializing_ = false;
       ROS_INFO("Vo sensor activated");      
     }
    }
  };




  // callback function for vel data
  void OdomEstimationNode::velCallback()
  {
    // receive data
    vel_mutex_.lock();
    vel_desi_(1) = vel_.vx;   vel_desi_(2) = vel_.vw;
    vel_mutex_.unlock();

    // active
    //if (!vel_active_) vel_active_ = true;
  };




  // update filter
  void OdomEstimationNode::update(const Time& time)
  {
    // update filter
    if ( my_filter_.isInitialized() )  {
      my_filter_.update(odom_active_, imu_active_, vo_active_,  time);
      
      // output most recent estimate and relative covariance
      my_filter_.getEstimate(output_);
      publish("odom_estimation", output_);

      // broadcast most recent estimate to TransformArray
      Stamped<Transform> tmp;
      my_filter_.getEstimate(0.0, tmp);
      odom_broadcaster_.sendTransform(Stamped<Transform>(tmp.inverse(), tmp.stamp_, "odom_combined", "base_footprint"));


#ifdef __EKF_DEBUG_FILE__
      // write to file
      ColumnVector estimate; 
      my_filter_.getEstimate(estimate);
      for (unsigned int i=1; i<=6; i++)
	corr_file_ << estimate(i) << " ";
      corr_file_ << endl;

      // write to file
      /*
      Stamped<Transform> est_now, est_last;
      my_filter_.GetEstimate(Time::now(), est_now);
      my_filter_.GetEstimate(0.0, est_last);
      double tmp, r_now, r_last;
      est_now.getBasis().getEulerZYX(r_now, tmp, tmp);
      est_last.getBasis().getEulerZYX(r_last, tmp, tmp);
      extra_file_ << est_now.getOrigin().x()  << " " << est_now.getOrigin().y()  << " " << est_now.getOrigin().z()  << " " << r_now << " "
      << est_last.getOrigin().x() << " " << est_last.getOrigin().y() << " " << est_last.getOrigin().z() << " " << r_last << endl;
      */
#endif
    }

    // initialize filer with odometry frame
    if ( odom_active_ && !my_filter_.isInitialized()){
      my_filter_.initialize(odom_meas_, odom_.header.stamp);
      ROS_INFO("Fiter initialized");
    }
  };






  // filter loop
  void OdomEstimationNode::spin()
  {
    while (ok()){
      odom_mutex_.lock();  imu_mutex_.lock();  vo_mutex_.lock();
#ifdef __EKF_DEBUG_FILE__
      // write to file
      time_file_ << (Time::now() - odom_time_).toSec() << " " << (Time::now() - imu_time_).toSec()  << " " << (Time::now() - vo_time_).toSec()   << " "
		 << (Time::now() - odom_.header.stamp).toSec() << " " << (Time::now() - imu_.header.stamp).toSec()  << " " << (Time::now() - vo_.header.stamp).toSec()   << " "
		 << (odom_time_ - imu_time_).toSec()   << " " << (odom_time_ - vo_time_).toSec()   << " " << (imu_time_  - vo_time_).toSec()   << " "
		 << (odom_.header.stamp - imu_.header.stamp).toSec()   << " " << (odom_.header.stamp - vo_.header.stamp).toSec()   << " " << (imu_.header.stamp  - vo_.header.stamp).toSec()   << endl;
#endif

      if (odom_active_ || imu_active_ || vo_active_){

	// check if sensors are still active
	if (odom_active_ && (Time::now() - odom_time_).toSec() > timeout_){
	  odom_active_ = false; odom_initializing_ = false;
	  ROS_INFO("Odom sensor not active any more");
	}
	if (imu_active_ && (Time::now() - imu_time_).toSec() > timeout_){
	  imu_active_ = false;  imu_initializing_ = false;
	  ROS_INFO("Imu sensor not active any more");
	}
	if (vo_active_ && (Time::now() - vo_time_).toSec() > timeout_){
	  vo_active_ = false;  vo_initializing_ = false;
	  ROS_INFO("VO sensor not active any more");
	}

	// update filter with exact time stamps
	Time min_time = Time::now();
	if (odom_active_)  min_time = min(min_time, odom_.header.stamp);
	if (imu_active_)   min_time = min(min_time, imu_.header.stamp);
	if (vo_active_)    min_time = min(min_time, vo_.header.stamp);
	this->update(min_time);
      }
      vo_mutex_.unlock();  imu_mutex_.unlock();  odom_mutex_.unlock();
      
      // sleep
      usleep(1e6/freq_);
    }
  };


}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv);

  // create filter class
  OdomEstimationNode my_filter_node;

  // wait for filter to finish
  my_filter_node.spin();

  // Clean up
  ros::fini();
  return 0;
}
