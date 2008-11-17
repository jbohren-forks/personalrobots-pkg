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

#include "odom_calib.h"

using namespace ros;
using namespace tf;


#define MAX_ROT_VEL    0.5
#define MAX_TRANS_VEL  0.4
#define MAX_DURATION   15.0

namespace calibration
{
  // constructor
  odom_calib::odom_calib()
    : ros::node("odom_calibration"),
      _odom_active(false),
      _imu_active(false),
      _mech_active(false),
      _completed(false),
      _mech_begin(12),
      _mech_end(12)
  {
    // advertise the velocity commands
    advertise<std_msgs::BaseVel>("cmd_vel",10);

    // subscribe to messages
    subscribe("odom",            _odom, &odom_calib::odom_callback, 10);
    subscribe("imu_data",        _imu,  &odom_calib::imu_callback,  10);
    subscribe("mechanism_state", _mech,  &odom_calib::mech_callback,  10);
  };



  // destructor
  odom_calib::~odom_calib(){};



  // callback function for odom data
  void odom_calib::odom_callback()
  {
    _odom_mutex.lock();
    if (!_odom_active){
      _odom_begin = _odom.pos.th;
      _odom_end   = _odom.pos.th;
      _odom_active = true;
    }
    else{
      double tmp = _odom.pos.th;
      AngleOverflowCorrect(tmp, _odom_end);
      _odom_end = tmp;
    }
    _odom_mutex.unlock();
  };



  // callback function for imu data
  void odom_calib::imu_callback()
  {
    _imu_mutex.lock();
    double tmp, yaw;  Transform tf;
    PoseMsgToTF(_imu.pos, tf);
    tf.getBasis().getEulerZYX(yaw, tmp, tmp);

    if (!_imu_active){
      _imu_begin = yaw;
      _imu_end   = yaw;
      _imu_active = true;
    }
    else{
      double tmp = yaw;
      AngleOverflowCorrect(tmp, _imu_end);
      _imu_end = tmp;
    }
    _imu_mutex.unlock();
  };



  // callback function for mech data
  void odom_calib::mech_callback()
  {
    _mech_mutex.lock();
    if (!_mech_active){
      for (unsigned int i=0; i<12; i++){
	_mech_begin[i] = _mech.joint_states[i+2].position;
	_mech_active = true;
      }
    }
    else
      for (unsigned int i=0; i<12; i++)
	_mech_end[i]   = _mech.joint_states[i+2].position;
    _mech_mutex.unlock();
  };



  // correct for angle overflow
  void odom_calib::AngleOverflowCorrect(double& a, double ref)
  {
    while ((a-ref) >  M_PI) a -= 2*M_PI;
    while ((a-ref) < -M_PI) a += 2*M_PI;
  };



  void odom_calib::start()
  {
    // get parameters
    param("odom_calibration/rot_vel",_rot_vel, 0.0);
    param("odom_calibration/trans_vel",_trans_vel, 0.0);
    param("odom_calibration/duration",_duration, 0.0);
    _rot_vel    = max(min(_rot_vel,   MAX_ROT_VEL),-MAX_ROT_VEL);
    _trans_vel  = max(min(_trans_vel, MAX_TRANS_VEL),-MAX_TRANS_VEL);
    _duration   = max(min(_duration,  MAX_DURATION),-MAX_DURATION);

    ROS_INFO("(Odometry Calibration)  Will rotate at %f [deg/sec], and translate at %f [m/sec]", _rot_vel*180/M_PI, _trans_vel);
    ROS_INFO("(Odometry Calibration)  Will move during at %f seconds", _duration);

    // wait for sensor measurements from odom, imu and mechanism state
    while (!(_odom_active && _imu_active && _mech_active)){
      _vel.vx = 0; _vel.vy = 0;  _vel.vw = 0;
      publish("cmd_vel", _vel);
      usleep(50000);
    }

    // get time
    _time_begin = Time::now();
  }




  void odom_calib::spin()
  {
    while (!_completed){
      // still moving
      Duration duration = Time::now() - _time_begin;
      if (duration.toSec() <= _duration){
	_vel.vx = _trans_vel;
	_vel.vw = _rot_vel;
      }
      // finished turning
      else{
	_completed = true;
	_vel.vx = 0;
	_vel.vw = 0;
      }
      publish("cmd_vel", _vel);
      usleep(50000);
    }
  }


  void odom_calib::stop()
  {
    // give robot time to stop
    for (unsigned int i=0; i<10; i++){
      _vel.vx = 0;
      _vel.vw = 0;
      publish("cmd_vel", _vel);
      usleep(50000);
    }

    _imu_mutex.lock();
    _odom_mutex.lock();
    _mech_mutex.lock();

    // rotation
    double d_imu  = _imu_end  - _imu_begin;
    double d_odom = _odom_end - _odom_begin;
    ROS_INFO("(Odometry Calibration)  Rotated imu  %f degrees", d_imu*180/M_PI);
    ROS_INFO("(Odometry Calibration)  Rotated wheel odom %f degrees", d_odom*180/M_PI);
    ROS_INFO("(Odometry Calibration)  Absolute angle error of wheel odometry is %f degrees",(d_odom - d_imu)*180/M_PI);
    ROS_INFO("(Odometry Calibration)  Relative angle error of wheel odometry is %f precent", (d_odom - d_imu) / d_imu * 100);
    ROS_INFO("(Odometry Calibration)  Sending correction ratio %f to controller", d_imu / d_odom);

    // wheel rotation
    for (unsigned int i=0; i<12; i++){
      ROS_INFO("(Odometry Calibration) %s moved %f radians.",_mech.joint_states[i+2].name.c_str(), _mech_end[i] - _mech_begin[i]);
    }

    // send results to base server
    _srv_snd.radius_multiplier =  d_imu / d_odom;
    if (service::call("base_controller/set_wheel_radius_multiplier", _srv_snd, _srv_rsp)) 
      ROS_INFO("(Odometry Calibration)  Correction ratio seccessfully sent");
    else
      ROS_INFO("(Odometry Calibration)  Failed to send correction ratio");

    _mech_mutex.unlock();
    _odom_mutex.unlock();
    _imu_mutex.unlock();
  }

}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace calibration;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv);

  // calibrate
  odom_calib my_odom_calibration_node;

  my_odom_calibration_node.start();
  my_odom_calibration_node.spin();
  my_odom_calibration_node.stop();

  // Clean up
  ros::fini();
  return 0;
}
