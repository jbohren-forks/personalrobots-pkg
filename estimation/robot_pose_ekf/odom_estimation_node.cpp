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
using namespace std_msgs;
using namespace ros;
using namespace KDL;


namespace estimation
{
  // constructor
  odom_estimation_node::odom_estimation_node()
    : ros::node("odom_estimation"),
      _vel_desi(2),
      _vel_initialized(false),
      _odom_initialized(false),
      _imu_initialized(false)
  {
    // advertise our estimation
    advertise<std_msgs::PoseStamped>("odom_estimation",10);

    // subscribe to messages
    subscribe("cmd_vel",      _vel,  &odom_estimation_node::vel_callback,  10);
    subscribe("odom",         _odom, &odom_estimation_node::odom_callback, 10);
    subscribe("imu_data",     _imu,  &odom_estimation_node::imu_callback,  10);

    _odom_file.open("odom_file.txt");
    _imu_file.open("imu_file.txt");
    _corr_file.open("corr_file.txt");
  };



  // destructor
  odom_estimation_node::~odom_estimation_node(){
    _odom_file.close();
    _imu_file.close();
    _corr_file.close();
  };




  // callback function for vel data
  void odom_estimation_node::vel_callback()
  {
    // receive data
    _filter_mutex.lock();
    _vel_desi(1) = _vel.vx;   _vel_desi(2) = _vel.vw;
    _filter_mutex.unlock();

    // initialized
    //if (!_vel_initialized) _vel_initialized = true;
  };




  // callback function for odom data
  void odom_estimation_node::odom_callback()
  {
    // receive data
    _filter_mutex.lock();
    _odom_time = _odom.header.stamp.to_double();
    _odom_meas =  Frame(Rotation::RPY(0,0,_odom.pos.th), Vector(_odom.pos.x, _odom.pos.y, 0));

    // initialize filer
    if ( !_odom_initialized && _imu_initialized)
      _my_filter.Initialize(_odom_meas, _odom_time, _imu_meas, _imu_time, _odom_time);

    // filter upate
    if ( _odom_initialized && _imu_initialized)
      {
	//cout << "time odom " << _odom_time << "  -  time imu " << _imu_time << endl;

	_my_filter.Update(_odom_meas, _odom_time, _imu_meas, _imu_time, _odom_time);
	ColumnVector estimate; double tm;
	_my_filter.GetEstimate(estimate, tm);

	// write to file
	for (unsigned int i=1; i<=6; i++)
	  _corr_file << estimate(i) << " ";
	_corr_file << tm << endl;

	// --> convert estimate to output message
	publish("odom_estimation", _output);
      }
    _filter_mutex.unlock();

    // initialized
    if (!_odom_initialized) _odom_initialized = true;
  };




  // callback function for imu data
  void odom_estimation_node::imu_callback()
  {
    // receive data
    _filter_mutex.lock();
    _imu_time = _imu.header.stamp.to_double();
    _imu_meas = Frame( Rotation::Quaternion(_imu.pos.orientation.x, _imu.pos.orientation.y,_imu.pos.orientation.z,_imu.pos.orientation.w),
		       Vector(0,0,0));
    // ASK TULLY FOR ANGLE SIGN CONVENTION
    _imu_meas = _imu_meas.Inverse();
    _filter_mutex.unlock();

    // initialized
    if (!_imu_initialized) _imu_initialized = true;


  };




}; // namespace



using namespace estimation;


int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv);

  // create filter class
  odom_estimation_node my_filter_node;

  // wait for filter to finish
  my_filter_node.spin();

  // Clean up
  ros::fini();
  return 0;
}
