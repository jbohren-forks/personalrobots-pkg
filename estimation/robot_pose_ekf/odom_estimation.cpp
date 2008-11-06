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

#include "odom_estimation.h"


using namespace MatrixWrapper;
using namespace BFL;
using namespace KDL;
using namespace std;
using namespace std_msgs;
using namespace ros;


namespace estimation
{
  // constructor
  odom_estimation::odom_estimation()
    : ros::node("odom_estimation"),
      _vel_desi(2),
      _filter_initialized(false),
      _odom_initialized(false),
      _imu_initialized(false)
  {
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
    double sys_covar_trans, sys_covar_rot;
    param("sys_covar_trans",sys_covar_trans,1000.0);
    param("sys_covar_rot",  sys_covar_rot,  1000.0);
    SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
    sysNoise_Cov(1,1) = pow(sys_covar_trans,2);
    sysNoise_Cov(2,2) = pow(sys_covar_trans,2);
    sysNoise_Cov(3,3) = pow(sys_covar_trans,2);
    sysNoise_Cov(4,4) = pow(sys_covar_rot,2);
    sysNoise_Cov(5,5) = pow(sys_covar_rot,2);
    sysNoise_Cov(6,6) = pow(sys_covar_rot,2);  
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    _sys_pdf   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    _sys_model = new AnalyticSystemModelGaussianUncertainty(_sys_pdf);
    

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(3);  measNoiseOdom_Mu = 0;
    double meas_odom_covar_trans, meas_odom_covar_rot;
    param("meas_odom_covar_trans",meas_odom_covar_trans,0.0000001);
    param("meas_odom_covar_rot",  meas_odom_covar_rot,  0.0000001);
    SymmetricMatrix measNoiseOdom_Cov(3);  measNoiseOdom_Cov = 0;
    measNoiseOdom_Cov(1,1) = pow(meas_odom_covar_trans,2);
    measNoiseOdom_Cov(2,2) = pow(meas_odom_covar_trans,2);
    measNoiseOdom_Cov(3,3) = pow(meas_odom_covar_rot,2);
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(3,6);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;    Hodom(3,6) = 1;
    _odom_meas_pdf   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    _odom_meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(_odom_meas_pdf);
    

    // create MEASUREMENT MODEL IMU
    ColumnVector measNoiseImu_Mu(3);  measNoiseImu_Mu = 0;
    double meas_imu_covar_trans, meas_imu_covar_rot;
    param("meas_imu_covar_trans",meas_imu_covar_trans,1.0);
    param("meas_imu_covar_rot",  meas_imu_covar_rot,  1.0);
    SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
    measNoiseImu_Cov(1,1) = pow(meas_imu_covar_trans,2);
    measNoiseImu_Cov(2,2) = pow(meas_imu_covar_trans,2);
    measNoiseImu_Cov(3,3) = pow(meas_imu_covar_rot,2);
    Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu(3,6);  Himu = 0;
    Himu(1,1) = 1;    Himu(2,2) = 1;    Himu(3,6) = 1;
    _imu_meas_pdf   = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    _imu_meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(_imu_meas_pdf);

    
    // advertise our estimation
    advertise<std_msgs::RobotBase2DOdom>("odom_estimation",10);

    // subscribe to messages
    subscribe("cmd_vel",      _vel,  &odom_estimation::vel_callback,  1);
    subscribe("odom",         _odom, &odom_estimation::odom_callback, 10);
    subscribe("eular_angles", _imu,  &odom_estimation::imu_callback,  10);

    _odom_file.open("odom_file.txt");
    _pred_file.open("pred_file.txt");
    _corr_file.open("corr_file.txt");
    _diff_file.open("diff_file.txt");
    _sum_file.open("sum_file.txt");
    _vel_file.open("vel_file.txt");
  };



  // destructor
  odom_estimation::~odom_estimation(){
    delete _filter;
    delete _prior;
    delete _odom_meas_model;
    delete _odom_meas_pdf;
    delete _imu_meas_model;
    delete _imu_meas_pdf;
    delete _sys_pdf;
    delete _sys_model;

    _vel_desi = 0;
    _odom_file.close();
    _pred_file.close();
    _corr_file.close();
    _diff_file.close();
    _sum_file.close();
    _vel_file.close();
  };




  // initialize prior density of filter with odom data
  void odom_estimation::Initialize_filter(const Frame& fr, double time)
  {
    // set prior of filter
    ColumnVector prior_Mu(6);   prior_Mu = 0;
    prior_Mu(1) = fr.p(0);  prior_Mu(2) = fr.p(1);   prior_Mu(3) = fr.p(2);
    fr.M.GetRPY( prior_Mu(4),  prior_Mu(5),  prior_Mu(6));
    SymmetricMatrix prior_Cov(6); 
    for (unsigned int i=1; i<=6; i++) {
      for (unsigned int j=1; j<=6; j++){
	if (i==j)  prior_Cov(i,j) = 0;
	else prior_Cov(i,j) = pow(0.001,2);
      }
    }
    _prior  = new Gaussian(prior_Mu,prior_Cov);
    _filter = new ExtendedKalmanFilter(_prior);

    // set timestamp of prior
    _filter_time = time;
    
    _filter_initialized = true;
  }



  // callback function for vel data
  void odom_estimation::vel_callback()
  {
    // receive data
    _vel_mutex.lock();
    _vel_desi(1) = _vel.vx;   _vel_desi(2) = _vel.vw;
    _vel_mutex.unlock();

    // write vel data to file
    _vel_file << _vel_desi(1) << " " << _vel_desi(2) << endl;
  };




  // callback function for odom data
  void odom_estimation::odom_callback()
  {
    // receive data
    _filter_mutex.lock();
    _odom_time  = _odom.header.stamp.to_double();
    _odom_frame = Frame(Rotation::RPY(0,0,_odom.pos.th), Vector(_odom.pos.x, _odom.pos.y, 0));

    if (_filter_initialized && _odom_initialized){
      OutputFrame(_odom_file, _odom_frame);

      // predict the state up to the time of new odom measurement
      _vel_mutex.lock();
      _filter->Update(_sys_model, _vel_desi * (_odom_time - _filter_time));
      //_vel_desi = 0;  _filter->Update(_sys_model, _vel_desi);
      _filter_time = _odom_time;
      _vel_mutex.unlock();
      Frame pred; GetState(pred);
      OutputFrame(_pred_file, pred);
      
      // update the state with new odom measurement
      Frame meas_frame = _odom_estimation * _odom_frame_old.Inverse() * _odom_frame;
      OutputFrame(_diff_file, _odom_frame_old.Inverse() * _odom_frame);
      ColumnVector meas_vec(3); double tmp;
      meas_vec(1) = meas_frame.p(0); meas_vec(2) = meas_frame.p(1); meas_frame.M.GetRPY(tmp,tmp,meas_vec(3));

      _filter->Update(_odom_meas_model, meas_vec);
      Frame corr; GetState(corr);
      OutputFrame(_corr_file, corr);

      _odom_sum_diff = _odom_sum_diff *  _odom_frame_old.Inverse() * _odom_frame;
      OutputFrame(_sum_file, _odom_sum_diff);

      // publish state
      GetState(_output);
      publish("odom_estimation", _output);
      // remember last state estimate with odom measurement
      GetState(_odom_estimation);
    }

    // initialize filter with odom data
    if (!_filter_initialized)  Initialize_filter(_odom_frame, _odom_time);

    // we received first odom data
    if (!_odom_initialized && _filter_initialized){
      GetState(_odom_estimation);
      _odom_sum_diff = _odom_estimation;
      _odom_initialized = true;
    }

    // store last frame
    _odom_frame_old = _odom_frame;

    _filter_mutex.unlock();
  };



  // callback function for imu data
  void odom_estimation::imu_callback()
  {
    // receive data
    _filter_mutex.lock();
    _imu_time = _imu.header.stamp.to_double();
    _imu_frame = Frame(Rotation::RPY(_imu.roll, _imu.pitch, _imu.yaw), Vector(0,0,0));

    // we received first imu data
    if (!_imu_initialized) _imu_initialized = true;

    // store last frame
    _imu_frame_old = _imu_frame;
    _filter_mutex.unlock();
  };



  void odom_estimation::OutputFrame(ofstream& f, const Frame& fr)
  {
    double r,p,y;
    fr.M.GetRPY(r,p,y);
    f << fr.p(0) << " " << fr.p(1) << " " << fr.p(2) << " " << r << " " << p << " " << y << endl;
  };



  // get filter state as frame
  void odom_estimation::GetState(Frame& f)
  {
    ColumnVector estimation = _filter->PostGet()->ExpectedValueGet();
    f = Frame(Rotation::RPY(estimation(4), estimation(5), estimation(6)), 
			Vector(estimation(1), estimation(2), estimation(3)));
  };


  // return filter state as pose
  void odom_estimation::GetState(PoseStamped& p)
  {
    ColumnVector estimation = _filter->PostGet()->ExpectedValueGet();
    p.pose.position.x    = estimation(1);   p.pose.position.y    = estimation(2);  p.pose.position.z    = estimation(3);
    p.pose.orientation.x = estimation(4);   p.pose.orientation.y = estimation(5);  p.pose.orientation.z = estimation(6);
    p.pose.orientation.w = 1;  p.header.stamp.from_double(_filter_time);
  };




}; // namespace



using namespace estimation;


int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv);

  // create filter class
  odom_estimation my_filter;

  // wait for filter to finish
  my_filter.spin();

  // Clean up
  ros::fini();
  return 0;
}
