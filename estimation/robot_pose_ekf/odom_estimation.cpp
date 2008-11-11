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


namespace estimation
{
  // constructor
  odom_estimation::odom_estimation():
    _prior(NULL),
    _filter(NULL),
    _filter_initialized(false)
  {
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
    sysNoise_Cov(1,1) = pow(1000.0,2);
    sysNoise_Cov(2,2) = pow(1000.0,2);
    sysNoise_Cov(3,3) = pow(1000.0,2);
    sysNoise_Cov(4,4) = pow(1000.0,2);
    sysNoise_Cov(5,5) = pow(1000.0,2);
    sysNoise_Cov(6,6) = pow(1000.0,2);  
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    _sys_pdf   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    _sys_model = new AnalyticSystemModelGaussianUncertainty(_sys_pdf);
    

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(3);  measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(3);  measNoiseOdom_Cov = 0;
    measNoiseOdom_Cov(1,1) = pow(0.001,2);
    measNoiseOdom_Cov(2,2) = pow(0.001,2);
    measNoiseOdom_Cov(3,3) = pow(0.01,2);
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(3,6);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;    Hodom(3,6) = 1;
    _odom_meas_pdf   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    _odom_meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(_odom_meas_pdf);
    

    // create MEASUREMENT MODEL IMU
    ColumnVector measNoiseImu_Mu(3);  measNoiseImu_Mu = 0;
    SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
    measNoiseImu_Cov(1,1) = pow(0.001,2);
    measNoiseImu_Cov(2,2) = pow(0.001,2);
    measNoiseImu_Cov(3,3) = pow(0.001,2);
    Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu(3,6);  Himu = 0;
    Himu(1,4) = 1;    Himu(2,5) = 1;    Himu(3,6) = 1;
    _imu_meas_pdf   = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    _imu_meas_model = new LinearAnalyticMeasurementModelGaussianUncertainty(_imu_meas_pdf);
  };



  // destructor
  odom_estimation::~odom_estimation(){
    if (_filter) delete _filter;
    if (_prior)  delete _prior;
    delete _odom_meas_model;
    delete _odom_meas_pdf;
    delete _imu_meas_model;
    delete _imu_meas_pdf;
    delete _sys_pdf;
    delete _sys_model;
  };


  // initialize prior density of filter with odom data
  void odom_estimation::Initialize(Frame& odom_meas, double odom_time,
				   Frame& imu_meas,  double imu_time, double filter_time)
  {
    // set prior of filter to odom data
    ColumnVector prior_Mu(6);   prior_Mu = 0;
    prior_Mu(1) = odom_meas.p(0);  prior_Mu(2) = odom_meas.p(1); 
    double tmp;  odom_meas.M.GetRPY(tmp, tmp, prior_Mu(6));
    SymmetricMatrix prior_Cov(6); 
    for (unsigned int i=1; i<=6; i++) {
      for (unsigned int j=1; j<=6; j++){
	if (i==j)  prior_Cov(i,j) = pow(0.001,2);
	else prior_Cov(i,j) = 0;
      }
    }
    _prior  = new Gaussian(prior_Mu,prior_Cov);
    _filter = new ExtendedKalmanFilter(_prior);

    // remember prior
    _filter_estimate_old_vec = prior_Mu;
    _filter_estimate_old = odom_meas;
    _filter_time_old     = filter_time;

    // remember sensor measurements
    _odom_meas_old = odom_meas;
    _imu_meas_old  = imu_meas;

    // filter initialized
    _filter_initialized = true;
  }





  // update filter
  void odom_estimation::Update(Frame& odom_meas, double odom_time,
			       Frame& imu_meas,  double imu_time, double filter_time)
  {
    if (_filter_initialized){
      // system update filter
      ColumnVector vel_desi(2); vel_desi = 0;
      _filter->Update(_sys_model, vel_desi * (filter_time - _filter_time_old));
      
      // convert absolute odom measurements to relative odom measurements
      Frame odom_rel_frame =  _filter_estimate_old * _odom_meas_old.Inverse() * odom_meas;
      ColumnVector odom_rel(3);
      odom_rel(1) = odom_rel_frame.p(0);   odom_rel(2) = odom_rel_frame.p(1); 
      double tmp; odom_rel_frame.M.GetRPY(tmp, tmp, odom_rel(3));
      AngleOverflowCorrect(odom_rel(3), _filter_estimate_old_vec(6));

      // convert absolute imu measurements to relative imu measurements
      Frame imu_rel_frame =  _filter_estimate_old * _imu_meas_old.Inverse() * imu_meas;
      ColumnVector imu_rel(3);
      imu_rel_frame.M.GetRPY(imu_rel(1), imu_rel(2), imu_rel(3));
      AngleOverflowCorrect(imu_rel(3), _filter_estimate_old_vec(6));
      
      // measurement update filter
      _filter->Update(_odom_meas_model, odom_rel);
      _filter->Update(_imu_meas_model,  imu_rel);
      
      // remember sensor measurements
      _odom_meas_old = odom_meas;
      _imu_meas_old  = imu_meas;

      // remember last estimate
      _filter_estimate_old_vec = _filter->PostGet()->ExpectedValueGet();
      _filter_estimate_old = Frame(Rotation::RPY(_filter_estimate_old_vec(4), _filter_estimate_old_vec(5), _filter_estimate_old_vec(6)), 
				   Vector(_filter_estimate_old_vec(1), _filter_estimate_old_vec(2), _filter_estimate_old_vec(3)));
      _filter_time_old     = filter_time;
      
    }
  };


  // get filter posterior as vector
  void odom_estimation::GetEstimate(ColumnVector& estimate, double& time)
  {
    estimate = _filter->PostGet()->ExpectedValueGet();
    time = _filter_time_old;
  };




  // correct for angle overflow
  void odom_estimation::AngleOverflowCorrect(double& a, double ref)
  {
    while ((a-ref) >  M_PI) a -= 2*M_PI;
    while ((a-ref) < -M_PI) a += 2*M_PI;
  };


}; // namespace
