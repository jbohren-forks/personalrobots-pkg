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
using namespace tf;
using namespace std;
using namespace ros;


namespace estimation
{
  // constructor
  OdomEstimation::OdomEstimation():
    prior_(NULL),
    filter_(NULL),
    filter_initialized_(false),
    odom_initialized_(false),
    imu_initialized_(false),
    vo_initialized_(false),
    odom_covar_multiplier_(1.0),
    imu_covar_multiplier_(1.0),
    vo_covar_multiplier_(1.0)
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
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);
    

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(3);  measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(3);  measNoiseOdom_Cov = 0;
    measNoiseOdom_Cov(1,1) = pow(0.002,2);    // = 2 mm / sec
    measNoiseOdom_Cov(2,2) = pow(0.002,2);    // = 2 mm / sec
    measNoiseOdom_Cov(3,3) = pow(0.017,2);    // = 1 degree / sec
    odom_covariance_ = measNoiseOdom_Cov;
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(3,6);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;    Hodom(3,6) = 1;
    odom_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);
    

    // create MEASUREMENT MODEL IMU
    ColumnVector measNoiseImu_Mu(3);  measNoiseImu_Mu = 0;
    SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
    measNoiseImu_Cov(1,1) = pow(0.0003,2);  // = 0.02 degrees / sec
    measNoiseImu_Cov(2,2) = pow(0.0003,2);  // = 0.02 degrees / sec
    measNoiseImu_Cov(3,3) = pow(0.0003,2);  // = 0.02 degrees / sec
    imu_covariance_ = measNoiseImu_Cov;
    Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu(3,6);  Himu = 0;
    Himu(1,4) = 1;    Himu(2,5) = 1;    Himu(3,6) = 1;
    imu_meas_pdf_   = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(imu_meas_pdf_);

    // create MEASUREMENT MODEL VO
    ColumnVector measNoiseVo_Mu(6);  measNoiseVo_Mu = 0;
    SymmetricMatrix measNoiseVo_Cov(6);  measNoiseVo_Cov = 0;
    measNoiseVo_Cov(1,1) = pow(0.01,2);  // = 1 cm / sec
    measNoiseVo_Cov(2,2) = pow(0.01,2);  // = 1 cm / sec
    measNoiseVo_Cov(3,3) = pow(0.01,2);  // = 1 cm / sec
    measNoiseVo_Cov(4,4) = pow(0.003,2); // = 0.2 degrees / sec
    measNoiseVo_Cov(5,5) = pow(0.003,2); // = 0.2 degrees / sec
    measNoiseVo_Cov(6,6) = pow(0.003,2); // = 0.2 degrees / sec
    vo_covariance_ = measNoiseVo_Cov;
    Gaussian measurement_Uncertainty_Vo(measNoiseVo_Mu, measNoiseVo_Cov);
    Matrix Hvo(6,6);  Hvo = 0;
    Hvo(1,1) = 1;    Hvo(2,2) = 1;    Hvo(3,3) = 1;    Hvo(4,4) = 1;    Hvo(5,5) = 1;    Hvo(6,6) = 1;
    vo_meas_pdf_   = new LinearAnalyticConditionalGaussian(Hvo, measurement_Uncertainty_Vo);
    vo_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(vo_meas_pdf_);
  };



  // destructor
  OdomEstimation::~OdomEstimation(){
    if (filter_) delete filter_;
    if (prior_)  delete prior_;
    delete odom_meas_model_;
    delete odom_meas_pdf_;
    delete imu_meas_model_;
    delete imu_meas_pdf_;
    delete vo_meas_model_;
    delete vo_meas_pdf_;
    delete sys_pdf_;
    delete sys_model_;
  };


  // initialize prior density of filter 
  void OdomEstimation::initialize(const Transform& prior, const Time& time)
  {
    // set prior of filter
    ColumnVector prior_Mu(6); 
    decomposeTransform(prior, prior_Mu(1), prior_Mu(2), prior_Mu(3), prior_Mu(4), prior_Mu(5), prior_Mu(6));
    SymmetricMatrix prior_Cov(6); 
    for (unsigned int i=1; i<=6; i++) {
      for (unsigned int j=1; j<=6; j++){
	if (i==j)  prior_Cov(i,j) = pow(0.001,2);
	else prior_Cov(i,j) = 0;
      }
    }
    prior_  = new Gaussian(prior_Mu,prior_Cov);
    filter_ = new ExtendedKalmanFilter(prior_);

    // remember prior
    addMeasurement(Stamped<Transform>(prior, time, "odom", "base_footprint"));
    filter_estimate_old_vec_ = prior_Mu;
    filter_estimate_old_ = prior;
    filter_time_old_     = time;

    // filter initialized
    filter_initialized_ = true;
  }





  // update filter
  void OdomEstimation::update(bool odom_active, bool imu_active, bool vo_active, const Time&  filter_time)
  {
    double dt = (filter_time - filter_time_old_).toSec();
    if (filter_initialized_ && dt > 0){

      // system update filter
      // --------------------
      // for now only add system noise
      ColumnVector vel_desi(2); vel_desi = 0;
      filter_->Update(sys_model_, vel_desi);


      // process odom measurement
      // ------------------------
      if (odom_active){
	transformer_.lookupTransform("base_footprint","wheelodom", filter_time, odom_meas_);
	if (odom_initialized_){
	  // convert absolute odom measurements to relative odom measurements in horizontal plane
	  Transform odom_rel_frame =  Transform(Quaternion(filter_estimate_old_vec_(6),0,0),filter_estimate_old_.getOrigin()) * odom_meas_old_.inverse() * odom_meas_;
	  ColumnVector odom_rel(3);  double tmp;
	  decomposeTransform(odom_rel_frame, odom_rel(1), odom_rel(2), tmp, tmp, tmp, odom_rel(3));
	  angleOverflowCorrect(odom_rel(3), filter_estimate_old_vec_(6));
	  // update filter
	  odom_meas_pdf_->AdditiveNoiseSigmaSet(odom_covariance_ * pow(odom_covar_multiplier_ * dt,2));
	  filter_->Update(odom_meas_model_, odom_rel);
	}
	else odom_initialized_ = true;
	odom_meas_old_ = odom_meas_;
      }
      // sensor not active
      else odom_initialized_ = false;



      // process imu measurement
      // -----------------------
      if (imu_active){
	transformer_.lookupTransform("base_footprint","imu", filter_time, imu_meas_);
	if (imu_initialized_){
	  // convert absolute imu yaw measurement to relative imu yaw measurement 
	  Transform imu_rel_frame =  filter_estimate_old_ * imu_meas_old_.inverse() * imu_meas_;
	  ColumnVector imu_rel(3); double tmp;
	  decomposeTransform(imu_rel_frame, tmp, tmp, tmp, tmp, tmp, imu_rel(3));
	  decomposeTransform(imu_meas_,     tmp, tmp, tmp, imu_rel(1), imu_rel(2), tmp);
	  angleOverflowCorrect(imu_rel(3), filter_estimate_old_vec_(6));
	  // update filter
	  imu_meas_pdf_->AdditiveNoiseSigmaSet(imu_covariance_ * pow(imu_covar_multiplier_ * dt,2));
	  filter_->Update(imu_meas_model_,  imu_rel);
	}
	else imu_initialized_ = true;
	imu_meas_old_ = imu_meas_; 
      }
      // sensor not active
      else imu_initialized_ = false;



      // process vo measurement
      // ----------------------
      if (vo_active){
	transformer_.lookupTransform("base_footprint","vo", filter_time, vo_meas_);
	if (vo_initialized_){
	  // convert absolute vo measurements to relative vo measurements
	  Transform vo_rel_frame =  filter_estimate_old_ * vo_meas_old_.inverse() * vo_meas_;
	  ColumnVector vo_rel(6);
	  decomposeTransform(vo_rel_frame, vo_rel(1),  vo_rel(2), vo_rel(3), vo_rel(4), vo_rel(5), vo_rel(6));
	  angleOverflowCorrect(vo_rel(6), filter_estimate_old_vec_(6));
	  // update filter
	  if (vo_covar_multiplier_ < 100.0){
	    vo_meas_pdf_->AdditiveNoiseSigmaSet(vo_covariance_ * pow(vo_covar_multiplier_ * dt,2));
	    filter_->Update(vo_meas_model_,  vo_rel);
	  }
	}
	else vo_initialized_ = true;
	vo_meas_old_ = vo_meas_;
      }
      // sensor not active
      else vo_initialized_ = false;



      // remember last estimate
      filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
      filter_estimate_old_ = Transform(Quaternion(filter_estimate_old_vec_(6), filter_estimate_old_vec_(5), filter_estimate_old_vec_(4)),
				       Vector3(filter_estimate_old_vec_(1), filter_estimate_old_vec_(2), filter_estimate_old_vec_(3)));
      filter_time_old_ = filter_time;
      addMeasurement(Stamped<Transform>(filter_estimate_old_, filter_time, "odom", "base_footprint"));
    }
    else if (filter_time < filter_time_old_)
      ROS_INFO("Will not update robot pose with time %f sec in the past.",(filter_time - filter_time_old_).toSec());
  };


  void OdomEstimation::addMeasurement(const Stamped<Transform>& meas, const double covar_multiplier)
  {
    transformer_.setTransform( meas );
    if (meas.frame_id_ == "wheelodom") odom_covar_multiplier_ = covar_multiplier;
    else if (meas.frame_id_ == "imu")  imu_covar_multiplier_  = covar_multiplier;
    else if (meas.frame_id_ == "vo")   vo_covar_multiplier_   = covar_multiplier;
  };


  // get latest filter posterior as vector
  void OdomEstimation::getEstimate(ColumnVector& estimate)
  {
    estimate = filter_estimate_old_vec_;
  };

  // get filter posterior at time 'time' as Transform
  void OdomEstimation::getEstimate(Time time, Transform& estimate)
  {
    Stamped<Transform> tmp;
    transformer_.lookupTransform("base_footprint","odom", time, tmp);
    estimate = tmp;
  };

  // get filter posterior at time 'time' as Stamped Transform
  void OdomEstimation::getEstimate(Time time, Stamped<Transform>& estimate)
  {
    transformer_.lookupTransform("base_footprint","odom", time, estimate);
  };

  // get most recent filter posterior as PoseWithCovariance
  void OdomEstimation::getEstimate(robot_msgs::PoseWithCovariance& estimate)
  {
    // pose
    Stamped<Transform> tmp;
    transformer_.lookupTransform("base_footprint","odom", ros::Time(), tmp);
    PoseTFToMsg(tmp, estimate.pose);

    // header
    estimate.header.stamp = tmp.stamp_;
    estimate.header.frame_id = "odom";

    // covariance
    SymmetricMatrix covar =  filter_->PostGet()->CovarianceGet();
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
	estimate.covariance[6*i+j] = covar(i+1,j+1);
  };

  // correct for angle overflow
  void OdomEstimation::angleOverflowCorrect(double& a, double ref)
  {
    while ((a-ref) >  M_PI) a -= 2*M_PI;
    while ((a-ref) < -M_PI) a += 2*M_PI;
  };

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void OdomEstimation::decomposeTransform(const Stamped<Transform>& trans, 
					   double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
    x = trans.getOrigin().x();   
    y = trans.getOrigin().y(); 
    z = trans.getOrigin().z(); 
    trans.getBasis().getEulerZYX(Rz, Ry, Rx);
  };

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void OdomEstimation::decomposeTransform(const Transform& trans, 
					   double& x, double& y, double&z, double&Rx, double& Ry, double& Rz){
    x = trans.getOrigin().x();   
    y = trans.getOrigin().y(); 
    z = trans.getOrigin().z(); 
    trans.getBasis().getEulerZYX(Rz, Ry, Rx);
  };

}; // namespace
