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

#ifndef __ODOM_ESTIMATION__
#define __ODOM_ESTIMATION__

// bayesian filtering
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.h"

// TF
#include <tf/tf.h>

// msgs
#include <robot_msgs/PoseWithCovariance.h>

// log files
#include <fstream>

namespace estimation
{

class OdomEstimation
{
public:
  /// constructor
  OdomEstimation();

  /// destructor
  virtual ~OdomEstimation();

  /// update filter
  bool update(bool odom_active, bool imu_active, bool vo_active, const ros::Time& filter_time);

  /// initialize filter
  void initialize(const tf::Transform& prior, const ros::Time& time);

  /// return if filter was initialized
  bool isInitialized() {return filter_initialized_;};

  /// get filter posterior
  void getEstimate(MatrixWrapper::ColumnVector& estimate);
  void getEstimate(ros::Time time, tf::Transform& estiamte);
  void getEstimate(ros::Time time, tf::Stamped<tf::Transform>& estiamte);
  void getEstimate(robot_msgs::PoseWithCovariance& estimate);

  /// Add a measurement to the measurement buffer
  void addMeasurement(const tf::Stamped<tf::Transform>& meas, double covar_multiplier=1);

private:
  /// correct for angle overflow
  void angleOverflowCorrect(double& a, double ref);

  // decompose Transform into x,y,z,Rx,Ry,Rz
  void decomposeTransform(const tf::Stamped<tf::Transform>& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);
  void decomposeTransform(const tf::Transform& trans,
			  double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);


  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 odom_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* odom_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 imu_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* imu_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                 vo_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* vo_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          odom_covariance_, imu_covariance_, vo_covariance_;

  // vars
  MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf::Transform filter_estimate_old_;
  tf::Stamped<tf::Transform> odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, vo_meas_, vo_meas_old_;
  ros::Time filter_time_old_;
  bool filter_initialized_, odom_initialized_, imu_initialized_, vo_initialized_;
  double odom_covar_multiplier_, imu_covar_multiplier_, vo_covar_multiplier_;

  // tf transformer
  tf::Transformer transformer_;

}; // class

}; // namespace

#endif
