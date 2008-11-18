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
#include "filter/extendedkalmanfilter.h"
#include "model/linearanalyticsystemmodel_gaussianuncertainty.h"
#include "model/linearanalyticmeasurementmodel_gaussianuncertainty.h"
#include "pdf/analyticconditionalgaussian.h"
#include "pdf/linearanalyticconditionalgaussian.h"
#include "nonlinearanalyticconditionalgaussianodo.h"

// TF
#include <tf/tf.h>

// log files
#include <fstream>

namespace estimation
{

class odom_estimation
{
public:
  /// constructor
  odom_estimation();

  /// destructor
  virtual ~odom_estimation();

  /// update filter
  void Update(bool odom_active, bool imu_active, bool vo_active, const ros::Time& filter_time);

  /// initialize filter
  void Initialize(const tf::Transform& prior, const ros::Time& time);


  /// get filter posterior
  void GetEstimate(MatrixWrapper::ColumnVector& estimate, ros::Time& time);
  void GetEstimate(tf::Transform& estiamte, ros::Time& time);
  void GetEstimate(std_msgs::PoseStamped& estimate);

  /// return if filter was initialized
  bool IsInitialized() {return _filter_initialized;};

  /// Add a measurement to the measurement buffer
  void AddMeasurement(const tf::Stamped<tf::Transform>& meas);

private:
  /// correct for angle overflow
  void AngleOverflowCorrect(double& a, double ref);

  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty* _sys_model;
  BFL::NonLinearAnalyticConditionalGaussianOdo* _sys_pdf;
  BFL::LinearAnalyticConditionalGaussian* _odom_meas_pdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* _odom_meas_model;
  BFL::LinearAnalyticConditionalGaussian* _imu_meas_pdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* _imu_meas_model;
  BFL::LinearAnalyticConditionalGaussian* _vo_meas_pdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* _vo_meas_model;
  BFL::Gaussian* _prior;
  BFL::ExtendedKalmanFilter* _filter;

  // vectors
  MatrixWrapper::ColumnVector _vel_desi, _filter_estimate_old_vec;
  tf::Transform _filter_estimate_old;
  tf::Stamped<tf::Transform> _odom_meas, _odom_meas_old, _imu_meas, _imu_meas_old, _vo_meas, _vo_meas_old;
  ros::Time _filter_time_old;
  bool _filter_initialized, _odom_initialized, _imu_initialized, _vo_initialized;

  // tf transformer
  tf::Transformer _transformer;

}; // class

}; // namespace

#endif
