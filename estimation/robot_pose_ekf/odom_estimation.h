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

// ros stuff
#include "ros/node.h" 
#include "kdl/frames.hpp"

// messages
#include "std_msgs/RobotBase2DOdom.h"
#include "std_msgs/EulerAngles.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/PoseStamped.h"

// bayesian fitlering
#include "filter/extendedkalmanfilter.h"
#include "model/linearanalyticsystemmodel_gaussianuncertainty.h"
#include "model/linearanalyticmeasurementmodel_gaussianuncertainty.h"
#include "pdf/analyticconditionalgaussian.h"
#include "pdf/linearanalyticconditionalgaussian.h"
#include "nonlinearanalyticconditionalgaussianodo.h"

// log files
#include <fstream>

namespace estimation
{

class odom_estimation: public ros::node
{
public:
  // constructor
  odom_estimation();

  // destructor
  virtual ~odom_estimation();

  // callback function for vel data
  void vel_callback();

  // callback function for odo data
  void odom_callback();

  // callback function for imu data
  void imu_callback();

private:

  // initialize filter in first step
  void Initialize_filter(const KDL::Frame& fr, double time);

  // get filter state
  void GetState(KDL::Frame& f);
  void GetState(std_msgs::PoseStamped& p);

  void OutputFrame(std::ofstream& f, const KDL::Frame& fr);

  // messages to receive
  std_msgs::BaseVel         _vel;  
  std_msgs::RobotBase2DOdom _odom;  
  std_msgs::EulerAngles     _imu;  

  // estimated robot pose message to send
  std_msgs::PoseStamped _output; 

  // pdf / model / filter
  BFL::AnalyticSystemModelGaussianUncertainty* _sys_model;
  BFL::NonLinearAnalyticConditionalGaussianOdo* _sys_pdf;
  BFL::LinearAnalyticConditionalGaussian* _odom_meas_pdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* _odom_meas_model;
  BFL::LinearAnalyticConditionalGaussian* _imu_meas_pdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* _imu_meas_model;
  BFL::Gaussian* _prior;
  BFL::ExtendedKalmanFilter* _filter;

  MatrixWrapper::ColumnVector _vel_desi;
  KDL::Frame _odom_estimation, _odom_frame, _odom_frame_old,_imu_frame, _imu_frame_old, _odom_sum_diff;

  double _odom_time, _imu_time, _filter_time;
  bool _filter_initialized, _odom_initialized, _imu_initialized;

  // mutex
  ros::thread::mutex _vel_mutex, _filter_mutex;

  // log files for debugging
  std::ofstream _odom_file, _pred_file,_corr_file,  _vel_file, _diff_file, _sum_file
;



}; // class

}; // namespace

#endif
