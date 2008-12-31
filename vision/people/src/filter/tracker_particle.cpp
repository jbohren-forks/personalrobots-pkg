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

/* Author: Wim Meeussen */

#include "tracker_particle.h"
#include "gaussian_pos_vel.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;
using namespace robot_msgs;




namespace estimation
{
  // constructor
  TrackerParticle::TrackerParticle(unsigned int num_particles, const StatePosVel& sysnoise, const Vector3& measnoise):
    prior_(num_particles),
    filter_(NULL),
    sys_model_(sysnoise),
    meas_model_(measnoise),
    tracker_initialized_(false),
    num_particles_(num_particles)
  {};



  // destructor
  TrackerParticle::~TrackerParticle(){
    if (filter_) delete filter_;
  };


  // initialize prior density of filter 
  void TrackerParticle::initialize(const StatePosVel& mu, const StatePosVel& sigma, const double time)
  {
    cout << "Initializing tracker with " << num_particles_ << " particles, with covariance " 
	 << sigma << " around " << mu << endl;


    GaussianPosVel gauss_pos_vel(mu, sigma);
    vector<Sample<StatePosVel> > prior_samples(num_particles_);
    gauss_pos_vel.SampleFrom(prior_samples, num_particles_, CHOLESKY, NULL);
    prior_.ListOfSamplesSet(prior_samples);
    filter_ = new BootstrapFilter<StatePosVel, Vector3>(&prior_, &prior_, 0, num_particles_/4.0);

    // tracker initialized
    tracker_initialized_ = true;
    quality_ = 1;
    filter_time_ = time;
  }




  // update filter prediction
  bool TrackerParticle::updatePrediction(const double  filter_time)
  {
    // set time step
    sys_model_.SetDt(filter_time - filter_time_);
    filter_time_ = filter_time;

    // update filter
    bool res = filter_->Update(&sys_model_);
    if (!res) quality_ = 0;

    return res;
  };



  // update filter correction
  bool TrackerParticle::updateCorrection(const Vector3&  meas, const MatrixWrapper::SymmetricMatrix& cov)
  {
    // set covariance
    ((MeasPdfPos*)(meas_model_.MeasurementPdfGet()))->CovarianceSet(cov);


    // update filter
    bool res = filter_->Update(&meas_model_, meas);
    if (!res) quality_ = 0;

    return res;
  };


  // get evenly spaced particle cloud
  void TrackerParticle::getParticleCloud(const tf::Vector3& step, double threshold, std_msgs::PointCloud& cloud) const
  {
    ((MCPdfPosVel*)(filter_->PostGet()))->getParticleCloud(step, threshold, cloud);
  };


  // get most recent filter posterior 
  void TrackerParticle::getEstimate(StatePosVel& est) const
  {
    est = ((MCPdfPosVel*)(filter_->PostGet()))->ExpectedValueGet();
  };


  void TrackerParticle::getEstimate(PositionMeasurement& est) const
  {
    StatePosVel tmp = filter_->PostGet()->ExpectedValueGet();

    est.pos.x = tmp.pos_[0];
    est.pos.y = tmp.pos_[1];
    est.pos.z = tmp.pos_[2];

    est.header.stamp.fromSec( filter_time_ );
    est.header.frame_id = "odom_combined";
  }





  /// Get histogram from certain area
  Matrix TrackerParticle::getHistogramPos(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const
  {
    return ((MCPdfPosVel*)(filter_->PostGet()))->getHistogramPos(min, max, step);
  };


  Matrix TrackerParticle::getHistogramVel(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const
  {
    return ((MCPdfPosVel*)(filter_->PostGet()))->getHistogramVel(min, max, step);
  };

}; // namespace



