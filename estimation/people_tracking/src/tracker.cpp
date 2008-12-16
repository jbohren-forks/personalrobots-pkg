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

#include "tracker.h"
#include "gaussian_pos_vel.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;


namespace estimation
{
  // constructor
  Tracker::Tracker(unsigned int num_particles, const StatePosVel& sysnoise, const Vector3& measnoise):
    prior_(num_particles),
    filter_(NULL),
    sys_model_(sysnoise),
    meas_model_(measnoise),
    tracker_initialized_(false),
    num_particles_(num_particles)
  {};



  // destructor
  Tracker::~Tracker(){
    if (filter_) delete filter_;
  };


  // initialize prior density of filter 
  void Tracker::initialize(const StatePosVel& mu, const StatePosVel& sigma, const Time& time)
  {
    cout << "Initializing tracker with " << num_particles_ << " particles, with covariance " 
	 << sigma << " around " << mu << endl;


    GaussianPosVel gauss_pos_vel(mu, sigma);
    vector<Sample<StatePosVel> > prior_samples(num_particles_);
    gauss_pos_vel.SampleFrom(prior_samples, num_particles_, CHOLESKY, NULL);
    prior_.ListOfSamplesSet(prior_samples);
    filter_ = new BootstrapFilter<StatePosVel, Vector3>(&prior_, 0, num_particles_/4.0);

    // tracker initialized
    tracker_initialized_ = true;
  }




  // update filter prediction
  void Tracker::updatePrediction(const Time&  filter_time)
  {
    // calculate dt
    double dt = 0.1;
    sys_model_.SetDt(dt);

    // update filter
    filter_->Update(&sys_model_);
  };



  // update filter correction
  void Tracker::updateCorrection(const Vector3&  meas)
  {
    // update filter
    filter_->Update(&meas_model_, meas);
  };



  // get most recent filter posterior as PositionMeasurement
  void Tracker::getEstimate(robot_msgs::PositionMeasurement& estimate)
  {
  };


  /// Get histogram from certain area
  Matrix Tracker::getHistogramPos(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const
  {
    return ((MCPdfPosVel*)(filter_->PostGet()))->getHistogramPos(min, max, step);
  };

  Matrix Tracker::getHistogramVel(const tf::Vector3& min, const tf::Vector3& max, const tf::Vector3& step) const
  {
    return ((MCPdfPosVel*)(filter_->PostGet()))->getHistogramVel(min, max, step);
  };

}; // namespace




using namespace estimation;


int main()
{
  StatePosVel prior_mu(Vector3(3, 3, 3), Vector3(0, 0, 0));
  StatePosVel prior_sigma(Vector3(0.3, 0.3, 0.00001), Vector3(0.00001, 0.00001, 0.00001));
  StatePosVel sys_sigma(Vector3(0.1, 0.1, 0.00001), Vector3(0.1, 0.1, 0.00001));
  Vector3 meas_sigma(0.5, 0.5, 1000);


  cout << "Initializing tracker" << endl;
  Tracker tracker(5000, sys_sigma, meas_sigma);
  tracker.initialize(prior_mu, prior_sigma, Time());

  cout << "Update tracker" << endl;
  Vector3 meas = prior_mu.pos_;
  Vector3 move(0.05, 0.0, 0.0);
  for (unsigned int i=0; i<100; i++){
    tracker.updatePrediction(Time());
    tracker.updateCorrection(meas);
    meas += move;
  }

  cout << "Get histogram" << endl;
  Vector3 min_pos(0,0,0);  Vector3 max_pos(12,6,8); Vector3 step_pos(0.05, 0.05, 0.05);
  Matrix hist_pos = tracker.getHistogramPos(min_pos, max_pos, step_pos);
  cout << hist_pos.rows() << " " << hist_pos.columns() << endl;
  ofstream file_pos; file_pos.open("pos_hist.txt");
  for (unsigned int i=1; i<= hist_pos.columns(); i++){
    for (unsigned int j=1; j<= hist_pos.rows(); j++)
      file_pos << hist_pos(j,i) << " ";
    file_pos << endl;
  }
  file_pos.close();

  Vector3 min_vel(-2,-2,0);  Vector3 max_vel(2,2,8); Vector3 step_vel(0.05, 0.05, 0.05);
  Matrix hist_vel = tracker.getHistogramVel(min_vel, max_vel, step_vel);
  cout << hist_vel.rows() << " " << hist_vel.columns() << endl;
  ofstream file_vel; file_vel.open("vel_hist.txt");
  for (unsigned int i=1; i<= hist_vel.columns(); i++){
    for (unsigned int j=1; j<= hist_vel.rows(); j++)
      file_vel << hist_vel(j,i) << " ";
    file_vel << endl;
  }
  file_vel.close();


  return 0;
}
