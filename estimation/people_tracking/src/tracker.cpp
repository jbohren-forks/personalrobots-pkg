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

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;

static const unsigned int NUM_CONDARG   = 1;

namespace estimation
{
  // constructor
  Tracker::Tracker(unsigned int num_particles):
    prior_(num_particles, NUM_CONDARG),
    filter_(NULL),
    tracker_initialized_(false),
    num_particles_(num_particles)
  {
    // create SYSTEM MODEL

    // create MEASUREMENT MODEL 
  };



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




  // update filter
  void Tracker::update(const Time&  filter_time)
  {
  };



  // get most recent filter posterior as PositionMeasurement
  void Tracker::getEstimate(robot_msgs::PositionMeasurement& estimate)
  {
  };

}; // namespace




using namespace estimation;


int main()
{
  StatePosVel mu(Vector3(1,2,3), Vector3(0,0,0));
  StatePosVel sigma(Vector3(0.5,0.5,0.00001), Vector3(0.00001,0.00001,0.00001));


  Tracker tracker(5000);
  tracker.initialize(mu, sigma, Time());


  return 0;
}
