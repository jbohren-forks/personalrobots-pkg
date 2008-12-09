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

#ifndef __TRACKER__
#define __TRACKER__

// bayesian filtering
#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>
#include "gaussian_pos_vel.h"
//#include "nonlinearSystemPdf.h"
//#include "nonlinearMeasurementPdf.h"

// TF
#include <tf/tf.h>

// msgs
#include <robot_msgs/PositionMeasurement.h>

// log files
#include <fstream>

namespace estimation
{

class Tracker
{
public:
  /// constructor
  Tracker(unsigned int num_particles);

  /// destructor
  virtual ~Tracker();

  /// update tracker
  void update(const ros::Time& filter_time);

  /// initialize tracker
  void initialize(const BFL::StatePosVel& mu, const BFL::StatePosVel& sigma, const ros::Time& time);

  /// return if tracker was initialized
  bool isInitialized() {return tracker_initialized_;};

  /// get filter posterior
  void getEstimate(robot_msgs::PositionMeasurement& estimate);

private:
  // pdf / model / filter
  BFL::MCPdf<BFL::StatePosVel>                              prior_;
  BFL::BootstrapFilter<BFL::StatePosVel, tf::Vector3>*      filter_;

  // vars
  bool tracker_initialized_;

  unsigned int num_particles_;


}; // class

}; // namespace

#endif
