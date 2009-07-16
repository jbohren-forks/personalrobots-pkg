/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#ifndef CHOMP_PARAMETERS_H_
#define CHOMP_PARAMETERS_H_

#include <ros/ros.h>

namespace chomp
{

class ChompParameters
{
public:
  ChompParameters();
  virtual ~ChompParameters();

  void initFromNodeHandle();

  double getPlanningTimeLimit() const;
  void setPlanningTimeLimit(double planning_time_limit);
  int getMaxIterations() const;
  double getSmoothnessCostWeight() const;
  double getObstacleCostWeight() const;
  bool getAnimatePath() const;
  double getLearningRate() const;

private:
  double planning_time_limit_;
  int max_iterations_;
  double smoothness_cost_weight_;
  double obstacle_cost_weight_;
  double learning_rate_;
  bool animate_path_;

};

/////////////////////// inline functions follow ////////////////////////

inline double ChompParameters::getPlanningTimeLimit() const
{
  return planning_time_limit_;
}

inline void ChompParameters::setPlanningTimeLimit(double planning_time_limit)
{
  planning_time_limit_ = planning_time_limit;
}

inline int ChompParameters::getMaxIterations() const
{
  return max_iterations_;
}

inline double ChompParameters::getSmoothnessCostWeight() const
{
  return smoothness_cost_weight_;
}

inline double ChompParameters::getObstacleCostWeight() const
{
  return obstacle_cost_weight_;
}

inline double ChompParameters::getLearningRate() const
{
  return learning_rate_;
}

inline bool ChompParameters::getAnimatePath() const
{
  return animate_path_;
}

} // namespace chomp

#endif /* CHOMP_PARAMETERS_H_ */
