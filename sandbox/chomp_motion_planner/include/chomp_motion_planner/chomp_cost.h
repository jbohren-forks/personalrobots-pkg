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

#ifndef CHOMP_COST_H_
#define CHOMP_COST_H_

#include <Eigen/Core>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <vector>

namespace chomp
{

/**
 * \brief Represents the smoothness cost for CHOMP, for a single joint
 */
class ChompCost
{
public:
  ChompCost(const ChompTrajectory& trajectory, int joint_number, const std::vector<double>& derivative_costs);
  virtual ~ChompCost();

  static const int DIFF_RULE_LENGTH = 7;

  static const double DIFF_RULES[3][DIFF_RULE_LENGTH];

  template<typename Derived>
  void getDerivative(Eigen::MatrixBase<Derived>& joint_trajectory, Eigen::MatrixBase<Derived>& derivative);

private:
  Eigen::MatrixXd quad_cost_full_;
  Eigen::MatrixXd quad_cost_;
  Eigen::VectorXd linear_cost_;
  Eigen::MatrixXd quad_cost_inv_;

  Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;
};

template<typename Derived>
void ChompCost::getDerivative(Eigen::MatrixBase<Derived>& joint_trajectory, Eigen::MatrixBase<Derived>& derivative)
{
  derivative = quad_cost_full_ * (2.0 * joint_trajectory);
}

}

#endif /* CHOMP_COST_H_ */
