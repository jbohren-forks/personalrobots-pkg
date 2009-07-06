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

#include <chomp_motion_planner/chomp_optimizer.h>

namespace chomp
{

ChompOptimizer::ChompOptimizer(ChompTrajectory *trajectory, const ChompRobotModel *robot_model,
    const ChompRobotModel::ChompPlanningGroup *planning_group, const ChompParameters *parameters):
      full_trajectory_(trajectory),
      robot_model_(robot_model),
      planning_group_(planning_group),
      parameters_(parameters),
      group_trajectory_(*full_trajectory_, planning_group_, ChompCost::DIFF_RULE_LENGTH)
{
  initialize();
}

void ChompOptimizer::initialize()
{
  // set up the joint costs:
  joint_costs_.reserve(planning_group_->num_joints_);

  // @TODO hardcoded derivative costs:
  std::vector<double> derivative_costs(3);
  derivative_costs[0] = 1.0;
  derivative_costs[1] = 1.0;
  derivative_costs[2] = 1.0;

  for (int i=0; i<group_trajectory_.getNumJoints(); i++)
  {
    joint_costs_.push_back(ChompCost(group_trajectory_, i, derivative_costs));
  }
}

ChompOptimizer::~ChompOptimizer()
{
}

void ChompOptimizer::optimize()
{

}

}
