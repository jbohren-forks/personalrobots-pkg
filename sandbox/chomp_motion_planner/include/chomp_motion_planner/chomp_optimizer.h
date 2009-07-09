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

#ifndef CHOMP_OPTIMIZER_H_
#define CHOMP_OPTIMIZER_H_

#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_robot_model.h>
#include <chomp_motion_planner/chomp_cost.h>

#include <Eigen/Core>

#include <vector>
#include <kdl/frames.hpp>

namespace chomp
{

class ChompOptimizer
{
public:
  ChompOptimizer(ChompTrajectory *trajectory, const ChompRobotModel *robot_model,
      const ChompRobotModel::ChompPlanningGroup *planning_group, const ChompParameters *parameters);
  virtual ~ChompOptimizer();

  void optimize();

private:

  int num_joints_;
  int num_vars_free_;
  int num_vars_all_;
  int num_collision_points_;
  int free_vars_start_;
  int free_vars_end_;
  int iteration_;
  ChompTrajectory *full_trajectory_;
  const ChompRobotModel *robot_model_;
  const ChompRobotModel::ChompPlanningGroup *planning_group_;
  const ChompParameters *parameters_;
  ChompTrajectory group_trajectory_;
  std::vector<ChompCost> joint_costs_;

  std::vector<std::vector<KDL::Vector> > joint_axis_;
  std::vector<std::vector<KDL::Vector> > joint_pos_;
  std::vector<std::vector<KDL::Frame> > segment_frames_;
  std::vector<std::vector<KDL::Vector> > collision_point_pos_;
  std::vector<std::vector<KDL::Vector> > collision_point_vel_;
  std::vector<std::vector<KDL::Vector> > collision_point_acc_;

  Eigen::MatrixXd smoothness_increments_;
  Eigen::MatrixXd collision_increments_;

  // temporary variables for all functions:
  Eigen::VectorXd smoothness_derivative_;
  KDL::JntArray kdl_joint_array_;

  void initialize();
  void calculateSmoothnessIncrements();
  void calculateCollisionIncrements();
  void performForwardKinematics();
  void addIncrementsToTrajectory();
  void updateFullTrajectory();
  void debugCost();

};

}

#endif /* CHOMP_OPTIMIZER_H_ */
