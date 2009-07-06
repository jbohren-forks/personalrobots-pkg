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

#ifndef CHOMP_TRAJECTORY_H_
#define CHOMP_TRAJECTORY_H_

#include <chomp_motion_planner/chomp_robot_model.h>

#include <vector>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>

namespace chomp
{

/**
 * \brief Represents a discretized joint-space trajectory for CHOMP
 */
class ChompTrajectory
{
public:
  /**
   * \brief Constructs a trajectory for a given robot model, trajectory duration, and discretization
   */
  ChompTrajectory(const ChompRobotModel* robot_model, double duration, double discretization);

  /**
   * \brief Constructs a trajectory for a given robot model, number of trajectory points, and discretization
   */
  ChompTrajectory(const ChompRobotModel* robot_model, int num_points, double discretization);

  /**
   * \brief Creates a new containing only the joints of interest, and adds padding to the start
   * and end if needed, to have enough trajectory points for the differentiation rules
   */
  ChompTrajectory(const ChompTrajectory& source_traj, const ChompRobotModel::ChompPlanningGroup* planning_group, int diff_rule_length);

  /**
   * \brief Destructor
   */
  virtual ~ChompTrajectory();

  double& operator() (int traj_point, int joint);

  double operator() (int traj_point, int joint) const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);
  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);

  /**
   * \brief Gets the number of points in the trajectory
   */
  int getNumPoints() const;

  /**
   * \brief Gets the number of joints in each trajectory point
   */
  int getNumJoints() const;

  /**
   * \brief Gets the discretization time interval of the trajectory
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   * Only modifies points from start_index_ to end_index_, inclusive.
   */
  void fillInMinJerk();

  /**
   * \brief Sets the start and end index for the modifiable part of the trajectory
   *
   * (Everything before the start and after the end index is considered fixed)
   * The values default to 1 and getNumPoints()-2
   */
  void setStartEndIndex(int start_index, int end_index);

  /**
   * \brief Gets the start index
   */
  int getStartIndex() const;

  /**
   * \brief Gets the end index
   */
  int getEndIndex() const;

private:

  void init();                                          /**< \brief Allocates memory for the trajectory */

  const ChompRobotModel* robot_model_;                  /**< Robot Model */
  const ChompRobotModel::ChompPlanningGroup* planning_group_;    /**< Planning group that this trajectory corresponds to, if any */
  int num_points_;                                      /**< Number of points in the trajectory */
  int num_joints_;                                      /**< Number of joints in each trajectory point */
  double discretization_;                               /**< Discretization of the trajectory */
  double duration_;                                     /**< Duration of the trajectory */
  Eigen::MatrixXd trajectory_;                          /**< Storage for the actual trajectory */
  int start_index_;                                     /**< Start index (inclusive) of trajectory to be optimized (everything before it will not be modified) */
  int end_index_;                                       /**< End index (inclusive) of trajectory to be optimized (everything after it will not be modified) */
};

///////////////////////// inline functions follow //////////////////////

inline double& ChompTrajectory::operator() (int traj_point, int joint)
{
  return trajectory_(traj_point, joint);
}

inline double ChompTrajectory::operator() (int traj_point, int joint) const
{
  return trajectory_(traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr ChompTrajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::ColXpr ChompTrajectory::getJointTrajectory(int joint)
{
  return trajectory_.col(joint);
}

inline int ChompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline int ChompTrajectory::getNumJoints() const
{
  return num_joints_;
}

inline double ChompTrajectory::getDiscretization() const
{
  return discretization_;
}

inline void ChompTrajectory::setStartEndIndex(int start_index, int end_index)
{
  start_index_ = start_index;
  end_index_ = end_index;
}

inline int ChompTrajectory::getStartIndex() const
{
  return start_index_;
}

inline int ChompTrajectory::getEndIndex() const
{
  return end_index_;
}

}

#endif /* CHOMP_TRAJECTORY_H_ */
