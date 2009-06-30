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
   * \brief Destructor
   */
  virtual ~ChompTrajectory();

  double& operator() (int traj, int point);

  KDL::JntArray& operator() (int traj);

  /**
   * \brief Gets the number of points in the trajectory
   */
  int getNumPoints() const;

  /**
   * \brief Gets the discretization time interval of the trajectory
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum jerk trajectory from the start index to end index
   *
   * Keeps the startIndex and endIndex trajectory points intact, modifies everything in between.
   */
  void fillInMinJerk(int startIndex, int endIndex);

private:

  void init();                                          /**< \brief Allocates memory for the trajectory */

  const ChompRobotModel* robot_model_;                  /**< Robot Model */
  int num_points_;                                      /**< Number of points in the trajectory */
  int num_joints_;                                      /**< Number of joints in each trajectory point */
  double discretization_;                               /**< Discretization of the trajectory */
  double duration_;                                     /**< Duration of the trajectory */
  std::vector<KDL::JntArray> trajectory_;               /**< Storage for the actual trajectory */
};

///////////////////////// inline functions follow //////////////////////

inline double& ChompTrajectory::operator() (int traj, int point)
{
  return trajectory_[traj](point);
}

inline KDL::JntArray& ChompTrajectory::operator() (int traj)
{
  return trajectory_[traj];
}

inline int ChompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline double ChompTrajectory::getDiscretization() const
{
  return discretization_;
}

}

#endif /* CHOMP_TRAJECTORY_H_ */
