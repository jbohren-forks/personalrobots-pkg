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


#ifndef SPLINE_SMOOTHER_UTILS_H_
#define SPLINE_SMOOTHER_UTILS_H_

#include <manipulation_msgs/WaypointTraj.h>
#include <ros/ros.h>

namespace spline_smoother
{

/**
 * \brief Ensures the consistency of a WaypointTraj message, and resizes vel and acc arrays
 *
 * Ensures that the number of (joint) names matches the number of positions in each waypoint
 * Resizes the velocities and accelerations for every waypoint, filling in zeros if necessary
 * Ensures that time is strictly increasing
 */
bool checkTrajectoryConsistency(manipulation_msgs::WaypointTraj& waypoint_traj);

template <typename T>
void differentiate(const std::vector<T>& x, std::vector<T>& xd);

/**
 * \brief Solves the tridiagonal system of equations, Ax = d
 * A is an n by n square matrix which consists of:
 *      diagonal b (0 ... n-1)
 *      upper diagonal c (0 ... n-2)
 *      lower diagonal a (0 ... n-1)
 *
 * The solution goes into x. Time complexity: O(n)
 *
 * WARNING: modifies input arrays!!
 */
template <typename T>
void tridiagonalSolve(std::vector<T>& a,
    std::vector<T>& b,
    std::vector<T>& c,
    std::vector<T>& d,
    std::vector<T>& x);

/////////////////////////// inline implementations follow //////////////////////////////

template <typename T>
void differentiate(const std::vector<T>& x, std::vector<T>& xd)
{
  int size = x.size();
  xd.resize(size-1);
  for (int i=0; i<size-1; ++i)
  {
    xd[i] = x[i+1] - x[i];
  }
}

template <typename T>
void tridiagonalSolve(std::vector<T>& a,
    std::vector<T>& b,
    std::vector<T>& c,
    std::vector<T>& d,
    std::vector<T>& x)
{
  int n = (int)d.size();

  x.resize(n);

  // forward elimination
  for (int i=1; i<n; i++)
  {
    double m = a[i] / b[i-1];
    b[i] -= m*c[i-1];
    d[i] -= m*d[i-1];
  }

  // backward substitution
  x[n-1] = d[n-1]/b[n-1];
  for (int i=n-2; i>=0; i--)
  {
    x[i] = (d[i] - c[i]*x[i+1])/b[i];
  }
}

inline bool checkTrajectoryConsistency(manipulation_msgs::WaypointTraj& waypoint_traj)
{
  unsigned int length = waypoint_traj.points.size();
  unsigned int num_joints = waypoint_traj.names.size();

  double prev_time = -1.0;

  for (unsigned int i=0; i<length; i++)
  {
    if (waypoint_traj.points[i].positions.size() != num_joints)
    {
      ROS_ERROR("Number of positions (%d) at trajectory index %d doesn't match number of joint names (%d)",
          waypoint_traj.points[i].positions.size(), i, num_joints);
      return false;
    }
    if (waypoint_traj.points[i].time < prev_time)
    {
      ROS_ERROR("Time of waypoint at trajectory index %d (%f) is not greater than or equal to the previous time (%f)",
          i, waypoint_traj.points[i].time, prev_time);
      return false;
    }
    prev_time = waypoint_traj.points[i].time;
    waypoint_traj.points[i].velocities.resize(num_joints, 0.0);
    waypoint_traj.points[i].accelerations.resize(num_joints, 0.0);
  }
  return true;
}

}

#endif /* SPLINE_SMOOTHER_UTILS_H_ */
