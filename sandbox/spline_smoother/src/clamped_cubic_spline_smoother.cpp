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

#include <spline_smoother/clamped_cubic_spline_smoother.h>
#include <spline_smoother/spline_smoother_utils.h>

namespace spline_smoother
{

ClampedCubicSplineSmoother::ClampedCubicSplineSmoother()
{
}

ClampedCubicSplineSmoother::~ClampedCubicSplineSmoother()
{
}

bool ClampedCubicSplineSmoother::smooth(const manipulation_msgs::WaypointTraj& trajectory_in, manipulation_msgs::WaypointTraj& trajectory_out) const
{
  int length = trajectory_in.points.size();
  trajectory_out = trajectory_in;

  if (!checkTrajectoryConsistency(trajectory_out))
    return false;

  if (length<3)
    return true;

  if (length <= MAX_TRIDIAGONAL_SOLVER_ELEMENTS)
  {
    smoothSegment(trajectory_out.points);
  }
  else
  {
    if (!num_diff_spline_smoother_.smooth(trajectory_in, trajectory_out))
      return false;

    std::vector<manipulation_msgs::Waypoint> waypoints(MAX_TRIDIAGONAL_SOLVER_ELEMENTS);

    // smooth smaller segments of it:
    for (int start=0; start<=length - MAX_TRIDIAGONAL_SOLVER_ELEMENTS; start++)
    {
      // fill the vectors up using the main vectors:
      for (int i=0; i<MAX_TRIDIAGONAL_SOLVER_ELEMENTS; i++)
      {
        waypoints[i] = trajectory_out.points[start+i];
      }
      // solve it:
      if (!smoothSegment(waypoints))
        return false;
      // copy the first filled in waypoint back
      trajectory_out.points[start+1] = waypoints[1];
    }
  }

  return true;
}

bool ClampedCubicSplineSmoother::smoothSegment(std::vector<manipulation_msgs::Waypoint>& wpts) const
{
  int length = wpts.size();
  int num_joints = wpts[0].positions.size();
  if (length < 3)
    return true;

  std::vector<double> intervals(length-1);

  // generate time intervals:
  for (int i=0; i<length-1; i++)
    intervals[i] = wpts[i+1].time - wpts[i].time;

  // arrays for tridiagonal matrix
  std::vector<double> a(length-2);
  std::vector<double> b(length-2);
  std::vector<double> c(length-2);
  std::vector<double> d(length-2);
  std::vector<double> x(length-2);

  // for each joint:
  for (int j=0; j<num_joints; j++)
  {
    a[0] = 0.0;
    c[length-3] = 0.0;
    for (int i=0; i<length-2; i++)
    {
      c[i] = intervals[i];
      if (i<length-3)
        a[i+1] = intervals[i+2];
      b[i] = 2.0*(intervals[i] + intervals[i+1]);
      d[i] = (3.0/(intervals[i]*intervals[i+1]))*
          ((intervals[i]*intervals[i])*(wpts[i+2].positions[j]-wpts[i+1].positions[j]) +
              (intervals[i+1]*intervals[i+1])*(wpts[i+1].positions[j]-wpts[i].positions[j]));
    }
    d[0] -= wpts[0].velocities[j]*intervals[1];
    d[length-3] -= wpts[length-1].velocities[j]*intervals[length-3];

    tridiagonalSolve(a, b, c, d, x);
    for (int i=0; i<length-2; i++)
      wpts[i+1].velocities[j] = x[i];
  }
  return true;
}

REGISTER_SPLINE_SMOOTHER(ClampedCubicSplineSmoother)
}
