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

bool ClampedCubicSplineSmoother::smooth(const std::vector<double>& positions,
    std::vector<double>& velocities,
    std::vector<double>& accelerations,
    const std::vector<double>& times) const
{
  int length = positions.size();

  if (length<3)
    return true;

  velocities.resize(length);
  accelerations.resize(length);

  if (length <= MAX_TRIDIAGONAL_SOLVER_ELEMENTS)
  {
    smoothSegment(positions, velocities, accelerations, times);
  }
  else
  {
    num_diff_spline_smoother_.smooth(positions, velocities, accelerations, times);

    std::vector<double> p(MAX_TRIDIAGONAL_SOLVER_ELEMENTS);
    std::vector<double> v(MAX_TRIDIAGONAL_SOLVER_ELEMENTS);
    std::vector<double> a(MAX_TRIDIAGONAL_SOLVER_ELEMENTS);
    std::vector<double> t(MAX_TRIDIAGONAL_SOLVER_ELEMENTS);

    // smooth smaller segments of it:
    for (int start=0; start<=length - MAX_TRIDIAGONAL_SOLVER_ELEMENTS; start++)
    {
      // fill the vectors up using the main vectors:
      for (int i=0; i<MAX_TRIDIAGONAL_SOLVER_ELEMENTS; i++)
      {
        p[i] = positions[start+i];
        v[i] = velocities[start+i];
        a[i] = accelerations[start+i];
        t[i] = times[start+i];
      }
      // solve it:
      smoothSegment(p, v, a, t);
      // copy the first filled in velocity back:
      velocities[start+1] = v[1];
    }
  }
  return true;
}

bool ClampedCubicSplineSmoother::smoothSegment(const std::vector<double>& positions,
    std::vector<double>& velocities,
    std::vector<double>& accelerations,
    const std::vector<double>& times) const
{
  std::vector<double> intervals;
  int length = positions.size();
  velocities.resize(length);
  accelerations.resize(length);

  // generate time intervals:
  differentiate(times, intervals);

  // fill up a tridiagonal matrix:
  std::vector<double> a(length-2);
  std::vector<double> b(length-2);
  std::vector<double> c(length-2);
  std::vector<double> d(length-2);
  std::vector<double> x(length-2);

  a[0] = 0.0;
  c[length-3] = 0.0;
  for (int i=0; i<length-2; i++)
  {
    c[i] = intervals[i];
    if (i<length-3)
      a[i+1] = intervals[i+2];
    b[i] = 2.0*(intervals[i] + intervals[i+1]);
    d[i] = (3.0/(intervals[i]*intervals[i+1]))*
        ((intervals[i]*intervals[i])*(positions[i+2]-positions[i+1]) +
            (intervals[i+1]*intervals[i+1])*(positions[i+1]-positions[i]));
  }
  d[0] -= velocities[0]*intervals[1];
  d[length-3] -= velocities[length-1]*times[length-3];

  tridiagonalSolve(a, b, c, d, x);
  for (int i=0; i<length-2; i++)
    velocities[i+1] = x[i];

  return true;
}

}
