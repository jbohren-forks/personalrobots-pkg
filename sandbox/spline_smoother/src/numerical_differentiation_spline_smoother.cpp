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

#include <spline_smoother/numerical_differentiation_spline_smoother.h>

namespace spline_smoother
{

NumericalDifferentiationSplineSmoother::NumericalDifferentiationSplineSmoother()
{
}

NumericalDifferentiationSplineSmoother::~NumericalDifferentiationSplineSmoother()
{
}

bool NumericalDifferentiationSplineSmoother::smooth(const WaypointTrajectory& trajectory_in, WaypointTrajectory& trajectory_out) const
{
  bool success = true;
  int size = trajectory_in.waypoints.size();
  trajectory_out = trajectory_in;

  // keep the first and last velocities intact

  for (int i=1; i<size-1; i++)
  {
    double dt1 = trajectory_in.waypoints[i].time - trajectory_in.waypoints[i-1].time;
    double dt2 = trajectory_in.waypoints[i+1].time - trajectory_in.waypoints[i].time;
    double dx1 = trajectory_in.waypoints[i].position - trajectory_in.waypoints[i-1].position;
    double dx2 = trajectory_in.waypoints[i+1].position - trajectory_in.waypoints[i].position;
    double dt = dt1+dt2;

    double v1 = dx1/dt1;
    double v2 = dx2/dt2;

    trajectory_out.waypoints[i].velocity = v1*(dt1/dt) + v2*(dt2/dt);
  }

  // all accelerations are 0 for now:
  for (int i=0; i<size; i++)
    trajectory_out.waypoints[i].acceleration = 0.0;

  return success;
}

}
