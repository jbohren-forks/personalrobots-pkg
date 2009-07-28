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

#include <gtest/gtest.h>
#include <spline_smoother/splines.h>
#include <spline_smoother/clamped_cubic_spline_smoother.h>
#include <spline_smoother/numerical_differentiation_spline_smoother.h>
#include <stdlib.h>

using namespace spline_smoother;

TEST(TestClampedCubicSplineSmoother, TestZeroPositionsSmall)
{
  int length = ClampedCubicSplineSmoother::MAX_TRIDIAGONAL_SOLVER_ELEMENTS - 2;

  WaypointTrajectory wpt;
  WaypointTrajectory wpt_out;
  wpt.waypoints.resize(length);
  for (int i=0; i<length; i++)
  {
    wpt.waypoints[i].position = 0.0;
    wpt.waypoints[i].velocity = 0.0;
    wpt.waypoints[i].acceleration = 0.0;
    wpt.waypoints[i].time = i;
  }

  ClampedCubicSplineSmoother ccss;
  ccss.smooth(wpt, wpt_out);

  // verify that velocities are 0:
  for (int i=0; i<length; i++)
  {
    EXPECT_NEAR(wpt_out.waypoints[i].velocity, 0.0, 1e-8);
  }
}

TEST(TestClampedCubicSplineSmoother, TestZeroPositionsLarge)
{
  int length = ClampedCubicSplineSmoother::MAX_TRIDIAGONAL_SOLVER_ELEMENTS*10;

  WaypointTrajectory wpt;
  WaypointTrajectory wpt_out;
  wpt.waypoints.resize(length);
  for (int i=0; i<length; i++)
  {
    wpt.waypoints[i].position = 0.0;
    wpt.waypoints[i].velocity = 0.0;
    wpt.waypoints[i].acceleration = 0.0;
    wpt.waypoints[i].time = i;
  }

  ClampedCubicSplineSmoother ccss;
  ccss.smooth(wpt, wpt_out);

  // verify that velocities are 0:
  for (int i=0; i<length; i++)
  {
    EXPECT_NEAR(wpt_out.waypoints[i].velocity, 0.0, 1e-8);
  }
}

TEST(TestClampedCubicSplineSmoother, TestStraightLineLarge)
{
  int length = ClampedCubicSplineSmoother::MAX_TRIDIAGONAL_SOLVER_ELEMENTS*10;

  WaypointTrajectory wpt;
  WaypointTrajectory wpt_out;
  wpt.waypoints.resize(length);
  for (int i=0; i<length; i++)
  {
    wpt.waypoints[i].position = i;
    wpt.waypoints[i].velocity = 1.0;
    wpt.waypoints[i].acceleration = 0.0;
    wpt.waypoints[i].time = i;
  }

  ClampedCubicSplineSmoother ccss;
  ccss.smooth(wpt, wpt_out);

  // verify that velocities are still 1:
  for (int i=0; i<length; i++)
  {
    EXPECT_NEAR(wpt.waypoints[i].velocity, 1.0, 1e-8);
  }
}
