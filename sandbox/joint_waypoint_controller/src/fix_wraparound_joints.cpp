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

#include <joint_waypoint_controller/fix_wraparound_joints.h>
#include <spline_smoother/spline_smoother_utils.h>
#include <angles/angles.h>

namespace joint_waypoint_controller
{

FixWraparoundJoints::FixWraparoundJoints()
{

}

FixWraparoundJoints::~FixWraparoundJoints()
{
}

bool FixWraparoundJoints::smooth(const manipulation_msgs::WaypointTrajWithLimits& data_in, manipulation_msgs::WaypointTrajWithLimits& data_out) const
{
  data_out = data_in;

  int size = data_in.points.size();
  int num_joints = data_in.names.size();

  if (!spline_smoother::checkTrajectoryConsistency(data_out))
    return false;

  for (int i=0; i<num_joints; ++i)
  {
    if (data_out.limits[i].angle_wraparound)
    {
      for (int j=1; j<size; ++j)
      {
        double& cur = data_out.points[j].positions[i];
        double prev = data_out.points[j-1].positions[i];

        cur = prev + angles::shortest_angular_distance(prev, cur);
      }
    }
  }

  return true;
}

}

REGISTER_SPLINE_SMOOTHER(FixWraparoundJoints, joint_waypoint_controller::FixWraparoundJoints)
