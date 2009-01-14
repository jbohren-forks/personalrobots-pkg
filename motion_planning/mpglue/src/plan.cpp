/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "plan.h"
#include "sbpl_environment.h"
#include <sfl/util/numeric.hpp>

namespace mpglue {
  
  
  PlanConverter::
  PlanConverter(waypoint_plan_t * plan)
    : plan_length(0),
      tangent_change(0),
      direction_change(0),
      plan_(plan),
      count_(0),
      prevx_(0),
      prevy_(0),
      prevtan_(0),
      prevdir_(0)
  {
  }
  
  
  void PlanConverter::
  addWaypoint(std_msgs::Pose2DFloat32 const & waypoint)
  {
    if (0 < count_) {
      double const dx(waypoint.x - prevx_);
      double const dy(waypoint.y - prevy_);
      plan_length += sqrt(pow(dx, 2) + pow(dy, 2));
      direction_change += fabs(sfl::mod2pi(waypoint.th - prevdir_));
      double const tangent(atan2(dy, dx));
      if (1 < count_) // tangent change only available after 2nd point
	tangent_change += fabs(sfl::mod2pi(tangent - prevtan_));
      prevtan_ = tangent;
    }
    plan_->push_back(waypoint);
    ++count_;
    prevx_ = waypoint.x;
    prevy_ = waypoint.y;
    prevdir_ = waypoint.th;
  }
  
  
  void PlanConverter::
  addWaypoint(double px, double py, double pth)
  {
    std_msgs::Pose2DFloat32 pp;
    pp.x = px;
    pp.y = py;
    pp.th = pth;
    addWaypoint(pp);
  }
  
  
  void convertPlan(SBPLEnvironment const & environment,
		   raw_sbpl_plan_t const & raw,
		   waypoint_plan_t * plan,
		   double * optPlanLengthM,
		   double * optTangentChangeRad,
		   double * optDirectionChangeRad)
  {
    PlanConverter pc(plan);
    for (raw_sbpl_plan_t::const_iterator it(raw.begin()); it != raw.end(); ++it)
      pc.addWaypoint(environment.GetPoseFromState(*it));
    if (optPlanLengthM)
      *optPlanLengthM = pc.plan_length;
    if (optTangentChangeRad)
      *optTangentChangeRad = pc.tangent_change;
    if (optDirectionChangeRad)
      *optDirectionChangeRad = pc.direction_change;
  }
  
  
  /**
     \todo Interpolation could easily be made optional using a bool param.
  */
  void convertPlan(IndexTransform const & itransform,
		   float const * path_x,
		   float const * path_y,
		   int path_len,
		   waypoint_plan_t * plan,
		   double * optPlanLengthM,
		   double * optTangentChangeRad,
		   double * optDirectionChangeRad)
  {
    PlanConverter pc(plan);
    double const resolution(itransform.getResolution());
    for (int ii(0); ii < path_len; ++ii, ++path_x, ++path_y) {
      ssize_t ix(static_cast<ssize_t>(rint(*path_x)));
      ssize_t iy(static_cast<ssize_t>(rint(*path_y)));
      double px, py;
      itransform.indexToGlobal(ix, iy, &px, &py);
      pc.addWaypoint(interpolateIndexToGlobal(ix, px, *path_x, resolution),
		     interpolateIndexToGlobal(iy, py, *path_y, resolution),
		     0);
    }
    if (optPlanLengthM)
      *optPlanLengthM = pc.plan_length;
    if (optTangentChangeRad)
      *optTangentChangeRad = pc.tangent_change;
    if (optDirectionChangeRad)
      *optDirectionChangeRad = pc.direction_change;
  }
  
}
