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

#include <mpglue/plan.h>
#include <mpglue/sbpl_environment.h>
#include <sfl/util/numeric.hpp>

namespace mpglue {
  
  double const waypoint_s::default_dr(0.1);
  double const waypoint_s::default_dtheta(M_PI);
  
  waypoint_s::waypoint_s(double _x, double _y, double _theta, double _dr, double _dtheta)
    : x(_x), y(_y), theta(_theta), dr(_dr), dtheta(_dtheta) {}
  
  waypoint_s::waypoint_s(double _x, double _y, double _theta)
    : x(_x), y(_y), theta(_theta), dr(default_dr), dtheta(default_dtheta) {}
  
  waypoint_s::waypoint_s(waypoint_s const & orig)
    : x(orig.x), y(orig.y), theta(orig.theta), dr(orig.dr), dtheta(orig.dtheta) {}

  waypoint_s::waypoint_s(robot_msgs::Pose const & pose, double _dr, double _dtheta)
    : x(pose.position.x), y(pose.position.y),
      theta(atan2(pose.orientation.z, pose.orientation.w)),
      dr(_dr), dtheta(_dtheta) {}
  
  waypoint_s::waypoint_s(robot_msgs::Pose const & pose)
    : x(pose.position.x), y(pose.position.y),
      theta(atan2(pose.orientation.z, pose.orientation.w)),
      dr(default_dr), dtheta(default_dtheta) {}
  
  bool waypoint_s::ignoreTheta() const
  {
    return dtheta >= M_PI;
  }
  
  
  PlanConverter::
  PlanConverter(waypoint_plan_t * plan)
    : default_dr(waypoint_s::default_dr),
      default_dtheta(waypoint_s::default_dtheta),
      plan_length(0),
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
  
  
  PlanConverter::
  PlanConverter(waypoint_plan_t * plan, double _default_dr, double _default_dtheta)
    : default_dr(_default_dr),
      default_dtheta(_default_dtheta),
      plan_length(0),
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
  addWaypoint(waypoint_s const & wp)
  {
    addWaypoint(boost::shared_ptr<waypoint_s>(new waypoint_s(wp)));
  }
  
  
  void PlanConverter::
  addWaypoint(boost::shared_ptr<waypoint_s> wp)
  {
    if (0 < count_) {
      double const dx(wp->x - prevx_);
      double const dy(wp->y - prevy_);
      plan_length += sqrt(pow(dx, 2) + pow(dy, 2));
      direction_change += fabs(sfl::mod2pi(wp->theta - prevdir_));
      double const tangent(atan2(dy, dx));
      if (1 < count_) // tangent change only available after 2nd point
	tangent_change += fabs(sfl::mod2pi(tangent - prevtan_));
      prevtan_ = tangent;
    }
    plan_->push_back(wp);
    ++count_;
    prevx_ = wp->x;
    prevy_ = wp->y;
    prevdir_ = wp->theta;
  }
  
  
  void PlanConverter::
  convertSBPL(SBPLEnvironment const & environment,
	      raw_sbpl_plan_t const & raw,
	      double dr,
	      double dtheta,
	      waypoint_plan_t * plan,
	      double * optPlanLengthM,
	      double * optTangentChangeRad,
	      double * optDirectionChangeRad)
  {
    PlanConverter pc(plan, dr, dtheta);
    for (raw_sbpl_plan_t::const_iterator it(raw.begin()); it != raw.end(); ++it) {
      rfct_pose pose(environment.GetPoseFromState(*it));
      pc.addWaypoint(waypoint_s(pose.x, pose.y, pose.th, dr, dtheta));
    }
    if (optPlanLengthM)
      *optPlanLengthM = pc.plan_length;
    if (optTangentChangeRad)
      *optTangentChangeRad = pc.tangent_change;
    if (optDirectionChangeRad)
      *optDirectionChangeRad = pc.direction_change;
  }
  
  
  void PlanConverter::
  convertXY(IndexTransform const & itransform,
	    float const * path_x,
	    float const * path_y,
	    int path_len,
	    double dr,
	    waypoint_plan_t * plan,
	    double * optPlanLengthM,
	    double * optTangentChangeRad)
  {
    PlanConverter pc(plan, dr, M_PI);
    ssize_t prev_ix(0), prev_iy(0);
    for (int ii(0); ii < path_len; ++ii, ++path_x, ++path_y) {
      ssize_t const ix(static_cast<ssize_t>(rint(*path_x)));
      ssize_t const iy(static_cast<ssize_t>(rint(*path_y)));
      if ((0 == ii) || (ix != prev_ix) || (iy != prev_iy)) {
	double px, py;
	itransform.indexToGlobal(ix, iy, &px, &py);
	pc.addWaypoint(px, py, 0);
      }
      prev_ix = ix;
      prev_iy = iy;
    }
    if (optPlanLengthM)
      *optPlanLengthM = pc.plan_length;
    if (optTangentChangeRad)
      *optTangentChangeRad = pc.tangent_change;
  }
  
  
  void PlanConverter::
  convertXYInterpolate(IndexTransform const & itransform,
		       float const * path_x,
		       float const * path_y,
		       int path_len,
		       double dr,
		       waypoint_plan_t * plan,
		       double * optPlanLengthM,
		       double * optTangentChangeRad)
  {
    PlanConverter pc(plan, dr, M_PI);
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
  }
  
}
