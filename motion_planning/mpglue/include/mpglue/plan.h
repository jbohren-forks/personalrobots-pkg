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

#ifndef MPGLUE_PLAN_HPP
#define MPGLUE_PLAN_HPP

#include <robot_msgs/Pose.h>
#include <vector>

namespace mpglue {
  
  typedef std::vector<int> raw_sbpl_plan_t;
  
  class SBPLEnvironment;
  class IndexTransform;
  
  
  struct waypoint_s {
    static double const default_dr;
    static double const default_dtheta;
    
    waypoint_s(double x, double y, double theta, double dr, double dtheta);
    waypoint_s(double x, double y, double theta);
    waypoint_s(waypoint_s const & orig);
    waypoint_s(robot_msgs::Pose const & pose, double dr, double dtheta);
    explicit waypoint_s(robot_msgs::Pose const & pose);
    
    virtual ~waypoint_s() {}
      
    bool ignoreTheta() const;
    
    double x, y, theta, dr, dtheta;
  };
  
  typedef std::vector<boost::shared_ptr<waypoint_s> > waypoint_plan_t;
  
  
  /**
     Helper class for constructing waypoint_plan_t while updating plan
     statistics.
  */
  class PlanConverter
  {
  public:
    explicit PlanConverter(waypoint_plan_t * plan);
    PlanConverter(waypoint_plan_t * plan, double default_dr, double default_dtheta);
    
    void addWaypoint(double px, double py, double pth)
    { addWaypoint(waypoint_s(px, py, pth, default_dr, default_dtheta)); }
    
    void addWaypoint(double px, double py, double pth, double dr, double dtheta)
    { addWaypoint(waypoint_s(px, py, pth, dr, dtheta)); }
    
    void addWaypoint(waypoint_s const & wp);
    
    void addWaypoint(boost::shared_ptr<waypoint_s> wp);
    
    /**
       Convert a plan from a raw state ID sequence (as computed by
       SBPLPlanner subclasses) to waypoints that are
       understandable. Optionally provides some statistics on the
       plan.
    */
    static void convertSBPL(/** in: how to translate state IDs to (x, y) or (x, y, theta)  */
			    SBPLEnvironment const & environment,
			    /** in: the raw plan */
			    raw_sbpl_plan_t const & raw,
			    /** in: the radial tolerance of each waypoint */
			    double dr,
			    /** in: the angular tolerance of each
				waypoint (use M_PI to ignore theta) */
			    double dtheta,
			    /** out: the converted plan (it is just appended
				to, not cleared for you) */
			    waypoint_plan_t * plan,
			    /** optional out: the cumulated path length */
			    double * optPlanLengthM,
			    /** optional out: the cumulated change in the path
				tangential direction (the angle between the
				x-axis and the delta between two successive
				waypoints) */
			    double * optTangentChangeRad,
			    /** optional out: the cumulated change in the
				direction of the waypoints (the delta of
				theta values).
				\note This only makes sense for plans that are
				aware of the robot's heading though, i.e. dtheta < M_PI. */
			    double * optDirectionChangeRad);
    
    /**
       Convert a plan from an float index-pair sequence (as computed by
       NavFn) to waypoints that are understandable. Optionally provides
       some statistics on the plan. Have a look at
       convertXYInterpolate() though, it takes advantage of the
       sub-pixel resolution available in NavFn.
    */
    static void convertXY(/** in: how to translate map (x, y) to global (x, y) */
			  IndexTransform const & itransform,
			  /** in: array of X-coordinates (continuous grid index). */
			  float const * path_x,
			  /** in: array of Y-coordinates (continuous grid index). */
			  float const * path_y,
			  /** in: the length of path_x[] and path_y[]. */
			  int path_len,
			  /** in: the radial tolerance of each waypoint */
			  double dr,
			  /** out: the converted plan (it is just appended
			      to, not cleared for you) */
			  waypoint_plan_t * plan,
			  /** optional out: the cumulated path length */
			  double * optPlanLengthM,
			  /** optional out: the cumulated change in the path
			      tangential direction (the angle between the
			      x-axis and the delta between two successive
			      waypoints) */
			  double * optTangentChangeRad);
    
    /**
       Uses interpolateIndexToGlobal() for sub-pixel resolution.
    */
    static void convertXYInterpolate(/** in: how to translate map (x, y) to global (x, y) */
				     IndexTransform const & itransform,
				     /** in: array of X-coordinates (continuous grid index). */
				     float const * path_x,
				     /** in: array of Y-coordinates (continuous grid index). */
				     float const * path_y,
				     /** in: the length of path_x[] and path_y[]. */
				     int path_len,
				     /** in: the radial tolerance of each waypoint */
				     double dr,
				     /** out: the converted plan (it is just appended
					 to, not cleared for you) */
				     waypoint_plan_t * plan,
				     /** optional out: the cumulated path length */
				     double * optPlanLengthM,
				     /** optional out: the cumulated change in the path
					 tangential direction (the angle between the
					 x-axis and the delta between two successive
					 waypoints) */
				     double * optTangentChangeRad);
    
    double default_dr, default_dtheta;
    double plan_length, tangent_change, direction_change;
    
  private:
    waypoint_plan_t * plan_;
    size_t count_;
    double prevx_, prevy_, prevtan_, prevdir_;
  };
  
}

#endif // MPGLUE_PLAN_HPP
