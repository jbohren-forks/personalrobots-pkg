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

/** \file setup.h */

#ifndef MPGLUE_SETUP_HPP
#define MPGLUE_SETUP_HPP

#include <mpglue/footprint.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <stdexcept>

namespace mpglue {
  
  
  /**
     Specification for the start pose (of a mobile base) of a planning
     request. It includes some additional parameters that you might
     want to set on a per-request basis.
  */
  struct startspec
  {
    startspec(bool from_scratch,
	      bool use_initial_solution,
	      bool allow_iteration,
	      double alloc_time,
	      double start_x,
	      double start_y,
	      double start_th);
    
    bool from_scratch;
    bool use_initial_solution;
    bool allow_iteration;
    double alloc_time;
    double px;
    double py;
    double pth;
  };
  
  
  /**
     Specification for the goal (of the mobile base) of a planning
     request.
  */
  struct goalspec {
    goalspec(double goal_x,
	     double goal_y,
	     double goal_th, 
	     double goal_tol_xy,
	     double goal_tol_th);
    
    double px;
    double py;
    double pth;
    double tol_xy;
    double tol_th;
  };
  
  
  /**
     Specification for a door, in case you are sending a planning
     request that should use the door planner (experimental).
  */
  struct doorspec {
    doorspec(doorspec const & orig);
    doorspec(double px,
	     double py,
	     double th_shut,
	     double th_open,
	     double width,
	     double dhandle);
    
    static boost::shared_ptr<doorspec> convert(double hinge_x, double hinge_y,
					       double door_x, double door_y,
					       double handle_distance,
					       double angle_range);
    
    double px;		/**< x-coordinate of hinge */
    double py;		/**< y-coordinate of hinge */
    double th_shut;     /**< angle between global X and door when fully shut */
    double th_open;	/**< angle between global X and door when fully open */
    double width;	/**< distance from hinge to extremity of door */
    double dhandle;	/**< distance from hinge to handle */
  };
  
  
  typedef std::vector<std::string> tokenlist_t;
  
  
  /**
     All parameters required for initializing a planning request.
  */
  struct requestspec {
    requestspec(requestspec const & orig);
    
    requestspec(std::string const & planner_spec,
		std::string const & robot_spec);
    
    static void help(/** The stream to write the help to. */
		     std::ostream & os,
		     /** The title will be printed as-is, followed by
			 a newline. If the title is an empty string,
			 nothing is printed. */
		     std::string const & title,
		     /** The prefix is prepended to each line of the
			 help, except for the title. */
		     std::string const & prefix);
    
    void dump(/** The stream to write the description to. */
	      std::ostream & os,
	      /** The title will be printed as-is, followed by a
		  newline. If the title is an empty string, nothing is
		  printed. */
	      std::string const & title,
	      /** The prefix is prepended to each line of the
		  description, except for the title. */
	      std::string const & prefix) const;
    
    requestspec & operator = (requestspec const & rhs);
    
    inline bool operator == (requestspec const & rhs) const
    { return (planner_spec == rhs.planner_spec) && (robot_spec == rhs.robot_spec); }
    
    std::string planner_spec;
    std::string robot_spec;
    
    tokenlist_t planner_tok;
    tokenlist_t robot_tok;
    
    std::string robot_name;
    double robot_inscribed_radius;
    double robot_circumscribed_radius;
    double robot_nominal_forward_speed;
    double robot_nominal_rotation_speed;
  };
  
  
  class CostmapPlanner;
  class CostmapAccessor;
  class IndexTransform;
  class SBPLEnvironment;
  
  
  CostmapPlanner *
  createNavFnPlanner(requestspec const & request,
		     boost::shared_ptr<CostmapAccessor const> costmap,
		     boost::shared_ptr<IndexTransform const> indexTransform,
		     /** set to null if you do not want progress output */
		     std::ostream * verbose_os)
  throw(std::runtime_error);
  
  
  CostmapPlanner *
  createEstarPlanner(requestspec const & request,
		     boost::shared_ptr<CostmapAccessor const> costmap,
		     boost::shared_ptr<IndexTransform const> indexTransform,
		     /** set to null if you do not want progress output */
		     std::ostream * verbose_os)
  throw(std::runtime_error);
  
  
  SBPLEnvironment *
  createSBPLEnvironment(requestspec const & request,
			boost::shared_ptr<CostmapAccessor const> costmap,
			boost::shared_ptr<IndexTransform const> indexTransform,
			footprint_t const & footprint,
			/** a bit of a hack to allow door planners to
			    say they want to search forwards (others
			    usually dont). */
			bool & default_fwd_search,
			/** only for door planner... should be refactored into something cleaner */
			doorspec * optional_door,
			/** set to null if you do not want progress output */
			std::ostream * verbose_os)
  throw(std::runtime_error);
  
  
  CostmapPlanner *
  createCostmapPlanner(requestspec const & request,
		       boost::shared_ptr<CostmapAccessor const> costmap,
		       boost::shared_ptr<IndexTransform const> indexTransform,
		       footprint_t const & footprint,
		       /** only for door planner... should be refactored into something cleaner */
		       doorspec * optional_door,
		       /** set to null if you do not want progress output */
		       std::ostream * verbose_os)
  throw(std::runtime_error);
  
}

#endif // MPGLUE_SETUP_HPP
