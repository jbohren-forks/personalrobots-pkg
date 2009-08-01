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

/** \file setup.cpp  */

#include <mpglue/setup.h>
#include <mpglue/navfn_planner.h>
#include <mpglue/sbpl_environment.h>
#include <mpglue/sbpl_planner.h>
#include <mpglue/estar_planner.h>
#include <costmap_2d/cost_values.h>
#include <sfl/util/numeric.hpp>
#include <sfl/util/strutil.hpp>
#include <cmath>

using namespace boost;
using namespace std;


namespace mpglue {  
  
  
  requestspec::
  requestspec(requestspec const & orig)
  {
    *this = orig;
  }
  
  
  requestspec & requestspec::
  operator = (requestspec const & rhs)
  {
    if (&rhs == this)
      return *this;
    planner_spec = rhs.planner_spec;
    robot_spec = rhs.robot_spec;
    planner_tok = rhs.planner_tok;
    robot_tok = rhs.robot_tok;
    robot_name = rhs.robot_name;
    robot_inscribed_radius = rhs.robot_inscribed_radius;
    robot_circumscribed_radius = rhs.robot_circumscribed_radius;
    robot_nominal_forward_speed = rhs.robot_nominal_forward_speed;
    robot_nominal_rotation_speed = rhs.robot_nominal_rotation_speed;
    return *this;
  }
  
  
  void requestspec::
  dump(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "planner spec:                 " << planner_spec << "\n"
       << prefix << "robot spec:                   " << robot_spec << "\n"
       << prefix << "robot_name:                   " << robot_name << "\n"
       << prefix << "robot_inscribed_radius:       " << robot_inscribed_radius << "\n"
       << prefix << "robot_circumscribed_radius:   " << robot_circumscribed_radius << "\n"
       << prefix << "robot_nominal_forward_speed:  " << robot_nominal_forward_speed << "\n"
       << prefix << "robot_nominal_rotation_speed: " << robot_nominal_rotation_speed << "\n";
  }
  
  
  startspec::
  startspec(bool _from_scratch,
	    bool _use_initial_solution,
	    bool _allow_iteration,
	    double _alloc_time,
	    double start_x,
	    double start_y,
	    double start_th)
    : from_scratch(_from_scratch),
      use_initial_solution(_use_initial_solution),
      allow_iteration(_allow_iteration),
      alloc_time(_alloc_time),
      px(start_x),
      py(start_y),
      pth(start_th)
  {
  }
  
  
  goalspec::
  goalspec(double goal_x,
	   double goal_y,
	   double goal_th, 
	   double goal_tol_xy,
	   double goal_tol_th)
    : px(goal_x),
      py(goal_y),
      pth(goal_th),
      tol_xy(goal_tol_xy),
      tol_th(goal_tol_th)
  {
  }
  
  
  doorspec::
  doorspec(doorspec const & orig)
    : px(orig.px),
      py(orig.py),
      th_shut(orig.th_shut),
      th_open(orig.th_open),
      width(orig.width),
      dhandle(orig.dhandle)
  {
  }
  
  
  doorspec::
  doorspec(double _px,
	   double _py,
	   double _th_shut,
	   double _th_open,
	   double _width,
	   double _dhandle)
    : px(_px),
      py(_py),
      th_shut(_th_shut),
      th_open(_th_open),
      width(_width),
      dhandle(_dhandle)
  {
  }
  
  
  boost::shared_ptr<doorspec> doorspec::
  convert(double hinge_x, double hinge_y,
	  double door_x, double door_y,
	  double handle_distance,
	  double angle_range)
  {
    double const dx(door_x - hinge_x);
    double const dy(door_y - hinge_y);
    double const th_shut(atan2(dy, dx));
    boost::shared_ptr<doorspec> door(new doorspec(hinge_x, hinge_y,
						  th_shut, th_shut + angle_range,
						  sqrt(pow(dx, 2) + pow(dy, 2)),
						  handle_distance));
    return door;
  }
  
  
  requestspec::
  requestspec(std::string const & _planner_spec,
	      std::string const & _robot_spec)
    : planner_spec(_planner_spec),
      robot_spec(_robot_spec),
      robot_name("pr2"),
      robot_inscribed_radius(0.325),
      robot_circumscribed_radius(0.46),
      robot_nominal_forward_speed(0.6),	// approx human walking speed
      robot_nominal_rotation_speed(0.6) // XXXX guesstimate
  {
    sfl::tokenize(planner_spec, ':', planner_tok);
    sfl::tokenize(robot_spec, ':', robot_tok);
    
    sfl::token_to(robot_tok, 0, robot_name);
    if (sfl::token_to(robot_tok, 1, robot_inscribed_radius))
      robot_inscribed_radius *= 1e-3;
    if (sfl::token_to(robot_tok, 2, robot_circumscribed_radius))
      robot_circumscribed_radius *= 1e-3;
    if (sfl::token_to(robot_tok, 3, robot_nominal_forward_speed))
      robot_nominal_forward_speed *= 1e-3;
    if (sfl::token_to(robot_tok, 4, robot_nominal_rotation_speed))
      robot_nominal_rotation_speed *= 1e-3;
  }
  
  
  void requestspec::
  help(std::ostream & os, std::string const & title, std::string const & prefix)
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "\navailable planner specs (can use registered aliases instead):\n"
       << prefix << "  NavFn [: int | dsc ]\n"
       << prefix << "        int = interpolate path (default)\n"
       << prefix << "        dsc = use discretized path instead\n"
       << prefix << "  ADStar | ARAStar [: 2d | 3dkin | xythetalat [: custom [: bwd | fwd ]]]\n"
       << prefix << "        2d = use 2D environment (default)\n"
       << prefix << "        3dkin = use 3DKIN environment\n"
       << prefix << "        xythetalat = use XYTHETALAT environment\n"
       << prefix << "        xythetadoor = use DOOR environment\n"
       << prefix << "        custom = uses by XYTHETALAT to specify the motion primitive file\n"
       << prefix << "                 defaults to data/pr2.mprim\n"
       << prefix << "        bwd = use backward search (default)\n"
       << prefix << "        fwd = use forward search\n"
       << prefix << "  EStar\n"
       << prefix << "        (no further options yet)\n"
       << prefix << "\navailable robot specs:\n"
       << prefix << "  pr2 [: inscribed [: circumscribed [: fwd_speed [: rot_speed ]]]]\n"
       << prefix << "        inscribed radius (millimeters), default 325mm\n"
       << prefix << "        circumscribed radius (millimeters), default 460mm\n"
       << prefix << "        nominal forward speed (millimeters per second), default 600mm/s\n"
       << prefix << "        nominal rotation speed (milliradians per second), default 600mrad/s\n";
  }
  
  
  CostmapPlanner *
  createNavFnPlanner(requestspec const & request,
		     boost::shared_ptr<CostmapAccessor const> costmap,
		     boost::shared_ptr<IndexTransform const> indexTransform,
		     /** set to null if you do not want progress output */
		     std::ostream * verbose_os)
    throw(std::runtime_error)
  {
    string int_str("int");
    sfl::token_to(request.planner_tok, 1, int_str);
    bool interpolate_path(true);
    if ("dsc" == int_str)
      interpolate_path = false;
    else if ("int" != int_str)
      throw runtime_error("mpglue::createNavFnPlanner(): invalid interpolate_path \"" + int_str
			  + "\", must be \"int\" or \"dsc\"");
    
    if (verbose_os)
      *verbose_os << "mpglue::createNavFnPlanner(): interpolate_path = "
		  << sfl::to_string(interpolate_path) << "\n" << flush;
    
    return new NavFnPlanner(costmap, indexTransform, interpolate_path);
  }
  
  
  CostmapPlanner *
  createEstarPlanner(requestspec const & request,
		     boost::shared_ptr<CostmapAccessor const> costmap,
		     boost::shared_ptr<IndexTransform const> indexTransform,
		     /** set to null if you do not want progress output */
		     std::ostream * verbose_os)
    throw(std::runtime_error)
  {
#ifndef MPGLUE_HAVE_ESTAR
    throw runtime_error("mpglue::createEstarPlanner(): no support for EStar planner\n"
			"  to enable it, install EStar,"
			" set the ROS_ESTAR_DIR environment variable,"
			" and recompile mpglue");
#else // MPGLUE_HAVE_ESTAR
    if (verbose_os)
      *verbose_os << "mpglue::createEstarPlanner()\n" << flush;
    return new EstarPlanner(costmap, indexTransform, &cerr);
#endif // MPGLUE_HAVE_ESTAR
  }
  
  
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
    throw(std::runtime_error)
  {
    default_fwd_search = false;
    string envstr("2d");
    sfl::token_to(request.planner_tok, 1, envstr);
    
    if ("2d" == envstr) {
      if (verbose_os)
	*verbose_os << "mpglue::createSBPLEnvironment(): 8-connected 2D\n" << flush;
      return SBPLEnvironment::create2D(costmap, indexTransform, false);
    }
    
    if ("2d16" == envstr) {
      if (verbose_os)
	*verbose_os << "mpglue::createSBPLEnvironment(): 16-connected 2D\n" << flush;
      return SBPLEnvironment::create2D(costmap, indexTransform, true);
    }
    
    double const
      timetoturn45degsinplace_secs(45.0 * M_PI / 180.0 / request.robot_nominal_rotation_speed);
    if (verbose_os)
      *verbose_os << "mpglue::createSBPLEnvironment(): timetoturn45degsinplace_secs = "
		  << timetoturn45degsinplace_secs << "\n" << flush;
    
    if ("3dkin" == envstr) {
      if (verbose_os)
	*verbose_os << "mpglue::createSBPLEnvironment(): 3DKIN\n" << flush;
      return SBPLEnvironment::create3DKIN(costmap,
						  indexTransform,
						  footprint,
						  request.robot_nominal_forward_speed,
						  timetoturn45degsinplace_secs,
						  verbose_os);
    }
    
    string mprim_filename("data/pr2.mprim");
    sfl::token_to(request.planner_tok, 2, mprim_filename);
    if (verbose_os)
      *verbose_os << "mpglue::createSBPLEnvironment(): motion primitive file: "
		  << mprim_filename << "\n" << flush;	
    
    if ("xythetalat" == envstr) {
      if (verbose_os)
	*verbose_os << "mpglue::createSBPLEnvironment(): XYTHETALAT\n" << flush;
      return SBPLEnvironment::createXYThetaLattice(costmap,
							   indexTransform,
							   footprint,
							   request.robot_nominal_forward_speed,
							   timetoturn45degsinplace_secs,
							   mprim_filename,
							   verbose_os);
    }
    
    if ("xythetadoor" == envstr) {
      if (verbose_os)
	*verbose_os << "mpglue::createSBPLEnvironment(): XYTHETADOOR\n" << flush;
      if ( ! optional_door)
	throw runtime_error("mpglue::createSBPLEnvironment(): XYTHETADOOR environment requires... a door!");
      
      door_msgs::Door doormsg;
      doormsg.frame_p1.x = optional_door->px; // hinge
      doormsg.frame_p1.y = optional_door->py;
      doormsg.frame_p2.x = optional_door->px + optional_door->width * cos(optional_door->th_shut); // other end
      doormsg.frame_p2.y = optional_door->py + optional_door->width * sin(optional_door->th_shut);
      doormsg.handle.x = optional_door->px + optional_door->dhandle * cos(optional_door->th_shut); // handle
      doormsg.handle.y = optional_door->py + optional_door->dhandle * sin(optional_door->th_shut);
      doormsg.hinge = 0;
      if (sfl::mod2pi(optional_door->th_open - optional_door->th_shut) > 0)
	doormsg.rot_dir = 1;
      else
	doormsg.rot_dir = -1;
      // XXXX to do: make this configurable (but where?)
      doormsg.header.frame_id = "map";
      
      default_fwd_search = true;
      return SBPLEnvironment::createXYThetaDoor(costmap,
							indexTransform,
							footprint,
							request.robot_nominal_forward_speed,
							timetoturn45degsinplace_secs,
							mprim_filename,
							verbose_os,
							doormsg);
    }
    
    throw runtime_error("mpglue::createSBPLEnvironment(): invalid environment token \""
			+ envstr + "\", must be 2d, 2d16, 3dkin, xythetalat, or xythetadoor");
    return 0;
  }
  
  
  CostmapPlanner *
  createCostmapPlanner(requestspec const & request,
		       boost::shared_ptr<CostmapAccessor const> costmap,
		       boost::shared_ptr<IndexTransform const> indexTransform,
		       footprint_t const & footprint,
		       /** only for door planner... should be refactored into something cleaner */
		       doorspec * optional_door,
		       /** set to null if you do not want progress output */
		       std::ostream * verbose_os)
    throw(std::runtime_error)
  {
    if (request.planner_tok.empty())
      throw runtime_error("mpglue::createCostmapPlanner(): no planner tokens in request");
    string const canonical_planner_name(canonicalPlannerName(request.planner_tok[0]));
    
    if ("NavFn" == canonical_planner_name)
      return createNavFnPlanner(request, costmap, indexTransform, verbose_os);
    
    if ("EStar" == canonical_planner_name)
      return createEstarPlanner(request, costmap, indexTransform, verbose_os);
    
    // all others must be SBPL... UNLESS the user made an error, so
    // check for that before creating the SBPLEnvironment instance.
    //
    // XXXX this check is done twice, here and further down, make sure
    // to update both places when adding or removing SBPL planners
    if (("ARAStar" != canonical_planner_name) && ("ADStar" != canonical_planner_name))
      throw runtime_error("mpglue::createCostmapPlanner(): invalid canonical planner name \""
			  + canonical_planner_name + "\" from planner spec \""
			  + request.planner_spec + "\"");
    
    bool default_forwardsearch(false);
    shared_ptr<SBPLEnvironment>
      sbpl_environment(createSBPLEnvironment(request, costmap, indexTransform, footprint,
					     default_forwardsearch, optional_door, verbose_os));
    if ( ! sbpl_environment)
      throw runtime_error("mpglue::createCostmapPlanner(): failed to create SBPLEnvironment from \""
			  + request.planner_spec + "\"");
    
    string dirstr;
    if (default_forwardsearch)
      dirstr = "fwd";
    else
      dirstr = "bwd";
    if (verbose_os)
      *verbose_os << "mpglue::createCostmapPlanner(): default search direction: forward\n" << flush;
    
    // token index 2 is "custom", read for lattice planner elsewhere
    sfl::token_to(request.planner_tok, 3, dirstr);
    
    // yes yes , it's a bit stupid to handle the default flag through
    // a string... have to find out a cleaner way for integrating the
    // door planner cerational pattern, which is a bit tricky because
    // it was a hack "back when"
    bool forwardsearch;
    if ("fwd" == dirstr)
      forwardsearch = true;
    else if ("bwd" == dirstr)
      forwardsearch = false;
    else
      throw runtime_error("mpglue::createCostmapPlanner(): invalid search direction \"" + dirstr
			  + "\", should be fwd or bwd");
    if (verbose_os)
      *verbose_os << "mpglue::createCostmapPlanner(): effective search direction: forward\n" << flush;
    
    // XXXX this check is done twice, here and further up, make sure
    // to update both places when adding or removing SBPL planners
    shared_ptr<SBPLPlanner> sbpl_planner;
    if ("ARAStar" == canonical_planner_name) {
      if (verbose_os)
	*verbose_os << "mpglue::createCostmapPlanner(): ARAPlanner\n" << flush;	
      sbpl_planner.reset(new ARAPlanner(sbpl_environment->getDSI(), forwardsearch));
    }
    else if ("ADStar" == canonical_planner_name) {
      if (verbose_os)
	*verbose_os << "mpglue::createCostmapPlanner(): ADPlanner\n" << flush;	
      sbpl_planner.reset(new ADPlanner(sbpl_environment->getDSI(), forwardsearch));
    }
    else
      throw runtime_error("mpglue::createCostmapPlanner(): invalid canonical planner name \""
			  + canonical_planner_name + "\" from planner spec \""
			  + request.planner_spec + "\"");
    
    return new SBPLPlannerWrap(sbpl_environment, sbpl_planner);
  }


}
