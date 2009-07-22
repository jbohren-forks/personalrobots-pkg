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
// #include "parse.h"
// #include "world.h"
// #include <mpglue/navfn_planner.h>
// #include <mpglue/sbpl_environment.h>
// #include <mpglue/sbpl_planner.h>
// #include <mpglue/estar_planner.h>
#include <costmap_2d/cost_values.h>
// #include <sbpl/headers.h>
// #include <sfl/util/numeric.hpp>
#include <sfl/util/strutil.hpp>
// #include <door_msgs/Door.h>
#include <cmath>

using namespace boost;
using namespace std;


namespace mpglue {  
  
  
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
  
}
