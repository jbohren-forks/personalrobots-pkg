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

#include "setup.h"
#include "parse.h"
#include <mpglue/navfn_planner.h>
#include <mpglue/sbpl_environment.h>
#include <mpglue/sbpl_planner.h>
#include <mpglue/estar_planner.h>
#include <costmap_2d/obstacle_map_accessor.h>
#include <sbpl/headers.h>
////#include <costmap_2d/costmap_2d.h>
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/util/strutil.hpp>
#include <errno.h>
#include <cstring>
#include <err.h>

#ifdef MPBENCH_HAVE_NETPGM
extern "C" {
#include <stdio.h>
// pgm.h is not very friendly with system headers... need to undef max() and min() afterwards
#include <pgm.h>
#undef max
#undef min
}
#endif // MPBENCH_HAVE_NETPGM

using sfl::minval;
using sfl::maxval;
using sfl::to_string;
using namespace boost;
using namespace std;

namespace {
  
  
  struct obstacle_adder: public sfl::GridFrame::draw_callback {
    obstacle_adder(mpbench::indexlist_t & _il, ostream * _dbgos)
      : il(_il), dbgos(_dbgos) {}
    
    virtual void operator () (ssize_t ix, ssize_t iy) {
      if (dbgos)
	*dbgos << "obstacle_adder: " << ix << " " << iy << "\n";
      il.insert(mpglue::index_pair(ix, iy));
    }
    
    mpbench::indexlist_t & il;
    ostream * dbgos;
  };
  
  
  std::string canonicalPlannerName(std::string const & name_or_alias)
  {
    static map<string, string> planner_alias;
    if (planner_alias.empty()) {
      planner_alias.insert(make_pair("ARAStar",      "ARAStar"));
      planner_alias.insert(make_pair("ara",          "ARAStar"));
      planner_alias.insert(make_pair("ARA",          "ARAStar"));
      planner_alias.insert(make_pair("arastar",      "ARAStar"));
    
      planner_alias.insert(make_pair("ADStar",       "ADStar"));
      planner_alias.insert(make_pair("ad",           "ADStar"));
      planner_alias.insert(make_pair("AD",           "ADStar"));
      planner_alias.insert(make_pair("adstar",       "ADStar"));
      
      planner_alias.insert(make_pair("NavFn",        "NavFn"));
      planner_alias.insert(make_pair("navfn",        "NavFn"));
      planner_alias.insert(make_pair("nf",           "NavFn"));
      planner_alias.insert(make_pair("NF",           "NavFn"));
      
      planner_alias.insert(make_pair("EStar",        "EStar"));
      planner_alias.insert(make_pair("estar",        "EStar"));
      planner_alias.insert(make_pair("Estar",        "EStar"));
    }
    
    map<string, string>::const_iterator is(planner_alias.find(name_or_alias));
    if (planner_alias.end() == is)
      return "";
    return is->second;
  }
  
  
  void drawDots(mpbench::Setup & setup,
		double hall, double door,
		std::ostream * progress_os)
  {
    setup.drawPoint(   0,     0, progress_os);
    setup.drawPoint(   0,  hall, progress_os);
    setup.drawPoint(hall,     0, progress_os);
    setup.drawPoint(hall,  hall, progress_os);
    setup.drawPoint(door,  door, progress_os);
    
    double const tol_xy(0.5 * door);
    double const tol_th(M_PI);
    setup.legacyAddTask("left to right", true,
		     0, 0.5 * hall, 0,
		  hall, 0.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("right to left", false,
		  hall, 0.5 * hall, 0,
		     0, 0.5 * hall, 0, tol_xy, tol_th);
  }
  
  
  void drawSquare(mpbench::Setup & setup,
		  double hall, double door,
		  std::ostream * progress_os, std::ostream * debug_os)
  {
    setup.drawLine(   0,     0,  hall,     0, progress_os, debug_os);
    setup.drawLine(   0,     0,     0,  hall, progress_os, debug_os);
    setup.drawLine(   0,  hall,  hall,  hall, progress_os, debug_os);
    setup.drawLine(hall,     0,  hall,  hall, progress_os, debug_os);

    double const tol_xy(0.5 * door);
    double const tol_th(M_PI);
    setup.legacyAddTask("left to right", true,
		         door, 0.5 * hall,   M_PI / 4,
		  hall - door, 0.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("right to left", false,
		  hall - door, 0.5 * hall, - M_PI / 4,
		         door, 0.5 * hall, 0, tol_xy, tol_th);
  }
  
  
  void drawOffice1(mpbench::Setup & setup,
		   double hall, double door,
		   std::ostream * progress_os, std::ostream * debug_os)
  {
    // outer bounding box
    setup.drawLine(       0,         0,  3 * hall,         0, progress_os, debug_os);
    setup.drawLine(       0,         0,         0,  5 * hall, progress_os, debug_os);
    setup.drawLine(       0,  5 * hall,  3 * hall,  5 * hall, progress_os, debug_os);
    setup.drawLine(3 * hall,         0,  3 * hall,  5 * hall, progress_os, debug_os);
    
    // two long walls along y-axis, each with a door near the northern end
    setup.drawLine(    hall,  3 * hall,      hall,  5 * hall - 2 * door, progress_os, debug_os);
    setup.drawLine(    hall,  5 * hall,      hall,  5 * hall -     door, progress_os, debug_os);
    setup.drawLine(2 * hall,      hall,  2 * hall,  5 * hall - 2 * door, progress_os, debug_os);
    setup.drawLine(2 * hall,  5 * hall,  2 * hall,  5 * hall -     door, progress_os, debug_os);
    
    // some shorter walls along x-axis
    setup.drawLine(         0,      hall,  0.5 * hall,      hall, progress_os, debug_os);
    setup.drawLine(         0,  2 * hall,  0.5 * hall,  2 * hall, progress_os, debug_os);
    setup.drawLine(1.5 * hall,      hall,  2   * hall,      hall, progress_os, debug_os);
    
    // y-axis wall with two office doors
    setup.drawLine(0.5*hall,              0,  0.5*hall,    hall - 2*door, progress_os, debug_os);
    setup.drawLine(0.5*hall,  hall -   door,  0.5*hall,    hall +   door, progress_os, debug_os);
    setup.drawLine(0.5*hall,  hall + 2*door,  0.5*hall,  2*hall         , progress_os, debug_os);
    
    // tasks...
    if (progress_os)
      *progress_os << "adding tasks...\n" << flush;
    
    double const tol_xy(0.25 * door);
    double const tol_th(M_PI);
    setup.legacyAddTask("through door or around hall", true,
		  0.5 * hall, 4.5 * hall, 0,
		  1.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("through door or around hall, return trip", false,
		  1.5 * hall, 4.5 * hall, 0,
		  0.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("through door or around long hall", true,
		  1.5 * hall, 4.5 * hall, 0,
		  2.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("through door or around long hall, return trip", false,
		  2.5 * hall, 4.5 * hall, 0,
		  1.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("office to office", true,
		  0.25 * hall, 0.5 * hall, 0,
		  0.25 * hall, 1.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("office to office, return trip", false,
		  0.25 * hall, 1.5 * hall, 0,
		  0.25 * hall, 0.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("office to hall", true,
		  0.25 * hall, 0.5 * hall, 0,
		         hall,       hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("office to hall, return trip", false,
		         hall,        hall, 0,
		  0.25 * hall, 0.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("hall to office", true,
		         hall,       hall, 0,
		  0.25 * hall, 1.5 * hall, 0, tol_xy, tol_th);
    setup.legacyAddTask("hall to office, return trip", false,
		  0.25 * hall, 1.5 * hall, 0,
		         hall,       hall, 0, tol_xy, tol_th);
  }
  
  
  void drawCubicle(mpbench::Setup & setup,
		   bool incremental,
		   bool minimal,
		   double hall, double door,
		   size_t ncube,
		   std::ostream * progress_os, std::ostream * debug_os)
  {
    if (ncube < 2)
      ncube = 2;
    
    double const alpha(M_PI / (4 * (ncube - 1)));
    double const R0(hall / alpha);
    double const R1(R0 + hall);
    double const yoff(0.5 * hall * R1 / R0);
    
    // lowermost wall
    setup.drawLine(R0,
		   - 0.5 * hall + yoff,
		   R1 * cos( - alpha / 2),
		   R1 * sin( - alpha / 2) + yoff, progress_os, debug_os);
    
    for (size_t ii(0); ii < ncube; ++ii) {
      double const ux(cos(ii * alpha));
      double const uy(sin(ii * alpha));
      double const nx(-uy);
      double const ny(ux);
      
      // door
      setup.drawLine(R0 * ux - 0.5 * hall * nx,
		     R0 * uy - 0.5 * hall * ny + yoff,
		     R0 * ux - 0.5 * door * nx,
		     R0 * uy - 0.5 * door * ny + yoff, progress_os, debug_os);
      setup.drawLine(R0 * ux + 0.5 * door * nx,
		     R0 * uy + 0.5 * door * ny + yoff,
		     R0 * ux + 0.5 * hall * nx,
		     R0 * uy + 0.5 * hall * ny + yoff, progress_os, debug_os);
      // upper wall
      setup.drawLine(R0 * ux + 0.5 * hall * nx,
		     R0 * uy + 0.5 * hall * ny + yoff,
		     R1 * cos((ii + 0.5) * alpha),
		     R1 * sin((ii + 0.5) * alpha) + yoff, progress_os, debug_os);
      // hind wall
      setup.drawLine(R1 * cos((ii + 0.5) * alpha),
		     R1 * sin((ii + 0.5) * alpha) + yoff,
		     R1 * cos((ii - 0.5) * alpha),
		     R1 * sin((ii - 0.5) * alpha) + yoff, progress_os, debug_os);
    }
    
    // tasks...
    if (progress_os)
      *progress_os << "adding tasks...\n" << flush;
    
    double const tol_xy(0.25 * door);
    double const tol_th(M_PI);
    double const gx(0.5 * R0 * cos(M_PI / 8));
    double const gy(0.5 * R0 * sin(M_PI / 8) + yoff);
    double const alloc_time(0.001);
    for (size_t ii(0); ii < ncube; ++ii) {
      ostringstream os;
      os << "from hall to cubicle " << ii;
      if (incremental) {
	mpbench::task::setup foo(os.str(),
				 mpbench::task::goalspec((R0 + hall / 2) * cos(ii * alpha),
							 (R0 + hall / 2) * sin(ii * alpha) + yoff,
							 0, tol_xy, tol_th));
	foo.start.push_back(mpbench::task::startspec(true, true, false, 3600, gx, gy, 0));
	foo.start.push_back(mpbench::task::startspec(false, false, true, alloc_time, gx, gy, 0));
	setup.addTask(foo);
      }
      else
	setup.legacyAddTask(os.str(), true, gx, gy, 0,
			    (R0 + hall / 2) * cos(ii * alpha),
			    (R0 + hall / 2) * sin(ii * alpha) + yoff,
			    0, tol_xy, tol_th);
      if (minimal)
	break;
      os << ", return trip";
      if (incremental) {
	mpbench::task::setup foo(os.str(),
				 mpbench::task::goalspec(gx, gy, 0, tol_xy, tol_th));
	foo.start.push_back(mpbench::task::startspec(true, true, false, 3600,
						     (R0 + hall / 2) * cos(ii * alpha),
						     (R0 + hall / 2) * sin(ii * alpha) + yoff,
						     0));
	foo.start.push_back(mpbench::task::startspec(false, false, true, alloc_time,
						     (R0 + hall / 2) * cos(ii * alpha),
						     (R0 + hall / 2) * sin(ii * alpha) + yoff,
						     0));
	setup.addTask(foo);
      }
      else
	setup.legacyAddTask(os.str(), false,
			    (R0 + hall / 2) * cos(ii * alpha),
			    (R0 + hall / 2) * sin(ii * alpha) + yoff,
			    0, gx, gy, 0, tol_xy, tol_th);
    }
  }
  
  
#ifdef MPBENCH_HAVE_NETPGM
  void readNetPGM(mpbench::Setup & setup,
		  FILE * pgmfile,
		  unsigned int obstacle_gray,
		  bool invert_gray,
		  double resolution)
  {
    // not sure how to properly handle this, the doc does not say
    // whether it supports being initialized more than once...
    static bool pgm_has_been_initialized(false);
    if ( ! pgm_has_been_initialized) {
      static int fake_argc(1);
      static char * fake_arg("foo");
      pgm_init(&fake_argc, &fake_arg);
      pgm_has_been_initialized = true;
    }
    
    int ncols, nrows;
    gray maxval;
    int format;
    pgm_readpgminit(pgmfile, &ncols, &nrows, &maxval, &format);
    gray * row(pgm_allocrow(ncols));
    for (int ii(nrows - 1); ii >= 0; --ii) {
      pgm_readpgmrow(pgmfile, row, ncols, maxval, format);
      for (int jj(ncols - 1); jj >= 0; --jj)
	if ((       invert_gray  && (obstacle_gray >= row[jj]))
	    || (( ! invert_gray) && (obstacle_gray <= row[jj])))
	  setup.drawPoint(jj * resolution, ii * resolution, 0);
    }
    pgm_freerow(row);
  }
#endif // MPBENCH_HAVE_NETPGM
  
  
  class OfficeBenchmark
    : public mpbench::Setup
  {
  protected:
    OfficeBenchmark(mpbench::SetupOptions const & options,
		    std::ostream * progress_os);
    
  public:
    static shared_ptr<OfficeBenchmark> create(mpbench::SetupOptions const & options,
					      std::ostream * progress_os,
					      std::ostream * debug_os);
    
    virtual void dumpSubDescription(std::ostream & os,
				    std::string const & prefix) const;
    
    double door_width;
    double hall_width;
  };
  
  
  mpbench::Setup * createNetPGMBenchmark(mpbench::SetupOptions const & opt,
					 std::ostream * progress_os) throw(runtime_error);
  
  void readXML(string const & xmlFileName, mpbench::Setup * setup, std::ostream * progress_os)
    throw(runtime_error);  
  
  mpbench::Setup * createXMLBenchmark(mpbench::SetupOptions const & opt,
				      std::ostream * progress_os) throw(runtime_error);
  
}


namespace mpbench {
  
  
  Setup::
  Setup(SetupOptions const & options)
    : opt_(options),
      gridframe_(options.costmap_resolution),
      bbx0_(0),
      bby0_(0),
      bbx1_(0),
      bby1_(0)
  {
  }
  
  
  Setup::
  ~Setup()
  {
  }
  
  
  void Setup::
  drawLine(double x0, double y0, double x1, double y1,
	   std::ostream * progress_os, std::ostream * debug_os)
  {
    if (progress_os)
      *progress_os << "Setup::drawLine(" << x0 << "  " << y0 << "  "
		   << x1 << "  " << y1 << ")\n" << flush;
    obstacle_adder obstadd(wspace_obstacles_, debug_os);
    gridframe_.DrawGlobalLine(x0, y0, x1, y1,
			      0, std::numeric_limits<ssize_t>::max(),
			      0, std::numeric_limits<ssize_t>::max(),
			      obstadd);
    bbx0_ = minval(bbx0_, minval(x0, x1));
    bby0_ = minval(bby0_, minval(y0, y1));
    bbx1_ = maxval(bbx1_, maxval(x0, x1));
    bby1_ = maxval(bby1_, maxval(y0, y1));
  }
  
  
  void Setup::
  drawPoint(double xx, double yy,
	    std::ostream * progress_os)
  {
    if (progress_os)
      *progress_os << "Setup::drawPoint(" << xx << "  " << yy << ")\n" << flush;
    sfl::GridFrame const gridframe(opt_.costmap_resolution);
    sfl::GridFrame::index_t const idx(gridframe_.GlobalIndex(xx, yy));
    wspace_obstacles_.insert(mpglue::index_pair(idx.v0, idx.v1));
    bbx0_ = minval(bbx0_, xx);
    bby0_ = minval(bby0_, yy);
    bbx1_ = maxval(bbx1_, xx);
    bby1_ = maxval(bby1_, yy);
  }
  
  
  void Setup::
  legacyAddTask(std::string const & description,
		bool from_scratch,
		double start_x, double start_y, double start_th, 
		double goal_x, double goal_y, double goal_th, 
		double goal_tol_xy, double goal_tol_th)
  {
    shared_ptr<task::setup>
      setup(new task::setup(description, task::goalspec(goal_x, goal_y, goal_th,
							goal_tol_xy, goal_tol_th)));
    setup->start.push_back(task::startspec(from_scratch,
					   false, false, numeric_limits<double>::max(),
					   start_x, start_y, start_th));
    tasklist_.push_back(setup);
  }
  
  
  void Setup::
  addTask(task::setup const & setup)
  {
    shared_ptr<task::setup> foo(new task::setup(setup));
    tasklist_.push_back(foo);
  }
  
  
  tasklist_t const & Setup::
  getTasks() const
  {
    return tasklist_;
  }
  
}

using namespace mpbench;

namespace {
  
  OfficeBenchmark::
  OfficeBenchmark(SetupOptions const & options,
		  std::ostream * progress_os)
    : Setup(options),
      door_width(1.2),
      hall_width(3)
  {
    if (sfl::token_to(options.world_tok, 2, door_width) && progress_os)
      *progress_os << "mpbench::OfficeBenchmark: door_width set to " << door_width
		   << "\n" << flush;
    if (sfl::token_to(options.world_tok, 3, hall_width) && progress_os)
      *progress_os << "mpbench::OfficeBenchmark: hall_width set to " << hall_width
		   << "\n" << flush;
  }
  
  
  shared_ptr<OfficeBenchmark> OfficeBenchmark::
  create(SetupOptions const & options,
	 std::ostream * progress_os,
	 std::ostream * debug_os)
  {
    shared_ptr<OfficeBenchmark> setup;
    string name;
    if ( ! sfl::token_to(options.world_tok, 1, name)) {
      if (progress_os)
	*progress_os << "OfficeBenchmark::create(): could not extract name from spec\n" << flush;
      return setup;
    }
    setup.reset(new OfficeBenchmark(options, progress_os));
    if ("dots" == name)
      drawDots(*setup, setup->hall_width, setup->door_width, progress_os);
    else if ("square" == name)
      drawSquare(*setup, setup->hall_width, setup->door_width, progress_os, debug_os);
    else if ("office1" == name)
      drawOffice1(*setup, setup->hall_width, setup->door_width, progress_os, debug_os);
    else if ("cubicle" == name)
      drawCubicle(*setup, false, false, setup->hall_width, setup->door_width, 3,
		  progress_os, debug_os);
    else if ("cubicle2" == name)
      drawCubicle(*setup, true, false, setup->hall_width, setup->door_width, 3,
		  progress_os, debug_os);
    else if ("cubicle3" == name)
      drawCubicle(*setup, true, true, setup->hall_width, setup->door_width, 3,
		  progress_os, debug_os);
    else {
      setup.reset();
      if (progress_os)
	*progress_os << "OfficeBenchmark::create(): invalid name \"" << name << "\", use one of:\n"
		     << "  dots: 4 dots in a square of hall_width side length\n"
		     << "  square: a square of hall_width side length\n"
		     << "  office1: two offices, two hallways, some doors\n" << flush;
    }
    return setup;
  }
  
  
  void OfficeBenchmark::
  dumpSubDescription(std::ostream & os,
		     std::string const & prefix) const
  {
    os << prefix << "door_width:           " << door_width << "\n"
       << prefix << "hall_width:           " << hall_width << "\n";
  }

}

namespace mpbench {  
  
  
  void SetupOptions::
  dump(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "world spec:                   " << world_spec << "\n"
       << prefix << "planner spec:                 " << planner_spec << "\n"
       << prefix << "robot spec:                   " << robot_spec << "\n"
       << prefix << "costmap spec:                 " << costmap_spec << "\n"
       << prefix << "robot_name:                   " << robot_name << "\n"
       << prefix << "robot_inscribed_radius:       " << robot_inscribed_radius << "\n"
       << prefix << "robot_circumscribed_radius:   " << robot_circumscribed_radius << "\n"
       << prefix << "robot_nominal_forward_speed:  " << robot_nominal_forward_speed << "\n"
       << prefix << "robot_nominal_rotation_speed: " << robot_nominal_rotation_speed << "\n"
       << prefix << "costmap_name:                 " << costmap_name << "\n"
       << prefix << "costmap_resolution:           " << costmap_resolution << "\n"
       << prefix << "costmap_inscribed_radius:     " << costmap_inscribed_radius << "\n"
       << prefix << "costmap_circumscribed_radius: " << costmap_circumscribed_radius << "\n"
       << prefix << "costmap_inflation_radius:     " << costmap_inflation_radius << "\n"
       << prefix << "costmap_obstacle_cost:        " << costmap_obstacle_cost << "\n"
       << prefix << "pgm_obstacle_gray:            " << pgm_obstacle_gray << "\n"
       << prefix << "pgm_invert_gray:              " << pgm_invert_gray << "\n";
  }
  
  
  void Setup::
  dumpDescription(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    opt_.dump(os, title, prefix);
    dumpSubDescription(os, prefix);
    os << prefix << "tasks:\n";
    for (tasklist_t::const_iterator it(tasklist_.begin()); it != tasklist_.end(); ++it) {
      os << prefix << "  - " << (*it)->description;
      if ((*it)->start.empty())
	os << prefix << " (no episode)\n";
      else if (1 == (*it)->start.size())
	os << prefix << " (1 episode)\n";
      else
	os << prefix << " (" << (*it)->start.size() << " episodes)\n";
      os << prefix << "    goal " << (*it)->goal.px << "  " << (*it)->goal.py << "  "
	 << (*it)->goal.pth << "  " << (*it)->goal.tol_xy << "  " << (*it)->goal.tol_th << "\n";
      for (vector<task::startspec>::const_iterator is((*it)->start.begin());
	   is != (*it)->start.end(); ++is)
	os << prefix << "    start " << to_string(is->from_scratch)
	   << " " << to_string(is->use_initial_solution)
	   << " " << to_string(is->allow_iteration)
	   << " " << is->alloc_time
	   << " " << is->px << "  " << is->py << "  " << is->pth << "\n";
    }
  }
  
  
  void Setup::
  dumpSubDescription(std::ostream & os,
		     std::string const & prefix) const
  {
    // nop
  }
  
  
  namespace task {
    
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
    
    setup::
    setup(std::string const & _description, goalspec const & _goal)
      : description(_description),
	goal(_goal)
    {
    }
    
    setup::
    setup(setup const & orig)
      : description(orig.description),
	goal(orig.goal),
	start(orig.start)
    {
    }
    
  }
  
  
  void Setup::
  getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_resolution;
    y0 = bby0_ - opt_.costmap_resolution;
    x1 = bbx1_ + opt_.costmap_resolution;
    y1 = bby1_ + opt_.costmap_resolution;
  }
  
  
  void Setup::
  getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_inscribed_radius;
    y0 = bby0_ - opt_.costmap_inscribed_radius;
    x1 = bbx1_ + opt_.costmap_inscribed_radius;
    y1 = bby1_ + opt_.costmap_inscribed_radius;
  }
  
  
  void Setup::
  getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_circumscribed_radius;
    y0 = bby0_ - opt_.costmap_circumscribed_radius;
    x1 = bbx1_ + opt_.costmap_circumscribed_radius;
    y1 = bby1_ + opt_.costmap_circumscribed_radius;
  }
  
  
  void Setup::
  getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - opt_.costmap_inflation_radius;
    y0 = bby0_ - opt_.costmap_inflation_radius;
    x1 = bbx1_ + opt_.costmap_inflation_radius;
    y1 = bby1_ + opt_.costmap_inflation_radius;
  }
  
  
  boost::shared_ptr<mpglue::CostmapPlanner> Setup::
  getPlanner()
  {
    return planner_;
  }
  
  
  boost::shared_ptr<mpglue::CostmapAccessor const> Setup::
  getCostmap() const
  {
    return costmapper_->getAccessor();
  }
  
  
  boost::shared_ptr<mpglue::IndexTransform const> Setup::
  getIndexTransform() const
  {
    return costmapper_->getIndexTransform();
  }
  
  
  SetupOptions::
  SetupOptions(std::string const & _world_spec,
	       std::string const & _planner_spec,
	       std::string const & _robot_spec,
	       std::string const & _costmap_spec)
    : world_spec(_world_spec),
      planner_spec(_planner_spec),
      robot_spec(_robot_spec),
      costmap_spec(_costmap_spec),
      robot_name("pr2"),
      robot_inscribed_radius(0.325),
      robot_circumscribed_radius(0.46),
      robot_nominal_forward_speed(0.6),	// approx human walking speed
      robot_nominal_rotation_speed(0.6), // XXXX guesstimate
      costmap_name("sfl"),
      costmap_resolution(0.05),
      costmap_inscribed_radius(0.325),
      costmap_circumscribed_radius(0.46),
      costmap_inflation_radius(0.55),
      costmap_obstacle_cost(costmap_2d::ObstacleMapAccessor::INSCRIBED_INFLATED_OBSTACLE),
      pgm_obstacle_gray(64),
      pgm_invert_gray(true)
  {
    sfl::tokenize(world_spec, ':', world_tok);
    sfl::tokenize(planner_spec, ':', planner_tok);
    sfl::tokenize(robot_spec, ':', robot_tok);
    sfl::tokenize(costmap_spec, ':', costmap_tok);
    
    sfl::token_to(robot_tok, 0, robot_name);
    if (sfl::token_to(robot_tok, 1, robot_inscribed_radius))
      robot_inscribed_radius *= 1e-3;
    if (sfl::token_to(robot_tok, 2, robot_circumscribed_radius))
      robot_circumscribed_radius *= 1e-3;
    if (sfl::token_to(robot_tok, 3, robot_nominal_forward_speed))
      robot_nominal_forward_speed *= 1e-3;
    if (sfl::token_to(robot_tok, 4, robot_nominal_rotation_speed))
      robot_nominal_rotation_speed *= 1e-3;
    
    sfl::token_to(costmap_tok, 0, costmap_name);
    if (sfl::token_to(costmap_tok, 1, costmap_resolution))
      costmap_resolution *= 1e-3;
    if (sfl::token_to(costmap_tok, 2, costmap_inscribed_radius))
      costmap_inscribed_radius *= 1e-3;
    if (sfl::token_to(costmap_tok, 3, costmap_circumscribed_radius))
      costmap_circumscribed_radius *= 1e-3;
    if (sfl::token_to(costmap_tok, 4, costmap_inflation_radius))
      costmap_inflation_radius *= 1e-3;
    sfl::token_to(costmap_tok, 5, costmap_obstacle_cost);
  }


  void SetupOptions::
  help(std::ostream & os, std::string const & title, std::string const & prefix)
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "available world specs:\n"
       << prefix << "  hc  : name [: door_width [: hall_width ]]\n"
       << prefix << "        name can be dots, square, office1, cubicle, cubcile2, cubicle3\n"
       << prefix << "        door_width and hall_width are in meters (default 1.2 and 3)\n"
       << prefix << "  pgm : pgm_filename [: xml_filename ]\n"
       << prefix << "        xml_filename is optional, but needed to define tasks etc\n"
       << prefix << "  xml : xml_filename\n"
       << prefix << "available planner specs (can use registered aliases instead):\n"
       << prefix << "  NavFn [: int | dsc ]\n"
       << prefix << "        int = interpolate path (default)\n"
       << prefix << "        dsc = use discretized path instead\n"
       << prefix << "  ADStar | ARAStar [: 2d | 3dkin [: bwd | fwd ]]\n"
       << prefix << "        2d = use 2D environment (default)\n"
       << prefix << "        3dkin = use 3DKIN environment\n"
       << prefix << "        bwd = use backward search (default)\n"
       << prefix << "        fwd = use forward search\n"
       << prefix << "  EStar\n"
       << prefix << "        (no further options yet)\n"
       << prefix << "available robot specs:\n"
       << prefix << "  pr2 [: inscribed [: circumscribed [: fwd_speed [: rot_speed ]]]]\n"
       << prefix << "        inscribed radius (millimeters), default 325mm\n"
       << prefix << "        circumscribed radius (millimeters), default 460mm\n"
       << prefix << "        nominal forward speed (millimeters per second), default 600mm/s\n"
       << prefix << "        nominal rotation speed (milliradians per second), default 600mrad/s\n"
       << prefix << "available costmap specs:\n"
       << prefix << "  sfl | ros [: resolution [: inscribed [: circumscribed [: inflation ]]]]\n"
       << prefix << "        all lengths and radii in millimeters\n"
       << prefix << "        defaults: resol. 50mm, inscr. 325mm, circ. 460mm, infl. 550mm\n";
  }
  
  
  shared_ptr<Setup> Setup::
  create(std::string const & world_spec,
	 std::string const & planner_spec,
	 std::string const & robot_spec,
	 std::string const & costmap_spec,
	 std::ostream * progress_os,
	 std::ostream * debug_os) throw(std::runtime_error)
  {
    SetupOptions const opt(SetupOptions(world_spec,
					planner_spec,
					robot_spec,
					costmap_spec));
    
    if (progress_os)
      *progress_os << "creating world from \"" << opt.world_spec << "\"\n" << flush;
    
    shared_ptr<Setup> setup;
    if (opt.world_tok.empty())
      throw runtime_error("mpbench::Setup::create(): could not extract world format from \""
			  + opt.world_spec + "\"");
    if ("hc" == opt.world_tok[0])
      setup = OfficeBenchmark::create(opt, progress_os, debug_os);
    else if ("pgm" == opt.world_tok[0])
      setup.reset(createNetPGMBenchmark(opt, progress_os));
    else if ("xml" == opt.world_tok[0])
      setup.reset(createXMLBenchmark(opt, progress_os));
    else
      throw runtime_error("mpbench::Setup::create(): invalid format \"" + opt.world_tok[0]
			  + "\" in spec \"" + opt.world_spec + "\"");
    
    if (progress_os)
      *progress_os << "creating CostmapManager (currently hardcoded sfl::Mapper2d)\n" << flush;
    boost::shared_ptr<sfl::Mapper2d::travmap_grow_strategy>
      growstrategy(new sfl::Mapper2d::always_grow());
    double const buffer_zone(opt.costmap_circumscribed_radius - opt.costmap_inscribed_radius);
    double const padding_factor(0);
    shared_ptr<sfl::Mapper2d> m2d(new sfl::Mapper2d(setup->gridframe_,
						    0, 0, // grid_xbegin, grid_xend
						    0, 0, // grid_ybegin, grid_yend
						    opt.costmap_inscribed_radius,
						    sfl::maxval(0.0, buffer_zone),
						    padding_factor,
						    0, // freespace cost
						    opt.costmap_obstacle_cost,
						    // costmap_2d
						    // seems to use a
						    // quadratic decay
						    // in r7215
						    sfl::exponential_travmap_cost_decay(2),
						    "m2d",
						    sfl::RWlock::Create("m2d"),
						    growstrategy));
    setup->costmapper_ = mpglue::createCostmapper(m2d);
    
    if (progress_os)
      *progress_os << "creating cost map (currently just one episode)\n" << flush;
    setup->costmapper_->addObstacles(setup->wspace_obstacles_, debug_os);
    
    if (progress_os)
      *progress_os << "creating planner from \"" << opt.planner_spec << "\"\n" << flush;
    
    if (opt.planner_tok.empty())
      throw runtime_error("mpbench::Setup::create(): no planner tokens");
    string planner_name(canonicalPlannerName(opt.planner_tok[0]));
    shared_ptr<mpglue::CostmapPlanner> planner;
    
    if ("NavFn" == planner_name) {
      if (progress_os)
	*progress_os << "  creating NavFnPlanner\n" << flush;
      string int_str("int");
      sfl::token_to(opt.planner_tok, 1, int_str);
      bool interpolate_path(true);
      if ("dsc" == int_str)
	interpolate_path = false;
      else if ("int" != int_str)
	throw runtime_error("mpbench::Setup::create(): invalid environment interpolate_path \""
			    + int_str + "\", must be \"int\" or \"dsc\"");
      setup->planner_.reset(new mpglue::NavFnPlanner(setup->getCostmap(),
						     setup->getIndexTransform(),
						     interpolate_path));
    }

    else if ("EStar" == planner_name) {
#ifndef MPBENCH_HAVE_ESTAR
      throw runtime_error("mpbench::Setup::create(): no support for EStar planner, set the ESTAR_DIR environment variable and recompile mpglue and mpbench\"");
#else // MPBENCH_HAVE_ESTAR
      if (progress_os)
	*progress_os << "  creating EstarPlanner\n" << flush;
      //  string int_str("int");
      //  sfl::token_to(opt.planner_tok, 1, int_str);
      //  bool interpolate_path(true);
      //  if ("dsc" == int_str)
      // 	interpolate_path = false;
      //  else if ("int" != int_str)
      //throw runtime_error("mpbench::Setup::create(): invalid environment interpolate_path \""
      //		    + int_str + "\", must be \"int\" or \"dsc\"");
      setup->planner_.reset(new mpglue::EstarPlanner(setup->getCostmap(),
						     setup->getIndexTransform(),
						     &cerr));
#endif // MPBENCH_HAVE_ESTAR
    }
    
    else {
      if (progress_os)
	*progress_os << "  creating SBPLPlanner subtype\n" << flush;
      
      // The remaining possibilities are (currently) all derived from
      // SBPL and thus all need an SBPLEnvironment instance, so we
      // create that first.
      string envstr("2d");
      sfl::token_to(opt.planner_tok, 1, envstr);
      shared_ptr<mpglue::SBPLEnvironment> sbpl_environment;
      if ("2d" == envstr) {
	if (progress_os)
	  *progress_os << "  creating 8-connected 2DEnvironment\n" << flush;
	sbpl_environment.reset(mpglue::SBPLEnvironment::create2D(setup->getCostmap(),
								 setup->getIndexTransform(),
								 false));
      }
      else if ("2d16" == envstr) {
	if (progress_os)
	  *progress_os << "  creating 16-connected 2DEnvironment\n" << flush;
	sbpl_environment.reset(mpglue::SBPLEnvironment::create2D(setup->getCostmap(),
								 setup->getIndexTransform(),
								 true));
      }
      else if ("3dkin" == envstr) {
	if (progress_os)
	  *progress_os << "  creating 3DKINEnvironment\n" << flush;
	double const
	  timetoturn45degsinplace_secs(45.0 * M_PI / 180.0 / opt.robot_nominal_rotation_speed);
	sbpl_environment.
	  reset(mpglue::SBPLEnvironment::create3DKIN(setup->getCostmap(),
						     setup->getIndexTransform(),
						     setup->getFootprint(),
						     opt.robot_nominal_forward_speed,
						     timetoturn45degsinplace_secs,
						     progress_os));
      }
      else
	throw runtime_error("mpbench::Setup::create(): invalid environment token \""
			    + envstr + "\", must be \"2d\", \"2d16\" or \"3dkin\"");
      if ( ! sbpl_environment)
	throw runtime_error("mpbench::Setup::create(): failed to create SBPLEnvironment from \""
			    + envstr + "\"");
      
      string dirstr("bwd");
      sfl::token_to(opt.planner_tok, 2, dirstr);
      bool forwardsearch(false);
      if ("fwd" == dirstr) {
	if (progress_os)
	  *progress_os << "  search direction: forward\n" << flush;
	forwardsearch = true;
      }
      else if ("bwd" == dirstr) {
	if (progress_os)
	  *progress_os << "  search direction: backward\n" << flush;
      }
      else
	throw runtime_error("mpbench::Setup::create(): invalid search direction \"" + dirstr
			    + "\", should be \"fwd\" or \"bwd\"");
      
      shared_ptr<SBPLPlanner> sbpl_planner;
      if ("ARAStar" == planner_name) {
	if (progress_os)
	  *progress_os << "  creating ARAPlanner\n" << flush;	
	sbpl_planner.reset(new ARAPlanner(sbpl_environment->getDSI(), forwardsearch));
      }
      else if ("ADStar" == planner_name) {
	if (progress_os)
	  *progress_os << "  creating ADPlanner\n" << flush;	
	sbpl_planner.reset(new ADPlanner(sbpl_environment->getDSI(), forwardsearch));
      }
      else
	throw runtime_error("mpbench::Setup::create(): invalid planner name \"" + planner_name
			    + "\"");
      
      shared_ptr<mpglue::AnytimeCostmapPlanner>
	foo(new mpglue::SBPLPlannerWrap(sbpl_environment, sbpl_planner));
      foo->stopAtFirstSolution(false); // XXXX to do: use task::start
      foo->setAllocatedTime(numeric_limits<double>::max()); // XXXX to do: use task::start
      
      setup->planner_ = foo;
    }
    
    if ( ! setup->planner_)
      throw runtime_error("mpbench::Setup::create(): failed to create planner from spec \""
			  + opt.planner_spec + "\"");
    
    if (progress_os)
      *progress_os << "finished creating setup\n" << flush;
    return setup;
  }
  
  
  mpglue::footprint_t const & Setup::
  getFootprint() const
  {
    if ( ! footprint_) {
      footprint_.reset(new mpglue::footprint_t());
      mpglue::initSimpleFootprint(*footprint_,
				  opt_.robot_inscribed_radius,
				  opt_.robot_circumscribed_radius);
    }
    return *footprint_;
  }
  
}

namespace {
  
  mpbench::Setup * createNetPGMBenchmark(SetupOptions const & opt,
					 std::ostream * progress_os) throw(runtime_error)
  {
    mpbench::Setup * bench(0);

#ifndef MPBENCH_HAVE_NETPGM
    
    if (progress_os)
      *progress_os << "ERROR in createNetPGMBenchmark(): no support for netpgm!\n"
		   << "  Install the netpbm libraries and headers and recompile.\n"
		   << "  For example under Ubuntu Feisty the package\n"
		   << "  libnetpbm10-dev does the trick.\n";
    throw runtime_error("sorry, no support for netpgm");
    
#else // MPBENCH_HAVE_NETPGM
    
    if (opt.world_tok.size() < 2) {
      ostringstream os;
      os << "createNetPGMBenchmark(): no PGM filename in world spec \"" << opt.world_spec << "\"";
      if (progress_os)
	*progress_os << "ERROR in " << os.str() << "\n";
      throw runtime_error(os.str());
    }
    
    FILE * pgmfile(fopen(opt.world_tok[1].c_str(), "rb"));
    if ( ! pgmfile) {
      ostringstream os;
      os << "createNetPGMBenchmark(): fopen(" << opt.world_tok[1] << "): " << strerror(errno);
      if (progress_os)
	*progress_os << "ERROR in " << os.str() << "\n";
      throw runtime_error(os.str());
    }
    bench = new mpbench::Setup(opt);
    if (progress_os)
      *progress_os << "createNetPGMBenchmark(): translating PGM file " << opt.world_tok[1] << "\n";
    readNetPGM(*bench, pgmfile, opt.pgm_obstacle_gray, opt.pgm_invert_gray,
	       opt.costmap_resolution);
    
    if (opt.world_tok.size() > 2)
      readXML(opt.world_tok[2], bench, progress_os);
    
#endif // MPBENCH_HAVE_NETPGM
    
    return bench;
  }
  
  
  void readXML(string const & xmlFileName, mpbench::Setup * setup, std::ostream * progress_os)
    throw(runtime_error)
  {
    
#ifndef MPBENCH_HAVE_EXPAT
    
    if (progress_os)
      *progress_os << "no support for XML!\n"
		   << "  Install the expat library and recompile.\n"
		   << "  For example under Ubuntu Feisty the package\n"
		   << "  libexpat1-dev does the trick.\n";
    throw runtime_error("sorry, no support for XML");
    
#else // MPBENCH_HAVE_EXPAT
    
    if (progress_os)
      *progress_os << "parsing XML file " << xmlFileName << "\n";
    mpbench::SetupParser sp;
    sp.Parse(xmlFileName, setup, progress_os);
    
#endif // MPBENCH_HAVE_EXPAT    
    
  }
  
  
  mpbench::Setup * createXMLBenchmark(SetupOptions const & opt,
				      std::ostream * progress_os) throw(runtime_error)
  {
    mpbench::Setup * bench(0);
    
#ifndef MPBENCH_HAVE_EXPAT
    
    if (progress_os)
      *progress_os << "no support for XML!\n"
		   << "  Install the expat library and recompile.\n"
		   << "  For example under Ubuntu Feisty the package\n"
		   << "  libexpat1-dev does the trick.\n";
    throw runtime_error("sorry, no support for XML");
    
#else // MPBENCH_HAVE_EXPAT
    
    if (opt.world_tok.size() < 2) {
      ostringstream os;
      os << "createXMLBenchmark(): no XML filename in world spec \"" << opt.world_spec << "\"";
      if (progress_os)
	*progress_os << "ERROR in " << os.str() << "\n";
      throw runtime_error(os.str());
    }
    
    bench = new mpbench::Setup(opt);
    if (progress_os)
      *progress_os << "createXMLBenchmark(): parsing " << opt.world_tok[1] << "\n";
    readXML(opt.world_tok[1], bench, progress_os);
    
#endif // MPBENCH_HAVE_EXPAT
    
    return bench;
  }

}
