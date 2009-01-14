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

#include "setup.hpp"
#include "parse.h"
#include <costmap_2d/costmap_2d.h>
#include <sfl/gplan/GridFrame.hpp>
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
using namespace boost;
using namespace std;

namespace {
  
  
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
		  std::ostream * progress_os)
  {
    setup.drawLine(   0,     0,  hall,     0, progress_os);
    setup.drawLine(   0,     0,     0,  hall, progress_os);
    setup.drawLine(   0,  hall,  hall,  hall, progress_os);
    setup.drawLine(hall,     0,  hall,  hall, progress_os);

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
		   std::ostream * progress_os)
  {
    // outer bounding box
    setup.drawLine(       0,         0,  3 * hall,         0, progress_os);
    setup.drawLine(       0,         0,         0,  5 * hall, progress_os);
    setup.drawLine(       0,  5 * hall,  3 * hall,  5 * hall, progress_os);
    setup.drawLine(3 * hall,         0,  3 * hall,  5 * hall, progress_os);
    
    // two long walls along y-axis, each with a door near the northern end
    setup.drawLine(    hall,  3 * hall,      hall,  5 * hall - 2 * door, progress_os);
    setup.drawLine(    hall,  5 * hall,      hall,  5 * hall -     door, progress_os);
    setup.drawLine(2 * hall,      hall,  2 * hall,  5 * hall - 2 * door, progress_os);
    setup.drawLine(2 * hall,  5 * hall,  2 * hall,  5 * hall -     door, progress_os);
    
    // some shorter walls along x-axis
    setup.drawLine(         0,      hall,  0.5 * hall,      hall, progress_os);
    setup.drawLine(         0,  2 * hall,  0.5 * hall,  2 * hall, progress_os);
    setup.drawLine(1.5 * hall,      hall,  2   * hall,      hall, progress_os);
    
    // y-axis wall with two office doors
    setup.drawLine(0.5 * hall,              0,  0.5 * hall,    hall - 2*door, progress_os);
    setup.drawLine(0.5 * hall,  hall -   door,  0.5 * hall,    hall +   door, progress_os);
    setup.drawLine(0.5 * hall,  hall + 2*door,  0.5 * hall,  2*hall         , progress_os);
    
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
		   double hall, double door,
		   size_t ncube,
		   std::ostream * progress_os)
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
		   R1 * sin( - alpha / 2) + yoff, progress_os);
    
    for (size_t ii(0); ii < ncube; ++ii) {
      double const ux(cos(ii * alpha));
      double const uy(sin(ii * alpha));
      double const nx(-uy);
      double const ny(ux);
      
      // door
      setup.drawLine(R0 * ux - 0.5 * hall * nx,
		     R0 * uy - 0.5 * hall * ny + yoff,
		     R0 * ux - 0.5 * door * nx,
		     R0 * uy - 0.5 * door * ny + yoff, progress_os);
      setup.drawLine(R0 * ux + 0.5 * door * nx,
		     R0 * uy + 0.5 * door * ny + yoff,
		     R0 * ux + 0.5 * hall * nx,
		     R0 * uy + 0.5 * hall * ny + yoff, progress_os);
      // upper wall
      setup.drawLine(R0 * ux + 0.5 * hall * nx,
		     R0 * uy + 0.5 * hall * ny + yoff,
		     R1 * cos((ii + 0.5) * alpha),
		     R1 * sin((ii + 0.5) * alpha) + yoff, progress_os);
      // hind wall
      setup.drawLine(R1 * cos((ii + 0.5) * alpha),
		     R1 * sin((ii + 0.5) * alpha) + yoff,
		     R1 * cos((ii - 0.5) * alpha),
		     R1 * sin((ii - 0.5) * alpha) + yoff, progress_os);
    }
    
    // tasks...
    if (progress_os)
      *progress_os << "adding tasks...\n" << flush;
    
    double const tol_xy(0.25 * door);
    double const tol_th(M_PI);
    double const gx(0.5 * R0 * cos(M_PI / 8));
    double const gy(0.5 * R0 * sin(M_PI / 8) + yoff);
    for (size_t ii(0); ii < ncube; ++ii) {
      ostringstream os;
      os << "from hall to cubicle " << ii;
      setup.legacyAddTask(os.str(), true, gx, gy, 0,
		    (R0 + hall / 2) * cos(ii * alpha),
		    (R0 + hall / 2) * sin(ii * alpha) + yoff,
		    0, tol_xy, tol_th);
      os << ", return trip";
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
    OfficeBenchmark(std::string const & name,
		    double resolution,
		    double inscribed_radius,
		    double circumscribed_radius,
		    double inflation_radius,
		    int obstacle_cost,
		    bool use_sfl_cost,
		    double door_width,
		    double hall_width);
    
  public:
    /**
       Valid names are:
       - dots: 4 dots in a square of hall_width side length
       - square: a square of hall_width side length
       - office1: two offices, two hallways, some doors
    */
    static OfficeBenchmark * create(std::string const & name,
				    double resolution,
				    double inscribed_radius,
				    double circumscribed_radius,
				    double inflation_radius,
				    int obstacle_cost,
				    bool use_sfl_cost,
				    double door_width,
				    double hall_width,
				    std::ostream * progress_os);
    
    virtual void dumpSubDescription(std::ostream & os,
				    std::string const & prefix) const;
    
    double const door_width;
    double const hall_width;
  };
  
  
  mpbench::Setup * createNetPGMBenchmark(std::string const & pgmFileName,
					 std::string const & xmlFileName,
					 unsigned int obstacle_gray,
					 bool invert_gray,
					 double resolution,
					 double inscribed_radius,
					 double circumscribed_radius,
					 double inflation_radius,
					 int obstacle_cost,
					 bool use_sfl_cost,
					 std::ostream * progress_os) throw(runtime_error);
  
  void readXML(string const & xmlFileName, mpbench::Setup * setup, std::ostream * progress_os)
    throw(runtime_error);  
  
  mpbench::Setup * createXMLBenchmark(std::string const & xmlFileName,
				      double resolution,
				      double inscribed_radius,
				      double circumscribed_radius,
				      double inflation_radius,
				      int obstacle_cost,
				      bool use_sfl_cost,
				      std::ostream * progress_os) throw(runtime_error);
  
}


namespace mpbench {
  
  
  Setup::
  Setup(std::string const & _name,
		     double _resolution,
		     double _inscribed_radius,
		     double _circumscribed_radius,
		     double _inflation_radius,
		     int _obstacle_cost,
		     bool _use_sfl_cost)
    : name(_name),
      resolution(_resolution),
      inscribed_radius(_inscribed_radius),
      circumscribed_radius(_circumscribed_radius),
      inflation_radius(_inflation_radius),
      obstacle_cost(_obstacle_cost),
      use_sfl_cost(_use_sfl_cost),
      bbx0_(0),
      bby0_(0),
      bbx1_(0),
      bby1_(0)
  {
    sfl::GridFrame const gframe(resolution);
    boost::shared_ptr<sfl::Mapper2d::travmap_grow_strategy>
      growstrategy(new sfl::Mapper2d::always_grow());
    //    double const buffer_zone(sfl::maxval(0.0, inflation_radius - inscribed_radius));
    double const buffer_zone(sfl::maxval(0.0, circumscribed_radius - inscribed_radius));
    double const padding_factor(0);
    m2d_.reset(new sfl::Mapper2d(gframe, 0, 0, 0, 0,
				 inscribed_radius, buffer_zone, padding_factor,
				 0, obstacle_cost,
				 // costmap_2d seems to use a quadratic decay in r7215
				 sfl::exponential_travmap_cost_decay(2),
				 name, sfl::RWlock::Create(name), growstrategy));
    

  }
  
  
  Setup::
  ~Setup()
  {
  }


  void Setup::
  dumpTravmap(std::ostream & os) const
  {
    getRawSFLTravmap()->DumpMap(&os);
  }
  
  
  void Setup::
  drawLine(double x0, double y0, double x1, double y1,
	   std::ostream * progress_os)
  {
    if (progress_os)
      *progress_os << "Setup::drawLine(" << x0 << "  " << y0 << "  "
		   << x1 << "  " << y1 << ")\n" << flush;
    sfl::Mapper2d::buffered_obstacle_adder boa(m2d_.get(), 0);
    m2d_->gridframe.DrawGlobalLine(x0, y0, x1, y1,
				   0, std::numeric_limits<ssize_t>::max(),
				   0, std::numeric_limits<ssize_t>::max(),
				   boa);
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
    m2d_->AddBufferedObstacle(xx, yy, 0);
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
    setup->start.push_back(task::startspec(from_scratch, start_x, start_y, start_th));
    tasklist_.push_back(setup);
  }
  
  
  void Setup::
  addTask(task::setup const & setup)
  {
    shared_ptr<task::setup> foo(new task::setup(setup));
    tasklist_.push_back(foo);
  }
  
  
  boost::shared_ptr<sfl::RDTravmap> Setup::
  getRawSFLTravmap() const
  {
    if ( ! rdtravmap_)
      rdtravmap_ = m2d_->CreateRDTravmap();
    return rdtravmap_;
  }
  
  
  costmap_2d::CostMap2D const & Setup::
  getRaw2DCostmap() const
  {
    if ( ! costmap_)
      costmap_.reset(createCostMap2D());
    return *costmap_;
  }
  
  
  tasklist_t const & Setup::
  getTasks() const
  {
    return tasklist_;
  }
  
  
  costmap_2d::CostMap2D * Setup::
  createCostMap2D() const
  {
    boost::shared_ptr<sfl::RDTravmap> rdt(m2d_->CreateRDTravmap());
    ssize_t const width(rdt->GetXEnd());
    ssize_t const height(rdt->GetYEnd());
    std::vector<unsigned char> data;
    data.reserve(width * height);
    for (ssize_t iy(0); iy < height; ++iy)
      for (ssize_t ix(0); ix < width; ++ix) {
	
	if (rdt->IsWObst(ix, iy))
	  data.push_back(costmap_2d::ObstacleMapAccessor::LETHAL_OBSTACLE);
	
	else if (use_sfl_cost) {
	  int cost;
	  if (rdt->GetValue(ix, iy, cost)) {
	    if (cost >= 0xff)
	      data.push_back(0xff);
	    else
	      data.push_back(cost & 0xff);
	  }
	}
	
	else
	  data.push_back(0);	// suppose 0 means freespace
      }
    
    // hm... what if our obstacle cost needs more than 8 bits?    
    unsigned char const threshold(obstacle_cost & 0xff);
    double const maxZ(0.5);
    double const zLB(0.10);
    double const zUB(0.15);
    double const weight(1);
    costmap_2d::CostMap2D * cm;
    if (use_sfl_cost)
      cm = new costmap_2d::CostMap2D(width, height, data, resolution,
				     threshold, maxZ, zLB, zUB,
				     0,	// inflationRadius
				     0,	// circumscribedRadius
				     0,	// inscribedRadius
				     weight);
    else
      cm = new costmap_2d::CostMap2D(width, height, data, resolution,
				     threshold, maxZ, zLB, zUB,
				     inflation_radius, circumscribed_radius, inscribed_radius,
				     weight);
    return cm;
  }
    
}

namespace {
  
  OfficeBenchmark::
  OfficeBenchmark(std::string const & name,
		  double resolution,
		  double inscribed_radius,
		  double circumscribed_radius,
		  double inflation_radius,
		  int obstacle_cost,
		  bool use_sfl_cost,
		  double _door_width,
		  double _hall_width)
    : Setup(name, resolution,
			 inscribed_radius, circumscribed_radius, inflation_radius,
			 obstacle_cost, use_sfl_cost),
      door_width(_door_width),
      hall_width(_hall_width)
  {
  }
  
  
  OfficeBenchmark * OfficeBenchmark::
  create(std::string const & name,
	 double resolution,
	 double inscribed_radius,
	 double circumscribed_radius,
	 double inflation_radius,
	 int obstacle_cost,
	 bool use_sfl_cost,
	 double door_width,
	 double hall_width,
	 std::ostream * progress_os)
  {
    OfficeBenchmark * setup(new OfficeBenchmark(name, resolution,
						inscribed_radius,
						circumscribed_radius,
						inflation_radius,
						obstacle_cost,
						use_sfl_cost,
						door_width,
						hall_width));
    if ("dots" == name)
      drawDots(*setup, hall_width, door_width, progress_os);
    else if ("square" == name)
      drawSquare(*setup, hall_width, door_width, progress_os);
    else if ("office1" == name)
      drawOffice1(*setup, hall_width, door_width, progress_os);
    else if ("cubicle" == name)
      drawCubicle(*setup, hall_width, door_width, 3, progress_os);
    else {
      delete setup;
      if (progress_os)
	*progress_os << "OfficeBenchmark::create(): invalid name \"" << name << "\", use one of:\n"
		     << "  dots: 4 dots in a square of hall_width side length\n"
		     << "  square: a square of hall_width side length\n"
		     << "  office1: two offices, two hallways, some doors\n" << flush;
      return 0;
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
  
  void Setup::
  dumpDescription(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "name:                 " << name << "\n"
       << prefix << "resolution:           " << resolution << "\n"
       << prefix << "inscribed_radius:     " << inscribed_radius << "\n"
       << prefix << "circumscribed_radius: " << circumscribed_radius << "\n"
       << prefix << "inflation_radius:     " << inflation_radius << "\n"
       << prefix << "obstacle_cost:        " << obstacle_cost << "\n"
       << prefix << "use_sfl_cost:         " << (use_sfl_cost ? "true\n" : "false\n");
    dumpSubDescription(os, prefix);
    os << prefix << "tasks:\n";
    for (tasklist_t::const_iterator it(tasklist_.begin()); it != tasklist_.end(); ++it) {
      os << prefix << "  - " << (*it)->description << "\n";
      if ((*it)->start.empty())
	os << prefix << "    (no episode)\n";
      else if (1 == (*it)->start.size())
	os << prefix << "    (1 episode)\n";
      else
	os << prefix << "    (" << (*it)->start.size() << " episodes)\n";
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
	      double start_x,
	      double start_y,
	      double start_th)
      : from_scratch(_from_scratch),
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
    
    result::
    result(size_t _task_id,
	   size_t _episode_id,
	   startspec const & _start,
	   goalspec const & _goal,
	   boost::shared_ptr<mpglue::waypoint_plan_t> _plan,
	   boost::shared_ptr<mpglue::CostmapPlannerStats> _stats)
      : task_id(_task_id),
	episode_id(_episode_id),
	start(_start),
	goal(_goal),
	plan(_plan),
	stats(_stats)      
    {
    }
    
  }
  
  
  void Setup::
  getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - resolution;
    y0 = bby0_ - resolution;
    x1 = bbx1_ + resolution;
    y1 = bby1_ + resolution;
  }
  
  
  void Setup::
  getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - inscribed_radius;
    y0 = bby0_ - inscribed_radius;
    x1 = bbx1_ + inscribed_radius;
    y1 = bby1_ + inscribed_radius;
  }
  
  
  void Setup::
  getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - circumscribed_radius;
    y0 = bby0_ - circumscribed_radius;
    x1 = bbx1_ + circumscribed_radius;
    y1 = bby1_ + circumscribed_radius;
  }
  
  
  void Setup::
  getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ - inflation_radius;
    y0 = bby0_ - inflation_radius;
    x1 = bbx1_ + inflation_radius;
    y1 = bby1_ + inflation_radius;
  }
  
  
  boost::shared_ptr<mpglue::Costmap> Setup::
  getCostmap() const
  {
    if (costmapWrap_)
      return costmapWrap_;
    if (use_sfl_cost)
      costmapWrap_.reset(mpglue::createCostmap(getRawSFLTravmap().get()));
    else
      costmapWrap_.reset(mpglue::createCostmap(&getRaw2DCostmap()));
    return costmapWrap_;
  }
  
  
  boost::shared_ptr<mpglue::IndexTransform> Setup::
  getIndexTransform() const
  {
    if (indexTransform_)
      return indexTransform_;
    if (use_sfl_cost)
      indexTransform_.reset(mpglue::createIndexTransform(&getRawSFLTravmap()->GetGridFrame()));
    else
      indexTransform_.reset(mpglue::createIndexTransform(&getRaw2DCostmap()));
    return indexTransform_;
  }
  
  
  SetupOptions::
  SetupOptions()
    : spec("hc:office1"),
      resolution(0.05),
      inscribed_radius(0.325),	// from highlevel_controllers/test/launch_move_base.xml r7215
      circumscribed_radius(0.46), // dito
      inflation_radius(0.55),	// dito
      obstacle_cost(costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE),
      use_sfl_cost(false),
      door_width(1.2),
      hall_width(3),
      obstacle_gray(64),
      invert_gray(true)
  {
  }
  
  
  Setup * createSetup(SetupOptions const & opt,
		      std::ostream * progress_os)
    throw(std::runtime_error)
  {
    string spec(opt.spec);
    string format;
    if ( ! sfl::splitstring(spec, ':', format, spec))
      throw runtime_error("mpbench::createSetup(): could not extract format from \""
			  + opt.spec + "\"");
    if ("hc" == format)
      return OfficeBenchmark::create(spec,
				     opt.resolution,
				     opt.inscribed_radius,
				     opt.circumscribed_radius,
				     opt.inflation_radius,
				     opt.obstacle_cost,
				     opt.use_sfl_cost,
				     opt.door_width,
				     opt.hall_width,
				     progress_os);
    if ("pgm" == format) {
      string pgm_filename;
      string xml_filename;
      sfl::splitstring(spec, ':', pgm_filename, xml_filename);
      return createNetPGMBenchmark(pgm_filename,
				   xml_filename,
				   opt.obstacle_gray,
				   opt.invert_gray,
				   opt.resolution,
				   opt.inscribed_radius,
				   opt.circumscribed_radius,
				   opt.inflation_radius,
				   opt.obstacle_cost,
				   opt.use_sfl_cost,
				   progress_os);
    }
    if ("xml" == format)
      return createXMLBenchmark(spec,
				opt.resolution,
				opt.inscribed_radius,
				opt.circumscribed_radius,
				opt.inflation_radius,
				opt.obstacle_cost,
				opt.use_sfl_cost,
				progress_os);
    throw runtime_error("mpbench::createSetup(): invalid format \"" + format + "\"");
    return 0;
  }
  
}

namespace {
  
  mpbench::Setup * createNetPGMBenchmark(std::string const & pgmFileName,
					 std::string const & xmlFileName,
					 unsigned int obstacle_gray,
					 bool invert_gray,
					 double resolution,
					 double inscribed_radius,
					 double circumscribed_radius,
					 double inflation_radius,
					 int obstacle_cost,
					 bool use_sfl_cost,
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
    
    FILE * pgmfile(fopen(pgmFileName.c_str(), "rb"));
    if ( ! pgmfile) {
      ostringstream os;
      os << "createNetPGMBenchmark(): fopen(" << pgmFileName << "): " << strerror(errno);
      if (progress_os)
	*progress_os << "ERROR in " << os.str() << "\n";
      throw runtime_error(os.str());
    }
    bench = new mpbench::Setup(pgmFileName + ":" + xmlFileName,
			       resolution, inscribed_radius, circumscribed_radius,
			       inflation_radius, obstacle_cost, use_sfl_cost);
    if (progress_os)
      *progress_os << "createNetPGMBenchmark(): translating PGM file " << pgmFileName << "\n";
    readNetPGM(*bench, pgmfile, obstacle_gray, invert_gray, resolution);
    if ( ! xmlFileName.empty())
      readXML(xmlFileName, bench, progress_os);
    
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
  
  
  mpbench::Setup * createXMLBenchmark(std::string const & xmlFileName,
				      double resolution,
				      double inscribed_radius,
				      double circumscribed_radius,
				      double inflation_radius,
				      int obstacle_cost,
				      bool use_sfl_cost,
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
    
    bench = new mpbench::Setup(xmlFileName,
			       resolution, inscribed_radius, circumscribed_radius,
			       inflation_radius, obstacle_cost, use_sfl_cost);
    if (progress_os)
      *progress_os << "createXMLBenchmark(): parsing " << xmlFileName << "\n";
    readXML(xmlFileName, bench, progress_os);
    
#endif // MPBENCH_HAVE_EXPAT
    
    return bench;    
  }

}
