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
#include <costmap_2d/costmap_2d.h>
#include <sfl/gplan/GridFrame.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/numeric.hpp>
#include <errno.h>
#include <cstring>

using sfl::minval;
using sfl::maxval;
using namespace std;

namespace {
  
  
  void drawDots(ompl::SBPLBenchmarkSetup & setup,
		double hall, double door,
		std::ostream * progress_os)
  {
    setup.drawPoint(       0,         0, progress_os);
    setup.drawPoint(       0,  hall, progress_os);
    setup.drawPoint(hall,         0, progress_os);
    setup.drawPoint(hall,  hall, progress_os);

    double const tol_xy(0.5 * door);
    double const tol_th(M_PI);
    setup.addTask("left to right", true,
		     0, 0.5 * hall, 0,
		  hall, 0.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("right to left", false,
		  hall, 0.5 * hall, 0,
		     0, 0.5 * hall, 0, tol_xy, tol_th);
  }
  
  
  void drawSquare(ompl::SBPLBenchmarkSetup & setup,
		  double hall, double door,
		  std::ostream * progress_os)
  {
    setup.drawLine(   0,     0,  hall,     0, progress_os);
    setup.drawLine(   0,     0,     0,  hall, progress_os);
    setup.drawLine(   0,  hall,  hall,  hall, progress_os);
    setup.drawLine(hall,     0,  hall,  hall, progress_os);

    double const tol_xy(0.5 * door);
    double const tol_th(M_PI);
    setup.addTask("left to right", true,
		         door, 0.5 * hall, 0,
		  hall - door, 0.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("right to left", false,
		  hall - door, 0.5 * hall, 0,
		         door, 0.5 * hall, 0, tol_xy, tol_th);
  }
  
  
  void drawOffice1(ompl::SBPLBenchmarkSetup & setup,
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
    setup.addTask("through door or around hall", true,
		  0.5 * hall, 4.5 * hall, 0,
		  1.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("through door or around hall, return trip", false,
		  1.5 * hall, 4.5 * hall, 0,
		  0.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("through door or around long hall", true,
		  1.5 * hall, 4.5 * hall, 0,
		  2.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("through door or around long hall, return trip", false,
		  2.5 * hall, 4.5 * hall, 0,
		  1.5 * hall, 4.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("office to office", true,
		  0.25 * hall, 0.5 * hall, 0,
		  0.25 * hall, 1.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("office to office, return trip", false,
		  0.25 * hall, 1.5 * hall, 0,
		  0.25 * hall, 0.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("office to hall", true,
		  0.25 * hall, 0.5 * hall, 0,
		         hall,       hall, 0, tol_xy, tol_th);
    setup.addTask("office to hall, return trip", false,
		         hall,        hall, 0,
		  0.25 * hall, 0.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("hall to office", true,
		         hall,       hall, 0,
		  0.25 * hall, 1.5 * hall, 0, tol_xy, tol_th);
    setup.addTask("hall to office, return trip", false,
		  0.25 * hall, 1.5 * hall, 0,
		         hall,       hall, 0, tol_xy, tol_th);
  }
  
}


namespace ompl {
  
  
  SBPLBenchmarkSetup::
  SBPLBenchmarkSetup(std::string const & name,
		     double _resolution,
		     double _inscribed_radius,
		     double _circumscribed_radius,
		     double _inflation_radius,
		     int _obstacle_cost,
		     bool _use_sfl_cost)
    : resolution(_resolution),
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
    double const buffer_zone(sfl::maxval(0.0, inflation_radius - inscribed_radius));
    double const padding_factor(0);
    m2d_.reset(new sfl::Mapper2d(gframe, 0, 0, 0, 0,
				 inscribed_radius, buffer_zone, padding_factor,
				 0, obstacle_cost,
				 name, sfl::RWlock::Create(name), growstrategy));
    

  }
  
  
  SBPLBenchmarkSetup::
  ~SBPLBenchmarkSetup()
  {
  }


  void SBPLBenchmarkSetup::
  dumpTravmap(std::ostream & os) const
  {
    getRDTravmap()->DumpMap(&os);
  }
  
  
  void SBPLBenchmarkSetup::
  drawLine(double x0, double y0, double x1, double y1,
	   std::ostream * progress_os)
  {
    if (progress_os)
      *progress_os << "SBPLBenchmarkSetup::drawLine(" << x0 << "  " << y0 << "  "
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
  
  
  void SBPLBenchmarkSetup::
  drawPoint(double xx, double yy,
	    std::ostream * progress_os)
  {
    if (progress_os)
      *progress_os << "SBPLBenchmarkSetup::drawPoint(" << xx << "  " << yy << ")\n" << flush;
    m2d_->AddBufferedObstacle(xx, yy, 0);
    bbx0_ = minval(bbx0_, xx);
    bby0_ = minval(bby0_, yy);
    bbx1_ = maxval(bbx1_, xx);
    bby1_ = maxval(bby1_, yy);
  }
  
  
  void SBPLBenchmarkSetup::
  addTask(std::string const & description,
	  bool from_scratch,
	  double start_x, double start_y, double start_th, 
	  double goal_x, double goal_y, double goal_th, 
	  double goal_tol_xy, double goal_tol_th)
  {
    tasklist_.push_back(task(description, from_scratch, start_x, start_y, start_th,
			     goal_x, goal_y, goal_th, goal_tol_xy, goal_tol_th));
  }
  
  
  boost::shared_ptr<sfl::RDTravmap> SBPLBenchmarkSetup::
  getRDTravmap() const
  {
    if ( ! rdtravmap_)
      rdtravmap_ = m2d_->CreateRDTravmap();
    return rdtravmap_;
  }
  
  
  costmap_2d::CostMap2D const & SBPLBenchmarkSetup::
  getCostmap() const
  {
    if ( ! costmap_)
      costmap_.reset(createCostMap2D());
    return *costmap_;
  }
  
  
  SBPLBenchmarkSetup::tasklist_t const & SBPLBenchmarkSetup::
  getTasks() const
  {
    return tasklist_;
  }
  
  
  costmap_2d::CostMap2D * SBPLBenchmarkSetup::
  createCostMap2D() const
  {
    boost::shared_ptr<sfl::RDTravmap> rdt(m2d_->CreateRDTravmap());
    unsigned int const width(rdt->GetXEnd());
    unsigned int const height(rdt->GetYEnd());
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
    
    // whatever, we won't update dynamic obstacles anyway (???)
    double const window_length(0);
    
    // hm... what if our obstacle cost needs more than 8 bits?    
    unsigned char const threshold(obstacle_cost & 0xff);
    double const maxZ(1);
    double const freeSpaceProjectionHeight(1);
    costmap_2d::CostMap2D * cm;
    if (use_sfl_cost)
      cm = new costmap_2d::CostMap2D(width, height, data, resolution, window_length,
				     threshold, maxZ, freeSpaceProjectionHeight,
				     0, 0, 0);
    else
      cm = new costmap_2d::CostMap2D(width, height, data, resolution, window_length,
				     threshold, maxZ, freeSpaceProjectionHeight,
				     inflation_radius, circumscribed_radius, inscribed_radius);
    return cm;
  }
    
    
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
    : SBPLBenchmarkSetup(name, resolution,
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
	 std::ostream * progress_os,
	 std::ostream * travmap_os)
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
    else {
      delete setup;
      if (progress_os)
	*progress_os << "OfficeBenchmark::create(): invalid name \"" << name << "\", use one of:\n"
		     << "  dots: 4 dots in a square of hall_width side length\n"
		     << "  square: a square of hall_width side length\n"
		     << "  office1: two offices, two hallways, some doors\n" << flush;
      return 0;
    }
    if (travmap_os) {
      if (progress_os)
	*progress_os << "saving sfl::TraversabilityMap\n";
      setup->dumpTravmap(*travmap_os);
    }
    return setup;
  }
  
  
  void OfficeBenchmark::
  dumpDescription(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "name:                 OfficeBenchmark1\n"
       << prefix << "resolution:           " << resolution << "\n"
       << prefix << "inscribed_radius:     " << inscribed_radius << "\n"
       << prefix << "circumscribed_radius: " << circumscribed_radius << "\n"
       << prefix << "inflation_radius:     " << inflation_radius << "\n"
       << prefix << "obstacle_cost:        " << obstacle_cost << "\n"
       << prefix << "use_sfl_cost:         " << (use_sfl_cost ? "true\n" : "false\n")
       << prefix << "door_width:           " << door_width << "\n"
       << prefix << "hall_width:           " << hall_width << "\n"
       << prefix << "tasks:\n";
    for (size_t ii(0); ii < tasklist_.size(); ++ii)
      os << prefix << "  [" << ii << "] " << tasklist_[ii].description << "\n";
  }
  


  SBPLBenchmarkSetup::task::
  task(std::string const & _description,
       bool _from_scratch,
       double _start_x, double _start_y, double _start_th, 
       double _goal_x, double _goal_y, double _goal_th, 
       double _goal_tol_xy, double _goal_tol_th)
    : description(_description),
      from_scratch(_from_scratch),
      start_x(_start_x),
      start_y(_start_y),
      start_th(_start_th),
      goal_x(_goal_x),
      goal_y(_goal_y),
      goal_th(_goal_th),
      goal_tol_xy(_goal_tol_xy),
      goal_tol_th(_goal_tol_th)
  {
  }
  
  
  void SBPLBenchmarkSetup::
  getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ + resolution;
    y0 = bby0_ + resolution;
    x1 = bbx1_ - resolution;
    y1 = bby1_ - resolution;
  }
  
  
  void SBPLBenchmarkSetup::
  getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ + inscribed_radius;
    y0 = bby0_ + inscribed_radius;
    x1 = bbx1_ - inscribed_radius;
    y1 = bby1_ - inscribed_radius;
  }
  
  
  void SBPLBenchmarkSetup::
  getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ + circumscribed_radius;
    y0 = bby0_ + circumscribed_radius;
    x1 = bbx1_ - circumscribed_radius;
    y1 = bby1_ - circumscribed_radius;
  }
  
  
  void SBPLBenchmarkSetup::
  getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const
  {
    x0 = bbx0_ + inflation_radius;
    y0 = bby0_ + inflation_radius;
    x1 = bbx1_ - inflation_radius;
    y1 = bby1_ - inflation_radius;
  }
  
}
