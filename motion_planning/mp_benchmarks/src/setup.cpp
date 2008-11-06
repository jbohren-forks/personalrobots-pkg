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
// #include <headers.h>
// #include <sys/time.h>
// #include <sys/resource.h>
// #include <time.h>
// #include <sstream>
#include <errno.h>
#include <cstring>
// #include <limits>


namespace {
  
  
  void drawLine(sfl::Mapper2d & m2d, double x0, double y0, double x1, double y1)
  {
    sfl::Mapper2d::buffered_obstacle_adder boa(&m2d, 0);
    m2d.gridframe.DrawGlobalLine(x0, y0, x1, y1,
				 0, std::numeric_limits<ssize_t>::max(),
				 0, std::numeric_limits<ssize_t>::max(),
				 boa);
  }
  
  
  void drawOfficeBenchmark1(sfl::Mapper2d & m2d,
			    ompl::SBPLBenchmarkSetup::tasklist_t & tasklist,
			    double hall,
			    double door)
  {
    // outer bounding box
    drawLine(m2d,        0,         0,  3 * hall,         0);
    drawLine(m2d,        0,         0,         0,  4 * hall);
    drawLine(m2d,        0,  4 * hall,  3 * hall,  4 * hall);
    drawLine(m2d, 3 * hall,         0,  3 * hall,  4 * hall);
    
    // two long walls along y-axis, each with a door near the northern end
    drawLine(m2d,      hall,  2 * hall,      hall,  4 * hall - 2 * door);
    drawLine(m2d,      hall,  2 * hall,      hall,  4 * hall -     door);
    drawLine(m2d,  2 * hall,      hall,  2 * hall,  4 * hall - 2 * door);
    drawLine(m2d,  2 * hall,      hall,  2 * hall,  4 * hall -     door);
    
    // some shorter walls along x-axis
    drawLine(m2d,           0,        hall,  0.5 * hall,        hall);
    drawLine(m2d,           0,  0.5 * hall,  0.5 * hall,  0.5 * hall);
    drawLine(m2d,  1.5 * hall,        hall,  2   * hall,        hall);
    
    // y-axis wall with two office doors
    drawLine(m2d,  0.5 * hall,                      0,  0.5 * hall,  0.5 * hall - 2 * door);
    drawLine(m2d,  0.5 * hall,  0.5 * hall -     door,  0.5 * hall,  0.5 * hall +     door);
    drawLine(m2d,  0.5 * hall,  0.5 * hall + 2 * door,  0.5 * hall,        hall           );

#warning 'construct the task list'
  }
  
  
  costmap_2d::CostMap2D * createCostMap2D(sfl::Mapper2d const & m2d)
  {
    boost::shared_ptr<sfl::RDTravmap> rdt(m2d.CreateRDTravmap());
    unsigned int const width(rdt->GetXEnd());
    unsigned int const height(rdt->GetYEnd());
    std::vector<unsigned char> data;
    data.reserve(width * height);
    for (unsigned int iy(0); iy < height; ++iy)
      for (unsigned int ix(0); ix < width; ++ix) {
	int value;
	if (rdt->GetValue(ix, iy, value)) // "always" succeeds though
	  data.push_back(value & 0xff);
	else
	  data.push_back(0);	// suppose 0 means freespace
      }
    
    // whatever, we won't update dynamic obstacles anyway
    double const window_length(0);
    
    // hm... what if our obstacle cost needs more than 8 bits?    
    unsigned char const threshold(m2d.obstacle & 0xff);
    
    return new costmap_2d::CostMap2D(width, height, data, m2d.gridframe.Delta(), window_length, threshold);
  }
  
}


namespace ompl {

    
  SBPLBenchmarkSetup::
  ~SBPLBenchmarkSetup()
  {
  }
    
    
  SBPLBenchmarkSetup::tasklist_t const & SBPLBenchmarkSetup::
  getTasks() const
  {
    return tasklist_;
  }
    
    
  OfficeBenchmark1::
  OfficeBenchmark1(double resolution,
		   double robot_radius,
		   double freespace_distance,
		   int obstacle_cost)
  {
    sfl::GridFrame gframe(resolution);
    boost::shared_ptr<sfl::Mapper2d::travmap_grow_strategy>
      growstrategy(new sfl::Mapper2d::always_grow());
    double const buffer_zone(sfl::maxval(0.0, freespace_distance - robot_radius));
    sfl::Mapper2d m2d(gframe, 0, 0, 0, 0, robot_radius, buffer_zone, 0, obstacle_cost,
		      "OfficeBenchmark1", sfl::RWlock::Create("OfficeBenchmark1"), growstrategy);
    
    drawOfficeBenchmark1(m2d, tasklist_, 4.5, 1.2);
    costmap_ = createCostMap2D(m2d);
  }
  
  
  OfficeBenchmark1::
  ~OfficeBenchmark1()
  {
    delete costmap_;
  }
  
  
  costmap_2d::CostMap2D const & OfficeBenchmark1::
  getCostmap() const
  {
    return *costmap_;
  }
  
  
  void OfficeBenchmark1::
  dumpDescription(char const * filename, char const * title, char const * prefix) const
  {
    FILE * ff(fopen(filename, "a"));
    if (0 == ff) {
      ROS_WARN("OfficeBenchmark1::dumpDescription(): fopen(%s): %s",
	       filename, strerror(errno));
      return;
    }
    fprintf(ff,
	    "%s\n"
	    "%sname: OfficeBenchmark1\n",
	    title, prefix);
    if (0 != fclose(ff))
      ROS_WARN("OfficeBenchmark1::dumpDescription(): fclose() on %s: %s",
	       filename, strerror(errno));
  }
  
}
