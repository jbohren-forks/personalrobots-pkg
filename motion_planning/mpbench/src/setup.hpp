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

/** \file setup.hpp */

#ifndef MPBENCH_BENCHMARK_SETUP_HPP
#define MPBENCH_BENCHMARK_SETUP_HPP

#include <mpglue/costmap.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace sfl {
  class Mapper2d;
  class RDTravmap;
}

namespace costmap_2d {
  class CostMap2D;
}

namespace mpbench {
  
  
  class SBPLBenchmarkSetup
  {
  public:
    struct task {
      task(std::string const & description,
	   bool from_scratch,
	   double start_x, double start_y, double start_th, 
	   double goal_x, double goal_y, double goal_th, 
	   double goal_tol_xy, double goal_tol_th);
      
      std::string description;
      bool from_scratch;
      double start_x, start_y, start_th;
      double goal_x, goal_y, goal_th;
      double goal_tol_xy, goal_tol_th;
    };
    
    typedef std::vector<task> tasklist_t;
    
    
    SBPLBenchmarkSetup(/** name of the setup */
		       std::string const & name,
		       /** cell size [m] (square cells) */
		       double resolution,
		       /** inscribed radius of the robot [m] */
		       double inscribed_radius,
		       /** circumscribed radius of the robot [m] */
		       double circumscribed_radius,
		       /** distance from obstacles where cells become
			   freespace [m] */
		       double inflation_radius,
		       /** the cost value at or above which a cell is
			   considered an obstacle */
		       int obstacle_cost,
		       bool use_sfl_costs);
    
    virtual ~SBPLBenchmarkSetup();
    
    /**
       Print a human-readable description of the setup to a
       std::ostream.
    */
    void dumpDescription(/** The stream to write the description to. */
			 std::ostream & os,
			 /** The title will be printed as-is, followed
			     by a newline. If the title is an empty
			     string, nothing is printed. */
			 std::string const & title,
			 /** The prefix is prepended to each line of
			     the description, except for the title. */
			 std::string const & prefix) const;
    
    /** Default implementation does nothing. */
    virtual void dumpSubDescription(/** The stream to write the
					description to. */
				    std::ostream & os,
				    /** The prefix is prepended to
					each line of the description,
					except for the title. */
				    std::string const & prefix) const;
    
    /**
       Call sfl::TraversabilityMap::DumpMap() on the instance stored
       in the underlying sfl::Mapper2d.
    */
    void dumpTravmap(std::ostream & os) const;
    
    /**
       Draw an obstacle line into the costmap, expanded by the robot
       radius and with costs descending out up to the freespace
       distance. This uses the underlying sfl::Mapper2d instance to
       automatically grow it sfl::TraversabilityMap.
    */
    void drawLine(double x0, double y0, double x1, double y1,
		  /** optional: verbose operation if non-null */
		  std::ostream * progress_os);
    
    void drawPoint(double xx, double yy,
		   std::ostream * progress_os);
    
    /**
       Add a task to the task list.
    */
    void addTask(std::string const & description,
		 bool from_scratch,
		 double start_x, double start_y, double start_th, 
		 double goal_x, double goal_y, double goal_th, 
		 double goal_tol_xy, double goal_tol_th);
    
    boost::shared_ptr<sfl::RDTravmap> getRawSFLTravmap() const;
    costmap_2d::CostMap2D const & getRaw2DCostmap() const;
    
    boost::shared_ptr<mpglue::Costmap> getCostmap() const;
    boost::shared_ptr<mpglue::IndexTransform> getIndexTransform() const;
    
    tasklist_t const & getTasks() const;
    void getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const;
    
    std::string const name;
    double const resolution;
    double const inscribed_radius;
    double const circumscribed_radius;
    double const inflation_radius;
    int const obstacle_cost;
    bool const use_sfl_cost;
    
  protected:
    boost::shared_ptr<sfl::Mapper2d> m2d_;
    tasklist_t tasklist_;
    double bbx0_, bby0_, bbx1_, bby1_; // workspace bounding box
    
  private:
    mutable boost::shared_ptr<costmap_2d::CostMap2D> costmap_; // lazy init
    mutable boost::shared_ptr<sfl::RDTravmap> rdtravmap_; // lazy init
    mutable boost::shared_ptr<mpglue::Costmap> costmapWrap_; // lazy init
    mutable boost::shared_ptr<mpglue::IndexTransform> indexTransform_; // lazy init
    
    costmap_2d::CostMap2D * createCostMap2D() const;
  };
  
  
  struct SBPLBenchmarkOptions {
    // fills in some "sensible" default values
    SBPLBenchmarkOptions();
    
    std::string name;
    double resolution;
    double inscribed_radius;
    double circumscribed_radius;
    double inflation_radius;
    int obstacle_cost;
    bool use_sfl_cost;
    double door_width;
    double hall_width;
    std::string pgm_filename;
    unsigned int obstacle_gray;
    bool invert_gray;
  };
  
  
  SBPLBenchmarkSetup * createBenchmark(SBPLBenchmarkOptions const & opt,
				       std::ostream * progress_os,
				       std::ostream * travmap_os);
  
}

#endif // MPBENCH_BENCHMARK_SETUP_HPP
