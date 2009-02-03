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

#ifndef MPBENCH_BENCHMARK_SETUP_HPP
#define MPBENCH_BENCHMARK_SETUP_HPP

#include <mpglue/costmapper.h>
#include <mpglue/plan.h>
#include <mpglue/planner.h>
#include <mpglue/footprint.h>
#include <sfl/gplan/GridFrame.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <set>

namespace mpbench {
  
  namespace task {
    
    struct startspec {
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
    
    struct setup {
      setup(std::string const & description, goalspec const & goal);
      setup(setup const & orig);
      
      std::string description;
      goalspec goal;
      std::vector<startspec> start;
    };
    
    struct result {
      result(size_t task_id,
	     size_t episode_id,
	     startspec const & start,
	     goalspec const & goal,
	     boost::shared_ptr<mpglue::waypoint_plan_t> plan,
	     boost::shared_ptr<mpglue::CostmapPlannerStats> stats);
      
      size_t task_id;
      size_t episode_id;
      startspec start;
      goalspec goal;
      boost::shared_ptr<mpglue::waypoint_plan_t> plan;
      boost::shared_ptr<mpglue::CostmapPlannerStats> stats;
    };
    
  }
  
  typedef std::vector<std::string> tokenlist_t;
  
  struct SetupOptions {    
    SetupOptions(std::string const & world_spec,
		 std::string const & planner_spec,
		 std::string const & robot_spec,
		 std::string const & costmap_spec);
    
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
    
    std::string const world_spec;
    std::string const planner_spec;
    std::string const robot_spec;
    std::string const costmap_spec;
    
    tokenlist_t world_tok;
    tokenlist_t planner_tok;
    tokenlist_t robot_tok;
    tokenlist_t costmap_tok;
    
    std::string robot_name;
    double robot_inscribed_radius;
    double robot_circumscribed_radius;
    double robot_nominal_forward_speed;
    double robot_nominal_rotation_speed;
    
    std::string costmap_name;
    double costmap_resolution;	/**< cell size [m] (square cells) */
    double costmap_inscribed_radius; /**< radius [m] of CSpace "lethal" inflation */
    double costmap_circumscribed_radius; /**< radius [m] of "non-lethal" inflation */
    double costmap_inflation_radius; /**< distance [m] of freespace cells from obstacles */
    int costmap_obstacle_cost;
    
    unsigned int pgm_obstacle_gray;
    bool pgm_invert_gray;
  };
  
  typedef std::set<mpglue::index_pair> indexlist_t;
  typedef std::vector<boost::shared_ptr<task::setup> > tasklist_t;
  typedef std::vector<boost::shared_ptr<task::result> > resultlist_t;
  
  
  class Setup
  {
  public:
    explicit Setup(SetupOptions const & options);
    
    virtual ~Setup();
    
    static boost::shared_ptr<Setup> create(std::string const & world_spec,
					   std::string const & planner_spec,
					   std::string const & robot_spec,
					   std::string const & costmap_spec,
					   std::ostream * progress_os,
					   std::ostream * debug_os)
      throw(std::runtime_error);
    
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
       \todo re-implement
    */
    void dumpTravmap(std::ostream & os) const;
    
    /**
       Draw an obstacle line into the costmap, expanded by the robot
       radius and with costs descending out up to the freespace
       distance.
    */
    void drawLine(double x0, double y0, double x1, double y1,
		  /** optional: verbose operation if non-null */
		  std::ostream * progress_os, std::ostream * debug_os);
    
    void drawPoint(double xx, double yy,
		   std::ostream * progress_os);
    
    /**
       Add a (copy of a) task to the task list.
    */
    void addTask(task::setup const & setup);
    
    void legacyAddTask(std::string const & description,
		       bool from_scratch,
		       double start_x, double start_y, double start_th, 
		       double goal_x, double goal_y, double goal_th, 
		       double goal_tol_xy, double goal_tol_th);
    
    boost::shared_ptr<mpglue::CostmapPlanner> getPlanner();
    boost::shared_ptr<mpglue::CostmapAccessor const> getCostmap() const;
    boost::shared_ptr<mpglue::IndexTransform const> getIndexTransform() const;
    
    tasklist_t const & getTasks() const;
    void getWorkspaceBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getInscribedBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getCircumscribedBounds(double & x0, double & y0, double & x1, double & y1) const;
    void getInflatedBounds(double & x0, double & y0, double & x1, double & y1) const;
    
    mpglue::footprint_t const & getFootprint() const;
    SetupOptions const & getOptions() const { return opt_; }
    
  protected:
    SetupOptions opt_;
    sfl::GridFrame gridframe_;
    indexlist_t wspace_obstacles_;
    tasklist_t tasklist_;
    double bbx0_, bby0_, bbx1_, bby1_; // workspace bounding box
    
  private:
    mutable boost::shared_ptr<mpglue::footprint_t> footprint_;
    boost::shared_ptr<mpglue::CostmapPlanner> planner_;
    boost::shared_ptr<mpglue::Costmapper> costmapper_;
  };
  
}

#endif // MPBENCH_BENCHMARK_SETUP_HPP
