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

#include <mpglue/setup.h>

// #include <mpglue/costmapper.h>
// #include <mpglue/plan.h>
#include <mpglue/planner.h>
#include <mpglue/footprint.h>
// #include <sfl/gplan/GridFrame.hpp>
// #include <boost/shared_ptr.hpp>
// #include <string>
// #include <vector>
// #include <set>

#include <stdexcept>

namespace mpbench {
  
  class World;
  
  namespace episode {
    
    struct taskspec {
      taskspec(std::string const & description, mpglue::goalspec const & goal);
      taskspec(std::string const & description, mpglue::goalspec const & goal, mpglue::doorspec const & door);
      taskspec(taskspec const & orig);
      
      std::string description;
      mpglue::goalspec goal;
      std::vector<mpglue::startspec> start; // per episode
      boost::shared_ptr<mpglue::doorspec> door; /**< only used by door planner (for now, anyway) */
    };
    
  }
  
  
  struct SetupOptions
    : public mpglue::requestspec
  {    
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
    
    mpglue::tokenlist_t world_tok;
  };
  
  typedef std::vector<boost::shared_ptr<episode::taskspec> > tasklist_t;
  
  
  class Setup
  {
  public:
    Setup(SetupOptions const & options, std::ostream * verbose_os, std::ostream * debug_os);
    
    virtual ~Setup();
    
    static boost::shared_ptr<Setup> create(std::string const & world_spec,
					   std::string const & planner_spec,
					   std::string const & robot_spec,
					   std::string const & costmap_spec,
					   std::ostream * verbose_os,
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
    
    void legacyDrawLine(double x0, double y0, double x1, double y1);
    
    void legacyDrawPoint(double xx, double yy);
    
    /**
       Add a (copy of a) task to the task list.
    */
    void addTask(episode::taskspec const & setup);
    
    void legacyAddTask(std::string const & description,
		       bool from_scratch,
		       double start_x, double start_y, double start_th, 
		       double goal_x, double goal_y, double goal_th, 
		       double goal_tol_xy, double goal_tol_th);
    
    boost::shared_ptr<mpglue::CostmapPlanner> getPlanner(size_t task_id) throw(std::exception);
    boost::shared_ptr<World> getWorld()             { return world_; }
    boost::shared_ptr<World const> getWorld() const { return world_; }
    
    tasklist_t const & getTasks() const;
    
    mpglue::footprint_t const & getFootprint() const;
    SetupOptions const & getOptions() const { return opt_; }
    
  protected:
    SetupOptions const opt_;
    std::ostream * verbose_os_;
    std::ostream * debug_os_;
    tasklist_t tasklist_;
    boost::shared_ptr<World> world_;
    
  private:
    mutable boost::shared_ptr<mpglue::footprint_t> footprint_;
    std::vector<boost::shared_ptr<mpglue::CostmapPlanner> > planner_; // one per task
  };
  
}

#endif // MPBENCH_BENCHMARK_SETUP_HPP
