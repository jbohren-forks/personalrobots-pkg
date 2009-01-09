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

/** \file sbpl_util.hh Utilities for handling SBPL planners independent of the exact subtype. */

#ifndef MPGLUE_SBPL_UTIL_HPP
#define MPGLUE_SBPL_UTIL_HPP

#include <std_msgs/Pose2DFloat32.h>
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <string>
#include <vector>

class SBPLPlanner;		/**< see motion_planning/sbpl/src/planners/planner.h */
class DiscreteSpaceInformation; /**< see motion_planning/sbpl/src/discrete_space_information/environment.h */

// would like to forward-declare, but in mdpconfig.h it's a typedef'ed
// anonymous struct and GCC chokes on that... great
//   struct MDPConfig; /**< see motion_planning/sbpl/src/utils/mdpconfig.h */
#include <utils/mdpconfig.h>

// Would like to forward-declare, but nav2dcell_t is used within a
// std::vector<> ... see also the comments in
// sbpl/src/planners/planner.h about the ChangedCellsGetter
// class. Also, environment_nav2D.h needs some other includes to be
// present and uses std::vector without the std:: prefix, so we
// unfortunately have to add a using directive here.
using std::vector;
#include <utils/mdp.h>
#include <discrete_space_information/environment.h>
#include <discrete_space_information/nav2d/environment_nav2D.h>

namespace mpglue {
  
  
  /**
     Translate a name or alias into a string that createSBPLPlanner()
     understands.
  */
  std::string canonicalPlannerName(std::string const & name_or_alias);
  
  
  /**
     Create a planner subclass based on its name.
     
     \todo Use some sort of registry instead of hard-coded
     string-to-subclass mappings.
     
     \return A freshly allocated planner, or zero if there is no
     planner subclass associated with that name.
  */
  SBPLPlanner * createSBPLPlanner(/** Name of the planner class, e.g. "ARAPlanner". */
				  std::string const & name,
				  /** Required by all (so far) SBPLPlanners. */
				  DiscreteSpaceInformation* environment,
				  /** Required by some planners (ARAPlanner, ADPlanner). */
				  bool bforwardsearch,
				  /** Required by some planners (VIPlanner). */
				  MDPConfig* mdpCfg,
				  /** optional stream to which error messages get written */
				  std::ostream * opt_err_os);
  
#ifdef UNDEFINED
  struct SBPLPlannerStatsEntry {
    SBPLPlannerStatsEntry(std::string const & plannerType, std::string const & environmentType);
    
    std::string plannerType;         /**< name of the planner (an SBPLPlanner subclass) */
    std::string environmentType;     /**< name of the environment type (2D, 3DKIN, ...) */
    std_msgs::Pose2DFloat32 goal;    /**< from the std_msgs::Planner2DGoal we received (map frame) */
    unsigned int goalIx;             /**< x-index of the goal in the costmap */
    unsigned int goalIy;             /**< y-index of the goal in the costmap */
    int goalState;                   /**< stateID of the goal (from costmap indices) */
    std_msgs::Pose2DFloat32 start;   /**< global pose (map frame) "just before" before planning */
    unsigned int startIx;            /**< x-index of the start in the costmap */
    unsigned int startIy;            /**< y-index of the start in the costmap */
    int startState;                  /**< stateID of the start (from costmap indices) */
    bool stop_at_first_solution;     /**< whether to just plan until any plan is found */
    bool plan_from_scratch;          /**< whether to discard any previous solutions */
    double allocated_time_sec;       /**< the amount of time we had available for planning */
    double actual_time_wall_sec;     /**< the amount of time actually used for planning (wallclock) */
    double actual_time_user_sec;     /**< the amount of time actually used for planning (user time) */
    double actual_time_system_sec;   /**< the amount of time actually used for planning (system time) */
    
    int status;                      /**< return value of replan() (i.e. success == 1, or -42 if replan() never got called) */
    int number_of_expands;           /**< number of state expansions, or -1 if not available */
    int solution_cost;               /**< cost of the solution, as given by replan() */
    double solution_epsilon;         /**< the "epsilon" value used to compute the solution */
    double plan_length_m;            /**< cumulated Euclidean distance between planned waypoints */
    double plan_angle_change_rad;    /**< cumulated abs(delta(angle)) along planned waypoints */
    
    /** Use ROS_INFO() to log this entry to rosconsole.
	\todo needs to be unified with the other logXXX() methods
    */
    void logInfo(char const * prefix = "") const;
    
    /** Append this entry to a logfile (which is opened and closed each time). */
    void logFile(char const * filename, char const * title, char const * prefix) const;
    
    /** Append this entry to a stream. */
    void logStream(std::ostream & os, std::string const & title, std::string const & prefix) const;
  };
#endif
  
  class Environment;
  
  /** Wraps around SBPLPlanner subclasses, which you can select by
      name, providing almost the same interface (augmented slightly to
      handle statistics). We do subclass SBPLPlanner because we want
      to stay flexible here and do not need the data shared by all its
      subclasses. */
  class SBPLPlannerManager
  {
  public:
    struct no_planner_selected: public std::runtime_error { no_planner_selected(); };
    
    SBPLPlannerManager(/** Required by all (so far) SBPLPlanners. */
		       DiscreteSpaceInformation* environment,
		       /** Required by some planners (ARAPlanner, ADPlanner). */
		       bool bforwardsearch,
		       /** Required by some planners (VIPlanner). */
		       MDPConfig* mdpCfg);
    
    virtual ~SBPLPlannerManager();
    
    /** \return Name of the currently select()-ed planner, or the
	empty string if you never called select(). */
    std::string const & getName() const;
    
    /** Attempt to createSBPLPlanner() based on the name (which might
	come e.g. from a ROS parameter). If we already have a planner
	instance and the name is valid, behavior depends on the
	recycle parameter:
	- if recycle is false, the old instance gets destroyed.
	- if recycle is true AND the name is the same that was given
          at the previous select(), then we keep the old instance.
	  
	If the name is invalid, the old instance is not touched and
	the method returns false.
	
	\return true if createSBPLPlanner() succeeded OR the old instance was recycled.
    */
    bool select(std::string const & name, bool recycle,
		/** optional stream to which error messages get written */
		std::ostream * opt_err_os);
    
    /** Dispatch to the currently select()-ed planner's
	SBPLPlanner::replan(), measuring the time it actually takes to
	run it.
	
	\note You can use mpglue::convertPlan() to translate the
	solution_stateIDs_V into something more generic.
	
	\return The status returned by SBPLPlanner::replan().
    */
    int replan(/** in: whether to just return the first feasible solution */
	       bool stop_at_first_solution,
	       /** in: whether to forcefully re-initialize the planner */
	       bool plan_from_scratch,
	       /** in: how much time is allocated for planning */
	       double allocated_time_sec,
	       /** out: how much time was actually used by the planner (wallclock) */
	       double * actual_time_wall_sec,
	       /** out: how much time was actually used by the planner (user time) */
	       double * actual_time_user_sec,
	       /** out: how much time was actually used by the planner (system time) */
	       double * actual_time_system_sec,
	       /** out: number of state expansions (or -1 if not available) */
	       int * number_of_expands,
	       /** out: the cost of the planned path */
	       int * solution_cost,
	       /** out: the epsilon-value of the planned path */
	       double * solution_epsilon,
	       /** out: the planned path, as a succession of state IDs */
	       std::vector<int>* solution_stateIDs_V) throw(no_planner_selected);

    /** Dispatch to the currently select()-ed planner's
	SBPLPlanner::set_goal(). */
    int set_goal(int goal_stateID) throw(no_planner_selected);
    
    /** Dispatch to the currently select()-ed planner's
	SBPLPlanner::set_start(). */
    int set_start(int start_stateID) throw(no_planner_selected);
    
    /** Dispatch to the currently select()-ed planner's
	SBPLPlanner::force_planning_from_scratch(). */
    int force_planning_from_scratch() throw(no_planner_selected);
        
    /** Notify the planner that costs have changed.
	
	\todo Clarify the roles between SBPLPlannerManager,
	Environment, DiscreteSpaceInformation, and
	SBPLPlanner. Actually, the user should not deal with separate
	instances, but always combinations of planners and
	environments, so probably the best way to put this is in a
	facade.
    */
    void flush_cost_changes(Environment & ewrap) throw(no_planner_selected);
    
    /** \note Just for refactoring... */
    boost::shared_ptr<SBPLPlanner> get_planner() { return planner_; }

    /** \note Just for refactoring... */
    boost::shared_ptr<SBPLPlanner const> get_planner() const { return planner_; }
    
  protected:
    DiscreteSpaceInformation* environment_;
    bool bforwardsearch_;
    MDPConfig* mdpCfg_;
    boost::shared_ptr<SBPLPlanner> planner_;
    std::string name_;
  };
  
}

#endif // MPGLUE_SBPL_UTIL_HPP
