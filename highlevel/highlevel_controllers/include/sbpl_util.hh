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

#include <std_msgs/Pose2DFloat32.h>
#include <stdexcept>
#include <string>
#include <vector>

class SBPLPlanner;		/**< see motion_planning/sbpl/src/planners/planner.h */
class DiscreteSpaceInformation; /**< see motion_planning/sbpl/src/discrete_space_information/environment.h */

// would like to forward-declare, but in mdpconfig.h it's a typedef'ed
// anonymous struct and GCC chokes on that... great
//   struct MDPConfig; /**< see motion_planning/sbpl/src/utils/mdpconfig.h */
#include <utils/mdpconfig.h>

namespace ros {
namespace highlevel_controllers {
  
  
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
				  MDPConfig* mdpCfg);
  
  
  /**
     Collection of statistic for SBPL planner runs.
  */
  class SBPLPlannerStatistics
  {
  public:
    /** One element of the statistics. Represents one SBPLPlanner::replan() cycle. */
    struct entry {
      explicit entry(std::string const & plannerName);
      
      std::string plannerName;         /**< name of the planner (an SBPLPlanner subclass) */
      std_msgs::Pose2DFloat32 goal;    /**< from the std_msgs::Planner2DGoal we received (map frame) */
      unsigned int goalIx;             /**< x-index of the goal in the costmap */
      unsigned int goalIy;             /**< y-index of the goal in the costmap */
      int goalState;                   /**< stateID of the goal (from costmap indices) */
      std_msgs::Pose2DFloat32 start;   /**< global pose (map frame) "just before" before planning */
      unsigned int startIx;            /**< x-index of the start in the costmap */
      unsigned int startIy;            /**< y-index of the start in the costmap */
      int startState;                  /**< stateID of the start (from costmap indices) */
      double allocated_time_sec;       /**< the amount of time we had available for planning */
      double actual_time_sec;          /**< the amount of time actually used for planning (wallclock) */
      //      bool from_scratch;               /**< whether force_planning_from_scratch() was called */
      int status;                      /**< return value of replan() (i.e. success == 1, or -42 if replan() never got called) */
      double plan_length_m;            /**< cumulated Euclidean distance between planned waypoints */
      double plan_angle_change_rad;    /**< cumulated abs(delta(angle)) along planned waypoints */
      
      /** Use ROS_INFO() to log this entry to rosconsole. */
      void logInfo(char const * prefix = "") const;
      
      /** Append this entry to a logfile (which is opened and closed each time). */
      void logFile(char const * filename, char const * prefix = "") const;
    };
    
    typedef std::vector<entry> stats_t;
    
    /** Create a fresh entry and append it to the end of the
	accumulated statistics. */
    void pushBack(std::string const & plannerName);
    
    /** \return The topmost (latest) element, which is only defined if
	you called pushBack() at least once. */
    entry & top();
    
    /** Read-only access to the entire history. */
    stats_t const & getAll() const;
    
  protected:
    stats_t stats_;
  };
  
  
  /** Wraps around SBPLPlanner subclasses, which you can select by
      name, providing almost the same interface (augmented slightly to
      handle statistics). We do subclass SBPLPlanner because we want
      to stay flexible here and do not need the data shared by all its
      subclasses. */
  class SBPLPlannerManager
  {
  public:
    struct no_planner_selected: public std::runtime_error
    { no_planner_selected(): std::runtime_error("SBPLPlannerManager: no planner selected") {} };
    
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
    bool select(std::string const & name, bool recycle);
    
    /** Dispatch to the currently select()-ed planner's
	SBPLPlanner::replan(), measuring the time it actually takes to
	run it. */
    int replan(double allocated_time_sec, double * actual_time_sec,
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
    
  protected:
    DiscreteSpaceInformation* environment_;
    bool bforwardsearch_;
    MDPConfig* mdpCfg_;
    SBPLPlanner * planner_;
    std::string name_;
  };
  
}
}
