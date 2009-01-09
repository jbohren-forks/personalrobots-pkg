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

/** \file sbpl_planner.h Wrap an SBPLPlanner into mpglue::IncrementalCostmapPlanner. */

#ifndef MPGLUE_SBPL_PLANNER_HPP
#define MPGLUE_SBPL_PLANNER_HPP

#include <mpglue/planner.h>

class SBPLPlanner;		/**< see motion_planning/sbpl/src/planners/planner.h */

namespace mpglue {
  
  struct SBPLPlannerStats
    : public IncrementalCostmapPlannerStats {
    SBPLPlannerStats(std::string const & planner_type,
		     std::string const & environment_type);
    virtual SBPLPlannerStats * copy() const;
    
    std::string const planner_type;     /**< name of the planner (an SBPLPlanner subclass) */
    std::string const environment_type; /**< name of the environment type (2D, 3DKIN, ...) */
    int goal_state;                     /**< stateID of the goal (from costmap indices) */
    int start_state;                    /**< stateID of the start (from costmap indices) */
    int status;                         /**< return value of replan() (i.e. success == 1, or -42 if replan() never got called) */
    int number_of_expands;              /**< number of state expansions, or -1 if not available */
    int solution_cost;                  /**< cost of the solution, as given by replan() */
    double solution_epsilon;            /**< the "epsilon" value used to compute the solution */
    
    virtual void logStream(std::ostream & os,
			   std::string const & title,
			   std::string const & prefix) const;
  };
  
  
  class SBPLPlannerWrap
    : public IncrementalCostmapPlanner
  {
  public:
    SBPLPlannerWrap(std::string const & planner_type,
		    std::string const & environment_type,
		    boost::shared_ptr<SBPLPlanner> planner,
		    boost::shared_ptr<Environment> environment);
    
  protected:
    virtual void preCreatePlan() throw(std::exception);
    virtual boost::shared_ptr<waypoint_plan_t> doCreatePlan() throw(std::exception);
    virtual void postCreatePlan() throw(std::exception);
    
    boost::shared_ptr<SBPLPlanner> planner_;
    boost::shared_ptr<Environment> environment_;
    SBPLPlannerStats stats_;
  };
  
}

#endif // MPGLUE_SBPL_PLANNER_HPP
