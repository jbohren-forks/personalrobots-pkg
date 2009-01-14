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
#include <mpglue/footprint.h>

class SBPLPlanner;		/**< see motion_planning/sbpl/src/planners/planner.h */

namespace mpglue {
  
  class SBPLEnvironment;
  
  struct SBPLPlannerStats
    : public AnytimeCostmapPlannerStats {
    SBPLPlannerStats();
    virtual SBPLPlannerStats * copy() const;
    
    int goal_state;                     /**< stateID of the goal (from costmap indices) */
    int start_state;                    /**< stateID of the start (from costmap indices) */
    int status;                         /**< return value of replan() (i.e. success == 1) */
    int number_of_expands;              /**< number of state expansions, or -1 if not available */
    int solution_cost;                  /**< cost of the solution, as given by replan() */
    double solution_epsilon;            /**< the "epsilon" value used to compute the solution */
    
    virtual void logStream(std::ostream & os,
			   std::string const & title,
			   std::string const & prefix) const;
  };
  
  
  class SBPLPlannerWrap
    : public AnytimeCostmapPlanner
  {
  public:
    SBPLPlannerWrap(boost::shared_ptr<SBPLEnvironment> environment,
		    boost::shared_ptr<SBPLPlanner> planner);
    
  protected:
    virtual void preCreatePlan() throw(std::exception);
    virtual boost::shared_ptr<waypoint_plan_t> doCreatePlan() throw(std::exception);
    virtual void postCreatePlan() throw(std::exception);
    
    boost::shared_ptr<SBPLPlanner> planner_;
    boost::shared_ptr<SBPLEnvironment> environment_;
    SBPLPlannerStats stats_;
  };
  
  
  SBPLPlannerWrap * createARAStar2D(boost::shared_ptr<Costmap> cm,
				    boost::shared_ptr<IndexTransform const> it,
				    bool forward_search,
				    int obst_cost_thresh);
  
  SBPLPlannerWrap * createADStar2D(boost::shared_ptr<Costmap> cm,
				   boost::shared_ptr<IndexTransform const> it,
				   bool forward_search,
				   int obst_cost_thresh);
  
  SBPLPlannerWrap * createARAStar3DKIN(boost::shared_ptr<Costmap> cm,
				       boost::shared_ptr<IndexTransform const> it,
				       bool forward_search,
				       int obst_cost_thresh,
				       footprint_t const & footprint,
				       double nominalvel_mpersecs,
				       double timetoturn45degsinplace_secs);
  
  SBPLPlannerWrap * createADStar3DKIN(boost::shared_ptr<Costmap> cm,
				      boost::shared_ptr<IndexTransform const> it,
				      bool forward_search,
				      int obst_cost_thresh,
				      footprint_t const & footprint,
				      double nominalvel_mpersecs,
				      double timetoturn45degsinplace_secs);
  
}

#endif // MPGLUE_SBPL_PLANNER_HPP
