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
    : public AnytimeCostmapPlannerStats
  {
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
    
    virtual const char * const getClassName() const;
    
    virtual void dumpSubclassXML(std::ostream & os,
				 std::string const & prefix) const;
  };
  
  
  struct door_waypoint_s: public waypoint_s {
    door_waypoint_s(waypoint_s const & wpt, double _min_door_angle, double _cost)
      : waypoint_s(wpt), min_door_angle(_min_door_angle), cost(_cost) {}
    
    door_waypoint_s(double x, double y, double theta, double dr, double dtheta, double _min_door_angle, double _cost)
      : waypoint_s(x, y, theta, dr, dtheta), min_door_angle(_min_door_angle), cost(_cost) {}
    
    double min_door_angle;
    double cost;
    std::vector<int> valid_angle; // for dbg... might be removed at some point
    std::vector<int> valid_cost;
    std::vector<unsigned char> valid_interval;
  };
  
  
  class SBPLPlannerWrap
    : public AnytimeCostmapPlanner
  {
  public:
    SBPLPlannerWrap(boost::shared_ptr<SBPLEnvironment> environment,
		    boost::shared_ptr<SBPLPlanner> planner);
    
    // grr, shared_ptr cannot be used covariantly...
    virtual boost::shared_ptr<SBPLPlannerStats> copyMyStats() const
    {
      boost::shared_ptr<SBPLPlannerStats> stats(stats_.copy());
      return stats;
    }
    
  protected:
    virtual void preCreatePlan() throw(std::exception);
    virtual boost::shared_ptr<waypoint_plan_t> doCreatePlan() throw(std::exception);
    virtual void postCreatePlan() throw(std::exception);
    
    boost::shared_ptr<SBPLPlanner> planner_;
    boost::shared_ptr<SBPLEnvironment> environment_;
    SBPLPlannerStats stats_;
  };
  
}

#endif // MPGLUE_SBPL_PLANNER_HPP
