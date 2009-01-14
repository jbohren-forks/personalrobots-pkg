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

#include "sbpl_planner.h"
#include "environment.h"
#include <sbpl/headers.h>
#include <sstream>

using namespace boost;
using namespace std;

namespace mpglue {
  
  
  SBPLPlannerStats::
  SBPLPlannerStats(std::string const & _planner_type,
		   std::string const & _environment_type)
    : IncrementalCostmapPlannerStats(),
      planner_type(_planner_type),
      environment_type(_environment_type),
      goal_state(-1),
      start_state(-1),
      status(0),
      number_of_expands(0),
      solution_cost(-1),
      solution_epsilon(0)
  {
  }
  
  
  SBPLPlannerStats * SBPLPlannerStats::
  copy() const
  {
    SBPLPlannerStats * foo(new SBPLPlannerStats(*this));
    return foo;
  }
  
  
  void SBPLPlannerStats::
  logStream(std::ostream & os,
	    std::string const & title,
	    std::string const & prefix) const
  {
    IncrementalCostmapPlannerStats::logStream(os, title, prefix);
    os << prefix << "planner_type:              " << planner_type << "\n"
       << prefix << "environment_type:          " << environment_type << "\n"
       << prefix << "goal_state:                " << goal_state << "\n"
       << prefix << "start_state:               " << start_state << "\n"
       << prefix << "status:                    " << status << "\n"
       << prefix << "number_of_expands:         " << number_of_expands << "\n"
       << prefix << "solution_cost:             " << solution_cost << "\n"
       << prefix << "solution_epsilon:          " << solution_epsilon << "\n";
  }
  
  
  SBPLPlannerWrap::
  SBPLPlannerWrap(std::string const & planner_type,
		  std::string const & environment_type,
		  boost::shared_ptr<SBPLPlanner> planner,
		  boost::shared_ptr<Environment> environment)
    : IncrementalCostmapPlanner(stats_,
				environment->getCostmap(),
				environment->getIndexTransform()),
      planner_(planner),
      environment_(environment),
      stats_(planner_type, environment_type)
  {
  }
  
  
  void SBPLPlannerWrap::
  preCreatePlan() throw(std::exception)
  {
    if (start_changed_) {
      stats_.start_state = environment_->SetStart(stats_.start_x, stats_.start_y, stats_.start_th);
      if (0 > stats_.start_state) {
	ostringstream os;
	os << "mpglue::SBPLPlannerWrap::preCreatePlan(): invalid start\n"
	   << "  start pose: " << stats_.start_x << " " << stats_.start_y << " "
	   << stats_.start_th << "\n"
	   << "  start index: " << stats_.start_ix << " " << stats_.start_iy << "\n"
	   << "  start state: " << stats_.start_state << "\n";
	throw out_of_range(os.str());
      }
      if (1 != planner_->set_start(stats_.start_state)) {
	ostringstream os;
	os << "mpglue::SBPLPlannerWrap::preCreatePlan(): SBPLPlanner::set_start() failed\n"
	   << "  start pose: " << stats_.start_x << " " << stats_.start_y << " "
	   << stats_.start_th << "\n"
	   << "  start index: " << stats_.start_ix << " " << stats_.start_iy << "\n"
	   << "  start state: " << stats_.start_state << "\n";
	throw runtime_error(os.str());
      }
    }
    
    if (goal_pose_changed_) {
      stats_.goal_state = environment_->SetGoal(stats_.goal_x, stats_.goal_y, stats_.goal_th);
      if (0 > stats_.goal_state) {
	ostringstream os;
	os << "mpglue::SBPLPlannerWrap::preCreatePlan(): invalid goal\n"
	   << "  goal pose: " << stats_.goal_x << " " << stats_.goal_y << " "
	   << stats_.goal_th << "\n"
	   << "  goal index: " << stats_.goal_ix << " " << stats_.goal_iy << "\n"
	   << "  goal state: " << stats_.goal_state << "\n";
	throw out_of_range(os.str());
      }
      if (1 != planner_->set_goal(stats_.goal_state)) {
	ostringstream os;
	os << "mpglue::SBPLPlannerWrap::preCreatePlan(): SBPLPlanner::set_goal() failed\n"
	   << "  goal pose: " << stats_.goal_x << " " << stats_.goal_y << " "
	   << stats_.goal_th << "\n"
	   << "  goal index: " << stats_.goal_ix << " " << stats_.goal_iy << "\n"
	   << "  goal state: " << stats_.goal_state << "\n";
	throw runtime_error(os.str());
      }
    }
    
    if (stats_.plan_from_scratch)
      planner_->force_planning_from_scratch();
    
    if (stats_.flush_cost_changes)
      environment_->FlushCostUpdates(planner_.get());
    
    if (stats_.stop_at_first_solution)
      planner_->set_search_mode(true);
    else
      planner_->set_search_mode(false);
    
    // do not forget to call superclass
    IncrementalCostmapPlanner::preCreatePlan();
  }
  
  
  boost::shared_ptr<waypoint_plan_t> SBPLPlannerWrap::
  doCreatePlan() throw(std::exception)
  {
    vector<int> solution;
    stats_.status = planner_->replan(stats_.allocated_time, &solution, &stats_.solution_cost);
    shared_ptr<waypoint_plan_t> plan;
    if (1 == stats_.status) {
      plan.reset(new waypoint_plan_t());
      if (1 < solution.size())
	convertPlan(*environment_, solution, plan.get(),
		    &stats_.plan_length,
		    &stats_.plan_angle_change,
		    0 // XXXX to do: if 3DKIN we actually want something here
		    );
    }
    return plan;
  }
  
  
  void SBPLPlannerWrap::
  postCreatePlan() throw(std::exception)
  {
    // do not forget to call superclass
    IncrementalCostmapPlanner::postCreatePlan();
    
    stats_.number_of_expands = planner_->get_n_expands();
    stats_.solution_epsilon = planner_->get_solution_eps();
  }
  
}
