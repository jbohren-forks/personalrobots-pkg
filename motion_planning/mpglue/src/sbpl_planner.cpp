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

#include <mpglue/sbpl_planner.h>
#include <mpglue/sbpl_environment.h>
#include <sbpl/headers.h>
#include <sstream>

// just for a quick hack
#include <set>

using namespace boost;
using namespace std;

namespace mpglue {
  
  
  SBPLPlannerStats::
  SBPLPlannerStats()
    : AnytimeCostmapPlannerStats(),
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
    AnytimeCostmapPlannerStats::logStream(os, title, prefix);
    os << prefix << "goal_state:                " << goal_state << "\n"
       << prefix << "start_state:               " << start_state << "\n"
       << prefix << "status:                    " << status << "\n"
       << prefix << "number_of_expands:         " << number_of_expands << "\n"
       << prefix << "solution_cost:             " << solution_cost << "\n"
       << prefix << "solution_epsilon:          " << solution_epsilon << "\n";
  }
  
  
  char const * SBPLPlannerStats::
  getClassName() const
  {
    return "mpglue::SBPLPlannerStats";
  }
  
  
  void SBPLPlannerStats::
  dumpSubclassXML(std::ostream & os,
		  std::string const & prefix) const
  {
    AnytimeCostmapPlannerStats::dumpSubclassXML(os, prefix);
    os << prefix << "  <goal_state>" << goal_state << "</goal_state>\n"
       << prefix << "  <start_state>" << start_state << "</start_state>\n"
       << prefix << "  <status>" << status << "</status>\n"
       << prefix << "  <number_of_expands>" << number_of_expands << "</number_of_expands>\n"
       << prefix << "  <solution_cost>" << solution_cost << "</solution_cost>\n"
       << prefix << "  <solution_epsilon>" << solution_epsilon << "</solution_epsilon>\n";
  }
  
  
  SBPLPlannerWrap::
  SBPLPlannerWrap(boost::shared_ptr<SBPLEnvironment> environment,
		  boost::shared_ptr<SBPLPlanner> planner)
    : AnytimeCostmapPlanner(stats_,
			    environment->getCostmap(),
			    environment->getIndexTransform()),
      planner_(planner),
      environment_(environment)
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
    
    if (goal_tol_changed_) {
      environment_->SetGoalTolerance(stats_.goal_tol_distance, stats_.goal_tol_angle);
      // XXXX to do: also tell planner? but goal tolerances are
      // completely ignored by sbpl anyway...
    }
    
    if (stats_.plan_from_scratch)
      planner_->force_planning_from_scratch();
    
    if (stats_.flush_cost_changes) {
      if (environment_->HavePendingCostUpdates())
	environment_->FlushCostUpdates(planner_.get());
      else
	stats_.flush_cost_changes = false;
    }
    
    if (stats_.stop_at_first_solution)
      planner_->set_search_mode(true);
    else
      planner_->set_search_mode(false);
    
    // do not forget to call superclass
    AnytimeCostmapPlanner::preCreatePlan();
  }
  
  
  boost::shared_ptr<waypoint_plan_t> SBPLPlannerWrap::
  doCreatePlan() throw(std::exception)
  {
    vector<int> solution;
    stats_.status = planner_->replan(stats_.allocated_time, &solution, &stats_.solution_cost);
    shared_ptr<waypoint_plan_t> plan;
    // using the cell size as waypoint tolerance seems like a good
    // heuristic
    double const dr(itransform_->getResolution());
    double const dtheta(environment_->GetAngularTolerance());
    if (1 == stats_.status) {
      plan.reset(new waypoint_plan_t());
      // XXXX to do: can this be generalized?
      EnvironmentNAVXYTHETADOOR * doorenv(dynamic_cast<EnvironmentNAVXYTHETADOOR *>(environment_->getDSI()));
      if (doorenv) {
	for (size_t ii(0); ii < solution.size(); ++ii) {
	  int xc, yc, thc;
	  unsigned char door_interval;
	  doorenv->GetCoordFromState(solution[ii], xc, yc, thc, door_interval);
	  double xx, yy, th;
	  if (doorenv->PoseDiscToCont(xc, yc, thc, xx, yy, th)) { // "always" succeeds
	    // XXXX to do: should extract angle from state instead of recomputing...
	    double angle, door_angle_cost;
	    doorenv->GetMinCostDoorAngle(xx, yy, th, door_interval, angle, door_angle_cost);
	    shared_ptr<door_waypoint_s> doorwpt(new door_waypoint_s(xx, yy, th, dr, dtheta, angle, door_angle_cost));
	    // dbg
	    EnvNAVXYTHETALAT3Dpt_t wrp;
	    wrp.x = xx;
	    wrp.y = yy;
	    wrp.theta = th;
	    doorenv->GetValidDoorAngles(wrp, &doorwpt->valid_angle, &doorwpt->valid_cost, &doorwpt->valid_interval);
	    { // very quickly hacked overlap detection: these are not the droids you are looking for
	      std::set<int> foo;
	      for (size_t jj(0); jj < doorwpt->valid_angle.size(); ++jj) {
		int const angle(doorwpt->valid_angle[jj]);
		if (foo.end() != foo.find(angle))
		  doorwpt->overlap_angle.push_back(angle);
		foo.insert(angle);
	      }
	    }
	    plan->push_back(doorwpt);
	    
	    printf("DBG doorwpt (%6.3f %6.3f %6.3f)  angle %6.3f  int %u  %zu valid  %zu overlap\n",
		   xx, yy, th, angle, (unsigned int) door_interval, doorwpt->valid_angle.size(), doorwpt->overlap_angle.size());
	    
	  }
	}
      }
      else {			// not a door environment
	if (1 < solution.size())
	  PlanConverter::convertSBPL(*environment_,
				     solution,
				     dr,
				     dtheta,
				     plan.get(),
				     &stats_.plan_length,
				     &stats_.plan_angle_change,
				     0 // XXXX to do: if 3DKIN we actually want something here
				     );
      }
    }
    return plan;
  }
  
  
  void SBPLPlannerWrap::
  postCreatePlan() throw(std::exception)
  {
    // do not forget to call superclass
    AnytimeCostmapPlanner::postCreatePlan();
    
    stats_.number_of_expands = planner_->get_n_expands();
    stats_.solution_epsilon = planner_->get_solution_eps();
  }
  
}
