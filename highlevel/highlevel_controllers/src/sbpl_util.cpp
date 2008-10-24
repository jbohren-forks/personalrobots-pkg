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

/** \file sbpl_util.cpp Implementation of sbpl_util.h */

#include <sbpl_util.hh>
#include <rosconsole/rosconsole.h>
#include <headers.h>
#include <sys/time.h>
#include <time.h>
#include <err.h>

namespace ros {
namespace highlevel_controllers {
  
  
  SBPLPlanner * createSBPLPlanner(std::string const & name,
				  DiscreteSpaceInformation* environment,
				  bool bforwardsearch,
				  MDPConfig* mdpCfg)
  {
    if ("ARAPlanner" == name)
      return new ARAPlanner(environment, bforwardsearch);
    if ("ADPlanner" == name)
      return new ADPlanner(environment, bforwardsearch);

#warning 'VIPlanner not instantiable... work in progress maybe?'
#ifdef UNDEFINED
    if ("VIPlanner" == name)
      return new VIPlanner(environment, mdpCfg);
#endif // UNDEFINED

    return 0;    
  }
  
  
  SBPLPlannerManager::
  SBPLPlannerManager(DiscreteSpaceInformation* environment,
		     bool bforwardsearch,
		     MDPConfig* mdpCfg)
    : environment_(environment),
      bforwardsearch_(bforwardsearch),
      mdpCfg_(mdpCfg),
      planner_(0),
      name_("")
  {
  }
  
  
  SBPLPlannerManager::
  ~SBPLPlannerManager()
  {
    delete planner_;
  }
  
  
  bool SBPLPlannerManager::
  select(std::string const & name, bool recycle)
  {
    if (recycle && (name == name_))
      return true;
    SBPLPlanner * new_planner(createSBPLPlanner(name, environment_, bforwardsearch_, mdpCfg_));
    if ( ! new_planner)
      return false;
    delete planner_;
    planner_ = new_planner;
    name_ = name;
    return true;    
  }
  
  
  std::string const & SBPLPlannerManager::
  getName() const
  {
    return name_;
  }
  
  
  int SBPLPlannerManager::
  replan(double allocated_time_sec, double * actual_time_sec,
	 vector<int>* solution_stateIDs_V) throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    
    struct timeval t_started;
    struct timeval t_finished;
    gettimeofday(&t_started, 0);
    int const status(planner_->replan(allocated_time_sec, solution_stateIDs_V));
    gettimeofday(&t_finished, 0);
    *actual_time_sec =
      t_finished.tv_sec - t_started.tv_sec
      + 1e-6 * t_finished.tv_usec - 1e-6 * t_started.tv_usec;
    
    return status;
  }
  
  
  int SBPLPlannerManager::
  set_goal(int goal_stateID) throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    return planner_->set_goal(goal_stateID);
  }
  
  
  int SBPLPlannerManager::
  set_start(int start_stateID) throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    return planner_->set_start(start_stateID);
  }
  
  
  int SBPLPlannerManager::
  force_planning_from_scratch() throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    return planner_->force_planning_from_scratch();
  }
  
  
  SBPLPlannerStatistics::entry::
  entry(std::string const & _plannerName)
    : plannerName(_plannerName),
      goalState(-1),
      startState(-1),
      status(-42)
  {
  }
  
  
  void SBPLPlannerStatistics::
  pushBack(std::string const & plannerName)
  {
    stats_.push_back(entry(plannerName));
  }
  
  
  SBPLPlannerStatistics::entry & SBPLPlannerStatistics::
  top()
  {
    return stats_.back();
  }
  
  
  SBPLPlannerStatistics::stats_t const & SBPLPlannerStatistics::
  getAll() const
  {
    return stats_;
  }
  
  
  void SBPLPlannerStatistics::entry::
  logInfo(char const * prefix) const
  {
    ROS_INFO("%s\n"
	     "%splanner:                 %s\n"
	     "%sgoal (map/grid/state):   %+8.3f %+8.3f %+8.3f / %u %u / %d\n"
	     "%sstart (map/grid/state):  %+8.3f %+8.3f %+8.3f / %u %u / %d\n"
	     "%stime [s] (actual/alloc): %g / %g\n"
	     "%sstatus (1 == SUCCESS):   %d\n"
	     "%splan_length [m]:         %+8.3f\n"
	     "%splan_rotation [rad]:     %+8.3f\n",
	     prefix,
	     prefix, plannerName.c_str(),
	     prefix, goal.x, goal.y, goal.th, goalIx, goalIy, goalState,
	     prefix, start.x, start.y, start.th, startIx, startIy, startState,
	     prefix, actual_time_sec, allocated_time_sec,
	     prefix, status,
	     prefix, plan_length_m,
	     prefix, plan_angle_change_rad);
  }
  
  
  void SBPLPlannerStatistics::entry::
  logFile(char const * filename, char const * prefix) const
  {
    FILE * ff(fopen(filename, "a"));
    if (0 == ff) {
      warn("SBPLPlannerStatistics::entry::logFile(): fopen(%s)", filename);
      return;
    }
    fprintf(ff,
	    "%splanner:                 %s\n"
	    "%sgoal (map/grid/state):   %+8.3f %+8.3f %+8.3f / %u %u / %d\n"
	    "%sstart (map/grid/state):  %+8.3f %+8.3f %+8.3f / %u %u / %d\n"
	    "%stime [s] (actual/alloc): %g / %g\n"
	    "%sstatus (1 == SUCCESS):   %d\n"
	    "%splan_length [m]:         %+8.3f\n"
	    "%splan_rotation [rad]:     %+8.3f\n",
	    prefix, plannerName.c_str(),
	    prefix, goal.x, goal.y, goal.th, goalIx, goalIy, goalState,
	    prefix, start.x, start.y, start.th, startIx, startIy, startState,
	    prefix, actual_time_sec, allocated_time_sec,
	    prefix, status,
	    prefix, plan_length_m,
	    prefix, plan_angle_change_rad);
    if (0 != fclose(ff))
      warn("SBPLPlannerStatistics::entry::logFile(): fclose() on %s", filename);
  }
  
}
}
