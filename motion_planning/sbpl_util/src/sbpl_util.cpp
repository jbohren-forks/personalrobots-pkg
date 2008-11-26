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

#include "sbpl_util.hh"
#include "environment_wrap.h"
#include <headers.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
#include <cstring>


using namespace std;


namespace {
  
  static map<string, string> planner_alias;
  
}


namespace ompl {
  
  
  std::string canonicalPlannerName(std::string const & name_or_alias)
  {
    if (planner_alias.empty()) {
      planner_alias.insert(make_pair("ARAPlanner", "ARAPlanner"));
      planner_alias.insert(make_pair("ara",        "ARAPlanner"));
      planner_alias.insert(make_pair("ARA",        "ARAPlanner"));
      planner_alias.insert(make_pair("arastar",    "ARAPlanner"));
      planner_alias.insert(make_pair("ARAStar",    "ARAPlanner"));

      planner_alias.insert(make_pair("ADPlanner",  "ADPlanner"));
      planner_alias.insert(make_pair("ad",         "ADPlanner"));
      planner_alias.insert(make_pair("AD",         "ADPlanner"));
      planner_alias.insert(make_pair("adstar",     "ADPlanner"));
      planner_alias.insert(make_pair("ADStar",     "ADPlanner"));
    }
    
    map<string, string>::const_iterator is(planner_alias.find(name_or_alias));
    if (planner_alias.end() == is)
      return "";
    return is->second;
  }
  
  
  SBPLPlanner * createSBPLPlanner(std::string const & name,
				  DiscreteSpaceInformation* environment,
				  bool bforwardsearch,
				  MDPConfig* mdpCfg,
				  std::ostream * opt_err_os)
  {
    string const canonical_name(canonicalPlannerName(name));
    if ("ARAPlanner" == canonical_name)
      return new ARAPlanner(environment, bforwardsearch);
    if ("ADPlanner" == canonical_name)
      return new ADPlanner(environment, bforwardsearch);

    // VIPlanner not instantiable... work in progress by Max
#ifdef UNDEFINED
    if ("VIPlanner" == canonical_name)
      return new VIPlanner(environment, mdpCfg);
#endif // UNDEFINED
    
    if (opt_err_os) {
      *opt_err_os << "ompl::createSBPLPlanner(): no planner called \"name\"\n"
		  << "  use \"ARAPlanner\" or \"ADPlanner\"\n"
		  << "  or one of the registered aliases:\n";
      for (map<string, string>::const_iterator is(planner_alias.begin()); is != planner_alias.end(); ++is)
	*opt_err_os << "    " << is->first << " --> " << is->second << "\n";
    }
    
    return 0;    
  }
  
  
  SBPLPlannerManager::no_planner_selected::
  no_planner_selected()
    : std::runtime_error("SBPLPlannerManager: no planner selected")
  {
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
  select(std::string const & name, bool recycle, std::ostream * opt_err_os)
  {
    if (recycle && (name == name_))
      return true;
    SBPLPlanner * new_planner(createSBPLPlanner(name, environment_, bforwardsearch_, mdpCfg_, opt_err_os));
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
  replan(bool stop_at_first_solution,
	 bool plan_from_scratch,
	 double allocated_time_sec,
	 double * actual_time_wall_sec,
	 double * actual_time_user_sec,
	 double * actual_time_system_sec,
	 int * solution_cost,
	 vector<int>* solution_stateIDs_V) throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    
    if (stop_at_first_solution)
      planner_->set_search_mode(true);
    else
      planner_->set_search_mode(false);
    
    if (plan_from_scratch)
      force_planning_from_scratch();
    
    struct rusage ru_started;
    struct timeval t_started;
    getrusage(RUSAGE_SELF, &ru_started);
    gettimeofday(&t_started, 0);
    
    int const status(planner_->replan(allocated_time_sec, solution_stateIDs_V, solution_cost));

    struct rusage ru_finished;
    struct timeval t_finished;
    getrusage(RUSAGE_SELF, &ru_finished);
    gettimeofday(&t_finished, 0);
    
    *actual_time_wall_sec =
      t_finished.tv_sec - t_started.tv_sec
      + 1e-6 * t_finished.tv_usec - 1e-6 * t_started.tv_usec;
    *actual_time_user_sec =
      ru_finished.ru_utime.tv_sec - ru_started.ru_utime.tv_sec
      + 1e-6 * ru_finished.ru_utime.tv_usec - 1e-6 * ru_started.ru_utime.tv_usec;
    *actual_time_system_sec =
      ru_finished.ru_stime.tv_sec - ru_started.ru_stime.tv_sec
      + 1e-6 * ru_finished.ru_stime.tv_usec - 1e-6 * ru_started.ru_stime.tv_usec;
    
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
  
  
  void SBPLPlannerManager::
  flush_cost_changes(EnvironmentWrapper & ewrap) throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    ewrap.FlushCostUpdates(planner_);
  }
  
  
  SBPLPlannerStatsEntry::
  SBPLPlannerStatsEntry(std::string const & _plannerType, std::string const & _environmentType)
    : plannerType(_plannerType),
      environmentType(_environmentType),
      goalState(-1),
      startState(-1),
      status(-42)
  {
  }
  
  
  void SBPLPlannerStatsEntry::
  logInfo(char const * prefix) const
  {
    ostringstream os;
    logStream(os, "ompl::SBPLPlannerStatistics", prefix);
    ROS_INFO("%s", os.str().c_str());
  }
  
  
  void SBPLPlannerStatsEntry::
  logFile(char const * filename, char const * title, char const * prefix) const
  {
    FILE * ff(fopen(filename, "a"));
    if (0 == ff) {
      ROS_WARN("SBPLPlannerStatsEntry::logFile(): fopen(%s): %s",
	       filename, strerror(errno));
      return;
    }
    ostringstream os;
    logStream(os, title, prefix);
    fprintf(ff, "%s", os.str().c_str());
    if (0 != fclose(ff))
      ROS_WARN("SBPLPlannerStatsEntry::logFile(): fclose() on %s: %s",
	       filename, strerror(errno));
  }
  
  
  void SBPLPlannerStatsEntry::
  logStream(std::ostream & os, std::string const & title, std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "planner:               " << plannerType << "\n"
       << prefix << "environment:           " << environmentType << "\n"
       << prefix << "goal  map:             " << goal.x << "  " << goal.y << "  " << goal.th << "\n"
       << prefix << "goal  grid:            " << goalIx << "  " << goalIy << "\n"
       << prefix << "goal  state:           " << goalState << "\n"
       << prefix << "start map:             " << start.x << "  " << start.y << "  " << start.th << "\n"
       << prefix << "start grid:            " << startIx << "  " << startIy << "\n"
       << prefix << "start state:           " << startState << "\n"
       << prefix << "stop at first solution:" << (stop_at_first_solution ? "true\n" : "false\n")
       << prefix << "plan from scratch:     " << (plan_from_scratch ? "true\n" : "false\n")
       << prefix << "time  alloc:           " << allocated_time_sec << "\n"
       << prefix << "time  actual (wall):   " << actual_time_wall_sec << "\n"
       << prefix << "time  actual (user):   " << actual_time_user_sec << "\n"
       << prefix << "time  actual (system): " << actual_time_system_sec << "\n"
       << prefix << "solution cost:         " << solution_cost << "\n"
       << prefix << "status (1 == SUCCESS): " << status << "\n"
       << prefix << "plan_length [m]:       " << plan_length_m << "\n"
       << prefix << "plan_rotation [rad]:   " << plan_angle_change_rad << "\n";
  }
  
}
