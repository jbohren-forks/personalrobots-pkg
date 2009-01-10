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

#include "planner.h"
#include "costmap.h"
#include <sbpl/headers.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <fstream>

using namespace boost;
using namespace std;

namespace mpglue {
  
  
  std::string canonicalPlannerName(std::string const & name_or_alias)
  {
    static map<string, string> planner_alias;
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

      planner_alias.insert(make_pair("NavFn",      "NavFn"));
      planner_alias.insert(make_pair("navfn",      "NavFn"));
      planner_alias.insert(make_pair("nf",         "NavFn"));
      planner_alias.insert(make_pair("NF",         "NavFn"));
    }
    
    map<string, string>::const_iterator is(planner_alias.find(name_or_alias));
    if (planner_alias.end() == is)
      return "";
    return is->second;
  }
  
  
  CostmapPlannerStats::
  CostmapPlannerStats()
    : start_x(0),
      start_y(0),
      start_th(0),
      start_ix(0),
      start_iy(0),
      goal_x(0),
      goal_y(0),
      goal_th(0),
      goal_ix(0),
      goal_iy(0),
      goal_tol_distance(0),
      goal_tol_angle(0),
      success(false),
      actual_time_wall(0),
      actual_time_user(0),
      plan_length(0),
      plan_angle_change(0)
  {
  }
  
  
  CostmapPlannerStats::
  ~CostmapPlannerStats()
  {
  }
  
  
  CostmapPlannerStats * CostmapPlannerStats::
  copy() const
  {
    CostmapPlannerStats * foo(new CostmapPlannerStats(*this));
    return foo;
  }
  
  
  void CostmapPlannerStats::
  logFile(char const * filename,
	  std::string const & title,
	  std::string const & prefix) const
  {
    ofstream os(filename, ios_base::app);
    if (os)
      logStream(os, title, prefix);
    // maybe throw an exception if something goes wrong?
  }
  
  
  void CostmapPlannerStats::
  logStream(std::ostream & os,
	    std::string const & title,
	    std::string const & prefix) const
  {
    if ( ! title.empty())
      os << title << "\n";
    os << prefix << "goal pose:                 " << goal_x << "  " << goal_y << "  " << goal_th << "\n"
       << prefix << "goal grid:                 " << goal_ix << "  " << goal_iy << "\n"
       << prefix << "start pose:                " << start_x << "  " << start_y << "  " << start_th << "\n"
       << prefix << "start grid:                " << start_ix << "  " << start_iy << "\n";
    if (success)
      os << prefix << "success:                   TRUE\n";
    else
      os << prefix << "success:                   FALSE\n";
    os << prefix << "time actual (wall) [ms]:   " << 1.0e3 * actual_time_wall << "\n"
       << prefix << "time actual (user) [ms]:   " << 1.0e3 * actual_time_user << "\n"
       << prefix << "plan_length [m]:           " << plan_length << "\n"
       << prefix << "plan_rotation [rad]:       " << plan_angle_change << "\n";
  }
  
  
  CostmapPlanner::
  CostmapPlanner(CostmapPlannerStats & stats,
		 boost::shared_ptr<Costmap const> costmap,
		 boost::shared_ptr<IndexTransform const> itransform)
    : stats__(stats),
      costmap_(costmap),
      itransform_(itransform)
  {
  }
  
  
  CostmapPlanner::
  ~CostmapPlanner()
  {
  }
  
  
  void CostmapPlanner::
  setStart(double px, double py, double pth) throw(std::out_of_range)
  {
    ssize_t ix, iy;
    itransform_->globalToIndex(px, py, &ix, &iy);
    if ( ! costmap_->isValidIndex(ix, iy)) {
      ostringstream os;
      os << "ERROR in mpglue::CostmapPlanner::setStart(): invalid grid index\n"
	 << "  start: " << px << " " << py << " " << pth << "\n"
	 << "  grid index: " << ix << " " << iy << "\n";
      throw out_of_range(os.str());
    }
    stats__.start_x = px;
    stats__.start_y = py;
    stats__.start_th = pth;
    stats__.start_ix = ix;
    stats__.start_iy = iy;
  }
  
  
  void CostmapPlanner::
  setGoal(double px, double py, double pth) throw(std::out_of_range)
  {
    ssize_t ix, iy;
    itransform_->globalToIndex(px, py, &ix, &iy);
    if ( ! costmap_->isValidIndex(ix, iy)) {
      ostringstream os;
      os << "ERROR in mpglue::CostmapPlanner::setGoal(): invalid grid index\n"
	 << "  goal: " << px << " " << py << " " << pth << "\n"
	 << "  grid index: " << ix << " " << iy << "\n";
      throw out_of_range(os.str());
    }
    stats__.goal_x = px;
    stats__.goal_y = py;
    stats__.goal_th = pth;
    stats__.goal_ix = ix;
    stats__.goal_iy = iy;
  }
  
  
  void CostmapPlanner::
  setGoalTolerance(double dist_tol, double angle_tol)
  {
    stats__.goal_tol_distance = dist_tol;
    stats__.goal_tol_angle = angle_tol;
  }
  
  
  boost::shared_ptr<waypoint_plan_t> CostmapPlanner::
  createPlan() throw(std::exception)
  {
    stats__.success = false;
    stats__.plan_length = 0;
    stats__.plan_angle_change = 0;
    
    preCreatePlan();
    
    struct rusage ru_started;
    struct timeval t_started;
    getrusage(RUSAGE_SELF, &ru_started);
    gettimeofday(&t_started, 0);
    
    boost::shared_ptr<waypoint_plan_t> plan(doCreatePlan());
    
    struct rusage ru_finished;
    struct timeval t_finished;
    getrusage(RUSAGE_SELF, &ru_finished);
    gettimeofday(&t_finished, 0);
    stats__.actual_time_wall =
      t_finished.tv_sec - t_started.tv_sec
      + 1e-6 * t_finished.tv_usec - 1e-6 * t_started.tv_usec;
    stats__.actual_time_user =
      ru_finished.ru_utime.tv_sec - ru_started.ru_utime.tv_sec
      + 1e-6 * ru_finished.ru_utime.tv_usec - 1e-6 * ru_started.ru_utime.tv_usec;
    
    postCreatePlan();
    
    if (plan)
      stats__.success = true;
    return plan;
  }
  
  
  void CostmapPlanner::
  preCreatePlan() throw(std::exception)
  {
  }
  
  
  void CostmapPlanner::
  postCreatePlan() throw(std::exception)
  {
  }
  
  
  boost::shared_ptr<CostmapPlannerStats> CostmapPlanner::
  copyStats() const
  {
    shared_ptr<CostmapPlannerStats> stats(stats__.copy());
    return stats;
  }
  
  
  DynamicCostmapPlannerStats::
  DynamicCostmapPlannerStats()
    : CostmapPlannerStats(),
      plan_from_scratch(false),
      flush_cost_changes(false)
  {
  }
  
  
  DynamicCostmapPlannerStats * DynamicCostmapPlannerStats::
  copy() const
  {
    DynamicCostmapPlannerStats * foo(new DynamicCostmapPlannerStats(*this));
    return foo;
  }
  
  
  void DynamicCostmapPlannerStats::
  logStream(std::ostream & os,
	    std::string const & title,
	    std::string const & prefix) const
  {
    CostmapPlannerStats::logStream(os, title, prefix);
    if (plan_from_scratch)
      os << prefix << "plan_from_scratch:         TRUE\n";
    else
      os << prefix << "plan_from_scratch:         FALSE\n";
    if (flush_cost_changes)
      os << prefix << "flush_cost_changes:        TRUE\n";
    else
      os << prefix << "flush_cost_changes:        FALSE\n";
  }
  
  
  DynamicCostmapPlanner::
  DynamicCostmapPlanner(DynamicCostmapPlannerStats & stats,
			boost::shared_ptr<Costmap const> costmap,
			boost::shared_ptr<IndexTransform const> itransform)
    : CostmapPlanner(stats, costmap, itransform),
      stats__(stats)
  {
  }
  
  
  void DynamicCostmapPlanner::
  forcePlanningFromScratch(bool flag)
  {
    stats__.plan_from_scratch = flag;
  }
  
  
  void DynamicCostmapPlanner::
  flushCostChanges(bool flag)
  {
    stats__.flush_cost_changes = flag;
  }
  
  
  void DynamicCostmapPlanner::
  postCreatePlan() throw(std::exception)
  {
    // do not forget to call superclass!
    CostmapPlanner::postCreatePlan();
    
    stats__.plan_from_scratch = false;
    stats__.flush_cost_changes = false;
  }
  
  
  IncrementalCostmapPlannerStats::
  IncrementalCostmapPlannerStats()
    : DynamicCostmapPlannerStats(),
      stop_at_first_solution(false),
      allocated_time(std::numeric_limits<double>::max())
  {
  }
  
  
  IncrementalCostmapPlannerStats * IncrementalCostmapPlannerStats::
  copy() const
  {
    IncrementalCostmapPlannerStats * foo(new IncrementalCostmapPlannerStats(*this));
    return foo;
  }
  
  
  void IncrementalCostmapPlannerStats::
  logStream(std::ostream & os,
	    std::string const & title,
	    std::string const & prefix) const
  {
    DynamicCostmapPlannerStats::logStream(os, title, prefix);
    if (stop_at_first_solution)
      os << prefix << "stop_at_first_solution:    TRUE\n";
    else
      os << prefix << "stop_at_first_solution:    FALSE\n";
    os << prefix << "time allocated [ms]:       " << 1.0e3 * allocated_time << "\n";
  }

  
  IncrementalCostmapPlanner::
  IncrementalCostmapPlanner(IncrementalCostmapPlannerStats & stats,
			    boost::shared_ptr<Costmap const> costmap,
			    boost::shared_ptr<IndexTransform const> itransform)
    : DynamicCostmapPlanner(stats, costmap, itransform),
      stats__(stats)
  {
  }
  
  
  void IncrementalCostmapPlanner::
  stopAtFirstSolution(bool flag)
  {
    stats__.stop_at_first_solution = flag;
  }


  void IncrementalCostmapPlanner::
  setAllocatedTime(double seconds)
  {
    stats__.allocated_time = seconds;
  }
  
}
