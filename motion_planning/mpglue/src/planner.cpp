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
#include <sfl/util/strutil.hpp>
#include <sbpl/headers.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <fstream>

using sfl::to_string;
using namespace boost;
using namespace std;

namespace mpglue {
  
  
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
      plan_from_scratch(false),
      flush_cost_changes(false),
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
       << prefix << "start grid:                " << start_ix << "  " << start_iy << "\n"
       << prefix << "plan_from_scratch:         " << to_string(plan_from_scratch) << "\n"
       << prefix << "flush_cost_changes:        " << to_string(flush_cost_changes) << "\n"
       << prefix << "success:                   " << to_string(success) << "\n"
       << prefix << "time actual (wall) [ms]:   " << 1.0e3 * actual_time_wall << "\n"
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
      itransform_(itransform),
      start_changed_(false),
      goal_pose_changed_(false),
      goal_tol_changed_(false),
      plan_from_scratch_changed_(false),
      flush_cost_changes_changed_(false)
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
    start_changed_ = true;
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
    goal_pose_changed_ = true;
  }
  
  
  void CostmapPlanner::
  setGoalTolerance(double dist_tol, double angle_tol)
  {
    stats__.goal_tol_distance = dist_tol;
    stats__.goal_tol_angle = angle_tol;
    goal_tol_changed_ = true;
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
    start_changed_ = false;
    goal_pose_changed_ = false;
    goal_tol_changed_ = false;
    plan_from_scratch_changed_ = false;
    flush_cost_changes_changed_ = false;
  }
  
  
  boost::shared_ptr<CostmapPlannerStats> CostmapPlanner::
  copyStats() const
  {
    shared_ptr<CostmapPlannerStats> stats(stats__.copy());
    return stats;
  }
  
  
  void CostmapPlanner::
  forcePlanningFromScratch(bool flag)
  {
    if (flag != stats__.plan_from_scratch) {
      stats__.plan_from_scratch = flag;
      plan_from_scratch_changed_ = true;
    }
  }
  
  
  void CostmapPlanner::
  flushCostChanges(bool flag)
  {
    if (flag != stats__.flush_cost_changes) {
      stats__.flush_cost_changes = flag;
      flush_cost_changes_changed_ = true;
    }
  }
  
  
  AnytimeCostmapPlannerStats::
  AnytimeCostmapPlannerStats()
    : CostmapPlannerStats(),
      stop_at_first_solution(false),
      allocated_time(std::numeric_limits<double>::max())
  {
  }
  
  
  AnytimeCostmapPlannerStats * AnytimeCostmapPlannerStats::
  copy() const
  {
    AnytimeCostmapPlannerStats * foo(new AnytimeCostmapPlannerStats(*this));
    return foo;
  }
  
  
  void AnytimeCostmapPlannerStats::
  logStream(std::ostream & os,
	    std::string const & title,
	    std::string const & prefix) const
  {
    CostmapPlannerStats::logStream(os, title, prefix);
    os << prefix << "stop_at_first_solution:    " << to_string(stop_at_first_solution) << "\n"
       << prefix << "time allocated [ms]:       " << 1.0e3 * allocated_time << "\n";
  }

  
  AnytimeCostmapPlanner::
  AnytimeCostmapPlanner(AnytimeCostmapPlannerStats & stats,
			boost::shared_ptr<Costmap const> costmap,
			boost::shared_ptr<IndexTransform const> itransform)
    : CostmapPlanner(stats, costmap, itransform),
      stats__(stats)
  {
  }
  
  
  void AnytimeCostmapPlanner::
  stopAtFirstSolution(bool flag)
  {
    if (flag != stats__.stop_at_first_solution) {
      stats__.stop_at_first_solution = flag;
      stop_at_first_solution_changed_ = true;
    }
  }


  void AnytimeCostmapPlanner::
  setAllocatedTime(double seconds)
  {
    stats__.allocated_time = seconds;
    allocated_time_changed_ = true;
  }
  
  
  void AnytimeCostmapPlanner::
  postCreatePlan() throw(std::exception)
  {
    stop_at_first_solution_changed_ = false;
    allocated_time_changed_ = false;
  }

  
}
