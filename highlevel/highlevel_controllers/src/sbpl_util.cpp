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
#include <costmap_2d/costmap_2d.h>
#include <headers.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <time.h>
#include <sstream>
#include <errno.h>
#include <cstring>


namespace {
  
  std::string mk_invalid_state_str(std::string const & method, int state) {
    std::ostringstream os;
    os << "EnvironmentWrapper::invalid_state in method " << method << ": state = " << state;
    return os.str();
  }
  
}

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

    // VIPlanner not instantiable... work in progress by Max
#ifdef UNDEFINED
    if ("VIPlanner" == name)
      return new VIPlanner(environment, mdpCfg);
#endif // UNDEFINED

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
  replan(double allocated_time_sec,
	 double * actual_time_wall_sec,
	 double * actual_time_user_sec,
	 double * actual_time_system_sec,
	 vector<int>* solution_stateIDs_V) throw(no_planner_selected)
  {
    if ( ! planner_)
      throw no_planner_selected();
    
    struct rusage ru_started;
    struct timeval t_started;
    getrusage(RUSAGE_SELF, &ru_started);
    gettimeofday(&t_started, 0);

    int const status(planner_->replan(allocated_time_sec, solution_stateIDs_V));

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
  
  
  SBPLPlannerStatistics::entry::
  entry(std::string const & _plannerType, std::string const & _environmentType)
    : plannerType(_plannerType),
      environmentType(_environmentType),
      goalState(-1),
      startState(-1),
      status(-42)
  {
  }
  
  
  void SBPLPlannerStatistics::
  pushBack(std::string const & plannerType, std::string const & environmentType)
  {
    stats_.push_back(entry(plannerType, environmentType));
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
	     prefix, plannerType.c_str(),
	     prefix, goal.x, goal.y, goal.th, goalIx, goalIy, goalState,
	     prefix, start.x, start.y, start.th, startIx, startIy, startState,
	     prefix, actual_time_wall_sec, allocated_time_sec,
	     prefix, status,
	     prefix, plan_length_m,
	     prefix, plan_angle_change_rad);
  }
  
  
  void SBPLPlannerStatistics::entry::
  logFile(char const * filename, char const * title, char const * prefix) const
  {
    FILE * ff(fopen(filename, "a"));
    if (0 == ff) {
      ROS_WARN("SBPLPlannerStatistics::entry::logFile(): fopen(%s): %s",
	       filename, strerror(errno));
      return;
    }
    fprintf(ff,
	    "%s\n"
	    "%splanner:               %s\n"
	    "%senvironment:           %s\n"
	    "%sgoal  map:             %+8.3f %+8.3f %+8.3f\n"
	    "%sgoal  grid:            %u %u\n"
	    "%sgoal  state:           %d\n"
	    "%sstart map:             %+8.3f %+8.3f %+8.3f\n"
	    "%sstart grid:            %u %u\n"
	    "%sstart state:           %d\n"
	    "%stime  alloc:           %g\n"
	    "%stime  actual (wall):   %g\n"
	    "%stime  actual (user):   %g\n"
	    "%stime  actual (system): %g\n"
	    "%sstatus (1 == SUCCESS): %d\n"
	    "%splan_length [m]:       %+8.3f\n"
	    "%splan_rotation [rad]:   %+8.3f\n",
	    title,
	    prefix, plannerType.c_str(),
	    prefix, environmentType.c_str(),
	    prefix, goal.x, goal.y, goal.th,
	    prefix, goalIx, goalIy,
	    prefix, goalState,
	    prefix, start.x, start.y, start.th,
	    prefix, startIx, startIy,
	    prefix, startState,
	    prefix, allocated_time_sec,
	    prefix, actual_time_wall_sec,
	    prefix, actual_time_user_sec,
	    prefix, actual_time_system_sec,
	    prefix, status,
	    prefix, plan_length_m,
	    prefix, plan_angle_change_rad);
    if (0 != fclose(ff))
      ROS_WARN("SBPLPlannerStatistics::entry::logFile(): fclose() on %s: %s",
	       filename, strerror(errno));
  }
  
  
  ////   EnvironmentWrapper::invalid_pose::
  ////   invalid_pose(std::string const & method, std_msgs::Pose2DFloat32 const & pose)
  ////     : std::runtime_error(mk_invalid_pose_str(method, pose))
  ////   {
  ////   }
  
  
  EnvironmentWrapper::invalid_state::
  invalid_state(std::string const & method, int state)
    : std::runtime_error(mk_invalid_state_str(method, state))
  {
  }
  
  
  EnvironmentWrapper2D::
  EnvironmentWrapper2D(costmap_2d::CostMap2D const & costmap,
		       int startx, int starty,
		       int goalx, int goaly,
		       unsigned char obsthresh)
    : EnvironmentWrapper(costmap),
      env_(new EnvironmentNAV2D())
  {
#warning 'Would be nice to get (at least) start as a pose from current data'
    env_->InitializeEnv(costmap.getWidth(), costmap.getHeight(), costmap.getMap(),
		       startx, starty, goalx, goaly, obsthresh);
  }
  
  
  EnvironmentWrapper2D::
  ~EnvironmentWrapper2D()
  {
    delete env_;
  }
  
  
  DiscreteSpaceInformation * EnvironmentWrapper2D::
  getDSI()
  {
    return env_;
  }
  
  
  bool EnvironmentWrapper2D::
  InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    return env_->InitializeMDPCfg(MDPCfg);
  }
  
  
  bool EnvironmentWrapper2D::
  UpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV2D::SetStart()
      return false;
    return env_->UpdateCost(ix, iy, newcost);
  }
  
  
  bool EnvironmentWrapper2D::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle)
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int EnvironmentWrapper2D::
  SetStart(std_msgs::Pose2DFloat32 const & start)
  {
    unsigned int ix, iy;
    costmap_.WC_MC(start.x, start.y, ix, iy);
    return env_->SetStart(ix, iy);
  }
  
  
  int EnvironmentWrapper2D::
  SetGoal(std_msgs::Pose2DFloat32 const & goal)
  {
    unsigned int ix, iy;
    costmap_.WC_MC(goal.x, goal.y, ix, iy);
    return env_->SetGoal(ix, iy);
  }
  
  
  /** \note Always sets pose.th == -42 so people can detect that it is
      undefined. */
  std_msgs::Pose2DFloat32 EnvironmentWrapper2D::
  GetPoseFromState(int stateID) const
    throw(invalid_state)
  {
    if (0 > stateID)
      throw invalid_state("EnvironmentWrapper2D::GetPoseFromState()", stateID);
    int ix, iy;
    env_->GetCoordFromState(stateID, ix, iy);
    // by construction (ix,iy) is always inside the map (otherwise we
    // wouldn't have a stateID)
    double px, py;
    costmap_.MC_WC(ix, iy, px, py);
    std_msgs::Pose2DFloat32 pose;
    pose.x = px;
    pose.y = py;
    pose.th = -42;
    return pose;
  }
  
  
  int EnvironmentWrapper2D::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    unsigned int ix, iy;
    costmap_.WC_MC(pose.x, pose.y, ix, iy);
    if ( ! env_->IsWithinMapCell(ix, iy))
      return -1;
    return env_->GetStateFromCoord(ix, iy);
  }
  
  
  std::string EnvironmentWrapper2D::
  getName() const
  {
    std::string name("2D");
    return name;
  }
  
  
  EnvironmentWrapper3DKIN::
  EnvironmentWrapper3DKIN(costmap_2d::CostMap2D const & costmap,
			  unsigned char obst_cost_thresh,
			  double startx, double starty, double starttheta,
			  double goalx, double goaly, double goaltheta,
			  double goaltol_x, double goaltol_y, double goaltol_theta,
			  footprint_t const & footprint,
			  double nominalvel_mpersecs,
			  double timetoturn45degsinplace_secs)
    : EnvironmentWrapper(costmap),
      obst_cost_thresh_(obst_cost_thresh),
      env_(new EnvironmentNAV3DKIN())
  {
    // Aarghh at least we only do this once at init time.
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }
    
#warning 'Would be nice to get (at least) start as a pose from current data'
    env_->InitializeEnv(costmap.getWidth(), costmap.getHeight(), costmap.getMap(),
			startx, starty, starttheta,
			goalx, goaly, goaltheta,
			goaltol_x, goaltol_y, goaltol_theta,
			perimeterptsV, costmap.getResolution(), nominalvel_mpersecs,
			timetoturn45degsinplace_secs);
  }
  
  
  EnvironmentWrapper3DKIN::
  ~EnvironmentWrapper3DKIN()
  {
    delete env_;
  }
  
  
  DiscreteSpaceInformation * EnvironmentWrapper3DKIN::
  getDSI()
  {
    return env_;
  }
  
  
  bool EnvironmentWrapper3DKIN::
  InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    return env_->InitializeMDPCfg(MDPCfg);
  }
  
  
  bool EnvironmentWrapper3DKIN::
  UpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV3DKIN::UpdateCost()
      return false;
    if (obst_cost_thresh_ >= newcost)
      return env_->UpdateCost(ix, iy, 1);
    return env_->UpdateCost(ix, iy, 0);
  }
  
  
  bool EnvironmentWrapper3DKIN::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle)
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int EnvironmentWrapper3DKIN::
  SetStart(std_msgs::Pose2DFloat32 const & start)
  {
#warning 'transform global to map frame? or is that the same?'
    return env_->SetStart(start.x, start.y, start.th);
  }
  
  
  int EnvironmentWrapper3DKIN::
  SetGoal(std_msgs::Pose2DFloat32 const & goal)
  {
#warning 'transform global to map frame? or is that the same?'
    return env_->SetGoal(goal.x, goal.y, goal.th);
  }
  
  
  std_msgs::Pose2DFloat32 EnvironmentWrapper3DKIN::
  GetPoseFromState(int stateID) const
    throw(invalid_state)
  {
    if (0 > stateID)
      throw invalid_state("EnvironmentWrapper3D::GetPoseFromState()", stateID);
    int ix, iy, ith;
    env_->GetCoordFromState(stateID, ix, iy, ith);
    // we know stateID is valid, thus we can ignore the
    // PoseDiscToCont() retval
    double px, py, pth;
    env_->PoseDiscToCont(ix, iy, ith, px, py, pth);
    std_msgs::Pose2DFloat32 pose;
    pose.x = px;
    pose.y = py;
    pose.th = pth;
    return pose;
  }
  
  
  int EnvironmentWrapper3DKIN::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    int ix, iy, ith;
    if ( ! env_->PoseContToDisc(pose.x, pose.y, pose.th, ix, iy, ith))
      return -1;
    return env_->GetStateFromCoord(ix, iy, ith);
  }
  
  
  std::string EnvironmentWrapper3DKIN::
  getName() const
  {
    std::string name("3DKIN");
    return name;
  }
  
}
}
