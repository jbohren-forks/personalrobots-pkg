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

#include "environment_wrap.h"
#include <costmap_2d/costmap_2d.h>
#include <sbpl/headers.h>

using namespace std;


namespace {
  
  string mk_invalid_state_str(string const & method, int state) {
    ostringstream os;
    os << "EnvironmentWrapper::invalid_state in method " << method << ": state = " << state;
    return os.str();
  }
  
  template<typename env_type>
  class myStateChangeQuery
    : public StateChangeQuery
  {
  public:
    myStateChangeQuery(env_type * env,
			 std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env),
	changedcellsV_(changedcellsV)
    {
    }
    
    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredecessors() const
    {
      if (predsOfChangedCells_.empty() && ( ! changedcellsV_.empty()))
	env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }
    
    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getSuccessors() const
    {
      if (succsOfChangedCells_.empty() && ( ! changedcellsV_.empty()))
	env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }
    
    env_type * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
  };
  
}


namespace ompl {
  
  
  EnvironmentWrapper::invalid_state::
  invalid_state(std::string const & method, int state)
    : std::runtime_error(mk_invalid_state_str(method, state))
  {
  }
  
  
  EnvironmentWrapper::
  EnvironmentWrapper(CostmapWrap * cm,
		     bool own_cm,
		     IndexTransformWrap const * it,
		     bool own_it)
    : cm_(cm),
      own_cm_(own_cm),
      it_(it),
      own_it_(own_it)
  {
  }
  
  
  EnvironmentWrapper::
  ~EnvironmentWrapper()
  {
    if (own_it_)
      delete it_;
    if (own_cm_)
      delete cm_;
  }
  
  
  bool EnvironmentWrapper::
  UpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! IsWithinMapCell(ix, iy))
      return false;
    unsigned char const oldcost(GetMapCost(ix, iy));
    if (oldcost == newcost)
      return true;
    if ( ! DoUpdateCost(ix, iy, newcost))
      return false;
    changedcellsV_.push_back(nav2dcell_t());
    changedcellsV_.back().x = ix;
    changedcellsV_.back().y = iy;
    return true;
  }
  
  
  void EnvironmentWrapper::
  FlushCostUpdates(SBPLPlanner * planner)
  {
    if (changedcellsV_.empty())
      return;

#warning 'what a hack...'
    StateChangeQuery const * ccg(createStateChangeQuery(changedcellsV_));
    planner->costs_changed(*ccg);
    delete ccg;
    changedcellsV_.clear();
  }
  
  
  EnvironmentWrapper2D::
  EnvironmentWrapper2D(CostmapWrap * cm,
		       bool own_cm,
		       IndexTransformWrap const * it,
		       bool own_it,
		       int startx, int starty,
		       int goalx, int goaly,
		       unsigned char obsthresh)
    : EnvironmentWrapper(cm, own_cm, it, own_it),
      env_(new EnvironmentNAV2D())
  {
    // good: Take advantage of the fact that InitializeEnv() can take
    // a NULL-pointer as mapdata in order to initialize to all
    // freespace.
    //
    // bad: Most costmaps do not support negative grid indices, so the
    // generic CostmapWrap::getXBegin() and getYBegin() are ignored
    // and simply assumed to always return 0 (which they won't if we
    // use growable costmaps).
    env_->InitializeEnv(cm->getXEnd(), // width
			cm->getYEnd(), // height
			0,	// mapdata
			startx, starty, goalx, goaly, obsthresh);
    
    // as above, assume getXBegin() and getYBegin() are always zero
    for (ssize_t ix(0); ix < cm->getXEnd(); ++ix)
      for (ssize_t iy(0); iy < cm->getYEnd(); ++iy) {
	int cost;
	if (cm->getCost(ix, iy, &cost))	// "always" succeeds though
	  env_->UpdateCost(ix, iy, rint(cost));
      }
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
  IsWithinMapCell(int ix, int iy) const
  {
    return env_->IsWithinMapCell(ix, iy);
  }
  
  
  bool EnvironmentWrapper2D::
  DoUpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV2D::SetStart()
      return false;
    return env_->UpdateCost(ix, iy, newcost);
  }


  StateChangeQuery const * EnvironmentWrapper2D::
  createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const
  {
    return new myStateChangeQuery<EnvironmentNAV2D>(env_, changedcellsV);
  }
  
  
  unsigned char EnvironmentWrapper2D::
  GetMapCost(int ix, int iy) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return costmap_2d::CostMap2D::NO_INFORMATION;
    return env_->GetMapCost(ix, iy);
  }
  
  
  bool EnvironmentWrapper2D::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int EnvironmentWrapper2D::
  SetStart(std_msgs::Pose2DFloat32 const & start)
  {
    ssize_t ix, iy;
    it_->globalToIndex(start.x, start.y, &ix, &iy);
    return env_->SetStart(ix, iy);
  }
  
  
  int EnvironmentWrapper2D::
  SetGoal(std_msgs::Pose2DFloat32 const & goal)
  {
    ssize_t ix, iy;
    it_->globalToIndex(goal.x, goal.y, &ix, &iy);
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
    it_->indexToGlobal(ix, iy, &px, &py);
    std_msgs::Pose2DFloat32 pose;
    pose.x = px;
    pose.y = py;
    pose.th = -42;
    return pose;
  }
  
  
  int EnvironmentWrapper2D::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    ssize_t ix, iy;
    it_->globalToIndex(pose.x, pose.y, &ix, &iy);
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
  EnvironmentWrapper3DKIN(CostmapWrap * cm,
			  bool own_cm,
			  IndexTransformWrap const * it,
			  bool own_it,
			  unsigned char obst_cost_thresh,
			  double startx, double starty, double starttheta,
			  double goalx, double goaly, double goaltheta,
			  double goaltol_x, double goaltol_y, double goaltol_theta,
			  footprint_t const & footprint,
			  double nominalvel_mpersecs,
			  double timetoturn45degsinplace_secs)
    : EnvironmentWrapper(cm, own_cm, it, own_it),
      obst_cost_thresh_(obst_cost_thresh),
      env_(new EnvironmentNAV3DKIN())
  {
    vector<sbpl_2Dpt_t> perimeterptsV;
    perimeterptsV.reserve(footprint.size());
    for (size_t ii(0); ii < footprint.size(); ++ii) {
      sbpl_2Dpt_t pt;
      pt.x = footprint[ii].x;
      pt.y = footprint[ii].y;
      perimeterptsV.push_back(pt);
    }
    
    // good: Take advantage of the fact that InitializeEnv() can take
    // a NULL-pointer as mapdata in order to initialize to all
    // freespace.
    //
    // bad: Most costmaps do not support negative grid indices, so the
    // generic CostmapWrap::getXBegin() and getYBegin() are ignored
    // and simply assumed to always return 0 (which they won't if we
    // use growable costmaps).
    //
    // also there is quite a bit of code duplication between this, the
    // EnvironmentWrapper2D ctor, and
    // EnvironmentWrapper3DKIN::DoUpdateCost()...
    env_->InitializeEnv(cm->getXEnd(), // width
			cm->getYEnd(), // height
			0,	// mapdata
			startx, starty, starttheta,
			goalx, goaly, goaltheta,
			goaltol_x, goaltol_y, goaltol_theta,
			perimeterptsV, it->getResolution(), nominalvel_mpersecs,
			timetoturn45degsinplace_secs, obst_cost_thresh);
    
    // as above, assume getXBegin() and getYBegin() are always zero
    for (ssize_t ix(0); ix < cm->getXEnd(); ++ix)
      for (ssize_t iy(0); iy < cm->getYEnd(); ++iy) {
	int cost;
	if (cm->getCost(ix, iy, &cost))	// "always" succeeds though
	  env_->UpdateCost(ix, iy, cost);
      }
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
  IsWithinMapCell(int ix, int iy) const
  {
    return env_->IsWithinMapCell(ix, iy);
  }
  
  
  /**
     \note Remapping the cost to binary obstacle info {0,1} actually
     confuses the check for actually changed costs in the
     EnvironmentWrapper::UpdateCost() method. However, this is bound
     to change as soon as the 3DKIN environment starts dealing with
     uniform obstacle costs.
  */
  bool EnvironmentWrapper3DKIN::
  DoUpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV3DKIN::UpdateCost()
      return false;
    if (obst_cost_thresh_ <= newcost)
      return env_->UpdateCost(ix, iy, 1); // see \note comment above if you change this!
    return env_->UpdateCost(ix, iy, 0); // see \note comment above if you change this!
  }


  StateChangeQuery const * EnvironmentWrapper3DKIN::
  createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const
  {
    return new myStateChangeQuery<EnvironmentNAV3DKIN>(env_, changedcellsV);
  }
  
  
  unsigned char EnvironmentWrapper3DKIN::
  GetMapCost(int ix, int iy) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return costmap_2d::CostMap2D::NO_INFORMATION;
    if (env_->IsObstacle(ix, iy))
      return costmap_2d::CostMap2D::LETHAL_OBSTACLE;
    return 0;
  }
  
  
  bool EnvironmentWrapper3DKIN::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int EnvironmentWrapper3DKIN::
  SetStart(std_msgs::Pose2DFloat32 const & start)
  {
    // assume global and map frame are the same
    return env_->SetStart(start.x, start.y, start.th);
  }
  
  
  int EnvironmentWrapper3DKIN::
  SetGoal(std_msgs::Pose2DFloat32 const & goal)
  {
    // assume global and map frame are the same
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
  
  
  std::string canonicalEnvironmentName(std::string const & name_or_alias)
  {
    static map<string, string> environment_alias;
    if (environment_alias.empty()) {
      environment_alias.insert(make_pair("2D", "2D"));
      environment_alias.insert(make_pair("2d", "2D"));
      environment_alias.insert(make_pair("2",  "2D"));
      environment_alias.insert(make_pair("3D", "3DKIN"));
      environment_alias.insert(make_pair("3d", "3DKIN"));
      environment_alias.insert(make_pair("3",  "3DKIN"));
    }
    
    map<string, string>::const_iterator is(environment_alias.find(name_or_alias));
    if (environment_alias.end() == is)
      return "";
    return is->second;
  }
  
}
