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

#include "environment.h"
#include <costmap_2d/costmap_2d.h>
#include <sbpl/headers.h>

using namespace std;


namespace {
  
  string mk_invalid_state_str(string const & method, int state) {
    ostringstream os;
    os << "Environment::invalid_state in method " << method << ": state = " << state;
    return os.str();
  }
  
  template<typename env_type>
  class myChangedCellsGetter
    : public ChangedCellsGetter
  {
  public:
    myChangedCellsGetter(env_type * env,
			 std::vector<nav2dcell_t> const & changedcellsV)
      : env_(env),
	changedcellsV_(changedcellsV)
    {
    }
    
    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const * getPredsOfChangedCells() const
    {
      if (predsOfChangedCells_.empty() && ( ! changedcellsV_.empty()))
	env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }
    
    env_type * env_;
    std::vector<nav2dcell_t> const & changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
  };
  
}


namespace mpglue {
  
  
  Environment::invalid_state::
  invalid_state(std::string const & method, int state)
    : std::runtime_error(mk_invalid_state_str(method, state))
  {
  }
  
  
  Environment::
  Environment(boost::shared_ptr<Costmap> cm,
	      boost::shared_ptr<IndexTransform const> it)
    : cm_(cm),
      it_(it)
  {
  }
  
  
  Environment::
  ~Environment()
  {
  }
  
  
  bool Environment::
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
  
  
  void Environment::
  FlushCostUpdates(SBPLPlanner * planner)
  {
    if (changedcellsV_.empty())
      return;

#warning 'what a hack...'
    ChangedCellsGetter const * ccg(createChangedCellsGetter(changedcellsV_));
    planner->costs_changed(*ccg);
    delete ccg;
    changedcellsV_.clear();
  }
  
  
  Environment2D::
  Environment2D(boost::shared_ptr<Costmap> cm,
		boost::shared_ptr<IndexTransform const> it,
		int startx, int starty,
		int goalx, int goaly,
		unsigned char obsthresh)
    : Environment(cm, it),
      env_(new EnvironmentNAV2D())
  {
    // good: Take advantage of the fact that InitializeEnv() can take
    // a NULL-pointer as mapdata in order to initialize to all
    // freespace.
    //
    // bad: Most costmaps do not support negative grid indices, so the
    // generic Costmap::getXBegin() and getYBegin() are ignored
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
	  env_->UpdateCost(ix, iy, cost);
      }
  }
  
  
  Environment2D::
  ~Environment2D()
  {
    delete env_;
  }
  
  
  DiscreteSpaceInformation * Environment2D::
  getDSI()
  {
    return env_;
  }
  
  
  bool Environment2D::
  InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    return env_->InitializeMDPCfg(MDPCfg);
  }
  
  
  bool Environment2D::
  IsWithinMapCell(int ix, int iy) const
  {
    return env_->IsWithinMapCell(ix, iy);
  }
  
  
  bool Environment2D::
  DoUpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV2D::SetStart()
      return false;
    return env_->UpdateCost(ix, iy, newcost);
  }


  ChangedCellsGetter const * Environment2D::
  createChangedCellsGetter(std::vector<nav2dcell_t> const & changedcellsV) const
  {
    return new myChangedCellsGetter<EnvironmentNAV2D>(env_, changedcellsV);
  }
  
  
  unsigned char Environment2D::
  GetMapCost(int ix, int iy) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return costmap_2d::CostMap2D::NO_INFORMATION;
    return env_->GetMapCost(ix, iy);
  }
  
  
  bool Environment2D::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int Environment2D::
  SetStart(double px, double py, double pth)
  {
    ssize_t ix, iy;
    it_->globalToIndex(px, py, &ix, &iy);
    return env_->SetStart(ix, iy);
  }
  
  
  int Environment2D::
  SetGoal(double px, double py, double pth)
  {
    ssize_t ix, iy;
    it_->globalToIndex(px, py, &ix, &iy);
    return env_->SetGoal(ix, iy);
  }
  
  
  /** \note Always sets pose.th == -42 so people can detect that it is
      undefined. */
  std_msgs::Pose2DFloat32 Environment2D::
  GetPoseFromState(int stateID) const
    throw(invalid_state)
  {
    if (0 > stateID)
      throw invalid_state("Environment2D::GetPoseFromState()", stateID);
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
  
  
  int Environment2D::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    ssize_t ix, iy;
    it_->globalToIndex(pose.x, pose.y, &ix, &iy);
    if ( ! env_->IsWithinMapCell(ix, iy))
      return -1;
    return env_->GetStateFromCoord(ix, iy);
  }
  
  
  std::string Environment2D::
  getName() const
  {
    std::string name("2D");
    return name;
  }
  
  
  Environment3DKIN::
  Environment3DKIN(boost::shared_ptr<Costmap> cm,
		   boost::shared_ptr<IndexTransform const> it,
		   unsigned char obst_cost_thresh,
		   double startx, double starty, double starttheta,
		   double goalx, double goaly, double goaltheta,
		   double goaltol_x, double goaltol_y, double goaltol_theta,
		   footprint_t const & footprint,
		   double nominalvel_mpersecs,
		   double timetoturn45degsinplace_secs)
    : Environment(cm, it),
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
    // generic Costmap::getXBegin() and getYBegin() are ignored
    // and simply assumed to always return 0 (which they won't if we
    // use growable costmaps).
    //
    // also there is quite a bit of code duplication between this, the
    // Environment2D ctor, and
    // Environment3DKIN::DoUpdateCost()...
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
  
  
  Environment3DKIN::
  ~Environment3DKIN()
  {
    delete env_;
  }
  
  
  DiscreteSpaceInformation * Environment3DKIN::
  getDSI()
  {
    return env_;
  }
  
  
  bool Environment3DKIN::
  InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    return env_->InitializeMDPCfg(MDPCfg);
  }
  
  
  bool Environment3DKIN::
  IsWithinMapCell(int ix, int iy) const
  {
    return env_->IsWithinMapCell(ix, iy);
  }
  
  
  /**
     \note Remapping the cost to binary obstacle info {0,1} actually
     confuses the check for actually changed costs in the
     Environment::UpdateCost() method. However, this is bound
     to change as soon as the 3DKIN environment starts dealing with
     uniform obstacle costs.
  */
  bool Environment3DKIN::
  DoUpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV3DKIN::UpdateCost()
      return false;
    if (obst_cost_thresh_ <= newcost)
      return env_->UpdateCost(ix, iy, 1); // see \note comment above if you change this!
    return env_->UpdateCost(ix, iy, 0); // see \note comment above if you change this!
  }


  ChangedCellsGetter const * Environment3DKIN::
  createChangedCellsGetter(std::vector<nav2dcell_t> const & changedcellsV) const
  {
    return new myChangedCellsGetter<EnvironmentNAV3DKIN>(env_, changedcellsV);
  }
  
  
  unsigned char Environment3DKIN::
  GetMapCost(int ix, int iy) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return costmap_2d::CostMap2D::NO_INFORMATION;
    if (env_->IsObstacle(ix, iy))
      return costmap_2d::CostMap2D::LETHAL_OBSTACLE;
    return 0;
  }
  
  
  bool Environment3DKIN::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int Environment3DKIN::
  SetStart(double px, double py, double pth)
  {
    // assume global and map frame are the same
    return env_->SetStart(px, py, pth);
  }
  
  
  int Environment3DKIN::
  SetGoal(double px, double py, double pth)
  {
    // assume global and map frame are the same
    return env_->SetGoal(px, py, pth);
  }
  
  
  std_msgs::Pose2DFloat32 Environment3DKIN::
  GetPoseFromState(int stateID) const
    throw(invalid_state)
  {
    if (0 > stateID)
      throw invalid_state("Environment3D::GetPoseFromState()", stateID);
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
  
  
  int Environment3DKIN::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    int ix, iy, ith;
    if ( ! env_->PoseContToDisc(pose.x, pose.y, pose.th, ix, iy, ith))
      return -1;
    return env_->GetStateFromCoord(ix, iy, ith);
  }
  
  
  std::string Environment3DKIN::
  getName() const
  {
    std::string name("3DKIN");
    return name;
  }
  
  
  std::string canonicalEnvironmentName(std::string const & name_or_alias)
  {
    static map<string, string> environment_alias;
    if (environment_alias.empty()) {
      environment_alias.insert(make_pair("2D",    "2D"));
      environment_alias.insert(make_pair("2d",    "2D"));
      environment_alias.insert(make_pair("2",     "2D"));
      environment_alias.insert(make_pair("3DKIN", "3DKIN"));
      environment_alias.insert(make_pair("3D",    "3DKIN"));
      environment_alias.insert(make_pair("3d",    "3DKIN"));
      environment_alias.insert(make_pair("3",     "3DKIN"));
    }
    
    map<string, string>::const_iterator is(environment_alias.find(name_or_alias));
    if (environment_alias.end() == is)
      return "";
    return is->second;
  }
  
}
