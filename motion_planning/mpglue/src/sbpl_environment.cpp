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

#include "sbpl_environment.h"
#include <costmap_2d/costmap_2d.h>
#include <sbpl/headers.h>

using namespace mpglue;
using namespace std;

namespace {
  
  string mk_invalid_state_str(string const & method, int state) {
    ostringstream os;
    os << "Environment::invalid_state in method " << method << ": state = " << state;
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
  
  
  class SBPLEnvironment2D
    : public SBPLEnvironment
  {
  public:
    SBPLEnvironment2D(boost::shared_ptr<Costmap> cm,
		      boost::shared_ptr<IndexTransform const> it,
		      int obsthresh);
    virtual ~SBPLEnvironment2D();
    
    virtual DiscreteSpaceInformation * getDSI();
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    virtual bool IsWithinMapCell(int ix, int iy) const;
    virtual unsigned char GetMapCost(int ix, int iy) const;
    virtual bool IsObstacle(int ix, int iy, bool outside_map_is_obstacle = false) const;
    virtual int SetStart(double px, double py, double pth);
    virtual int SetGoal(double px, double py, double pth);
    virtual void SetGoalTolerance(double tol_xy, double tol_th);
    virtual std_msgs::Pose2DFloat32 GetPoseFromState(int stateID) const throw(invalid_state);
    virtual int GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const;
    virtual std::string getName() const;
    
  protected:
    virtual bool DoUpdateCost(int ix, int iy, unsigned char newcost);
    virtual StateChangeQuery const * createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const;
    
    /** \note This is mutable because GetStateFromPose() can
	conceivable change the underlying EnvironmentNAV2D, which we
	don't care about here. */
    mutable EnvironmentNAV2D * env_;
  };
  
  
  /** Wraps an EnvironmentNAV3DKIN instance (which it construct and owns
      for you). */
  class SBPLEnvironment3DKIN
    : public SBPLEnvironment
  {
  public:
    SBPLEnvironment3DKIN(boost::shared_ptr<Costmap> cm,
			 boost::shared_ptr<IndexTransform const> it,
			 int obst_cost_thresh,
			 footprint_t const & footprint,
			 double nominalvel_mpersecs,
			 double timetoturn45degsinplace_secs);
    virtual ~SBPLEnvironment3DKIN();
    
    virtual DiscreteSpaceInformation * getDSI();
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    virtual bool IsWithinMapCell(int ix, int iy) const;
    virtual unsigned char GetMapCost(int ix, int iy) const;
    virtual bool IsObstacle(int ix, int iy, bool outside_map_is_obstacle = false) const;
    virtual int SetStart(double px, double py, double pth);
    virtual int SetGoal(double px, double py, double pth);
    virtual void SetGoalTolerance(double tol_xy, double tol_th);
    virtual std_msgs::Pose2DFloat32 GetPoseFromState(int stateID) const throw(invalid_state);
    virtual int GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const;
    virtual std::string getName() const;
    
  protected:
    virtual bool DoUpdateCost(int ix, int iy, unsigned char newcost);
    virtual StateChangeQuery const * createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const;
    
    int obst_cost_thresh_;
    
    /** \note This is mutable because GetStateFromPose() can
	conceivable change the underlying EnvironmentNAV3DKIN, which we
	don't care about here. */
    mutable EnvironmentNAV3DKIN * env_;
  };
  
}


namespace mpglue {
  
  
  SBPLEnvironment::invalid_state::
  invalid_state(std::string const & method, int state)
    : std::runtime_error(mk_invalid_state_str(method, state))
  {
  }
  
  
  SBPLEnvironment::
  SBPLEnvironment(boost::shared_ptr<Costmap> cm,
		  boost::shared_ptr<IndexTransform const> it)
    : cm_(cm),
      it_(it)
  {
  }
  
  
  SBPLEnvironment::
  ~SBPLEnvironment()
  {
  }
  
  
  bool SBPLEnvironment::
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
  
  
  bool SBPLEnvironment::
  HavePendingCostUpdates() const
  {
    return ! changedcellsV_.empty();
  }
  
  
  void SBPLEnvironment::
  FlushCostUpdates(SBPLPlanner * planner)
  {
    if (changedcellsV_.empty())
      return;

    // XXXX to do: what a hack...
    StateChangeQuery const * ccg(createStateChangeQuery(changedcellsV_));
    planner->costs_changed(*ccg);
    delete ccg;
    changedcellsV_.clear();
  }
  
  
  SBPLEnvironment * create2DEnvironment(boost::shared_ptr<Costmap> cm,
					boost::shared_ptr<IndexTransform const> it,
					int obst_cost_thresh)
  {
    SBPLEnvironment2D * env(new SBPLEnvironment2D(cm, it, obst_cost_thresh));
    return env;
  }
  
  
  SBPLEnvironment * create3DKINEnvironment(boost::shared_ptr<Costmap> cm,
					   boost::shared_ptr<IndexTransform const> it,
					   int obst_cost_thresh,
					   footprint_t const & footprint,
					   double nominalvel_mpersecs,
					   double timetoturn45degsinplace_secs)
  {
    SBPLEnvironment3DKIN * env(new SBPLEnvironment3DKIN(cm, it, obst_cost_thresh,
							footprint, nominalvel_mpersecs,
							timetoturn45degsinplace_secs));
    return env;
  }

}

namespace {
  
  SBPLEnvironment2D::
  SBPLEnvironment2D(boost::shared_ptr<Costmap> cm,
		    boost::shared_ptr<IndexTransform const> it,
		    int obsthresh)
    : SBPLEnvironment(cm, it),
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
			obsthresh);
    
    // as above, assume getXBegin() and getYBegin() are always zero
    for (ssize_t ix(0); ix < cm->getXEnd(); ++ix)
      for (ssize_t iy(0); iy < cm->getYEnd(); ++iy) {
	int cost;
	if (cm->getCost(ix, iy, &cost))	// "always" succeeds though
	  env_->UpdateCost(ix, iy, cost);
      }
  }
  
  
  SBPLEnvironment2D::
  ~SBPLEnvironment2D()
  {
    delete env_;
  }
  
  
  DiscreteSpaceInformation * SBPLEnvironment2D::
  getDSI()
  {
    return env_;
  }
  
  
  bool SBPLEnvironment2D::
  InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    return env_->InitializeMDPCfg(MDPCfg);
  }
  
  
  bool SBPLEnvironment2D::
  IsWithinMapCell(int ix, int iy) const
  {
    return env_->IsWithinMapCell(ix, iy);
  }
  
  
  bool SBPLEnvironment2D::
  DoUpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV2D::SetStart()
      return false;
    return env_->UpdateCost(ix, iy, newcost);
  }


  StateChangeQuery const * SBPLEnvironment2D::
  createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const
  {
    return new myStateChangeQuery<EnvironmentNAV2D>(env_, changedcellsV);
  }
  
  
  unsigned char SBPLEnvironment2D::
  GetMapCost(int ix, int iy) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return costmap_2d::CostMap2D::NO_INFORMATION;
    return env_->GetMapCost(ix, iy);
  }
  
  
  bool SBPLEnvironment2D::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int SBPLEnvironment2D::
  SetStart(double px, double py, double pth)
  {
    ssize_t ix, iy;
    it_->globalToIndex(px, py, &ix, &iy);
    return env_->SetStart(ix, iy);
  }
  
  
  int SBPLEnvironment2D::
  SetGoal(double px, double py, double pth)
  {
    ssize_t ix, iy;
    it_->globalToIndex(px, py, &ix, &iy);
    return env_->SetGoal(ix, iy);
  }
  
  
  void SBPLEnvironment2D::
  SetGoalTolerance(double tol_xy, double tol_th)
  {
    env_->SetGoalTolerance(tol_xy, tol_xy, tol_th);
  }
  
  
  /** \note Always sets pose.th == -42 so people can detect that it is
      undefined. */
  std_msgs::Pose2DFloat32 SBPLEnvironment2D::
  GetPoseFromState(int stateID) const
    throw(invalid_state)
  {
    if (0 > stateID)
      throw invalid_state("SBPLEnvironment2D::GetPoseFromState()", stateID);
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
  
  
  int SBPLEnvironment2D::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    ssize_t ix, iy;
    it_->globalToIndex(pose.x, pose.y, &ix, &iy);
    if ( ! env_->IsWithinMapCell(ix, iy))
      return -1;
    return env_->GetStateFromCoord(ix, iy);
  }
  
  
  std::string SBPLEnvironment2D::
  getName() const
  {
    std::string name("2D");
    return name;
  }
  
  
  SBPLEnvironment3DKIN::
  SBPLEnvironment3DKIN(boost::shared_ptr<Costmap> cm,
		       boost::shared_ptr<IndexTransform const> it,
		       int obst_cost_thresh,
		       footprint_t const & footprint,
		       double nominalvel_mpersecs,
		       double timetoturn45degsinplace_secs)
    : SBPLEnvironment(cm, it),
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
    // SBPLEnvironment2D ctor, and
    // SBPLEnvironment3DKIN::DoUpdateCost()...
    env_->InitializeEnv(cm->getXEnd(), // width
			cm->getYEnd(), // height
			0,	// mapdata
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
  
  
  SBPLEnvironment3DKIN::
  ~SBPLEnvironment3DKIN()
  {
    delete env_;
  }
  
  
  DiscreteSpaceInformation * SBPLEnvironment3DKIN::
  getDSI()
  {
    return env_;
  }
  
  
  bool SBPLEnvironment3DKIN::
  InitializeMDPCfg(MDPConfig *MDPCfg)
  {
    return env_->InitializeMDPCfg(MDPCfg);
  }
  
  
  bool SBPLEnvironment3DKIN::
  IsWithinMapCell(int ix, int iy) const
  {
    return env_->IsWithinMapCell(ix, iy);
  }
  
  
  /**
     \note Remapping the cost to binary obstacle info {0,1} actually
     confuses the check for actually changed costs in the
     SBPLEnvironment::UpdateCost() method. However, this is bound
     to change as soon as the 3DKIN environment starts dealing with
     uniform obstacle costs.
  */
  bool SBPLEnvironment3DKIN::
  DoUpdateCost(int ix, int iy, unsigned char newcost)
  {
    if ( ! env_->IsWithinMapCell(ix, iy)) // should be done inside EnvironmentNAV3DKIN::UpdateCost()
      return false;
    if (obst_cost_thresh_ <= newcost)
      return env_->UpdateCost(ix, iy, 1); // see \note comment above if you change this!
    return env_->UpdateCost(ix, iy, 0); // see \note comment above if you change this!
  }


  StateChangeQuery const * SBPLEnvironment3DKIN::
  createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const
  {
    return new myStateChangeQuery<EnvironmentNAV3DKIN>(env_, changedcellsV);
  }
  
  
  unsigned char SBPLEnvironment3DKIN::
  GetMapCost(int ix, int iy) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return costmap_2d::CostMap2D::NO_INFORMATION;
    if (env_->IsObstacle(ix, iy))
      return costmap_2d::CostMap2D::LETHAL_OBSTACLE;
    return 0;
  }
  
  
  bool SBPLEnvironment3DKIN::
  IsObstacle(int ix, int iy, bool outside_map_is_obstacle) const
  {
    if ( ! env_->IsWithinMapCell(ix, iy))
      return outside_map_is_obstacle;
    return env_->IsObstacle(ix, iy);
  }
  
  
  int SBPLEnvironment3DKIN::
  SetStart(double px, double py, double pth)
  {
    // assume global and map frame are the same
    return env_->SetStart(px, py, pth);
  }
  
  
  int SBPLEnvironment3DKIN::
  SetGoal(double px, double py, double pth)
  {
    // assume global and map frame are the same
    return env_->SetGoal(px, py, pth);
  }
  
  
  void SBPLEnvironment3DKIN::
  SetGoalTolerance(double tol_xy, double tol_th)
  {
    env_->SetGoalTolerance(tol_xy, tol_xy, tol_th);
  }
  
  
  std_msgs::Pose2DFloat32 SBPLEnvironment3DKIN::
  GetPoseFromState(int stateID) const
    throw(invalid_state)
  {
    if (0 > stateID)
      throw invalid_state("SBPLEnvironment3D::GetPoseFromState()", stateID);
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
  
  
  int SBPLEnvironment3DKIN::
  GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const
  {
    int ix, iy, ith;
    if ( ! env_->PoseContToDisc(pose.x, pose.y, pose.th, ix, iy, ith))
      return -1;
    return env_->GetStateFromCoord(ix, iy, ith);
  }
  
  
  std::string SBPLEnvironment3DKIN::
  getName() const
  {
    std::string name("3DKIN");
    return name;
  }
  
}
