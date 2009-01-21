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

#ifndef OMPL_ENVIRONMENT_WRAP_HPP
#define OMPL_ENVIRONMENT_WRAP_HPP

// should say #include <sbpl_util/costmap_wrap.h> but this requires
// changes in the code layout and maybe in the export statement in
// manifest.xml... to do later
#include "costmap_wrap.h"
#include "footprint.h"

#include <std_msgs/Pose2DFloat32.h>
#include <stdexcept>
#include <string>
#include <vector>

class SBPLPlanner;		/**< see motion_planning/sbpl/src/planners/planner.h */
class DiscreteSpaceInformation; /**< see motion_planning/sbpl/src/discrete_space_information/environment.h */
class EnvironmentNAV2D;	        /**< see motion_planning/sbpl/src/discrete_space_information/nav2d/environment_nav2D.h */
class EnvironmentNAV3DKIN;      /**< see motion_planning/sbpl/src/discrete_space_information/nav3dkin/environment_nav3Dkin.h */
class StateChangeQuery;

// would like to forward-declare, but in mdpconfig.h it's a typedef'ed
// anonymous struct and GCC chokes on that... great
//   struct MDPConfig; /**< see motion_planning/sbpl/src/utils/mdpconfig.h */
#include <utils/mdpconfig.h>

// Would like to forward-declare, but nav2dcell_t is used within a
// std::vector<> ... see also the comments in
// sbpl/src/planners/planner.h about the StateChangeQuery
// class. Also, environment_nav2D.h needs some other includes to be
// present and uses std::vector without the std:: prefix, so we
// unfortunately have to add a using directive here.
using std::vector;
#include <utils/mdp.h>
#include <discrete_space_information/environment.h>
#include <discrete_space_information/nav2d/environment_nav2D.h>

namespace ompl {
  
  
  std::string canonicalEnvironmentName(std::string const & name_or_alias);
  
  
  /** Helper class for abstracting away the usage (or not) of the
      robot's heading during planning. Represents a common (tweaked)
      subset of the DiscreteSpaceInformation-subclasses that are
      employed by SBPLPlanner subclasses. */
  class EnvironmentWrapper
  {
  public:
    ////     struct invalid_pose: public std::runtime_error
    ////     { invalid_pose(std::string const & method, std_msgs::Pose2DFloat32 const & pose); };
    
    struct invalid_state: public std::runtime_error
    { invalid_state(std::string const & method, int state); };
    
    EnvironmentWrapper(CostmapWrap * cm,
		       /** whether ~EnvironmentWrapper() should delete the CostmapWrap * cm */
		       bool own_cm,
		       IndexTransformWrap const * it,
		       /** whether ~EnvironmentWrapper() should delete the IndexTransformWrap const * it */
		       bool own_it);
    virtual ~EnvironmentWrapper();
    
    virtual DiscreteSpaceInformation * getDSI() = 0;
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg) = 0;
    
    /** Delegate to DoUpdateCost() and add the changed cell to
	changedcellsV_ if (i) it is a valid coordinate and (ii) the
	cost has actually changed.
	
	\return false if (ix,iy) is not in the map, or if the
	delegated cost update failed. It returns true even if the
	newcost is the same as the old cost at that cell, but then it
	does not put the cell onto the changedcellsV_. */
    bool UpdateCost(int ix, int iy, unsigned char newcost);
    
    /** If there are any pending cost updates, it calls
	SBPLPlanner::costs_changed() and then clears that buffer. */
    void FlushCostUpdates(SBPLPlanner * planner);
    
    /** \return true if the cell (ix,iy) lies within the bounds of the
	underlying costmap. */
    virtual bool IsWithinMapCell(int ix, int iy) const = 0;
    
    /** \return NO_INFORMATION if ix, iy not in the map. otherwise the cell value */  
    virtual unsigned char GetMapCost(int ix, int iy) const = 0;

    /** \return true if there is no obstacle at (ix,iy)... if (ix,iy)
	is not in the map, then outside_map_is_obstacle is
	returned. */
    virtual bool IsObstacle(int ix, int iy, bool outside_map_is_obstacle = false) const = 0;
    
    /** \return The stateID of the start, or -1 if it lies outside the map. */
    virtual int SetStart(std_msgs::Pose2DFloat32 const & start) = 0;
    
    /** \return The stateID of the goal, or -1 if it lies outside the map. */
    virtual int SetGoal(std_msgs::Pose2DFloat32 const & goal) = 0;
    
    virtual std_msgs::Pose2DFloat32 GetPoseFromState(int stateID) const throw(invalid_state) = 0;
    
    /** \return the stateID of a pose, or -1 if the pose lies outside
	of the map. */
    virtual int GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const = 0;
    
    virtual std::string getName() const = 0;
    
  protected:
    virtual bool DoUpdateCost(int ix, int iy, unsigned char newcost) = 0;
    
    virtual StateChangeQuery const * createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const = 0;
    
    CostmapWrap * cm_;
    bool const own_cm_;
    IndexTransformWrap const * it_;
    bool const own_it_;

  private:
    std::vector<nav2dcell_t> changedcellsV_;
  };
  
  
  /** Wraps an EnvironmentNAV2D instance (which it construct and owns
      for you). */
  class EnvironmentWrapper2D
    : public EnvironmentWrapper
  {
  public:
    EnvironmentWrapper2D(CostmapWrap * cm,
			 bool own_cm,
			 IndexTransformWrap const * it,
			 bool own_it,
			 int startx, int starty,
			 int goalx, int goaly,
			 unsigned char obsthresh);
    virtual ~EnvironmentWrapper2D();
    
    virtual DiscreteSpaceInformation * getDSI();
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    virtual bool IsWithinMapCell(int ix, int iy) const;
    virtual unsigned char GetMapCost(int ix, int iy) const;
    virtual bool IsObstacle(int ix, int iy, bool outside_map_is_obstacle = false) const;
    virtual int SetStart(std_msgs::Pose2DFloat32 const & start);
    virtual int SetGoal(std_msgs::Pose2DFloat32 const & goal);
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
  class EnvironmentWrapper3DKIN
    : public EnvironmentWrapper
  {
  public:    
    EnvironmentWrapper3DKIN(CostmapWrap * cm,
			    bool own_cm,
			    IndexTransformWrap const * it,
			    bool own_it,
			    /** Use
				costmap_2d::CostMap2D::LETHAL_OBSTACLE
				for workspace-only obstacles,
				costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE
				for obstacles blown up by the
				inscribed radius, etc */
			    unsigned char obst_cost_thresh,
			    double startx, double starty, double starttheta,
			    double goalx, double goaly, double goaltheta,
			    double goaltol_x, double goaltol_y, double goaltol_theta,
			    footprint_t const & footprint,
			    double nominalvel_mpersecs,
			    double timetoturn45degsinplace_secs);
    virtual ~EnvironmentWrapper3DKIN();
    
    virtual DiscreteSpaceInformation * getDSI();
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);
    
    virtual bool IsWithinMapCell(int ix, int iy) const;
    virtual unsigned char GetMapCost(int ix, int iy) const;
    virtual bool IsObstacle(int ix, int iy, bool outside_map_is_obstacle = false) const;
    virtual int SetStart(std_msgs::Pose2DFloat32 const & start);
    virtual int SetGoal(std_msgs::Pose2DFloat32 const & goal);
    virtual std_msgs::Pose2DFloat32 GetPoseFromState(int stateID) const throw(invalid_state);
    virtual int GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const;
    virtual std::string getName() const;
    
  protected:
    virtual bool DoUpdateCost(int ix, int iy, unsigned char newcost);
    virtual StateChangeQuery const * createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const;
    
    unsigned char obst_cost_thresh_;
    
    /** \note This is mutable because GetStateFromPose() can
	conceivable change the underlying EnvironmentNAV3DKIN, which we
	don't care about here. */
    mutable EnvironmentNAV3DKIN * env_;
  };

}

#endif // OMPL_ENVIRONMENT_WRAP_HPP
