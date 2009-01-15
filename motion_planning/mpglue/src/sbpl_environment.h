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

#ifndef MPGLUE_SBPL_ENVIRONMENT_HPP
#define MPGLUE_SBPL_ENVIRONMENT_HPP

#include <mpglue/costmap.h>
#include <mpglue/footprint.h>
#include <std_msgs/Pose2DFloat32.h>
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <string>
#include <vector>

class SBPLPlanner;		/**< see motion_planning/sbpl/src/planners/planner.h */
class DiscreteSpaceInformation; /**< see motion_planning/sbpl/src/discrete_space_information/environment.h */
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

namespace mpglue {
  
  
  /** Helper class for abstracting away the usage (or not) of the
      robot's heading during planning. Represents a common (tweaked)
      subset of the DiscreteSpaceInformation-subclasses that are
      employed by SBPLPlanner subclasses.
  */
  class SBPLEnvironment
  {
  public:
    ////     struct invalid_pose: public std::runtime_error
    ////     { invalid_pose(std::string const & method, std_msgs::Pose2DFloat32 const & pose); };
    
    struct invalid_state: public std::runtime_error
    { invalid_state(std::string const & method, int state); };
    
    SBPLEnvironment(boost::shared_ptr<Costmap> cm,
		    boost::shared_ptr<IndexTransform const> it);
    virtual ~SBPLEnvironment();
    
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
    virtual int SetStart(double px, double py, double pth) = 0;
    
    /** \note see SetStart(double, double, double) */
    int SetStart(std_msgs::Pose2DFloat32 const & start)
    { return SetStart(start.x, start.y, start.th); }
    
    /** \return The stateID of the goal, or -1 if it lies outside the map. */
    virtual int SetGoal(double px, double py, double pth) = 0;
    
    virtual void SetGoalTolerance(double tol_xy, double tol_th) = 0;
    
    /** \note see SetGoal(double, double, double) */
    int SetGoal(std_msgs::Pose2DFloat32 const & goal)
    { return SetGoal(goal.x, goal.y, goal.th) ; }
    
    virtual std_msgs::Pose2DFloat32 GetPoseFromState(int stateID) const throw(invalid_state) = 0;
    
    /** \return the stateID of a pose, or -1 if the pose lies outside
	of the map. */
    virtual int GetStateFromPose(std_msgs::Pose2DFloat32 const & pose) const = 0;
    
    virtual std::string getName() const = 0;
    
    boost::shared_ptr<Costmap> getCostmap() { return cm_; }
    boost::shared_ptr<Costmap const> getCostmap() const { return cm_; }
    boost::shared_ptr<IndexTransform const> getIndexTransform() const { return it_; }
    
  protected:
    virtual bool DoUpdateCost(int ix, int iy, unsigned char newcost) = 0;
    
    virtual StateChangeQuery const * createStateChangeQuery(std::vector<nav2dcell_t> const & changedcellsV) const = 0;
    
    boost::shared_ptr<Costmap> cm_;
    boost::shared_ptr<IndexTransform const> it_;
    
  private:
    std::vector<nav2dcell_t> changedcellsV_;
  };
  
  
  SBPLEnvironment * create2DEnvironment(boost::shared_ptr<Costmap> cm,
					boost::shared_ptr<IndexTransform const> it,
					int obst_cost_thresh);
  
  SBPLEnvironment * create3DKINEnvironment(boost::shared_ptr<Costmap> cm,
					   boost::shared_ptr<IndexTransform const> it,
					   int obst_cost_thresh,
					   footprint_t const & footprint,
					   double nominalvel_mpersecs,
					   double timetoturn45degsinplace_secs);
  
}

#endif // MPGLUE_SBPL_ENVIRONMENT_HPP
