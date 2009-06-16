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

/** \author Ioan Sucan */

#ifndef PLANNING_ENVIRONMENT_PLANNING_MONITOR_
#define PLANNING_ENVIRONMENT_PLANNING_MONITOR_

#include "planning_environment/collision_space_monitor.h"
#include <motion_planning_msgs/KinematicJoint.h>
#include <motion_planning_msgs/KinematicPath.h>
#include <motion_planning_msgs/KinematicConstraints.h>

namespace planning_environment
{

    /** @b PlanningMonitor is a class which in addition to being aware
	of a robot model, and the collision model is also aware of
	constraints and can check the validity of states and paths.
    */
    
    class PlanningMonitor : public CollisionSpaceMonitor
    {
    public:
	
	PlanningMonitor(CollisionModels *cm, std::string frame_id) : CollisionSpaceMonitor(static_cast<CollisionModels*>(cm), frame_id)
	{
	}
	
	PlanningMonitor(CollisionModels *cm) : CollisionSpaceMonitor(static_cast<CollisionModels*>(cm))
	{
	}
	
	virtual ~PlanningMonitor(void)
	{
	}

	/** Check if the full state of the robot is valid */
	bool isStateValidOnPath(const planning_models::KinematicModel::StateParams *state) const;

	/** Check if the full state of the robot is valid */
	bool isStateValidAtGoal(const planning_models::KinematicModel::StateParams *state) const;
	
	/** Check if the path is valid */
	bool isPathValid(const motion_planning_msgs::KinematicPath &path) const;
	
	/** Set the kinematic constraints the monitor should use when checking a path */
	void setPathConstraints(const motion_planning_msgs::KinematicConstraints &kc);

	/** Set the kinematic constraints the monitor should use when checking a path's last state (the goal) */
	void setGoalConstraints(const motion_planning_msgs::KinematicConstraints &kc);
	
	/** Transform the frames in which constraints are specified to the one requested */
	void transformConstraintsToFrame(motion_planning_msgs::KinematicConstraints &kc, const std::string &target) const;
	
	/** Transform the kinematic path to the frame requested */
	void transformPathToFrame(motion_planning_msgs::KinematicPath &kp, const std::string &target) const;

	/** Transform the kinematic joint to the frame requested */
	void transformJointToFrame(motion_planning_msgs::KinematicJoint &kj, const std::string &target) const;
	
    protected:
	
	/** Transform the joint parameters (if needed) to a target frame */
	void transformJoint(const std::string &name, unsigned int index, std::vector<double> &params, roslib::Header &header, const std::string& target) const;
	
	/** Check the path assuming it is in the frame of the model */
	bool isPathValidAux(const motion_planning_msgs::KinematicPath &path) const;
	
	motion_planning_msgs::KinematicConstraints kcPath_;
	motion_planning_msgs::KinematicConstraints kcGoal_;
    };
    
	
}

#endif
