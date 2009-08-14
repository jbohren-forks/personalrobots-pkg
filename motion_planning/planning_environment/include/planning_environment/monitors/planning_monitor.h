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

#ifndef PLANNING_ENVIRONMENT_MONITORS_PLANNING_MONITOR_
#define PLANNING_ENVIRONMENT_MONITORS_PLANNING_MONITOR_

#include "planning_environment/monitors/collision_space_monitor.h"
#include <motion_planning_msgs/KinematicJoint.h>
#include <motion_planning_msgs/KinematicPath.h>
#include <motion_planning_msgs/KinematicConstraints.h>
#include <iostream>

namespace planning_environment
{

    /** \breif @b PlanningMonitor is a class which in addition to being aware
	of a robot model, and the collision model is also aware of
	constraints and can check the validity of states and paths.
    */    
    class PlanningMonitor : public CollisionSpaceMonitor
    {
    public:
	
	PlanningMonitor(CollisionModels *cm, tf::TransformListener *tf, std::string frame_id) : CollisionSpaceMonitor(static_cast<CollisionModels*>(cm), tf, frame_id)
	{
	    onCollisionContact_ = NULL;
	    maxCollisionContacts_ = 1;
	    loadParams();
	}
	
	PlanningMonitor(CollisionModels *cm, tf::TransformListener *tf) : CollisionSpaceMonitor(static_cast<CollisionModels*>(cm), tf)
	{
	    onCollisionContact_ = NULL;	    
	    maxCollisionContacts_ = 1;
	    loadParams();
	}
	
	virtual ~PlanningMonitor(void)
	{
	}
	
	/** \brief Mask for validity testing functions */
	enum 
	{
	    COLLISION_TEST        = 1,
	    PATH_CONSTRAINTS_TEST = 2,
	    GOAL_CONSTRAINTS_TEST = 4
	};	
	    
	/** \brief Return true if recent enough data is available so that planning is considered safe */
	bool isEnvironmentSafe(void) const;
	
	/** \brief Check if the full state of the robot is valid */
	bool isStateValid(const planning_models::StateParams *state, const int test, bool verbose = false) const;

	/** \brief Check if the full state of the robot is valid including path constraints (wrapper for isStateValid) */
	bool isStateValidOnPath(const planning_models::StateParams *state, bool verbose = false) const;

	/** \brief Check if the full state of the robot is valid including path constraints (wrapper for isStateValid) */
	bool isStateValidAtGoal(const planning_models::StateParams *state, bool verbose = false) const;

	/** \brief Check if the path is valid. Path constraints are considered, but goal constraints are not  */
	bool isPathValid(const motion_planning_msgs::KinematicPath &path, const int test, bool verbose = false) const;

	/** \brief Check if a segment of the path is valid. Path constraints are considered, but goal constraints are not  */
	bool isPathValid(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, const int test, bool verbose = false) const;
	
	/** \brief Find the index of the state on the path that is closest to a given state */
	int  closestStateOnPath(const motion_planning_msgs::KinematicPath &path, const planning_models::StateParams *state) const;

	/** \brief Find the index of the state on the path segment that is closest to a given state */
	int  closestStateOnPath(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, const planning_models::StateParams *state) const;
	
	/** \brief Set the kinematic constraints the monitor should use when checking a path */
	void setPathConstraints(const motion_planning_msgs::KinematicConstraints &kc);

	/** \brief Get the kinematic constraints the monitor uses when checking a path */
	const motion_planning_msgs::KinematicConstraints& getPathConstraints(void) const
	{
	    return kcPath_;
	}
	
	/** \brief Set the kinematic constraints the monitor should use when checking a path's last state (the goal) */
	void setGoalConstraints(const motion_planning_msgs::KinematicConstraints &kc);
	
	/** \brief Get the kinematic constraints the monitor uses when checking a path's last state (the goal) */
	const motion_planning_msgs::KinematicConstraints& getGoalConstraints(void) const
	{
	    return kcGoal_;
	}

	/** \brief Print active constraints */
	void printConstraints(std::ostream &out = std::cout);

	/** \brief Clear previously set constraints */
	void clearConstraints(void);
	
	/** \brief Transform the frames in which constraints are specified to the one requested */
	bool transformConstraintsToFrame(motion_planning_msgs::KinematicConstraints &kc, const std::string &target) const;
	
	/** \brief Transform the kinematic path to the frame requested */
	bool transformPathToFrame(motion_planning_msgs::KinematicPath &kp, const std::string &target) const;

	/** \brief Transform the kinematic joint to the frame requested */
	bool transformJointToFrame(motion_planning_msgs::KinematicJoint &kj, const std::string &target) const;
	
	/** \brief Set the set of contacts allowed when collision checking */
	void setAllowedContacts(const std::vector<motion_planning_msgs::AcceptableContact> &allowedContacts);
	
	/** \brief Set the set of contacts allowed when collision checking */
	void setAllowedContacts(const std::vector<collision_space::EnvironmentModel::AllowedContact> &allowedContacts);
	
	/** \brief Get the set of contacts allowed when collision checking */
	const std::vector<collision_space::EnvironmentModel::AllowedContact>& getAllowedContacts(void) const;

	/** \brief Print allowed contacts */
	void printAllowedContacts(std::ostream &out = std::cout);
	
	/** \brief Clear the set of allowed contacts */
	void clearAllowedContacts(void);
	
	/** \brief Set a callback to be called when a collision is found */
	void setOnCollisionContactCallback(const boost::function<void(collision_space::EnvironmentModel::Contact&)> &callback, unsigned int maxContacts = 1)
	{
	    onCollisionContact_ = callback;
	    maxCollisionContacts_ = maxContacts;
	}

	/** \brief Return the maximum amount of time that is allowed to pass between updates to the map. */
	double getExpectedMapUpdateInterval(void) const
	{
	    return intervalCollisionMap_;
	}

	/** \brief Return the maximum amount of time that is allowed to pass between updates to the state. */
	double getExpectedJointStateUpdateInterval(void) const
	{
	    return intervalState_;
	}

	/** \brief Return the maximum amount of time that is allowed to pass between updates to the pose. */
	double getExpectedPoseUpdateInterval(void) const
	{
	    return intervalPose_;
	}
	
    protected:

	/** \brief Load ROS parameters */
	void loadParams(void);

	/** \brief Transform the joint parameters (if needed) to a target frame */
	bool transformJoint(const std::string &name, unsigned int index, std::vector<double> &params, roslib::Header &header, const std::string& target) const;
	
	/** \brief Check the path assuming it is in the frame of the model */
	bool isPathValidAux(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, const int test, bool verbose) const;

	/** \brief Find the index of the state on the path that is closest to a given state assuming the path is in the frame of the model */
	int closestStateOnPathAux(const motion_planning_msgs::KinematicPath &path, unsigned int start, unsigned int end, const planning_models::StateParams *state) const;
	
	/** \brief User callback when a collision is found */
	boost::function<void(collision_space::EnvironmentModel::Contact&)> onCollisionContact_;
	unsigned int                                                       maxCollisionContacts_;
	
	std::vector<collision_space::EnvironmentModel::AllowedContact>     allowedContacts_;

	motion_planning_msgs::KinematicConstraints kcPath_;
	motion_planning_msgs::KinematicConstraints kcGoal_;
	
	double intervalCollisionMap_;
	double intervalState_;
	double intervalPose_;
	
    };
    
	
}

#endif
