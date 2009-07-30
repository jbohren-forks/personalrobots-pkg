/*********************************************************************
*
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
*
* Authors: Sachin Chitta, Ioan Sucan  
*********************************************************************/
#ifndef MOVE_ARM_ACTION_H_
#define MOVE_ARM_ACTION_H_

/** Actions and messages */
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>

#include <manipulation_msgs/JointTraj.h>
#include <pr2_robot_actions/MoveArmState.h>
#include <pr2_robot_actions/MoveArmGoal.h>
#include <motion_planning_msgs/GetMotionPlan.h>

#include <ros/ros.h>
#include <planning_environment/monitors/planning_monitor.h>

namespace move_arm 
{    
    
    /**
     * @class MoveArm
     * @brief A class adhering to the robot_actions::Action interface that moves the robot base to a goal location.
     */
    class MoveArm : public robot_actions::Action<pr2_robot_actions::MoveArmGoal, int32_t> 
    {
    public:
	/**
	 * @brief  Constructor for the actions
	 */
	MoveArm(const std::string &arm_name);
	
	/**
	 * @brief  Destructor - Cleans up
	 */
	virtual ~MoveArm(void);
	
	/**
	 * @brief  Runs whenever a new goal is sent to the move_base
	 * @param goal The goal to pursue 
	 * @param feedback Feedback that the action gives to a higher-level monitor, in this case, the position of the robot
	 * @return The result of the execution, ie: Success, Preempted, Aborted, etc.
	 */
	virtual robot_actions::ResultStatus execute(const pr2_robot_actions::MoveArmGoal& goal, int32_t& feedback);
	
    private:

	// the state of the action; this should be true if initialized properly
	bool                     valid_;
	bool                     perform_ik_;      /**< Flag that enables the option of IK */
	bool                     show_collisions_; // enable showing collisions as visualization markers
	bool                     unsafe_paths_;    // flag to enable execution of paths without monitoring them for collision	 

	// the arm we are planning for
	std::string              arm_;
	std::vector<std::string> arm_joint_names_;	

	
	ros::NodeHandle          node_handle_;
	ros::Publisher           displayPathPublisher_;
	ros::Publisher           visMarkerPublisher_;
	
	planning_environment::CollisionModels *collisionModels_;
	planning_environment::PlanningMonitor *planningMonitor_;
	tf::TransformListener                  tf_;
	
	bool getControlJointNames(std::vector<std::string> &joint_names);
	void contactFound(collision_space::EnvironmentModel::Contact &contact);
	void printPath(const motion_planning_msgs::KinematicPath &path);
	bool fixState(planning_models::StateParams &sp, bool atGoal);
	bool fillStartState(std::vector<motion_planning_msgs::KinematicJoint> &start);	
	void fillTrajectoryPath(const motion_planning_msgs::KinematicPath &path, manipulation_msgs::JointTraj &traj);
	bool alterRequestUsingIK(motion_planning_msgs::GetMotionPlan::Request &req);
	bool computeIK(ros::ServiceClient &client, const robot_msgs::PoseStamped &pose_stamped_msg, std::vector<double> &solution);
    };
}

#endif
