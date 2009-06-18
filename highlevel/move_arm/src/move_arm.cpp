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
#include <move_arm/move_arm.h>

#include <motion_planning_srvs/KinematicPlan.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>

#include <manipulation_srvs/IKService.h>
#include <manipulation_srvs/IKQuery.h>

using namespace robot_actions;

namespace move_arm 
{
    // these are the strings used internally to access services
    // they should be remaped in the launch file
    static const std::string CONTROL_START_NAME  = "controller_start";
    static const std::string CONTROL_QUERY_NAME  = "controller_query";
    static const std::string CONTROL_CANCEL_NAME = "controller_cancel";
    static const std::string MOTION_PLAN_NAME    = "motion_plan";

    static const std::string ARM_IK_NAME         = "arm_ik";
    static const std::string ARM_IK_QUERY_NAME   = "arm_ik_query";

    MoveArm::MoveArm(void) : Action<pr2_robot_actions::MoveArmGoal, int32_t>("move_arm")
    {	
	node_handle_.param<std::string>("~arm", arm_, std::string());
	node_handle_.param<bool>("~perform_ik", perform_ik_, true);
	
	// monitor robot
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_);
	
	if (collisionModels_->getKinematicModel()->getGroupID(arm_) < 0)
	{
	    valid_ = false;
	    ROS_ERROR("Arm '%s' is not known", arm_.c_str());
	}
	else
	    ROS_INFO("Starting move_arm for '%s' (IK is %senabled)", arm_.c_str(), perform_ik_ ? "" : "not ");
	
	if (valid_)
	    valid_ = collisionModels_->loadedModels() && getControlJointNames(arm_joint_names_);
	
	if (!valid_)
	    ROS_ERROR("Move arm action is invalid");
	
	// advertise the topic for displaying kinematic plans
	displayPathPublisher_ = node_handle_.advertise<motion_planning_msgs::KinematicPath>("display_kinematic_path", 10);
    }
    
    MoveArm::~MoveArm()
    {
	delete planningMonitor_;
	delete collisionModels_;
    }

    robot_actions::ResultStatus MoveArm::execute(const pr2_robot_actions::MoveArmGoal& goal, int32_t& feedback)
    { 
	if (!valid_)
	{
	    ROS_ERROR("Move arm action has not been initialized properly");
	    return robot_actions::ABORTED;
	}
	
	motion_planning_srvs::KinematicPlan::Request  req;
	motion_planning_srvs::KinematicPlan::Response res;
	
	
	req.params.model_id = arm_;              // the model to plan for (should be defined in planning.yaml)
	req.params.planner_id = "KPIECE";        // this is optional; the planning node should be able to pick a planner
	req.params.distance_metric = "L2Square"; // the metric to be used in the robot's state space
	
	// this volume is only needed if planar or floating joints move in the space
	req.params.volumeMin.x = req.params.volumeMin.y = req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = req.params.volumeMax.y = req.params.volumeMax.z = 0.0;
	
	// forward the goal & path constraints
	req.goal_constraints = goal.goal_constraints;
	req.path_constraints = goal.path_constraints;
	
	// compute the path once
	req.times = 1;

	// do not spend more than this amount of time
	req.allowed_time = 0.5;

	// change pose constraints to joint constraints, if possible and so desired
	if (perform_ik_ &&                                           // IK is enabled,
	    req.goal_constraints.joint_constraint.empty() &&         // we have no joint constraints on the goal,
	    req.goal_constraints.pose_constraint.size() == 1 &&      // we have a single pose constraint on the goal
	    req.goal_constraints.pose_constraint[0].type == motion_planning_msgs::PoseConstraint::POSITION_XYZ +
	    motion_planning_msgs::PoseConstraint::ORIENTATION_RPY && // that is active on all 6 DOFs
	    req.goal_constraints.pose_constraint[0].position_distance < 0.1 && // and the desired position and
	    req.goal_constraints.pose_constraint[0].orientation_distance < 0.1) // orientation distances are small
	{
	    planning_models::KinematicModel::Link *link = planningMonitor_->getKinematicModel()->getLink(req.goal_constraints.pose_constraint[0].link_name);
	    if (link && link->before && link->before->name == arm_joint_names_.back())
	    {
		// we can do ik can turn the pose constraint into a joint one
		ROS_INFO("Converting pose constraint to joint constraint using IK...");
		
		std::vector<double> solution;
		if (computeIK(req.goal_constraints.pose_constraint[0].pose, solution))
		{
		    unsigned int n = 0;
		    for (unsigned int i = 0 ; i < arm_joint_names_.size() ; ++i)
		    {
			motion_planning_msgs::JointConstraint jc;
			jc.joint_name = arm_joint_names_[i];
			jc.header.frame_id = req.goal_constraints.pose_constraint[0].pose.header.frame_id;
			jc.header.stamp = planningMonitor_->lastStateUpdate();
			unsigned int u = planningMonitor_->getKinematicModel()->getJoint(arm_joint_names_[i])->usedParams;
			for (unsigned int j = 0 ; j < u ; ++j)
			{
			    jc.value.push_back(solution[n + j]);
			    jc.toleranceAbove.push_back(0.0);
			    jc.toleranceBelow.push_back(0.0);
			}
			n += u;			
			req.goal_constraints.joint_constraint.push_back(jc);
		    }
		    req.goal_constraints.pose_constraint.clear();
		}
		else
		    ROS_WARN("Unable to compute IK");
	    }
	}
	
	ResultStatus result = robot_actions::SUCCESS;
	
	feedback = pr2_robot_actions::MoveArmState::PLANNING;
	update(feedback);
	
	ros::ServiceClient clientPlan   = node_handle_.serviceClient<motion_planning_srvs::KinematicPlan>(MOTION_PLAN_NAME, true);
	ros::ServiceClient clientStart  = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryStart>(CONTROL_START_NAME, true);
	ros::ServiceClient clientQuery  = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(CONTROL_QUERY_NAME, true);
	ros::ServiceClient clientCancel = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryCancel>(CONTROL_CANCEL_NAME, true);

	int                trajectoryId = -1;
	ros::Duration      eps(0.01);
	while (true)
	{
	    // if we have to stop, do so
	    if (isPreemptRequested() || !node_handle_.ok())
		result = robot_actions::PREEMPTED;
	    
	    // if we have to plan, do so
	    if (result == robot_actions::SUCCESS && feedback == pr2_robot_actions::MoveArmState::PLANNING)
	    {
		
		// fill in start state with current one
		std::vector<planning_models::KinematicModel::Joint*> joints;
		planningMonitor_->getKinematicModel()->getJoints(joints);
		
		req.start_state.resize(joints.size());
		for (unsigned int i = 0 ; i < joints.size() ; ++i)
		{
		    req.start_state[i].header.frame_id = planningMonitor_->getFrameId();
		    req.start_state[i].header.stamp = planningMonitor_->lastStateUpdate();
		    req.start_state[i].joint_name = joints[i]->name;
		    planningMonitor_->getRobotState()->copyParamsJoint(req.start_state[i].value, joints[i]->name);
		}
		
		// call the planner and decide whether to use the path 
		if (clientPlan.call(req, res))
		{
		    if (res.path.states.empty())
			ROS_WARN("Unable to plan path to desired goal");
		    else
		    {
			if (res.unsafe)
			    ROS_WARN("Received path is unsafe (planning data was out of date). Ignoring");
			else
			{
			    if (res.path.model_id != req.params.model_id)
				ROS_ERROR("Received path for incorrect model: expected '%s', received '%s'", req.params.model_id.c_str(), res.path.model_id.c_str());
			    else
			    {
				if (!planningMonitor_->getTransformListener()->frameExists(res.path.header.frame_id))
				    ROS_ERROR("Received path in unknown frame: '%s'", res.path.header.frame_id.c_str());
				else
				{
				    if (res.approximate)
					ROS_INFO("Approximate path was found. Distance to goal is: %f", res.distance);
				    ROS_INFO("Received path with %u states from motion planner", (unsigned int)res.path.states.size());
				    currentPath_ = res.path;
				    displayPathPublisher_.publish(currentPath_);
				    feedback = pr2_robot_actions::MoveArmState::MOVING;	
				    update(feedback);
				}
			    }
			}
		    }
		}
		else
		{
		    ROS_ERROR("Motion planning service failed");
		    result = robot_actions::ABORTED;
		    break;
		}
	    }
	    
	    // if we have to stop, do so
	    if (isPreemptRequested() || !node_handle_.ok())
		result = robot_actions::PREEMPTED;


	    // stop the robot if we need to
	    if (feedback == pr2_robot_actions::MoveArmState::MOVING)
	    {
		bool safe = planningMonitor_->isEnvironmentSafe();
		bool valid = planningMonitor_->isPathValid(currentPath_);
		if (result == robot_actions::PREEMPTED || !safe || !valid)
		{
		    if (result == robot_actions::PREEMPTED)
			ROS_INFO("Preempt requested. Stopping arm.");
		    else
			if (safe)
			    ROS_WARN("Environment is no longer safe. Cannot decide if path is valid. Stopping & replanning...");
			else
			    ROS_INFO("Current path is no longer valid. Stopping & replanning...");

		    if (trajectoryId != -1)
		    {
			// we are already executing the path; we need to stop it
			pr2_mechanism_controllers::TrajectoryCancel::Request  send_traj_cancel_req;
			pr2_mechanism_controllers::TrajectoryCancel::Response send_traj_cancel_res;
			send_traj_cancel_req.trajectoryid = trajectoryId;
			if (clientCancel.call(send_traj_cancel_req, send_traj_cancel_res))
			{
			    // check if indeed the trajectory was canceled
			    pr2_mechanism_controllers::TrajectoryQuery::Request  send_traj_query_req;
			    pr2_mechanism_controllers::TrajectoryQuery::Response send_traj_query_res;
			    send_traj_query_req.trajectoryid = trajectoryId;
			    if (clientQuery.call(send_traj_query_req, send_traj_query_res))
			    {
				if (send_traj_query_res.done == pr2_mechanism_controllers::TrajectoryQuery::Response::State_Active)
				    ROS_WARN("Unable to confirm canceling trajectory %d. Continuing...", trajectoryId);
			    }
			    else
				ROS_ERROR("Unable to query trajectory %d. Continuing...", trajectoryId);
			}
			else
			    ROS_ERROR("Unable to cancel trajectory %d. Continuing...", trajectoryId);
			trajectoryId = -1;
		    }
		    
		    if (result != robot_actions::PREEMPTED)
		    {
			// if we were not preempted
			feedback = pr2_robot_actions::MoveArmState::PLANNING;	
			update(feedback);
			continue;
		    }
		    else
			break;
		}
	    }
	    
	    // execute & monitor a path if we need to 
	    if (result == robot_actions::SUCCESS && feedback == pr2_robot_actions::MoveArmState::MOVING)
	    {
		// start the controller if we have to, using trajectory start
		if (trajectoryId == -1)
		{
		    pr2_mechanism_controllers::TrajectoryStart::Request  send_traj_start_req;
		    pr2_mechanism_controllers::TrajectoryStart::Response send_traj_start_res;
		    
		    fillTrajectoryPath(currentPath_, send_traj_start_req.traj);
		    send_traj_start_req.hastiming = 0;
		    send_traj_start_req.requesttiming = 0;

		    if (clientStart.call(send_traj_start_req, send_traj_start_res))
		    {
			trajectoryId = send_traj_start_res.trajectoryid;
			if (trajectoryId < 0)
			{
			    ROS_ERROR("Invalid trajectory id: %d", trajectoryId);
			    result = robot_actions::ABORTED;
			    break;
			}
			ROS_INFO("Sent trajectory to controller");
		    }
		    else
		    {
			ROS_ERROR("Unable to start trajectory controller");
			result = robot_actions::ABORTED;
			break;
		    }
		}
		
		// monitor controller execution by calling trajectory query

		pr2_mechanism_controllers::TrajectoryQuery::Request  send_traj_query_req;
		pr2_mechanism_controllers::TrajectoryQuery::Response send_traj_query_res;
		send_traj_query_req.trajectoryid = trajectoryId;
		if (clientQuery.call(send_traj_query_req, send_traj_query_res))
		{
		    // we are done; exit with success
		    if (send_traj_query_res.done == pr2_mechanism_controllers::TrajectoryQuery::Response::State_Done)
			break;
		    // something bad happened in the execution
		    if (send_traj_query_res.done != pr2_mechanism_controllers::TrajectoryQuery::Response::State_Active && 
			send_traj_query_res.done != pr2_mechanism_controllers::TrajectoryQuery::Response::State_Queued)
		    {
			result = robot_actions::ABORTED;
			ROS_ERROR("Unable to execute trajectory %d: query returned status %d", trajectoryId, (int)send_traj_query_res.done);
			break;
		    }
		}
		else
		{
		    ROS_ERROR("Unable to query trajectory %d", trajectoryId);
		    result = robot_actions::ABORTED;
		    break;
		}
	    }
	    eps.sleep();	
	}
	
	if (result == robot_actions::SUCCESS)
	    ROS_INFO("Trajectory execution is complete");
	
	return result;
    }

    void MoveArm::fillTrajectoryPath(const motion_planning_msgs::KinematicPath &path, robot_msgs::JointTraj &traj)
    {
	traj.points.resize(path.states.size());
	for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	{
	    traj.points[i].positions = path.states[i].vals;
	    traj.points[i].time = path.times[i];
	}
    }
    
    bool MoveArm::getControlJointNames(std::vector<std::string> &joint_names)
    {
	ros::ServiceClient client_query = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(CONTROL_QUERY_NAME);
	pr2_mechanism_controllers::TrajectoryQuery::Request  req_query;
	pr2_mechanism_controllers::TrajectoryQuery::Response res_query;
	req_query.trajectoryid = -1;
	
	bool result = client_query.call(req_query, res_query);
	
	if (!result)
	{
	    ROS_WARN("Unable to retrieve controller joint names from control query service. Waiting a bit and retrying...");
	    ros::Duration(1.0).sleep();
	    result = client_query.call(req_query, res_query);
	    if (result)
		ROS_WARN("Retrieved controller joints on second attempt");
	}
	
	if (!result)
	{
	    ROS_ERROR("Unable to retrieve controller joint names from control query service");
	    return false;
	}
	
	joint_names = res_query.jointnames;
	
	// make sure we have the right joint names
	for(unsigned int i = 0; i < joint_names.size() ; ++i)
	{
	    if (planning_models::KinematicModel::Joint *j = planningMonitor_->getKinematicModel()->getJoint(joint_names[i]))
	    {
		ROS_DEBUG("Using joing '%s' with %u parameters", j->name.c_str(), j->usedParams);
		if (planningMonitor_->getKinematicModel()->getJointIndexInGroup(j->name, arm_) < 0)
		    return false;
	    }
	    else
	    {
		ROS_ERROR("Joint '%s' is not known", joint_names[i].c_str());
		return false;
	    }
	}
	
	std::vector<std::string> groupNames;
	planningMonitor_->getKinematicModel()->getJointsInGroup(groupNames, arm_);
	if (groupNames.size() != joint_names.size())
	{
	    ROS_ERROR("The group '%s' has more joints than the controller can handle", arm_.c_str());
	    return false;	    
	}
	
	if (groupNames != joint_names)
	    ROS_WARN("The group joints are not in the same order as the controller expects");
	
	return true;
    }
    
    bool MoveArm::computeIK(const robot_msgs::PoseStamped &pose_stamped_msg, std::vector<double> &solution)
    {
	// define the service messages
	manipulation_srvs::IKService::Request request;
	manipulation_srvs::IKService::Response response;
	
	request.data.pose_stamped = pose_stamped_msg;
	request.data.joint_names = arm_joint_names_;
	request.data.positions.clear();
	for(unsigned int i = 0; i < arm_joint_names_.size() ; ++i)
	{
	    const double *params = planningMonitor_->getRobotState()->getParamsJoint(arm_joint_names_[i]);
	    const unsigned int u = planningMonitor_->getKinematicModel()->getJoint(arm_joint_names_[i])->usedParams;
	    for (unsigned int j = 0 ; j < u ; ++j)
		request.data.positions.push_back(params[j]);
	}
	
	ros::ServiceClient client = node_handle_.serviceClient<manipulation_srvs::IKService>(ARM_IK_NAME);
	
	if (client.call(request, response))
	{ 
	    ROS_DEBUG("Obtained IK solution");
	    solution = response.solution;
	    for(unsigned int i = 0; i < solution.size() ; ++i)
		ROS_DEBUG("%f", solution[i]);
	    return true;      
	}
	else
	{
	    ROS_ERROR("IK service failed");
	    return false;
	}
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_arm");  
    ros::Node xx; // hack to get action to work
    
    move_arm::MoveArm move_arm;
    robot_actions::ActionRunner runner(20.0);
    runner.connect<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t>(move_arm);
    runner.run();
    ros::spin();
    return 0;
}
