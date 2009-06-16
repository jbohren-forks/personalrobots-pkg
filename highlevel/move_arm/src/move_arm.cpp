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
*  POSSIBILITY OF SUCH DAMAGE.    return robot_actions::PREEMPTED;

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
    MoveArm::MoveArm(const ArmType &arm) : Action<pr2_robot_actions::MoveArmGoal, int32_t>("move_arm")
    {
	if (arm == LEFT)
	    arm_ = "left_arm";
	else
	    arm_ = "right_arm";
	
	ROS_INFO("Starting move_arm for '%s'", arm_.c_str());
	
	// monitor robot
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_);

	node_handle_.param<bool>       ("~perform_ik",      perform_ik_,      true);
	node_handle_.param<std::string>("~ik_service_name", ik_service_name_, "pr2_ik");
	node_handle_.param<std::string>("~ik_query_name",   ik_query_name_,   "pr2_ik_query");
	
	node_handle_.param<std::string>("~motion_plan_name",    motion_plan_name_,    "plan_kinematic_path");
	node_handle_.param<std::string>("~control_start_name",  control_start_name_,  "right_arm/trajectory_controller/TrajectoryStart");
	node_handle_.param<std::string>("~control_query_name",  control_query_name_,  "right_arm/trajectory_controller/TrajectoryQuery");
	node_handle_.param<std::string>("~control_cancel_name", control_cancel_name_, "right_arm/trajectory_controller/TrajectoryCancel");
	
	if (!(valid_ = collisionModels_->loadedModels() && getControlJointNames(arm_joint_names_)))
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
	
	req.params.model_id = arm_;
	req.params.planner_id = "KPIECE";
	req.params.distance_metric = "L2Square";
	
	// this volume is only needed if planar or floating joints move in the space
	req.params.volumeMin.x = req.params.volumeMin.y = req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = req.params.volumeMax.y = req.params.volumeMax.z = 0.0;
	
	req.goal_constraints = goal.goal_constraints;
	req.path_constraints = goal.path_constraints;
	
	
	if (perform_ik_ &&                                           // IK is enabled,
	    req.goal_constraints.joint_constraint.empty() &&         // we have no joint constraints on the goal,
	    req.goal_constraints.pose_constraint.size() == 1 &&      // we have a single pose constraint on the goal
	    req.goal_constraints.pose_constraint[0].type == motion_planning_msgs::PoseConstraint::POSITION_XYZ +
	    motion_planning_msgs::PoseConstraint::ORIENTATION_RPY && // that is active on all 6 DOFs
	    req.goal_constraints.pose_constraint[0].position_distance < 0.1 && // and the desired position and
	    req.goal_constraints.pose_constraint[0].orientation_distance < 0.1 && // orientation distances are small,
	    ((arm_ == "left_arm"  && req.goal_constraints.pose_constraint[0].link_name == arm_joint_names_.back())
	     || (arm_ == "right_arm" && req.goal_constraints.pose_constraint[0].link_name == arm_joint_names_.back()))) // and acts on the last link of the arm
	{
	    // we can do ik can turn the pose constraint into a joint one
	    ROS_INFO("Converting pose constraint to joint constraint using IK...");
	    
	    std::vector<double> solution;
	    if (computeIK(req.goal_constraints.pose_constraint[0].pose, solution))
	    {
		req.goal_constraints.joint_constraint.resize(1);
		req.goal_constraints.joint_constraint[0].header.frame_id = req.goal_constraints.pose_constraint[0].pose.header.frame_id;
		req.goal_constraints.joint_constraint[0].header.stamp = planningMonitor_->lastStateUpdate();
		unsigned int n = 0;
		for (unsigned int i = 0 ; i < arm_joint_names_.size() ; ++i)
		{
		    motion_planning_msgs::JointConstraint jc;
		    jc.joint_name = arm_joint_names_[i];
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
	    
	req.times = 1;
	req.allowed_time = 0.5;
	
	ResultStatus result = robot_actions::SUCCESS;
	
	feedback = pr2_robot_actions::MoveArmState::PLANNING;
	update(feedback);
	
	ros::ServiceClient clientPlan   = node_handle_.serviceClient<motion_planning_srvs::KinematicPlan>("plan_kinematic_path", true);
	ros::ServiceClient clientStart  = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryStart>("right_arm/trajectory_controller/TrajectoryStart", true);
	ros::ServiceClient clientQuery  = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>("right_arm/trajectory_controller/TrajectoryQuery", true);
	ros::ServiceClient clientCancel = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryCancel>("right_arm/trajectory_controller/TrajectoryCancel", true);

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
				    currentPath_ = res.path;
				    feedback = pr2_robot_actions::MoveArmState::MOVING;	
				    update(feedback);
				}
			    }
			}
		    }
		}
		else
		{
		    ROS_ERROR("Service 'plan_kinematic_path' failed");
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
		if (result == robot_actions::PREEMPTED || !planningMonitor_->isPathValid(currentPath_))
		{
		    if (result == robot_actions::PREEMPTED)
			ROS_INFO("Preempt requested. Stopping arm.");
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
			ROS_ERROR("Service 'right_arm/trajectory_controller/TrajectoryStart' failed");
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

	return result;

	/*

	
	return robot_actions::SUCCESS;

	*/
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
	ros::ServiceClient client_query = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(control_query_name_);
	pr2_mechanism_controllers::TrajectoryQuery::Request  req_query;
	pr2_mechanism_controllers::TrajectoryQuery::Response res_query;
	req_query.trajectoryid = -1;
	
	if (!client_query.call(req_query, res_query))
	{
	    ROS_ERROR("Unable to retrieve controller joint names using '%s' service", control_query_name_.c_str());
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
	
	ros::ServiceClient client = node_handle_.serviceClient<manipulation_srvs::IKService>(ik_service_name_);
	
	if (client.call(request, response))
	{ 
	    ROS_DEBUG("MoveArm:: Got IK solution");
	    solution = response.solution;
	    for(unsigned int i = 0; i < solution.size() ; ++i)
		ROS_DEBUG("%f", solution[i]);
	    return true;      
	}
	else
	{
	    ROS_ERROR("MoveArm:: Service '%s' failed", ik_service_name_.c_str());
	    return false;
	}
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_arm");  
    ros::Node xx; // hack to get action to work
    
    move_arm::MoveArm move_arm(move_arm::RIGHT);
    robot_actions::ActionRunner runner(20.0);
    runner.connect<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t>(move_arm);
    runner.run();
    ros::spin();
    return 0;
}
