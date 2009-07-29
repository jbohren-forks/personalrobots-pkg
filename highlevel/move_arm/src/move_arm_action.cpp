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

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <pr2_mechanism_controllers/TrajectoryCancel.h>

#include <manipulation_srvs/IKService.h>
#include <manipulation_srvs/IKQuery.h>

#include <visualization_msgs/Marker.h>
#include <cstdlib>

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

    MoveArm::MoveArm(const::std::string &arm_name) : Action<pr2_robot_actions::MoveArmGoal, int32_t>("move_" + arm_name)
    {	
	valid_ = true;
	arm_ = arm_name;
	
	node_handle_.param<bool>("~perform_ik", perform_ik_, true);
	node_handle_.param<bool>("~show_collisions", show_collisions_, false);
	node_handle_.param<bool>("~unsafe_paths", unsafe_paths_, false);
	
	// monitor robot
	collisionModels_ = new planning_environment::CollisionModels("robot_description");
	planningMonitor_ = new planning_environment::PlanningMonitor(collisionModels_, &tf_);

	if (collisionModels_->getKinematicModel()->getGroupID(arm_) < 0)
	{
	    valid_ = false;
	    ROS_ERROR("Arm '%s' is not known", arm_.c_str());
	}
	else
	    ROS_INFO("Starting move_arm for '%s' (IK is %senabled)", arm_.c_str(), perform_ik_ ? "" : "not ");
	
	if (valid_)
	    valid_ = collisionModels_->loadedModels();
	
	if (valid_)
	{
	    if (show_collisions_)
	    {
		ROS_INFO("Found collisions will be displayed as visualization markers");
		visMarkerPublisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 128);
		planningMonitor_->setOnCollisionContactCallback(boost::bind(&MoveArm::contactFound, this, _1));
	    }
	    planningMonitor_->getEnvironmentModel()->setVerbose(true);
	    planningMonitor_->waitForState();
	    planningMonitor_->waitForMap();
	    valid_ = getControlJointNames(arm_joint_names_);
	}
	
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
	
	motion_planning_srvs::MotionPlan::Request  req;
	motion_planning_srvs::MotionPlan::Response res;
	
	
	req.params.model_id = arm_;              // the model to plan for (should be defined in planning.yaml)
	req.params.distance_metric = "L2Square"; // the metric to be used in the robot's state space
		
	// forward the goal & path constraints
	req.goal_constraints = goal.goal_constraints;
	req.path_constraints = goal.path_constraints;
	
	// compute the path once
	req.times = 1;

	// do not spend more than this amount of time
	req.allowed_time = 1.0;

	// tell the planning monitor about the constraints we will be following
	planningMonitor_->setPathConstraints(req.path_constraints);
	planningMonitor_->setGoalConstraints(req.goal_constraints);

	if (perform_ik_)
	    alterRequestUsingIK(req);
	

	ResultStatus result = robot_actions::SUCCESS;
	
	feedback = pr2_robot_actions::MoveArmState::PLANNING;
	update(feedback);
	
	ros::ServiceClient clientPlan   = node_handle_.serviceClient<motion_planning_srvs::MotionPlan>(MOTION_PLAN_NAME, true);
	ros::ServiceClient clientStart  = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryStart>(CONTROL_START_NAME, true);
	ros::ServiceClient clientQuery  = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryQuery>(CONTROL_QUERY_NAME, true);
	ros::ServiceClient clientCancel = node_handle_.serviceClient<pr2_mechanism_controllers::TrajectoryCancel>(CONTROL_CANCEL_NAME, true);

	motion_planning_msgs::KinematicPath currentPath;
	int                                 currentPos   = 0;
	bool                                approx       = false;
	int                                 trajectoryId = -1;
	ros::Duration                       eps(0.01);
	ros::Duration                       epsLong(0.1);

	while (true)
	{
	    // if we have to stop, do so
	    if (isPreemptRequested() || !node_handle_.ok())
		result = robot_actions::PREEMPTED;
	    
	    // if we have to plan, do so
	    if (result == robot_actions::SUCCESS && feedback == pr2_robot_actions::MoveArmState::PLANNING)
	    {

	        if (!planningMonitor_->isEnvironmentSafe())
		{
		    ROS_WARN("Environment is not safe. Will not issue request for planning");
		    epsLong.sleep();
		    continue;
		}
		
		fillStartState(req.start_state);
		
		ROS_DEBUG("Issued request for motion planning");
		
		// call the planner and decide whether to use the path 
		if (clientPlan.call(req, res))
		{
		    if (res.path.states.empty())
		    {
		        ROS_WARN("Unable to plan path to desired goal");
			epsLong.sleep();
		    }
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
				approx = res.approximate;
				if (res.approximate)
				    ROS_INFO("Approximate path was found. Distance to goal is: %f", res.distance);
				ROS_INFO("Received path with %u states from motion planner", (unsigned int)res.path.states.size());
				currentPath = res.path;
				currentPos = 0;
				if (planningMonitor_->transformPathToFrame(currentPath, planningMonitor_->getFrameId()))
				{
				    displayPathPublisher_.publish(currentPath);
				    printPath(currentPath);
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

	    // if preeemt was requested while we are planning, terminate
	    if (result != robot_actions::SUCCESS && feedback == pr2_robot_actions::MoveArmState::PLANNING)
	        break;

	    // stop the robot if we need to
	    if (feedback == pr2_robot_actions::MoveArmState::MOVING)
	    {
		bool safe = planningMonitor_->isEnvironmentSafe();
		bool valid = true;
		// we need to check if the path is still valid
		if (!unsafe_paths_ || (unsafe_paths_ && trajectoryId == -1))
		{
		    // we don't want to check the part of the path that was already executed
		    currentPos = planningMonitor_->closestStateOnPath(currentPath, currentPos, currentPath.states.size() - 1, planningMonitor_->getRobotState());
		    if (currentPos < 0)
		    {
			ROS_WARN("Unable to identify current state in path");
			currentPos = 0;
		    }
		    valid = planningMonitor_->isPathValid(currentPath, currentPos, currentPath.states.size() - 1, true);
		}
		
		if (result == robot_actions::PREEMPTED || !safe || !valid)
		{
		    if (result == robot_actions::PREEMPTED)
			ROS_INFO("Preempt requested. Stopping arm.");
		    else
			if (!safe)
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
			    ROS_INFO("Stopped trajectory %d", trajectoryId);
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
		    
		    fillTrajectoryPath(currentPath, send_traj_start_req.traj);
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
			ROS_INFO("Sent trajectory %d to controller", trajectoryId);
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
		    {
		        if (approx && !planningMonitor_->isStateValidAtGoal(planningMonitor_->getRobotState()))
			{
			    ROS_INFO("Completed approximate path (trajectory %d). Trying again to reach goal...", trajectoryId);
			    feedback = pr2_robot_actions::MoveArmState::PLANNING;	
			    update(feedback);
			    trajectoryId = -1;
			    continue;
			}
			ROS_INFO("Completed trajectory %d", trajectoryId);
			break;
		    }
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
	    ROS_INFO("Goal was reached");
	else
	    ROS_INFO("Goal was not reached");

	return result;
    }

    void MoveArm::fillTrajectoryPath(const motion_planning_msgs::KinematicPath &path, manipulation_msgs::JointTraj &traj)
    {
	/// \todo Joint controller does not take joint names; make sure we set them when the controller is updated
        traj.names = arm_joint_names_;
	traj.points.resize(path.states.size());
	for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	{
	    traj.points[i].positions = path.states[i].vals;
	    traj.points[i].time = path.times[i];
	}
    }
    
    void MoveArm::printPath(const motion_planning_msgs::KinematicPath &path)
    {
	ROS_DEBUG("Received path with %d states", (int)path.states.size());
	for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	{
	    std::stringstream ss;
	    for (unsigned int j = 0 ; j < path.states[i].vals.size() ; ++j)
		ss << path.states[i].vals[j] << " ";
	    ROS_DEBUG(ss.str().c_str());
	}
    }
    
    void MoveArm::contactFound(collision_space::EnvironmentModel::Contact &contact)
    {
	if (!planningMonitor_->getEnvironmentModel()->getVerbose())
	    return;
	
	static int count = 0;
	visualization_msgs::Marker mk;
	mk.header.stamp = planningMonitor_->lastMapUpdate();
	mk.header.frame_id = planningMonitor_->getFrameId();
	mk.ns = ros::this_node::getName();
	mk.id = count++;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;
	mk.pose.position.x = contact.pos.x();
	mk.pose.position.y = contact.pos.y();
	mk.pose.position.z = contact.pos.z();
	mk.pose.orientation.w = 1.0;
	
	mk.scale.x = mk.scale.y = mk.scale.z = 0.03;

	mk.color.a = 0.6;
	mk.color.r = 1.0;
	mk.color.g = 0.04;
	mk.color.b = 0.04;
	
	mk.lifetime = ros::Duration(30.0);
	
	visMarkerPublisher_.publish(mk);
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
	    ros::Duration(5.0).sleep();
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
    
    bool MoveArm::fixState(planning_models::StateParams &st, bool atGoal)
    {
	bool result = true;
	
	// just in case the system is a bit outside bounds, we enforce the bounds
	st.enforceBounds();
	
	// if the state is not valid, we try to fix it
	if (atGoal ? !planningMonitor_->isStateValidAtGoal(&st) : !planningMonitor_->isStateValidOnPath(&st))
	{
	    // try 2% change in each component
	    planning_models::StateParams temp(st);
	    int count = 0;
	    do 
	    {
		temp = st;
		temp.perturbStateGroup(0.02, arm_);
		count++;
	    } while ((atGoal ? !planningMonitor_->isStateValidAtGoal(&temp) : !planningMonitor_->isStateValidOnPath(&temp)) && count < 50);
	    
	    // try 10% change in each component
	    if (atGoal ? !planningMonitor_->isStateValidAtGoal(&temp) : !planningMonitor_->isStateValidOnPath(&temp))
	    {
		count = 0;
		do 
		{
		    temp = st;
		    temp.perturbStateGroup(0.1, arm_);
		    count++;
		} while ((atGoal ? !planningMonitor_->isStateValidAtGoal(&temp) : !planningMonitor_->isStateValidOnPath(&temp)) && count < 50);
	    }
	    
	    if (atGoal ? !planningMonitor_->isStateValidAtGoal(&temp) : !planningMonitor_->isStateValidOnPath(&temp))
		st = temp;
	    else
		result = false;
	}
	return result;
    }
    
    bool MoveArm::fillStartState(std::vector<motion_planning_msgs::KinematicJoint> &start_state)
    {
	// get the current state
	planning_models::StateParams st(*planningMonitor_->getRobotState());
	
	bool result = fixState(st, false);
	
	if (!result)
	    ROS_ERROR("Starting state for the robot is in collision and attempting to fix it failed");
	
	// fill in start state with current one
	std::vector<planning_models::KinematicModel::Joint*> joints;
	planningMonitor_->getKinematicModel()->getJoints(joints);
	
	start_state.resize(joints.size());
	for (unsigned int i = 0 ; i < joints.size() ; ++i)
	{
	    start_state[i].header.frame_id = planningMonitor_->getFrameId();
	    start_state[i].header.stamp = planningMonitor_->lastMechanismStateUpdate();
	    start_state[i].joint_name = joints[i]->name;
	    st.copyParamsJoint(start_state[i].value, joints[i]->name);
	}
	
	return result;
    }

    inline double uniformDouble(double lower_bound, double upper_bound)
    {
	return (upper_bound - lower_bound) * drand48() + lower_bound;     
    }
    
    bool MoveArm::alterRequestUsingIK(motion_planning_srvs::MotionPlan::Request &req)
    {
	bool result = false;
	
	// change pose constraints to joint constraints, if possible and so desired
	if (req.goal_constraints.joint_constraint.empty() &&         // we have no joint constraints on the goal,
	    req.goal_constraints.pose_constraint.size() == 1 &&      // we have a single pose constraint on the goal
	    req.goal_constraints.pose_constraint[0].type == 
	    motion_planning_msgs::PoseConstraint::POSITION_X + motion_planning_msgs::PoseConstraint::POSITION_Y + motion_planning_msgs::PoseConstraint::POSITION_Z +
	    motion_planning_msgs::PoseConstraint::ORIENTATION_R + motion_planning_msgs::PoseConstraint::ORIENTATION_P + motion_planning_msgs::PoseConstraint::ORIENTATION_Y)  // that is active on all 6 DOFs
	{
	    planning_models::KinematicModel::Link *link = planningMonitor_->getKinematicModel()->getLink(req.goal_constraints.pose_constraint[0].link_name);
	    if (link && link->before && link->before->name == arm_joint_names_.back())
	    {
		// we can do ik can turn the pose constraint into a joint one
		ROS_INFO("Converting pose constraint to joint constraint using IK...");
		
		planningMonitor_->getEnvironmentModel()->setVerbose(false);
		ros::ServiceClient client = node_handle_.serviceClient<manipulation_srvs::IKService>(ARM_IK_NAME, true);
		for (int t = 0 ; t < 2 ; ++t)
		{
		    geometry_msgs::PoseStamped tpose = req.goal_constraints.pose_constraint[0].pose;
		    if (t > 0)
		    {
			tpose.pose.position.x = uniformDouble(tpose.pose.position.x - req.goal_constraints.pose_constraint[0].position_tolerance_below.x,
							      tpose.pose.position.x + req.goal_constraints.pose_constraint[0].position_tolerance_above.x);
			tpose.pose.position.y = uniformDouble(tpose.pose.position.y - req.goal_constraints.pose_constraint[0].position_tolerance_below.y,
							      tpose.pose.position.y + req.goal_constraints.pose_constraint[0].position_tolerance_above.y);
			tpose.pose.position.z = uniformDouble(tpose.pose.position.z - req.goal_constraints.pose_constraint[0].position_tolerance_below.z,
							      tpose.pose.position.z + req.goal_constraints.pose_constraint[0].position_tolerance_above.z);
		    }
		    std::vector<double> solution;
		    if (computeIK(client, tpose, 5, solution))
		    {
			unsigned int n = 0;
			for (unsigned int i = 0 ; i < arm_joint_names_.size() ; ++i)
			{
			    motion_planning_msgs::JointConstraint jc;
			    jc.joint_name = arm_joint_names_[i];
			    jc.header.frame_id = tpose.header.frame_id;
			    jc.header.stamp = planningMonitor_->lastMechanismStateUpdate();
			    unsigned int u = planningMonitor_->getKinematicModel()->getJoint(arm_joint_names_[i])->usedParams;
			    for (unsigned int j = 0 ; j < u ; ++j)
			    {
				jc.value.push_back(solution[n + j]);
				jc.tolerance_above.push_back(0.0);
				jc.tolerance_below.push_back(0.0);
			    }
			    n += u;			
			    req.goal_constraints.joint_constraint.push_back(jc);
			}
			req.goal_constraints.pose_constraint.clear();

			// update the goal constraints for the planning monitor as well
			planningMonitor_->setGoalConstraints(req.goal_constraints);
			
			result = true;
		    }
		}
		planningMonitor_->getEnvironmentModel()->setVerbose(true);
		if (!result)
		    ROS_WARN("Unable to compute IK");
	    }
	}
	return result;
    }

    bool MoveArm::computeIK(ros::ServiceClient &client, const geometry_msgs::PoseStamped &pose_stamped_msg, int attempts, std::vector<double> &solution)
    {
	// define the service messages
	manipulation_srvs::IKService::Request request;
	manipulation_srvs::IKService::Response response;
	
	request.data.pose_stamped = pose_stamped_msg;
	request.data.joint_names = arm_joint_names_;

	bool validSolution = false;
	int ikSteps = 0;
	while (ikSteps < attempts && !validSolution)
	{
	    request.data.positions.clear();
	    planning_models::StateParams *sp = NULL;
	    if (ikSteps == 0)
		sp = new planning_models::StateParams(*planningMonitor_->getRobotState());
	    else
	    {
		sp = planningMonitor_->getKinematicModel()->newStateParams();
		sp->randomStateGroup(arm_);
	    }
	    ikSteps++;
	    
	    for(unsigned int i = 0; i < arm_joint_names_.size() ; ++i)
	    {
		const double *params = sp->getParamsJoint(arm_joint_names_[i]);
		const unsigned int u = planningMonitor_->getKinematicModel()->getJoint(arm_joint_names_[i])->usedParams;
		for (unsigned int j = 0 ; j < u ; ++j)
		    request.data.positions.push_back(params[j]);
	    }
	    delete sp;
	    
	    if (client.call(request, response))
	    { 
		ROS_DEBUG("Obtained IK solution");
		solution = response.solution;
		if (solution.size() != request.data.positions.size())
		{
		    ROS_ERROR("Incorrect number of elements in IK output");
		    return false;
		}
		
		for(unsigned int i = 0; i < solution.size() ; ++i)
		    ROS_DEBUG("IK[%d] = %f", (int)i, solution[i]);
		
		// if IK did not fail, check if the state is valid
		planning_models::StateParams spTest(*planningMonitor_->getRobotState());
		unsigned int n = 0;		    
		for (unsigned int i = 0 ; i < arm_joint_names_.size() ; ++i)
		{
		    unsigned int u = planningMonitor_->getKinematicModel()->getJoint(arm_joint_names_[i])->usedParams;
		    for (unsigned int j = 0 ; j < u ; ++j)
		    {
			std::vector<double> params(solution.begin() + n, solution.begin() + n + u);
			spTest.setParamsJoint(params, arm_joint_names_[i]);
		    }
		    n += u;			
		}
		
		// if state is not valid, we try to fix it
		fixState(spTest, true);
		
		if (planningMonitor_->isStateValidAtGoal(&spTest))
		{
		    validSolution = true;
		    
		    // update solution
		    solution.clear();
		    for (unsigned int i = 0 ; i < arm_joint_names_.size() ; ++i)
		    {
			std::vector<double> params;
			spTest.copyParamsJoint(params, arm_joint_names_[i]);
			solution.insert(solution.end(), params.begin(), params.end());
		    }
		}
	    }
	    else
	    {
		ROS_ERROR("IK service failed");
		return false;
	    }
	}
	
	return validSolution;
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_arm", ros::init_options::AnonymousName);  
    
    ros::NodeHandle nh;
    std::string arm_name;    
    nh.param<std::string>("~arm", arm_name, std::string());
    
    if (arm_name.empty())
	ROS_ERROR("No '~arm' parameter specified");
    else
    {
	move_arm::MoveArm move_arm(arm_name);
	robot_actions::ActionRunner runner(20.0);
	runner.connect<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t>(move_arm);
	runner.run();
	ros::spin();
    }
    
    return 0;
}
