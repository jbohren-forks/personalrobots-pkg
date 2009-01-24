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

/** This is a simple program for requesting a motion plan */

#include <kinematic_planning/KinematicStateMonitor.h>

#include <robot_srvs/KinematicPlanState.h>
#include <robot_srvs/KinematicPlanLinkPosition.h>
#include <robot_srvs/KinematicReplanState.h>
#include <robot_srvs/KinematicReplanLinkPosition.h>
#include <robot_msgs/DisplayKinematicPath.h>
#include <robot_srvs/ValidateKinematicPath.h>

#include <std_srvs/Empty.h>
#include <std_msgs/VisualizationMarker.h>
#include <pr2_mechanism_controllers/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

#include <sstream>
#include <cassert>

class PlanningRequest
{
public:
    
    enum 
	{
	    C_NONE,
	    C_ARM
	};
    
    PlanningRequest(ros::Node *node)
    {
	m_gripPos = 0.5;
	m_replanning = false;
	m_node = node;	
    }
    
    virtual ~PlanningRequest(void)
    {
    }
    
    void printPath(robot_msgs::KinematicPath &path, const double distance)
    {	
	unsigned int nstates = path.get_states_size();
	ROS_INFO("Obtained solution path with %u states, distance to goal = %f", nstates, distance);
	std::stringstream ss;
	ss << std::endl;
	for (unsigned int i = 0 ; i < nstates ; ++i)
	{
	    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
		ss <<  path.states[i].vals[j] << " ";
	    ss << std::endl;
	}
	ROS_INFO(ss.str().c_str());
    }
    
    bool verifyPath(robot_msgs::KinematicState &start, robot_msgs::KinematicConstraints &cstrs,
		    robot_msgs::KinematicPath &path, const std::string &model)
    {
	if (path.get_states_size() > 0)
	{
	    robot_srvs::ValidateKinematicPath::request  req;
	    robot_srvs::ValidateKinematicPath::response res;
	    req.model_id = model;
	    req.start_state = start;
	    req.constraints = cstrs;
	    
	    req.goal_state = path.states[path.get_states_size() - 1];
	    if (ros::service::call("validate_path", req, res))
	    {
		if (res.valid)
		    ROS_INFO("Direct path is valid");
		else
		    ROS_WARN("Direct path is not valid");
	    }
	    else
		ROS_INFO("Service 'validate_path' not available (or failed)");
	    return res.valid;
	}
	return false;
    }
    
    void sendDisplay(robot_msgs::KinematicState &start, robot_msgs::KinematicPath &path, const std::string &model)
    {
	robot_msgs::DisplayKinematicPath dpath;
	dpath.frame_id = "map";
	dpath.model_name = model;
	dpath.start_state = start;
	dpath.path = path;
	m_node->publish("display_kinematic_path", dpath);
	//	ROS_INFO("Sent planned path to display");
    }
    
    void sendArmCommand(robot_msgs::KinematicPath &path, const std::string &model)
    {
	pr2_mechanism_controllers::JointTraj traj;
	getTrajectoryMsg(path, traj);
	m_node->publish("right_arm_trajectory_command", traj);
	ROS_INFO("Sent trajectory to controller");
    }
    
    void sendArmCommandAndWait(robot_msgs::KinematicPath &path, const std::string &model)
    {
	pr2_mechanism_controllers::TrajectoryStart::request  send_traj_start_req;
	pr2_mechanism_controllers::TrajectoryStart::response send_traj_start_res;
	
	pr2_mechanism_controllers::TrajectoryQuery::request  send_traj_query_req;
	pr2_mechanism_controllers::TrajectoryQuery::response send_traj_query_res;
	
	pr2_mechanism_controllers::JointTraj traj;
	getTrajectoryMsg(path, traj);
	
	send_traj_start_req.traj = traj;
	int traj_done = -1;
	if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", send_traj_start_req, send_traj_start_res))
	{
	    ROS_INFO("Sent trajectory to controller");
	    
	    send_traj_query_req.trajectoryid =  send_traj_start_res.trajectoryid;
	    while(!(traj_done == send_traj_query_res.State_Done || traj_done == send_traj_query_res.State_Failed))
	    {
		if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery",  send_traj_query_req,  send_traj_query_res))  
		{
		    traj_done = send_traj_query_res.done;
		}
		else
		{
		    ROS_ERROR("Trajectory query failed");
		}
	    }
	    ROS_INFO("Trajectory execution is complete");	    
	}
    }
    
    void performCall(robot_srvs::KinematicReplanState::request &req, int controller = C_NONE)
    {
	if (m_replanning)
	    requestStopReplanning();
	m_replanning = true;
	m_activeRequestState = req.value;
	m_replanningController = controller;
	m_statePlanning = true;
	m_node->publish("replan_kinematic_path_state", req);
	ROS_INFO("Issued a replanning request");	    
    }
    
    void performCall(robot_srvs::KinematicPlanState::request &req, int controller = C_NONE)
    {
	robot_srvs::KinematicPlanState::response res;
	if (ros::service::call("plan_kinematic_path_state", req, res))
	    executePath(req.value, res.value.path, res.value.distance, controller);
	else
	    ROS_ERROR("Service 'plan_kinematic_path_state' failed");
    }
    
    void performCall(robot_srvs::KinematicReplanLinkPosition::request &req, int controller = C_NONE)
    {
	if (m_replanning)
	    requestStopReplanning();
	m_replanning = true;
	m_activeRequestLinkPosition = req.value;
	m_replanningController = controller;
	m_statePlanning = false;		
	m_node->publish("replan_kinematic_path_position", req);
	ROS_INFO("Issued a replanning request");	    
    }
    
    void performCall(robot_srvs::KinematicPlanLinkPosition::request &req, int controller = C_NONE)
    {
	robot_srvs::KinematicPlanLinkPosition::response res;	    
	if (ros::service::call("plan_kinematic_path_position", req, res))
	    executePath(req.value, res.value.path, res.value.distance, controller);
	else
	    ROS_ERROR("Service 'plan_kinematic_path_position' failed");
    }	
    
    void requestStopReplanning(void)
    {
	std_srvs::Empty::request req;
	std_srvs::Empty::response res;
	
	if (ros::service::call("replan_stop", req, res))
	    ROS_INFO("Issued a request to stop replanning");	
	else
	    ROS_WARN("Could not stop stop replanning");	
	m_replanning = false;
    }
    
    void useReplannedPath(robot_msgs::KinematicPath &path)
    {
	if (m_replanning)
	{
	    if (m_statePlanning)
		executePath(m_activeRequestState, path, -1.0, m_replanningController);
	    else
		executePath(m_activeRequestLinkPosition, path, -1.0, m_replanningController);
	}
	else
	    ROS_WARN("Received new path for replanning, but we are not in replanning mode");
    }
    
    void executePath(robot_msgs::KinematicPlanLinkPositionRequest &req,
		     robot_msgs::KinematicPath &path, const double distance,
		     int controller = C_NONE)
    {
	printPath(path, distance);
	sendDisplay(req.start_state, path, req.params.model_id);
	verifyPath(req.start_state, req.constraints, path, req.params.model_id);
	if (controller == C_ARM)
	    sendArmCommand(path, req.params.model_id);
    }
    
    void executePath(robot_msgs::KinematicPlanStateRequest &req,
		     robot_msgs::KinematicPath &path, const double distance,
		     int controller = C_NONE)
    {
	printPath(path, distance);
	sendDisplay(req.start_state, path, req.params.model_id);
	verifyPath(req.start_state, req.constraints, path, req.params.model_id);
	if (controller == C_ARM)
	    sendArmCommand(path, req.params.model_id);
    }
    
protected:
    
    void getTrajectoryMsg(robot_msgs::KinematicPath &path, pr2_mechanism_controllers::JointTraj &traj)
    {
	traj.set_points_size(path.get_states_size());
	
	for (unsigned int i = 0 ; i < path.get_states_size() ; ++i)
	{
	    traj.points[i].set_positions_size(path.states[i].get_vals_size());
	    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
		traj.points[i].positions[j] = path.states[i].vals[j];
	    traj.points[i].time = 0.0;
	}	
    }
    
    // ros node
    ros::Node                                     *m_node;
    
    // flag indicating whether we are in replanning mode
    bool                                           m_replanning;
    
    // flag is true if we are replanning towards a state 
    bool                                           m_statePlanning;
    
    // the desired gripper position during planning    
    double                                         m_gripPos;
    
    // in replanning mode, the controller to be used
    int                                            m_replanningController;	
    // in replanning mode, current starting state
    robot_msgs::KinematicPlanStateRequest          m_activeRequestState;	
    robot_msgs::KinematicPlanLinkPositionRequest   m_activeRequestLinkPosition;
    
};

class PlanKinematicPath : public ros::Node,
			  public kinematic_planning::KinematicStateMonitor
{
public:
    
    PlanKinematicPath(const std::string& robot_model) : ros::Node("plan_kinematic_path"),
							kinematic_planning::KinematicStateMonitor(dynamic_cast<ros::Node*>(this), robot_model),
							m_pr(dynamic_cast<ros::Node*>(this))
    {
	advertise<robot_msgs::DisplayKinematicPath>("display_kinematic_path", 1);
	advertise<robot_srvs::KinematicPlanState::request>("replan_kinematic_path_state", 1);
	advertise<robot_srvs::KinematicPlanState::request>("replan_kinematic_path_position", 1);
	advertise<pr2_mechanism_controllers::JointTraj>("right_arm_trajectory_command", 1);
	advertise<std_msgs::Empty>("replan_stop", 1);
	
	subscribe("path_to_goal", m_pathToGoal, &PlanKinematicPath::currentPathToGoal, this, 1);

	advertise<std_msgs::VisualizationMarker>("visualizationMarker", 10240);
	m_id = 0;
    }

    virtual ~PlanKinematicPath(void)
    {
    }
    
    void stateUpdate(void)
    {
	KinematicStateMonitor::stateUpdate();
	robot_msgs::KinematicPath empty_path;
	robot_msgs::KinematicState state;
	currentState(state);
	m_pr.sendDisplay(state, empty_path, "pr2");
    }
    
    void currentState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(m_kmodel->stateDimension);
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    state.vals[i] = m_robotState->getParams()[i];
    }    
    
    // execute this when a new path is received
    void currentPathToGoal(void)
    {
	m_pr.useReplannedPath(m_pathToGoal);
	//	m_pr.requestStopReplanning();
    }
    
    void requestStopReplanning(void)
    {
	m_pr.requestStopReplanning();
    }
    
    void runTestRightArm(bool replan = false)
    {
	robot_msgs::KinematicPlanStateRequest  req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "RRT";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 1;

	currentState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;

	req.goal_state.vals[1] = -0.2;
	req.goal_state.vals[0] = 0.5;
	
	req.allowed_time = 30.0;

	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	if (replan)
	{
	    robot_srvs::KinematicReplanState::request r;
	    r.value = req;
	    m_pr.performCall(r, PlanningRequest::C_ARM);
	}
	else
	{
	    robot_srvs::KinematicPlanState::request r;
	    r.value = req;
	    m_pr.performCall(r, PlanningRequest::C_ARM);
	}	
    }
    
    void runRightArmTo0(bool replan = false)
    {
	robot_msgs::KinematicPlanStateRequest  req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "RRT";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 1;

	currentState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;	
	req.goal_state.vals[1] = -0.2;
	req.goal_state.vals[0] = -0.2;

	req.allowed_time = 30.0;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;

	if (replan)
	{
	    robot_srvs::KinematicReplanState::request r;
	    r.value = req;
	    m_pr.performCall(r, PlanningRequest::C_ARM);
	}
	else
	{
	    robot_srvs::KinematicPlanState::request r;
	    r.value = req;
	    m_pr.performCall(r, PlanningRequest::C_ARM);
	}	
    }
	    
    void printLinkPoses(void)
    {
	m_kmodel->computeTransforms(m_robotState->getParams());
	m_kmodel->printLinkPoses();
    }    
    
    void runTestRightEEf(bool replan = false)
    {
	robot_msgs::KinematicPlanLinkPositionRequest req;
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "RRT";
	req.interpolate = 1;
	req.times = 1;
	
	currentState(req.start_state);
	printLinkPoses();

	req.set_goal_constraints_size(1);
	req.goal_constraints[0].type = robot_msgs::PoseConstraint::COMPLETE_POSE;
	req.goal_constraints[0].robot_link = "r_gripper_palm_link";
	req.goal_constraints[0].pose.position.x = .7;	
	req.goal_constraints[0].pose.position.y = 0;	
	req.goal_constraints[0].pose.position.z = 1;	

	req.goal_constraints[0].pose.orientation.x = 0;
	req.goal_constraints[0].pose.orientation.y = 0;
	req.goal_constraints[0].pose.orientation.z = sqrt(2)/2;
	req.goal_constraints[0].pose.orientation.w = sqrt(2)/2;	

	req.goal_constraints[0].position_distance = 0.02;
	req.goal_constraints[0].orientation_distance = 0.02;
	req.goal_constraints[0].orientation_importance = 0.5;
	
	// an example of constraints: do not move the elbow too much
	/*
	  req.constraints.set_pose_size(1);
	  req.constraints.pose[0].type = robot_msgs::PoseConstraint::ONLY_POSITION;
	  req.constraints.pose[0].robot_link = "elbow_flex_left";
	  req.constraints.pose[0].pose.position.x = 0.45;
	  req.constraints.pose[0].pose.position.y = 0.188;
	  req.constraints.pose[0].pose.position.z = 0.74;
	  req.constraints.pose[0].position_distance = 0.01;
	*/
	
	req.allowed_time = 0.5;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];
	req.params.volumeMin.y = -5.0 + m_basePos[1];
	req.params.volumeMin.z = 0.0;
	
	req.params.volumeMax.x = 5.0 + m_basePos[0];
	req.params.volumeMax.y = 5.0 + m_basePos[1];
	req.params.volumeMax.z = 0.0;
	
	if (replan)
	{
	    robot_srvs::KinematicReplanLinkPosition::request r;
	    r.value = req;
	    m_pr.performCall(r, PlanningRequest::C_ARM);
	}
	else
	{
	    robot_srvs::KinematicPlanLinkPosition::request r;
	    r.value = req;
	    m_pr.performCall(r, PlanningRequest::C_ARM);
	}	
    }
    


protected:

    void sendPoint(double x, double y, double z, double radius, const std::string &frame_id)
    {
	std_msgs::VisualizationMarker mk;
	mk.header.stamp = ros::Time::now();
	
	mk.header.frame_id = frame_id;

	mk.id = m_id++;
	mk.type = std_msgs::VisualizationMarker::SPHERE;
	mk.action = std_msgs::VisualizationMarker::ADD;
	mk.x = x;
	mk.y = y;
	mk.z = z;

	mk.roll = 0;
	mk.pitch = 0;
	mk.yaw = 0;
	
	mk.xScale = radius * 2.0;
	mk.yScale = radius * 2.0;
	mk.zScale = radius * 2.0;
		
	mk.alpha = 255;
	mk.r = 255;
	mk.g = 10;
	mk.b = 10;
	
	publish("visualizationMarker", mk);
    }

    void afterAttachBody(planning_models::KinematicModel::Link *link)
    {
	for (unsigned int i = 0 ; i < link->attachedBodies.size() ; ++i)
	{
	    planning_models::KinematicModel::Sphere *sphere = dynamic_cast<planning_models::KinematicModel::Sphere*>(link->attachedBodies[i]->shape);
	    if (sphere)
	    {
		btVector3 &v = link->attachedBodies[i]->attachTrans.getOrigin();
		sendPoint(v.x(), v.y(), v.z(), sphere->radius, link->name);
	    }
        }
    }
    
    // in replanning mode, current path to goal
    robot_msgs::KinematicPath m_pathToGoal;
    PlanningRequest           m_pr;
    unsigned int              m_id;
    
};


void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used when building the map.\n");
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    if (argc >= 2)
    {
	PlanKinematicPath *plan = new PlanKinematicPath(argv[1]);
	plan->loadRobotDescription();
	if (plan->loadedRobot())
	{
	    sleep(1);
	    plan->waitForState();
	    ROS_INFO("Received robot state");
	    
	    char test = (argc < 3) ? ' ' : argv[2][0];
	    
	    switch (test)
	    {
	    case '0':
		plan->runRightArmTo0(false);
		break;
	    case 'r':
		plan->runTestRightArm(false);    
		break;
	    case 'e':
		plan->runTestRightEEf(false);    
		break;
	    default:
		ROS_WARN("No test");
		break;
	    } 

	}
	plan->spin();
	plan->requestStopReplanning();
	
	plan->shutdown();
	delete plan;
    }
    else
	usage(argv[0]);
    
    return 0;    
}
