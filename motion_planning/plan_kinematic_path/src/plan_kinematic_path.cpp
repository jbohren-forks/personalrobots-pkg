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

#include <kinematic_planning/CollisionSpaceMonitor.h>

#include <robot_srvs/KinematicPlanState.h>
#include <robot_srvs/KinematicPlanLinkPosition.h>
#include <robot_srvs/KinematicReplanState.h>
#include <robot_srvs/KinematicReplanLinkPosition.h>
#include <robot_msgs/DisplayKinematicPath.h>
#include <robot_srvs/ValidateKinematicPath.h>

#include <std_srvs/Empty.h>
#include <robot_msgs/VisualizationMarker.h>
#include <robot_msgs/JointTraj.h>
#include <robot_srvs/IKService.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>


#include <sstream>
#include <cassert>


planning_models::KinematicModel::StateParams *robotState = NULL;
bool stopped = true;


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
	dpath.frame_id = "base_link";
	dpath.model_name = model;
	dpath.start_state = start;
	dpath.path = path;
	m_node->publish("display_kinematic_path", dpath);
	//	ROS_INFO("Sent planned path to display");
    }
    
    void sendArmCommand(robot_msgs::KinematicPath &path, const std::string &model)
    {
	robot_msgs::JointTraj traj;
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
	
	robot_msgs::JointTraj traj;
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

	robot_srvs::KinematicReplanState::response res;
	if (ros::service::call("replan_kinematic_path_state", req, res))
	    ROS_INFO("Succesfully issued a replanning request");  
	else
	    ROS_ERROR("Unable to issue replanning request");	
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

	robot_srvs::KinematicReplanLinkPosition::response res;
	if (ros::service::call("replan_kinematic_path_position", req, res))
	    ROS_INFO("Succesfully issued a replanning request");  
	else
	    ROS_ERROR("Unable to issue replanning request");	

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
    
    void stopRobot();
    
    void useReplannedPath(bool valid, robot_msgs::KinematicPath &path, double distance)
    {
	
	if (m_replanning)
	{
	    if (!valid)
		stopRobot();
	    else
	    {
		if (path.states.size() > 0)
		{
		    stopped = false;
		    
	    if (m_statePlanning)
		executePath(m_activeRequestState, path, distance, m_replanningController);
	    else
		executePath(m_activeRequestLinkPosition, path, distance, m_replanningController);
		}
		
		//		stopRobot();
		
	    }
	    
	    
	}
	//	else
	//	    ROS_WARN("Received new path for replanning, but we are not in replanning mode");
    }
    
    void executePath(robot_msgs::KinematicPlanLinkPositionRequest &req,
		     robot_msgs::KinematicPath &path, const double distance,
		     int controller = C_NONE)
    {
	printPath(path, distance);
	sendDisplay(req.start_state, path, req.params.model_id);
	//	verifyPath(req.start_state, req.constraints, path, req.params.model_id);
	//	if (controller == C_ARM)
	sendArmCommand(path, req.params.model_id);
    }
    
    void executePath(robot_msgs::KinematicPlanStateRequest &req,
		     robot_msgs::KinematicPath &path, const double distance,
		     int controller = C_NONE)
    {
	printPath(path, distance);
	sendDisplay(req.start_state, path, req.params.model_id);
	//	verifyPath(req.start_state, req.constraints, path, req.params.model_id);
	//	if (controller == C_ARM)
	sendArmCommand(path, req.params.model_id);
    }
    
protected:
    
    void getTrajectoryMsg(robot_msgs::KinematicPath &path, robot_msgs::JointTraj &traj)
    {
	traj.set_points_size(path.get_states_size());
	
	for (unsigned int i = 0 ; i < path.get_states_size() ; ++i)
	{
	    traj.points[i].set_positions_size(path.states[i].get_vals_size());
	    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
		traj.points[i].positions[j] = path.states[i].vals[j];

	    //	    traj.points[i].positions.insert(traj.points[i].positions.begin() + 2, robotState->getParams("r_upper_arm_roll_joint")[0]);
	    
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
			  public kinematic_planning::CollisionSpaceMonitor
{
public:
    
    PlanKinematicPath(void) : ros::Node("plan_kinematic_path"),
			      kinematic_planning::CollisionSpaceMonitor(dynamic_cast<ros::Node*>(this)),
			      m_pr(dynamic_cast<ros::Node*>(this))
    {
	advertise<robot_msgs::DisplayKinematicPath>("display_kinematic_path", 10);
	advertise<robot_msgs::JointTraj>("right_arm_trajectory_command", 1);
	
	subscribe("kinematic_planning_status", m_planStatus, &PlanKinematicPath::currentPathToGoal, this, 1);

	advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 10240);
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
	//	m_robotState->print();
	//	printLinkPoses();
	
	//	exit(0);
	
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
	m_pr.useReplannedPath(m_planStatus.valid, m_planStatus.path, m_planStatus.distance);
    }
    
    void requestStopReplanning(void)
    {
	m_pr.requestStopReplanning();
    }
    
    void runTestRightArm(bool replan = false)
    {
	robot_msgs::KinematicPlanStateRequest  req;
	stopped = false;
	robotState = m_robotState;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "IKSBL";
	req.threshold = 0.2;
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


    void runTestRightEx(bool replan = false)
    {
	robot_msgs::KinematicPlanStateRequest  req;

	robotState = m_robotState;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "IKSBL";
	req.threshold = 0.2;
	req.interpolate = 1;
	req.times = 1;

	currentState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;


	doIK(0.75025, -0.188, 0.859675, 0,0 ,0,1, req.goal_state);	
	/*

	req.goal_state.vals[0] = 0.114249;
	req.goal_state.vals[1] = 0.315477;	
	req.goal_state.vals[2] = 0.393613;	
	req.goal_state.vals[3] = -1.00633;	
	req.goal_state.vals[4] = 3.12397;	
	req.goal_state.vals[5] = 0.623655;	
	req.goal_state.vals[6] = 2.67552;	
	*/

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
	stopped = false;
	robotState = m_robotState;

	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "IKSBL";
	req.threshold = 0.2;
	req.interpolate = 1;
	req.times = 1;

	currentState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;	
	req.goal_state.vals[1] = -0.2;
	req.goal_state.vals[0] = -1.5;

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
	stopped = false;
	
	robotState = m_robotState;
	
	robot_msgs::KinematicPlanLinkPositionRequest req;
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "IKSBL";
	req.interpolate = 1;
	req.times = 1;
	
	//	currentState(req.start_state);

	currentState(req.start_state);//.set_vals_size(0);
       

	req.set_goal_constraints_size(1);
	req.goal_constraints[0].type = robot_msgs::PoseConstraint::COMPLETE_POSE;
	req.goal_constraints[0].robot_link = "r_gripper_palm_link";
	req.goal_constraints[0].pose.position.x = 0.75025;
	req.goal_constraints[0].pose.position.y = -0.188;	
	req.goal_constraints[0].pose.position.z = 0.829675;	

	req.goal_constraints[0].pose.orientation.x = 0;
	req.goal_constraints[0].pose.orientation.y = 0;
	req.goal_constraints[0].pose.orientation.z = 0;
	req.goal_constraints[0].pose.orientation.w = 1;	

	double d = 0.005;
	req.goal_constraints[0].position_distance = d;
	req.goal_constraints[0].orientation_distance = 0.05;
	req.goal_constraints[0].orientation_importance = 0.005;
	
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
	
	req.allowed_time = 16.67;
	
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
    
    void doIK(double px, double py, double pz,
	      double qx, double qy, double qz, double qw,
	      robot_msgs::KinematicState &state)
    {
	robot_srvs::IKService::request req;
	robot_srvs::IKService::response res;
	
	req.pose.position.x = px;
	req.pose.position.y = py;
	req.pose.position.z = pz;
	req.pose.orientation.x = qx;
	req.pose.orientation.y = qy;
	req.pose.orientation.z = qz;
	req.pose.orientation.w = qw;
	
	if (!ros::service::call("perform_pr2_ik", req, res))
	    ROS_ERROR("Failed calling IK service");
		
	if (res.traj.points.size() > 0)
	{
	    ROS_INFO("Got %d solutions", res.traj.points.size());
	    state.set_vals_size(res.traj.points[0].positions.size());
	    for (unsigned int i  = 0 ; i < res.traj.points[0].positions.size() ; ++i)
		state.vals[i] = res.traj.points[0].positions[i];
	}
	else
	    ROS_ERROR("IK Failed");
	
    }
    
protected:

    void sendPoint(double x, double y, double z, double radius, const std::string &frame_id)
    {
	robot_msgs::VisualizationMarker mk;
	mk.header.stamp = ros::Time::now();
	
	mk.header.frame_id = frame_id;

	mk.id = m_id++;
	mk.type = robot_msgs::VisualizationMarker::SPHERE;
	mk.action = robot_msgs::VisualizationMarker::ADD;
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
	    planning_models::KinematicModel::Box *sphere = dynamic_cast<planning_models::KinematicModel::Box*>(link->attachedBodies[i]->shape);
	    if (sphere)
	    {
		btVector3 &v = link->attachedBodies[i]->attachTrans.getOrigin();
		printf("extents: %g, %g, %g\n", sphere->size[0], sphere->size[1], sphere->size[2]);
		
		sendPoint(v.x(), v.y(), v.z(), std::max(std::max(sphere->size[0], sphere->size[1]), sphere->size[2] / 2.0), link->name);
	    }
        }
    }
    
    // in replanning mode, current path to goal
    robot_msgs::KinematicPlanStatus m_planStatus;
    PlanningRequest           m_pr;
    unsigned int              m_id;
    
};

void PlanningRequest::stopRobot()
{
    if (stopped)
	return;
    stopped = true;
    
    robot_msgs::KinematicPath stop_path;
    robot_msgs::KinematicState state;
    dynamic_cast<PlanKinematicPath*>(m_node)->currentState(state);
    stop_path.set_states_size(1);
    stop_path.states[0].set_vals_size(7);
    for (int i = 0 ; i < 7 ; i++)
    {
	stop_path.states[0].vals[i] = state.vals[20 + i];
	
    }
    

    
    ROS_WARN("************************* STOPPING ROBOT!");
    
    if (m_statePlanning)
	executePath(m_activeRequestState, stop_path, -1.0, m_replanningController);
    else
	executePath(m_activeRequestLinkPosition, stop_path, -1.0, m_replanningController);
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
	PlanKinematicPath *plan = new PlanKinematicPath();
	plan->loadRobotDescription();
	if (plan->loadedRobot())
	{
	    sleep(2);
	    plan->waitForState();
	    ROS_INFO("Received robot state");
	    plan->printCurrentState();
	    plan->printLinkPoses();

	    /*	    
	    sleep(3);		
	    

	    while(1)
	    {
		
		plan->runRightArmTo0(true);
	    sleep(10);
	    
	    plan->runTestRightArm(true);    
	    sleep(10);
	}
	    */
	    char test = (argc < 3) ? ' ' : argv[2][0];
	    
	    switch (test)
	    {
	    case '0':
		plan->runRightArmTo0(true);
		break;
	    case 'r':
		plan->runTestRightArm(true);    
		break;
	    case 'e':
		plan->runTestRightEEf(true);    
		break;
	    case 'x':
		plan->runTestRightEx(true);    
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
    
    return 0;    
}
