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
#include <robot_msgs/DisplayKinematicPath.h>
#include <robot_srvs/ValidateKinematicPath.h>

#include <std_msgs/Empty.h>
#include <pr2_mechanism_controllers/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

#include <sstream>
#include <cassert>

enum 
    {
	C_NONE,
	C_ARM
    };
    
class PlanKinematicPath : public ros::Node,
			  public kinematic_planning::KinematicStateMonitor
{
public:
    
    PlanKinematicPath(const std::string& robot_model) : ros::Node("plan_kinematic_path"),
							kinematic_planning::KinematicStateMonitor(dynamic_cast<ros::Node*>(this), robot_model)
    {
	advertise<robot_msgs::DisplayKinematicPath>("display_kinematic_path", 1);
	advertise<robot_srvs::KinematicPlanState::request>("replan_kinematic_path_state", 1);
	advertise<robot_srvs::KinematicPlanState::request>("replan_kinematic_path_position", 1);
	advertise<pr2_mechanism_controllers::JointTraj>("right_arm_trajectory_command", 1);
	advertise<std_msgs::Empty>("replan_stop", 1);
	
	subscribe("path_to_goal", m_pathToGoal, &PlanKinematicPath::currentPathToGoal, this, 1);
	m_controller = C_NONE;
	m_gripPos = 0.5;	
    }

    virtual ~PlanKinematicPath(void)
    {
    }
    
    void currentState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(m_kmodel->stateDimension);
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    state.vals[i] = m_robotState->getParams()[i];
    }
    
    void requestStopReplanning(void)
    {
	std_msgs::Empty dummy;
	publish("replan_stop", dummy);
    }
    
    // execute this when a new path is received
    void currentPathToGoal(void)
    {
	std_msgs::Empty dummy;
	publish("replan_stop", dummy);

	printPath(m_pathToGoal, -1.0);
	if (m_statePlanning)
	{
	    sendDisplay(m_activeRequestState.start_state, m_pathToGoal, m_activeRequestState.params.model_id);
	    verifyPath(m_activeRequestState.start_state, m_activeRequestState.constraints, m_pathToGoal, m_activeRequestState.params.model_id);
	    if (m_controller == C_ARM)
		sendArmCommand(m_pathToGoal, m_activeRequestState.params.model_id);	    
	}
	else
	{
	    sendDisplay(m_activeRequestPosition.start_state, m_pathToGoal, m_activeRequestPosition.params.model_id);
	    verifyPath(m_activeRequestPosition.start_state, m_activeRequestPosition.constraints, m_pathToGoal, m_activeRequestPosition.params.model_id);
	    if (m_controller == C_ARM)
		sendArmCommand(m_pathToGoal, m_activeRequestPosition.params.model_id);	    
	}
    }
    
    void runTestRightArm(bool replan = false)
    {
	robot_srvs::KinematicPlanState::request  req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 1;

	currentState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;
	req.goal_state.vals[0] = -0.2;    
        req.goal_state.vals[1] = -0.1;    
	req.goal_state.vals[3] = 0.5;    
	req.goal_state.vals[4] = 0.4;    
	req.goal_state.vals[2] = -0.3;    
	req.goal_state.vals[6] = -0.3;    

	req.allowed_time = 30.0;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	m_controller = C_ARM;
	m_statePlanning = true;

	if (replan)
	    sendGoal(req);
	else
	    performCall(req);
    }
    
    void runRightArmTo0(bool replan = false)
    {
	robot_srvs::KinematicPlanState::request  req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 1;

	currentState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;
	
	req.allowed_time = 30.0;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	m_controller = C_ARM;
	m_statePlanning = true;
	
	if (replan)
	    sendGoal(req);
	else
	    performCall(req);
    }
    
    void sendGoal(robot_srvs::KinematicPlanLinkPosition::request &req)
    {
	m_activeRequestPosition = req;
	publish("replan_kinematic_path_position", req);
    }

    void sendGoal(robot_srvs::KinematicPlanState::request &req)
    {
	m_activeRequestState = req;
	publish("replan_kinematic_path_state", req);
    }
    
    void performCall(robot_srvs::KinematicPlanLinkPosition::request &req)
    {	
	robot_srvs::KinematicPlanLinkPosition::response res;
	
	if (ros::service::call("plan_kinematic_path_position", req, res))
	{
	    printPath(res.path, res.distance);
	    sendDisplay(req.start_state, res.path, req.params.model_id);
	    verifyPath(req.start_state, req.constraints, res.path, req.params.model_id);
	    if (m_controller == C_ARM)
		sendArmCommand(res.path, req.params.model_id);	   
	}
	else
	    ROS_ERROR("Service 'plan_kinematic_path_position' failed");
    }

    void performCall(robot_srvs::KinematicPlanState::request &req)
    {	
	robot_srvs::KinematicPlanState::response res;
	
	if (ros::service::call("plan_kinematic_path_state", req, res))
	{
	    printPath(res.path, res.distance);
	    sendDisplay(req.start_state, res.path, req.params.model_id);
	    verifyPath(req.start_state, req.constraints, res.path, req.params.model_id);
	    if (m_controller == C_ARM)
		sendArmCommand(res.path, req.params.model_id);
	}
	else
	    ROS_ERROR("Service 'plan_kinematic_path_state' failed\n");
    }
    
    void printPath(robot_msgs::KinematicPath &path, const double distance)
    {	
	unsigned int nstates = path.get_states_size();
	ROS_INFO("Obtained solution path with %u states, distance to goal = %f", nstates, distance);
	std::stringstream ss;
	for (unsigned int i = 0 ; i < nstates ; ++i)
	{
	    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
		ss <<  path.states[i].vals[j] << " ";
	}
	ROS_INFO(ss.str().c_str());
    }
    
    void verifyPath(robot_msgs::KinematicState &start, robot_msgs::KinematicConstraints &cstrs,
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
	}
    }
    
    void sendDisplay(robot_msgs::KinematicState &start, robot_msgs::KinematicPath &path, const std::string &model)
    {
	robot_msgs::DisplayKinematicPath dpath;
	dpath.frame_id = "map";
	dpath.model_name = model;
	dpath.start_state = start;
	dpath.path = path;
	publish("display_kinematic_path", dpath);
	ROS_INFO("Sent planned path to display");
    }
    
    void getTrajectoryMsg(robot_msgs::KinematicPath &path, pr2_mechanism_controllers::JointTraj &traj)
    {
	traj.set_points_size(path.get_states_size());
	
	for (unsigned int i = 0 ; i < path.get_states_size() ; ++i)
	{
	    traj.points[i].set_positions_size(path.states[i].get_vals_size() + 1);
	    for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
		traj.points[i].positions[j] = path.states[i].vals[j];
	    traj.points[i].positions[path.states[i].get_vals_size()] = m_gripPos;
	    traj.points[i].time = 0.0;
	}	
    }
    
    void sendArmCommand(robot_msgs::KinematicPath &path, const std::string &model)
    {
	pr2_mechanism_controllers::JointTraj traj;
	getTrajectoryMsg(path, traj);
	publish("right_arm_trajectory_command", traj);
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
    
protected:

    // in replanning mode, current path to goal
    robot_msgs::KinematicPath                      m_pathToGoal;

    // request if planning towards a state
    robot_srvs::KinematicPlanState::request        m_activeRequestState;

    // request if planning towards a link position
    robot_srvs::KinematicPlanLinkPosition::request m_activeRequestPosition;

    // the desired gripper position during planning
    double                                         m_gripPos;
    
    // flag is true if we are planning towards a state 
    bool                                           m_statePlanning;

    // id of the controller to be used when executing the planned paths
    int                                            m_controller;
    
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
	    //	plan->waitForState();

	    sleep(2);

	    char test = (argc < 3) ? '0' : argv[2][0];
	    
	    switch (test)
	    {
	    case '0':
		plan->runRightArmTo0();
		break;
	    case 'r':
		plan->runTestRightArm(true);    
		break;
	    default:
		ROS_ERROR("Unknown test");
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
