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

/** This is a simple program that shows how to (re)plan a motion to a
    state or a link position.  The minimum number of things to get the
    arm to move (and replan) is done.
 */

#include <kinematic_planning/KinematicStateMonitor.h>

// service for (re)planning to a state
#include <motion_planning_srvs/KinematicReplanState.h>

// service for (re)planning to a link position
#include <motion_planning_srvs/KinematicReplanLinkPosition.h>

// message for receiving the planning status
#include <motion_planning_msgs/KinematicPlanStatus.h>

// message sent to visualizer to display the path
#include <motion_planning_msgs/DisplayKinematicPath.h>

// messages to interact with the joint trajectory controller
#include <robot_msgs/JointTraj.h>

// message to interact with the cartesian trajectory controller
#include <robot_msgs/PoseStamped.h>

static const std::string GROUPNAME = "pr2::right_arm";

class PlanKinematicPath : public kinematic_planning::KinematicStateMonitor
{
public:
    
    PlanKinematicPath(void) : kinematic_planning::KinematicStateMonitor()
    {
	plan_id_ = -1;
	robot_stopped_ = true;
	goalA_ = true;
	
	// we use the topic for sending commands to the controller, so we need to advertise it
	jointCommandPublisher_ = m_nodeHandle.advertise<robot_msgs::JointTraj>("right_arm/trajectory_controller/trajectory_command", 1);
	cartesianCommandPublisher_ = m_nodeHandle.advertise<robot_msgs::PoseStamped>("right_arm/?/command", 1);

	// advertise the topic for displaying kinematic plans
	displayKinematicPathPublisher_ = m_nodeHandle.advertise<motion_planning_msgs::DisplayKinematicPath>("display_kinematic_path", 10);

	kinematicPlanningStatusSubscriber_ = m_nodeHandle.subscribe("kinematic_planning_status", 1, &PlanKinematicPath::receiveStatus, this);
    }
        
    void runRightArmToPositionA(void)
    {
	// construct the request for the motion planner
	motion_planning_msgs::KinematicPlanStateRequest req;
	
	req.params.model_id = GROUPNAME;
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "KPIECE";
	req.threshold = 0.1;
	req.interpolate = 1;
	req.times = 1;

	// skip setting the start state
	
	// 7 DOF for the arm; pick a goal state (joint angles)
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = 0.0;	
	req.goal_state.vals[0] = -1.5;
	req.goal_state.vals[1] = -0.2;

	// allow 1 second computation time
	req.allowed_time = 1.0;
	
	// define the service messages
	motion_planning_srvs::KinematicReplanState::Request  s_req;
	motion_planning_srvs::KinematicReplanState::Response s_res;
	s_req.value = req;

	ros::ServiceClient client = m_nodeHandle.serviceClient<motion_planning_srvs::KinematicReplanState>("replan_kinematic_path_state");
	if (client.call(s_req, s_res))
	    plan_id_ = s_res.id;
	else
	    ROS_ERROR("Service 'replan_kinematic_path_state' failed");
    }

    void runRightArmToPositionB(void)
    {
	// construct the request for the motion planner
	motion_planning_msgs::KinematicPlanLinkPositionRequest req;
	
	req.params.model_id = GROUPNAME;
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "IKKPIECE";
	req.interpolate = 1;
	req.times = 1;

	// skip setting the start state
	
	// set the goal constraints
	req.set_goal_constraints_size(1);
	req.goal_constraints[0].type = motion_planning_msgs::PoseConstraint::POSITION_XYZ + motion_planning_msgs::PoseConstraint::ORIENTATION_RPY;
	req.goal_constraints[0].robot_link = "r_gripper_r_finger_tip_link";
	req.goal_constraints[0].x = 0.75025;
	req.goal_constraints[0].y = -0.188;	
	req.goal_constraints[0].z = 0.829675;	

	req.goal_constraints[0].roll = 0.0;
	req.goal_constraints[0].pitch = 0.0;
	req.goal_constraints[0].yaw = 0.0;
	
	req.goal_constraints[0].position_distance = 0.0001;
	req.goal_constraints[0].orientation_distance = 0.1;
	req.goal_constraints[0].orientation_importance = 0.0001;
	
	// allow 1 second computation time
	req.allowed_time = 0.5;
	
	// define the service messages
	motion_planning_srvs::KinematicReplanLinkPosition::Request  s_req;
	motion_planning_srvs::KinematicReplanLinkPosition::Response s_res;
	s_req.value = req;
	
	ros::ServiceClient client = m_nodeHandle.serviceClient<motion_planning_srvs::KinematicReplanLinkPosition>("replan_kinematic_path_position");

	if (client.call(s_req, s_res))
	    plan_id_ = s_res.id;
	else
	    ROS_ERROR("Service 'replan_kinematic_path_position' failed");
    }

    void runRightArmToCoordinates(void)
    {
	robot_msgs::PoseStamped ps;
	ps.header.frame_id = "base_link"; // make sure this is true; this should be take from input header
	ps.header.stamp = ros::Time::now(); // again, should be taken from input header
	ps.pose.position.x = 0.8;
	ps.pose.position.y = -0.188;
	ps.pose.position.z = 0.829675;
	ps.pose.orientation.x = 0;
	ps.pose.orientation.y = 0;
	ps.pose.orientation.z = 0;
	ps.pose.orientation.w = 1;
	cartesianCommandPublisher_.publish(ps);
    }

    void run(void)
    {
	loadRobotDescription();
	if (loadedRobot())
	{
	    goalTimer_ = m_nodeHandle.createTimer(ros::Duration(10.0), &PlanKinematicPath::changeGoal, this);
	    ros::spin();
	}	
    }
    
protected:
    
    void changeGoal(const ros::TimerEvent &e)
    {
	if (goalA_)
	    runRightArmToPositionA();
	else
	    runRightArmToPositionB();
	goalA_ = !goalA_;
    }
    
    // handle new status message
    void receiveStatus(const motion_planning_msgs::KinematicPlanStatusConstPtr &planStatus)
    {
	if (plan_id_ >= 0 && planStatus->id == plan_id_)
	{
	    if (planStatus->valid && !planStatus->unsafe)
	    {
		if (!planStatus->path.states.empty())
		{
		    robot_stopped_ = false;
		    sendArmCommand(planStatus->path, GROUPNAME);
		}
	    }
	    else
		stopRobot();
	    if (planStatus->done)
	    {
		plan_id_ = -1;
		robot_stopped_ = true;
		ROS_INFO("Execution is complete");
	    }
	}
    }
    
    void stopRobot(void)
    {
	if (robot_stopped_)
	    return;
	robot_stopped_ = true;
	
	// get the current params for the robot's right arm
	double cmd[7];
	m_robotState->copyParamsGroup(cmd, GROUPNAME);
	
	motion_planning_msgs::KinematicPath stop_path;	
	stop_path.set_states_size(1);
	stop_path.states[0].set_vals_size(7);
	for (unsigned int i = 0 ; i < 7 ; ++i)
	    stop_path.states[0].vals[i] = cmd[i];
	
	sendArmCommand(stop_path, GROUPNAME);
    }
    
    // get the current state from the StateParams instance monitored by the KinematicStateMonitor
    void currentState(motion_planning_msgs::KinematicState &state)
    {
	state.set_vals_size(m_kmodel->getModelInfo().stateDimension);
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    state.vals[i] = m_robotState->getParams()[i];	
    }

    // convert a kinematic path message to a trajectory for the controller
    void getTrajectoryMsg(const motion_planning_msgs::KinematicPath &path, robot_msgs::JointTraj &traj)
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
    
    // send a command to the trajectory controller using a topic
    void sendArmCommand(const motion_planning_msgs::KinematicPath &path, const std::string &model)
    {
	sendDisplay(path, model);
	printPath(path);
	robot_msgs::JointTraj traj;
	getTrajectoryMsg(path, traj);
	jointCommandPublisher_.publish(traj);
	ROS_INFO("Sent trajectory to controller");
    }

    void sendDisplay(const motion_planning_msgs::KinematicPath &path, const std::string &model)
    {
	motion_planning_msgs::DisplayKinematicPath dpath;
	dpath.frame_id = "base_link";
	dpath.model_name = model;
	currentState(dpath.start_state);
	dpath.path = path;
	displayKinematicPathPublisher_.publish(dpath);
	ROS_INFO("Sent planned path to display");
    }

    void printPath(const motion_planning_msgs::KinematicPath &path)
    {
	printf("Path with %d states\n", (int)path.states.size());	
	for (unsigned int i = 0 ; i < path.states.size() ; ++i)
	{
	    for (unsigned int j = 0 ; j < path.states[i].vals.size() ; ++j)
		printf("%f ", path.states[i].vals[j]);	    
	    printf("\n");
	}
	printf("\n");
    }
    
    int                                       plan_id_;
    bool                                      robot_stopped_;
    bool                                      goalA_;
    ros::Timer                                goalTimer_;
    
    ros::Publisher                            jointCommandPublisher_;
    ros::Publisher                            cartesianCommandPublisher_;
    ros::Publisher                            displayKinematicPathPublisher_;
    ros::Subscriber                           kinematicPlanningStatusSubscriber_;

};


int main(int argc, char **argv)
{  
    ros::init(argc, argv, "plan_kinematic_path");

    PlanKinematicPath plan;
    plan.run();

    return 0;    
}
