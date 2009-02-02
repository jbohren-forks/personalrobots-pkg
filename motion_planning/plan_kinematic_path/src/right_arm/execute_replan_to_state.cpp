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

/** This is a simple program that shows how to (re)plan a motion to a state.
    The minimum number of things to get the arm to move (and replan) is done.
 */

#include <kinematic_planning/KinematicStateMonitor.h>

// service for (re)planning to a state
#include <robot_srvs/KinematicReplanState.h>

// message for receiving the planning status
#include <robot_msgs/KinematicPlanStatus.h>

// messages to interact with the trajectory controller
#include <robot_msgs/JointTraj.h>

static const std::string GROUPNAME = "pr2::right_arm";
    
class Example : public ros::Node,
		public kinematic_planning::KinematicStateMonitor
{
public:
    
    Example() : ros::Node("example_execute_replan_to_state"),
		kinematic_planning::KinematicStateMonitor(dynamic_cast<ros::Node*>(this))
    {
	plan_id_ = -1;
	robot_stopped_ = true;
	
	// we use the topic for sending commands to the controller, so we need to advertise it
	advertise<robot_msgs::JointTraj>("right_arm_trajectory_command", 1);
	subscribe("kinematic_planning_status", plan_status_, &Example::receiveStatus, this, 1);
    }
        
    void runExample(void)
    {
	// construct the request for the motion planner
	robot_msgs::KinematicPlanStateRequest req;
	
	req.params.model_id = GROUPNAME;
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
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
	robot_srvs::KinematicReplanState::Request  s_req;
	robot_srvs::KinematicReplanState::Response s_res;
	s_req.value = req;
	
	if (ros::service::call("replan_kinematic_path_state", s_req, s_res))
	    plan_id_ = s_res.id;
	else
	    ROS_ERROR("Service 'replan_kinematic_path_state' failed");
    }
    
protected:

    // handle new status message
    void receiveStatus(void)
    {
	if (plan_id_ >= 0 && plan_status_.id == plan_id_)
	{
	    if (plan_status_.valid)
	    {
		if (!plan_status_.path.states.empty())
		{
		    robot_stopped_ = false;
		    sendArmCommand(plan_status_.path, GROUPNAME);
		}
	    }
	    else
		stopRobot();
	    if (plan_status_.done)
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
	m_robotState->copyParams(cmd, m_kmodel->getGroupID(GROUPNAME));
	
	robot_msgs::KinematicPath stop_path;	
	stop_path.set_states_size(1);
	stop_path.states[0].set_vals_size(7);
	for (unsigned int i = 0 ; i < 7 ; ++i)
	    stop_path.states[0].vals[i] = cmd[i];
	
	sendArmCommand(stop_path, GROUPNAME);
    }
    
    // get the current state from the StateParams instance monitored by the KinematicStateMonitor
    void currentState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(m_kmodel->stateDimension);
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    state.vals[i] = m_robotState->getParams()[i];	
    }

    // convert a kinematic path message to a trajectory for the controller
    void getTrajectoryMsg(robot_msgs::KinematicPath &path, robot_msgs::JointTraj &traj)
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
    void sendArmCommand(robot_msgs::KinematicPath &path, const std::string &model)
    {
	robot_msgs::JointTraj traj;
	getTrajectoryMsg(path, traj);
	m_node->publish("right_arm_trajectory_command", traj);
	ROS_INFO("Sent trajectory to controller (using a topic)");
    }

    robot_msgs::KinematicPlanStatus plan_status_;
    int                             plan_id_;
    bool                            robot_stopped_;
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    Example *plan = new Example();
    plan->loadRobotDescription();
    if (plan->loadedRobot())
    {
	sleep(1);
	plan->runExample();
	plan->spin();
    }
    
    plan->shutdown();
    delete plan;
    
    return 0;    
}
