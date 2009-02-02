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

/** This is a simple program that shows how to plan a motion to a link position
    The minimum number of things to get the arm to move is done.
 */

#include <kinematic_planning/KinematicStateMonitor.h>

// service for planning to a link position
#include <robot_srvs/KinematicPlanLinkPosition.h>

// messages to interact with the trajectory controller
#include <robot_msgs/JointTraj.h>

static const std::string GROUPNAME = "pr2::right_arm";
    
class Example : public ros::Node,
		public kinematic_planning::KinematicStateMonitor
{
public:
    
    Example() : ros::Node("example_execute_plan_to_position_minimal"),
		kinematic_planning::KinematicStateMonitor(dynamic_cast<ros::Node*>(this))
    {
	// we use the topic for sending commands to the controller, so we need to advertise it
	advertise<robot_msgs::JointTraj>("right_arm_trajectory_command", 1);
    }

    void runExample(void)
    {
	// construct the request for the motion planner
	robot_msgs::KinematicPlanLinkPositionRequest req;
	
	req.params.model_id = GROUPNAME;
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "IKSBL"; 
	req.interpolate = 1;
	req.times = 1;

	// skip setting the start state

	// allow 1 second computation time
	req.allowed_time = 1.0;


	// place some constraints on the goal
	req.set_goal_constraints_size(1);
	// see the constraints message definition
	req.goal_constraints[0].type = robot_msgs::PoseConstraint::POSITION_XYZ + robot_msgs::PoseConstraint::ORIENTATION_RY;
	req.goal_constraints[0].robot_link = "r_gripper_palm_link";
	req.goal_constraints[0].x = 0.75025;
	req.goal_constraints[0].y = -0.188;	
	req.goal_constraints[0].z = 0.829675;	

	req.goal_constraints[0].roll = 0.0;
	req.goal_constraints[0].yaw = 0.0;
	
	// the threshold for solution is position_distance + orientation_distance * orientation_importance
	// but the distance requirements must be satisfied by 
        req.goal_constraints[0].position_distance = 0.005;      // in L2square norm
        req.goal_constraints[0].orientation_distance = 0.05;	// difference in radians between the quaternions
        req.goal_constraints[0].orientation_importance = 0.005;	// factor of importance of orientation relative to importance of position
	
	// define the service messages
	robot_srvs::KinematicPlanLinkPosition::Request  s_req;
	robot_srvs::KinematicPlanLinkPosition::Response s_res;
	s_req.value = req;
	
	if (ros::service::call("plan_kinematic_path_position", s_req, s_res))
	    sendArmCommand(s_res.value.path, GROUPNAME);
	else
	    ROS_ERROR("Service 'plan_kinematic_path_state' failed");
    }
    
protected:

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
    }
    sleep(1);
    
    plan->shutdown();
    delete plan;
    
    return 0;    
}
