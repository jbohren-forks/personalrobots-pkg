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

#include <planning_environment/kinematic_model_state_monitor.h>

// service for (re)planning to a state
#include <motion_planning_srvs/MotionPlan.h>

// messages to interact with the trajectory controller
#include <manipulation_msgs/JointTraj.h>

#include <std_srvs/Empty.h>

#include <iostream>

static const std::string GROUPNAME = "right_arm";
    
class Example
{
public:
    
    Example(void)
    {
	plan_id_ = -1;
	robot_stopped_ = true;
	rm_ = new planning_environment::RobotModels("robot_description");
	kmsm_ = new planning_environment::KinematicModelStateMonitor(rm_, &tf_);
	
	// we use the topic for sending commands to the controller, so we need to advertise it
	jointCommandPublisher_ = nh_.advertise<manipulation_msgs::JointTraj>("right_arm/trajectory_controller/trajectory_command", 1);
	displayPathPublisher_ =  nh_.advertise<motion_planning_msgs::KinematicPath>("display_kinematic_path", 1);
    }
        
    ~Example(void)
    {
	delete kmsm_;
	delete rm_;
    }
    
    void runExampleArm(void)
    {
	// construct the request for the motion planner
	motion_planning_srvs::MotionPlan::Request req;
	
	req.params.model_id = GROUPNAME;
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "kinematic::KPIECE";
	req.times = 50;

	
	// 7 DOF for the arm; pick a goal state (joint angles)
	std::vector<std::string> names;
	rm_->getKinematicModel()->getJointsInGroup(names, GROUPNAME);
	req.goal_constraints.joint_constraint.resize(names.size());
	for (unsigned int i = 0 ; i < req.goal_constraints.joint_constraint.size(); ++i)
	{
	    req.goal_constraints.joint_constraint[i].header.stamp = ros::Time::now();
	    req.goal_constraints.joint_constraint[i].header.frame_id = "/base_link";
	    req.goal_constraints.joint_constraint[i].joint_name = names[i];
	    req.goal_constraints.joint_constraint[i].value.resize(1);
	    req.goal_constraints.joint_constraint[i].toleranceAbove.resize(1);
	    req.goal_constraints.joint_constraint[i].toleranceBelow.resize(1);
	    req.goal_constraints.joint_constraint[i].value[0] = 0.0;
	    req.goal_constraints.joint_constraint[i].toleranceBelow[0] = 0.0;
	    req.goal_constraints.joint_constraint[i].toleranceAbove[0] = 0.0;
	}
	
	req.goal_constraints.joint_constraint[0].value[0] = -1.0;
	req.goal_constraints.joint_constraint[3].value[0] = -0.1;
	req.goal_constraints.joint_constraint[5].value[0] = 0.15;


	// allow 1 second computation time
	req.allowed_time = 1.0;
	
	// define the service messages
	motion_planning_srvs::MotionPlan::Response res;
	
	ros::ServiceClient client = nh_.serviceClient<motion_planning_srvs::MotionPlan>("plan_kinematic_path");
	if (client.call(req, res))
	{
	    
	    sendArmCommand(res.path, GROUPNAME);
	}
	else
	    ROS_ERROR("Service 'plan_kinematic_path' failed");
    }

    void runExampleBase(void)
    {
	// construct the request for the motion planner
	motion_planning_srvs::MotionPlan::Request req;
	
	req.params.model_id = "base";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "dynamic::KPIECE";

	req.params.volumeMin.x = -3;
	req.params.volumeMin.y = -3;
	req.params.volumeMin.z = 0;

	req.params.volumeMax.x = 3;
	req.params.volumeMax.y = 3;
	req.params.volumeMax.z = 0;

	req.times = 1;

	
	req.goal_constraints.joint_constraint.resize(1);
	req.goal_constraints.joint_constraint[0].header.stamp = ros::Time::now();
	req.goal_constraints.joint_constraint[0].header.frame_id = "/base_link";
	req.goal_constraints.joint_constraint[0].joint_name = "base_joint";
	req.goal_constraints.joint_constraint[0].value.resize(3);
	req.goal_constraints.joint_constraint[0].toleranceAbove.resize(3);
	req.goal_constraints.joint_constraint[0].toleranceBelow.resize(3);

	req.goal_constraints.joint_constraint[0].value[0] = 0.0;
	req.goal_constraints.joint_constraint[0].toleranceBelow[0] = 0.05;
	req.goal_constraints.joint_constraint[0].toleranceAbove[0] = 0.05;

	req.goal_constraints.joint_constraint[0].value[1] = 1.0;
	req.goal_constraints.joint_constraint[0].toleranceBelow[1] = 0.05;
	req.goal_constraints.joint_constraint[0].toleranceAbove[1] = 0.05;

	req.goal_constraints.joint_constraint[0].value[2] = 0.0;
	req.goal_constraints.joint_constraint[0].toleranceBelow[2] = 0.1;
	req.goal_constraints.joint_constraint[0].toleranceAbove[2] = 0.1;


	// allow 1 second computation time
	req.allowed_time = 0.5;
	
	// define the service messages
	motion_planning_srvs::MotionPlan::Response res;
	
	ros::ServiceClient client = nh_.serviceClient<motion_planning_srvs::MotionPlan>("plan_kinematic_path");
	if (client.call(req, res))
	{
	    displayPathPublisher_.publish(res.path);
	    std::cout << "Path with " << res.path.get_states_size() << " states";
	    for (unsigned int i = 0 ; i < res.path.get_states_size() ; ++i)
	    {	    
		for (unsigned int j = 0 ; j < res.path.states[i].get_vals_size() ; ++j)
		    std::cout << " " << res.path.states[i].vals[j];
		std::cout << std::endl;
	    }
	}
	else
	    ROS_ERROR("Service 'plan_kinematic_path' failed");
    }
    
    void run(void)
    {
	if (rm_->loadedModels())
	{
	    sleep(1);
	    
	    runExampleBase();
	    ros::spin();
	}
    }

protected:

    // convert a kinematic path message to a trajectory for the controller
    void getTrajectoryMsg(const motion_planning_msgs::KinematicPath &path, manipulation_msgs::JointTraj &traj)
    {	
        traj.set_points_size(path.get_states_size());	
	for (unsigned int i = 0 ; i < path.get_states_size() ; ++i)
        {	    
            traj.points[i].set_positions_size(path.states[i].get_vals_size());	    
            for (unsigned int j = 0 ; j < path.states[i].get_vals_size() ; ++j)
                traj.points[i].positions[j] = path.states[i].vals[j];
            traj.points[i].time = path.times[i];	    
        }	
    }
    
    // send a command to the trajectory controller using a topic
    void sendArmCommand(const motion_planning_msgs::KinematicPath &path, const std::string &model)
    {
	manipulation_msgs::JointTraj traj;
	getTrajectoryMsg(path, traj);
	jointCommandPublisher_.publish(traj);
	ROS_INFO("Sent trajectory to controller (using a topic)");
    }
    
    int                                               plan_id_;
    bool                                              robot_stopped_;
    ros::NodeHandle                                   nh_;
    ros::Publisher                                    jointCommandPublisher_;
    ros::Publisher                                    displayPathPublisher_;
    planning_environment::RobotModels                *rm_;
    planning_environment::KinematicModelStateMonitor *kmsm_;
    tf::TransformListener                             tf_;
    
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv, "example_execute_replan_to_state");

    Example plan;
    plan.run();
    
    return 0;    
}
