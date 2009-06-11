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
#include <motion_planning_srvs/KinematicPlan.h>

// message for receiving the planning status
#include <motion_planning_msgs/KinematicPlanStatus.h>

// messages to interact with the trajectory controller
#include <robot_msgs/JointTraj.h>

static const std::string GROUPNAME = "right_arm";
    
class Example
{
public:
    
    Example(void)
    {
	plan_id_ = -1;
	robot_stopped_ = true;
	rm_ = new planning_environment::RobotModels("robot_description");
	kmsm_ = new planning_environment::KinematicModelStateMonitor(rm_, false);
	
	// we use the topic for sending commands to the controller, so we need to advertise it
	jointCommandPublisher_ = nh_.advertise<robot_msgs::JointTraj>("right_arm/trajectory_controller/trajectory_command", 1);
	kinematicPlanningStatusSubscriber_ = nh_.subscribe("kinematic_planning_status", 1, &Example::receiveStatus, this);
    }
        
    ~Example(void)
    {
	delete kmsm_;
	delete rm_;
    }
    
    void runExample(void)
    {
	// construct the request for the motion planner
	motion_planning_srvs::KinematicPlan::Request req;
	
	req.params.model_id = GROUPNAME;
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.interpolate = 1;
	req.times = 1;

	// skip setting the start state
	
	// 7 DOF for the arm; pick a goal state (joint angles)
	std::vector<std::string> names;
	rm_->getKinematicModel()->getJointsInGroup(names, GROUPNAME);
	req.goal_constraints.joint.resize(names.size());
	for (unsigned int i = 0 ; i < req.goal_constraints.joint.size(); ++i)
	{
	    req.goal_constraints.joint[i].joint_name = names[i];
	    req.goal_constraints.joint[i].min.resize(1);
	    req.goal_constraints.joint[i].max.resize(1);
	    req.goal_constraints.joint[i].min[0] = 0.0;
	    req.goal_constraints.joint[i].max[0] = 0.0;
	}
	
	req.goal_constraints.joint[0].min[0] = req.goal_constraints.joint[0].max[0] = -1.5;
	req.goal_constraints.joint[3].min[0] = req.goal_constraints.joint[3].max[0] = -0.2;
	req.goal_constraints.joint[5].min[0] = req.goal_constraints.joint[5].max[0] = 0.15;

	// allow 1 second computation time
	req.allowed_time = 1.0;
	
	// define the service messages
	motion_planning_srvs::KinematicPlan::Response res;
	
	ros::ServiceClient client = nh_.serviceClient<motion_planning_srvs::KinematicPlan>("plan_kinematic_path");
	if (client.call(req, res))
	    plan_id_ = res.id;
	else
	    ROS_ERROR("Service 'plan_kinematic_path' failed");
    }
    
    void run(void)
    {
	if (rm_->loadedModels())
	{
	    runExample();
	    ros::spin();
	}
    }

protected:

    // handle new status message
    void receiveStatus(const motion_planning_msgs::KinematicPlanStatusConstPtr &planStatus)
    {
	if (plan_id_ >= 0 && planStatus->id == plan_id_)
	{
	    if (planStatus->valid)
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
	kmsm_->getRobotState()->copyParamsGroup(cmd, GROUPNAME);

	std::vector<std::string> names;
	rm_->getKinematicModel()->getJointsInGroup(names, GROUPNAME);

	motion_planning_msgs::KinematicPath stop_path;	
	stop_path.states.resize(1);
	stop_path.states[0].set_vals_size(7);
	for (unsigned int i = 0 ; i < 7 ; ++i)
	    stop_path.states[0].vals[i] = cmd[i];
	
	sendArmCommand(stop_path, GROUPNAME);
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
            traj.points[i].time = path.times[i];	    
        }	
    }
    
    // send a command to the trajectory controller using a topic
    void sendArmCommand(const motion_planning_msgs::KinematicPath &path, const std::string &model)
    {
	robot_msgs::JointTraj traj;
	getTrajectoryMsg(path, traj);
	jointCommandPublisher_.publish(traj);
	ROS_INFO("Sent trajectory to controller (using a topic)");
    }
    
    int                                               plan_id_;
    bool                                              robot_stopped_;
    ros::NodeHandle                                   nh_;
    ros::Subscriber                                   kinematicPlanningStatusSubscriber_;
    ros::Publisher                                    jointCommandPublisher_;
    planning_environment::RobotModels                *rm_;
    planning_environment::KinematicModelStateMonitor *kmsm_;
    
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv, "example_execute_replan_to_state");

    Example plan;
    plan.run();
    
    return 0;    
}
