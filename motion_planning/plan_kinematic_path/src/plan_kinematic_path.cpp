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

#include <ros/node.h>
#include <ros/time.h>
#include <robot_srvs/KinematicPlanState.h>
#include <robot_srvs/KinematicPlanLinkPosition.h>
#include <robot_msgs/DisplayKinematicPath.h>
#include <std_msgs/RobotBase2DOdom.h>

class PlanKinematicPath : public ros::node
{
public:
    
    PlanKinematicPath(void) : ros::node("plan_kinematic_path")
    {
	advertise<robot_msgs::DisplayKinematicPath>("display_kinematic_path", 1);

	m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;
	m_haveBasePos = false;
	subscribe("localizedpose", m_localizedPose, &PlanKinematicPath::localizedPoseCallback, 1);
    }
  
    bool haveBasePos(void) const
    {
	return m_haveBasePos;
    }
    
    void initialState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(45);
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    state.vals[i] = 0.0;
	for (unsigned int i = 0 ; i < 3 ; ++i)
	    state.vals[i] = m_basePos[i];
    }
    
    
    void runTestBase(void)
    {
	robot_srvs::KinematicPlanState::request  req;
	
	req.params.model_id = "pr2::base";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 10;
	
	initialState(req.start_state);
	req.start_state.vals[0] -= 1.5;


	req.goal_state.set_vals_size(3);
	for (unsigned int i = 0 ; i < req.goal_state.vals_size ; ++i)
	    req.goal_state.vals[i] = m_basePos[i];

	req.goal_state.vals[0] += 7.5;
	req.goal_state.vals[1] += 0.5;
	req.goal_state.vals[2] = -M_PI/2.0;

	req.allowed_time = 15.0;
	
	req.params.volumeMin.x = -10.0 + m_basePos[0];	req.params.volumeMin.y = -10.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 10.0 + m_basePos[0];	req.params.volumeMax.y = 10.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	performCall(req);
    }
    
    void runTestLeftArm(void)
    {
	robot_srvs::KinematicPlanState::request  req;
	
	req.params.model_id = "pr2::left_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 1;

	initialState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.vals_size ; ++i)
	    req.goal_state.vals[i] = 0.0;
	req.goal_state.vals[0] = 1.0;    

	req.allowed_time = 3.0;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	performCall(req);
    }

    
    void runTestRightArm(void)
    {
	robot_srvs::KinematicPlanState::request  req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.threshold = 0.01;
	req.interpolate = 1;
	req.times = 1;

	initialState(req.start_state);
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.vals_size ; ++i)
	    req.goal_state.vals[i] = 0.0;
        req.goal_state.vals[0] = -1.0;    

	req.allowed_time = 30.0;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	performCall(req);
    }
    
    void performCall(robot_srvs::KinematicPlanState::request &req)
    {	
	robot_srvs::KinematicPlanState::response res;
	robot_msgs::DisplayKinematicPath dpath;
	dpath.frame_id = "FRAMEID_MAP";
	
	if (ros::service::call("plan_kinematic_path_state", req, res))
	{
	    unsigned int nstates = res.path.get_states_size();
	    printf("Obtained %ssolution path with %u states, distance to goal = %f\n",
		   res.distance > req.threshold ? "approximate " : "", nstates, res.distance);
	    for (unsigned int i = 0 ; i < nstates ; ++i)
	    {
		for (unsigned int j = 0 ; j < res.path.states[i].get_vals_size() ; ++j)
		    printf("%f ", res.path.states[i].vals[j]);
		printf("\n");
	    }
	    dpath.name = req.params.model_id;
	    dpath.path = res.path;
	    publish("display_kinematic_path", dpath);
	}
	else
	    fprintf(stderr, "Service 'plan_kinematic_path_state' failed\n");	 
    }
    
    void runTestLeftEEf(void)
    {
	robot_srvs::KinematicPlanLinkPosition::request req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "RRT";
	req.interpolate = 1;
	req.times = 2;

	initialState(req.start_state);

	req.start_state.vals[0] += 0.5;
	req.start_state.vals[1] += 1.5;
	req.start_state.vals[2] = -M_PI/2.0;


	// the goal region is basically the position of a set of bodies
	req.set_goal_constraints_size(1);
	req.goal_constraints[0].type = robot_msgs::PoseConstraint::ONLY_POSITION;
	req.goal_constraints[0].robot_link = "wrist_flex_right";
	req.goal_constraints[0].pose.position.x = 0.0;
	req.goal_constraints[0].pose.position.y = 0.0;
	req.goal_constraints[0].pose.position.z = -100.0;
	req.goal_constraints[0].position_distance = 0.01;
	
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
	req.allowed_time = 3.0;
	
	req.params.volumeMin.x = -5.0 + m_basePos[0];	req.params.volumeMin.y = -5.0 + m_basePos[1];	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0 + m_basePos[0];	req.params.volumeMax.y = 5.0 + m_basePos[1];	req.params.volumeMax.z = 0.0;
	
	performCall(req);
    }

    void performCall(robot_srvs::KinematicPlanLinkPosition::request &req)
    {	
	robot_srvs::KinematicPlanLinkPosition::response res;
	robot_msgs::DisplayKinematicPath dpath;
	dpath.frame_id = "FRAMEID_MAP";
	
	if (ros::service::call("plan_kinematic_path_position", req, res))
	{
	    unsigned int nstates = res.path.get_states_size();
	    printf("Obtained solution path with %u states\n", nstates);
	    
	    for (unsigned int i = 0 ; i < nstates ; ++i)
	    {
		for (unsigned int j = 0 ; j < res.path.states[i].get_vals_size() ; ++j)
		    printf("%f ", res.path.states[i].vals[j]);
		printf("\n");
	    }
	    dpath.name = req.params.model_id;
	    dpath.path = res.path;
	    publish("display_kinematic_path", dpath);
	}
	else
	    fprintf(stderr, "Service 'plan_kinematic_path_position' failed\n");	 
    }
    
private: 

    void localizedPoseCallback(void)
    {
	m_basePos[0] = m_localizedPose.pos.x;
	m_basePos[1] = m_localizedPose.pos.y;
	m_basePos[2] = m_localizedPose.pos.th;
	m_haveBasePos = true;
    }
    
    std_msgs::RobotBase2DOdom m_localizedPose;
    double                    m_basePos[3];
    bool                      m_haveBasePos;
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    PlanKinematicPath plan;
    sleep(1);

    /*
    ros::Duration dur(0.1);
    while (!plan.haveBasePos())
	dur.sleep();
    */

    char test = (argc < 2) ? 'b' : argv[1][0];
    
    switch (test)
    {
    case 'l':
	plan.runTestLeftArm();    
	break;
    case 'b':    
        plan.runTestBase();
	break;
    case 'r':
	plan.runTestRightArm();    
	break;
    case 'e':
	plan.runTestLeftEEf();    
	break;
    default:
	printf("Unknown test\n");
	break;
    } 
    sleep(1);
    
    plan.shutdown();
    
    return 0;    
}
