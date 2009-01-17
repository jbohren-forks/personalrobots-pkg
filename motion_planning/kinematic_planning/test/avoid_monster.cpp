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

#include <gtest/gtest.h>
#include <ros/node.h>

#include <robot_srvs/KinematicPlanState.h>
#include <robot_srvs/KinematicPlanLinkPosition.h>

class AvoidMonster : public ros::Node
{
public:
    
    AvoidMonster(void) : ros::Node("avoid_monster")
    {
    }
  
    void initialState(robot_msgs::KinematicState &state)
    {
	state.set_vals_size(45);
	for (unsigned int i = 0 ; i < state.get_vals_size() ; ++i)
	    state.vals[i] = 0.0;
    }
    
    void runTestBase(void)
    {
	robot_srvs::KinematicPlanState::request  req;
	
	req.params.model_id = "pr2::base";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "SBL";
	req.threshold = 0.01;
	req.interpolate = 0;
	req.times = 1;
	
	initialState(req.start_state);
	req.start_state.vals[0] -= 1.5;
	
	req.goal_state.set_vals_size(3);
	req.goal_state.vals[0] += 7.5;
	req.goal_state.vals[1] += 0.5;
	req.goal_state.vals[2] = -M_PI/2.0;

	req.allowed_time = 15.0;
	
	req.params.volumeMin.x = -10.0;	req.params.volumeMin.y = -10.0;	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 10.0;	req.params.volumeMax.y = 10.0;	req.params.volumeMax.z = 0.0;
	
	performCall(req);
    }
    
    
    void performCall(robot_srvs::KinematicPlanState::request &req)
    {	
	robot_srvs::KinematicPlanState::response res;
	
	if (ros::service::call("plan_kinematic_path_state", req, res))
	{
	    EXPECT_TRUE(res.path.get_states_size() > 2);
	    EXPECT_TRUE(res.distance == 0.0);
	}
	else
	{
	    fprintf(stderr, "Service 'plan_kinematic_path_state' failed\n");
	    FAIL();
	}
    }
    
    void runTestLeftEEf(void)
    {
	robot_srvs::KinematicPlanLinkPosition::request req;
	
	req.params.model_id = "pr2::right_arm";
	req.params.distance_metric = "L2Square";
	req.params.planner_id = "RRT";
	req.interpolate = 0;
	req.times = 1;

	initialState(req.start_state);

	req.start_state.vals[0] += 2.0;
	req.start_state.vals[2] = M_PI + M_PI/4.0;


	// the goal region is basically the position of a set of bodies
	req.set_goal_constraints_size(1);
	req.goal_constraints[0].type = robot_msgs::PoseConstraint::ONLY_POSITION;
	req.goal_constraints[0].robot_link = "wrist_flex_right";
	req.goal_constraints[0].pose.position.x = 0.0;
	req.goal_constraints[0].pose.position.y = 0.0;
	req.goal_constraints[0].pose.position.z = -100.0;
	req.goal_constraints[0].position_distance = 0.01;
	

	req.allowed_time = 0.5;
	
	req.params.volumeMin.x = -5.0;	req.params.volumeMin.y = -5.0;	req.params.volumeMin.z = 0.0;
	req.params.volumeMax.x = 5.0;	req.params.volumeMax.y = 5.0;	req.params.volumeMax.z = 0.0;
	
	performCall(req);
    }

    void performCall(robot_srvs::KinematicPlanLinkPosition::request &req)
    {	
	robot_srvs::KinematicPlanLinkPosition::response res;	
	if (ros::service::call("plan_kinematic_path_position", req, res))
	{
	    EXPECT_TRUE(res.path.get_states_size() > 0);
	    EXPECT_TRUE(res.distance >= 0.0);
	}
	else
	{
	    fprintf(stderr, "Service 'plan_kinematic_path_position' failed\n");	 
	    FAIL();
	}
    }
    
};

static int    g_argc = 0;
static char** g_argv = NULL;

class PlanAvoidMonster : public testing::Test
{
public:

    AvoidMonster *plan;

protected:
    
    PlanAvoidMonster(void)
    {
    }
    
    void SetUp(void)
    {      
	ros::init(g_argc, g_argv);
	plan = new AvoidMonster();
	sleep(8);
    }
    
    void TearDown(void)
    {
	sleep(1);
	ros::fini();
	delete plan;
    }
    
};


TEST_F(PlanAvoidMonster, Base)
{
    plan->runTestBase();
}

TEST_F(PlanAvoidMonster, Arm)
{
    plan->runTestLeftEEf();
}

int main(int argc, char **argv)
{  
    testing::InitGoogleTest(&argc, argv);
    g_argc = argc;
    g_argv = argv;
    return RUN_ALL_TESTS();
}
