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

/** This is a simple program that shows how to plan a motion to a state.
    The minimum number of things to get the base to move is done.

    This example is incomplete since I do not know how to interact
    with the base controller, and the kinematic_planning node still
    runs in the robot frame. We need to switch to the map frame at
    some point, if we want to do planning for the base.
*/

#include <kinematic_planning/KinematicStateMonitor.h>

// service for planning to a state
#include <robot_srvs/KinematicPlanState.h>


static const std::string GROUPNAME = "pr2::base";
    
class Example : public ros::Node,
		public kinematic_planning::KinematicStateMonitor
{
public:
    
    Example() : ros::Node("example_execute_plan_to_state_minimal"),
		kinematic_planning::KinematicStateMonitor(dynamic_cast<ros::Node*>(this))
    {
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
	
	// 3 DOF for the base; pick a goal state (joint angles)
	req.goal_state.set_vals_size(3);
	for (unsigned int i = 0 ; i < req.goal_state.get_vals_size(); ++i)
	    req.goal_state.vals[i] = m_basePos[i];	

	// move by one meter on X or Y
	req.goal_state.vals[0] = 1;
	
	// set the 2D space in which the base is allowed to move
	req.params.volumeMin.x = -5.0 + m_basePos[0];
	req.params.volumeMin.y = -5.0 + m_basePos[1];
	req.params.volumeMin.z = 0.0;
	
        req.params.volumeMax.x = 5.0 + m_basePos[0];
	req.params.volumeMax.y = 5.0 + m_basePos[1];
	req.params.volumeMax.z = 0.0;
	
	// allow 1 second computation time
	req.allowed_time = 1.0;
	
	// define the service messages
	robot_srvs::KinematicPlanState::request  s_req;
	robot_srvs::KinematicPlanState::response s_res;
	s_req.value = req;
	
	if (ros::service::call("plan_kinematic_path_state", s_req, s_res))
	{
	    // send command to the base
	    // no idea how to interface with that yet ...
	}
	else
	    ROS_ERROR("Service 'plan_kinematic_path_state' failed");
    }
    
protected:

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
