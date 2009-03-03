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

/** This is a simple program that shows how to plan a motion to a
    state.  The minimum number of things to get the arm to move is
    done. This code only interfaces with a highlevel controller, so
    getting the motion executed is no longer our responsibility.
 */

#include <kinematic_planning/KinematicStateMonitor.h>

// interface to a high level controller
#include <pr2_msgs/MoveArmGoal.h>
    
class Example : public kinematic_planning::KinematicStateMonitor
{
public:
    
    Example(ros::Node *node) : kinematic_planning::KinematicStateMonitor(node)
    {
	// we use the topic for sending commands to the controller, so we need to advertise it
	m_node->advertise<pr2_msgs::MoveArmGoal>("right_arm_goal", 1);
    }

    void runExample(void)
    {
	// construct the request for the highlevel controller
	pr2_msgs::MoveArmGoal ag;
	ag.enable = 1;
	ag.timeout = 10.0;

	ag.set_goal_configuration_size(1);
	ag.goal_configuration[0].name = "r_shoulder_pan_joint";
	ag.goal_configuration[0].position = -0.5;
	m_node->publish("right_arm_goal", ag);
    }
        
    void run(void)
    {
	loadRobotDescription();
	if (loadedRobot())
	{
	    sleep(1);
	    runExample();
	}
	sleep(1);
    }

};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);

    ros::Node node("example_call_plan_to_state");
    Example plan(&node);
    plan.run();
    
    return 0;    
}
