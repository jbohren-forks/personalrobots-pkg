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

/**

@mainpage

@htmlinclude manifest.html

@b test_kinematic_planning is a node that can be used to test a
kinematic planning node.

<hr>

@section usage Usage
@verbatim
$ test_kinematic_planning [standard ROS args]
@endverbatim

@par Example

@verbatim
$ test_kinematic_planning
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "left_pr2arm_pos"/PR2Arm : retrieve current left arm position
- @b "right_pr2arm_pos"/PR2Arm : retrieve current right arm position

Publishes to (name/type):
- @b "cmd_leftarmconfig"/PR2Arm : issue joint angle commands to the left arm
- @b "cmd_rightarmconfig"/PR2Arm : issue joint angle commands to the right arm

<hr>

@section services ROS services

Uses (name/type):
- @b "plan_kinematic_path"/KinematicMotionPlan : given a robot model, starting ang goal states, this service computes a collision free path

Provides (name/type):
- None

<hr>

@section parameters ROS parameters
- None


**/

#include <ros/node.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_srvs/KinematicMotionPlan.h>
#include <kinematic_planning/util.h>

using namespace std_msgs;
using namespace std_srvs;

class TestKinematicPlanning : public ros::node
{
public:

    TestKinematicPlanning(void) : ros::node("test_kinematic_planning")
    {
	advertise<PR2Arm>("cmd_leftarmconfig");
	advertise<PR2Arm>("cmd_rightarmconfig");
	subscribe("left_pr2arm_pos",  leftArmPos,  &TestKinematicPlanning::currentLeftArmPos);
	subscribe("right_pr2arm_pos", rightArmPos, &TestKinematicPlanning::currentRightArmPos);
    }
    
    void currentLeftArmPos(void)
    {
	// don't need to do anything -- we already have the data
    }

    void currentRightArmPos(void)
    {
	// don't need to do anything -- we already have the data
    }
    
    
    void testLeftArm1(void)
    {
	KinematicMotionPlan::request  req;
	KinematicMotionPlan::response res;
	
	req.model_id   = ros::kinematics::PR2_LEFT_ARM;
	req.resolution = 0.01;

	ros::kinematics::convertPR2ArmKinematicState(leftArmPos, req.start_state);
	req.goal_state = req.start_state;
	
	for (int i = 0 ; i < 9 ; ++i)
	    req.goal_state.vals[i] += 0.2;
	
	printf("Starting at state: ");
	ros::kinematics::printKinematicState(req.start_state);
	
	printf("Going towards state: ");
	ros::kinematics::printKinematicState(req.goal_state);
	

	if (ros::service::call("plan_kinematic_path", req, res))
	{
	    uint32_t nstates = res.path.get_states_size();
	    printf("Received a plan containing %u states\n", nstates);
	    
	    for (uint32_t i = 0 ; i < nstates ; ++i)
	    {
		PR2Arm cmd;
		ros::kinematics::convertKinematicStatePR2Arm(res.path.states[i], cmd);
		publish("cmd_leftarmconfig", cmd);
		printf("  - sent the state: "); 
		ros::kinematics::printKinematicState(res.path.states[i]);
		usleep(10000);
	    }	    
	}
	else
	    fprintf(stderr, "Service 'plan_kinematic_path' failed\n");
	printf("'%s' complete\n", __func__);
    }

private:
    
    PR2Arm leftArmPos, rightArmPos;
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    TestKinematicPlanning test;
    sleep(1);
    
    test.testLeftArm1();
    
    test.shutdown();
    
    return 0;    
}
