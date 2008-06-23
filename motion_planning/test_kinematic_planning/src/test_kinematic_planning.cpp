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

@b TestKinematicPlanning is a node that can be used to test a
kinematic planning node.

**/

#include "ros/node.h"
#include "std_msgs/PointCloudFloat32.h"
#include "std_msgs/KinematicPath.h"
#include "std_msgs/PR2Arm.h"
#include "std_srvs/KinematicMotionPlan.h"


using namespace std_msgs;
using namespace std_srvs;

class TestKinematicPlanning : public ros::node
{
public:

    TestKinematicPlanning(void) : ros::node("TestKinematicPlanning")
    {
	advertise<PR2Arm>("cmd_leftarmconfig");
	advertise<PR2Arm>("cmd_rightarmconfig");
    }
    
    void test1(void)
    {
	KinematicMotionPlan::request  req;
	KinematicMotionPlan::response res;

	req.model_id        = 0;
	req.resolution.data = 0.0;
	req.start_state.set_vals_size(9);
	req.start_state.vals[0].data = 0.1;
	req.start_state.vals[1].data = 0.1;
	req.start_state.vals[2].data = 0.1;
	req.start_state.vals[3].data = 0.1;
	req.start_state.vals[4].data = 0.1;
	req.start_state.vals[5].data = 0.1;
	req.start_state.vals[6].data = 0.1;
	req.start_state.vals[7].data = 0.1;
	req.start_state.vals[8].data = 0.1;
	
	req.goal_state.set_vals_size(9);
	req.goal_state.vals[0].data = 0.9;
	req.goal_state.vals[1].data = 0.9;
	req.goal_state.vals[2].data = 0.9;
	req.goal_state.vals[3].data = 0.9;
	req.goal_state.vals[4].data = 0.9;
	req.goal_state.vals[5].data = 0.9;
	req.goal_state.vals[6].data = 0.9;
	req.goal_state.vals[7].data = 0.9;
	req.goal_state.vals[8].data = 0.9;
	
	if (ros::service::call("plan_kinematic_path", req, res))
	{
	    uint32_t nstates = res.path.get_states_size();
	    for (uint32_t i = 0 ; i < nstates ; ++i)
	    {
		PR2Arm cmd;
		cmd.turretAngle       = res.path.states[i].vals[0].data;
		cmd.shoulderLiftAngle = res.path.states[i].vals[1].data;
		cmd.upperarmRollAngle = res.path.states[i].vals[2].data;
		cmd.elbowAngle        = res.path.states[i].vals[3].data;
		cmd.forearmRollAngle  = res.path.states[i].vals[4].data;
		cmd.wristPitchAngle   = res.path.states[i].vals[5].data;
		cmd.wristRollAngle    = res.path.states[i].vals[6].data;
		cmd.gripperForceCmd   = res.path.states[i].vals[7].data;
		cmd.gripperGapCmd     = res.path.states[i].vals[8].data;
		publish("cmd_leftarmconfig", cmd);

		usleep(10000);
	    }
	    printf("'%s' complete\n", __func__);
	}
	else
	    fprintf(stderr, "Service 'plan_kinematic_path' failed\n");
    }
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    TestKinematicPlanning test;

    test.test1();
    
    test.shutdown();
    
    return 0;    
}
