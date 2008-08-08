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

#include <ros/node.h>
#include <ros/time.h>
#include <robot_srvs/KinematicMotionPlan.h>
#include <robot_msgs/NamedKinematicPath.h>
#include <std_msgs/RobotBase2DOdom.h>

class PlanKinematicPath : public ros::node
{
public:
    
    PlanKinematicPath(void) : ros::node("plan_kinematic_path")
    {
	advertise<robot_msgs::NamedKinematicPath>("display_kinematic_path");

	m_basePos[0] = m_basePos[1] = m_basePos[2] = 0.0;
	m_haveBasePos = false;
	subscribe("localizedpose", m_localizedPose, &PlanKinematicPath::localizedPoseCallback);
    }
  
    bool haveBasePos(void) const
    {
	return m_haveBasePos;
    }
    
    void runTestBase(void)
    {
	robot_srvs::KinematicMotionPlan::request  req;
	
	req.model_id = "pr2::base";
	req.threshold = 0.01;
	req.distance_metric = "L2Square";
	
	req.start_state.set_vals_size(50);
	for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i)
	    req.start_state.vals[i] = 0.0;
	
	req.goal_state.set_vals_size(3);
	for (unsigned int i = 0 ; i < req.goal_state.vals_size ; ++i)
	{
	    req.goal_state.vals[i] = m_basePos[i];
	    req.start_state.vals[i] = m_basePos[i];
	}
	req.goal_state.vals[0] += 4.5;
	

	req.allowed_time = 5.0;
	
	req.volumeMin.x = -5.0 + m_basePos[0];	req.volumeMin.y = -5.0 + m_basePos[1];	req.volumeMin.z = 0.0;
	req.volumeMax.x = 5.0 + m_basePos[0];	req.volumeMax.y = 5.0 + m_basePos[1];	req.volumeMax.z = 0.0;
	
	performCall(req);
    }
    
    void runTestLeftArm(void)
    {
	robot_srvs::KinematicMotionPlan::request  req;
	
	req.model_id = "pr2::leftArm";
	req.threshold = 0.01;
	req.distance_metric = "L2Square";
	
	req.start_state.set_vals_size(50);
	for (unsigned int i = 0 ; i < req.start_state.vals_size ; ++i)
	    req.start_state.vals[i] = 0.0;
	
	req.goal_state.set_vals_size(7);
	for (unsigned int i = 0 ; i < req.goal_state.vals_size ; ++i)
	    req.goal_state.vals[i] = 0.1;
	
	req.allowed_time = 10.0;
	
	req.volumeMin.x = -5.0 + m_basePos[0];	req.volumeMin.y = -5.0 + m_basePos[1];	req.volumeMin.z = 0.0;
	req.volumeMax.x = 5.0 + m_basePos[0];	req.volumeMax.y = 5.0 + m_basePos[1];	req.volumeMax.z = 0.0;
	
	performCall(req);
    }
    
    void performCall(robot_srvs::KinematicMotionPlan::request &req)
    {	
	robot_srvs::KinematicMotionPlan::response res;
	robot_msgs::NamedKinematicPath dpath;

	if (ros::service::call("plan_kinematic_path", req, res))
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
	    dpath.name = req.model_id;
	    dpath.path = res.path;
	    publish("display_kinematic_path", dpath);
	}
	else
	    fprintf(stderr, "Service 'plan_kinematic_path' failed\n");	 
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
    
    ros::Duration dur(0.1);
    while (!plan.haveBasePos())
	dur.sleep();

    //    plan.runTestLeftArm();    
    plan.runTestBase();
    
    plan.shutdown();
    
    return 0;    
}
