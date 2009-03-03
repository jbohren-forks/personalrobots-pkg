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

/** This is a simple program for testing how the joints are moved
    between the defined limits. Agreement with the mechanism control
    is looked at as well.  */

#include <kinematic_planning/KinematicStateMonitor.h>
#include <robot_msgs/JointTraj.h>
#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>
#include <cassert>
    
class TestExecutionPath : public kinematic_planning::KinematicStateMonitor
{
public:
    
    TestExecutionPath(ros::Node *node) : kinematic_planning::KinematicStateMonitor(node)
    {
	m_node->advertise<robot_msgs::JointTraj>("right_arm_trajectory_command", 1);
	sleep_duration_ = 4;
	use_topic_ = false;
    }
    
    void testJointLimitsRightArm(const std::string& jname = "")
    {
	// we send a trajectory with one state
	robot_msgs::JointTraj traj;
	const int controllerDim = 7;
	std::string groupName = "pr2::right_arm";
	traj.set_points_size(1);
        traj.points[0].set_positions_size(controllerDim);


	std::vector<std::string> joints;
	if (jname.empty())
	{
	    // get the joints in the group
	    m_kmodel->getJointsInGroup(joints, m_kmodel->getGroupID(groupName));
	}
	else
	{
	    joints.push_back(jname);
	}
	
        for (unsigned int j = 0 ;  j < joints.size() ; ++j)
        {
	    // we only test revolute joints
            planning_models::KinematicModel::RevoluteJoint *joint = dynamic_cast<planning_models::KinematicModel::RevoluteJoint*>(m_kmodel->getJoint(joints[j]));
            if (!joint) continue;
            
	    printf("Testing '%s': [%f, %f]\n", joint->name.c_str(), joint->limit[0], joint->limit[1]);
	    fprintf(stderr, "%s:\n", joint->name.c_str());
	    
	    // check the value of the joint at small increments
            planning_models::KinematicModel::StateParams *sp = m_kmodel->newStateParams();
            for (double val = joint->limit[0] ; val <= joint->limit[1] ; val += 0.1)
            {
             	double to_send[controllerDim];
                for (int i = 0 ;  i < controllerDim ; ++i)
	            to_send[i] = 0.0;
                sp->setParamsJoint(&val, joints[j]);
                sp->copyParamsGroup(to_send, groupName);
		int index = sp->getJointIndexInGroup(joints[j], groupName);
		
                for (unsigned int i = 0 ;  i < traj.points[0].get_positions_size() ; ++i)
                    traj.points[0].positions[i] = to_send[i];
		
		if (use_topic_)
		{
		    m_node->publish("right_arm_trajectory_command", traj);
		    sleep(sleep_duration_);
		}		
		else
		{
		    pr2_mechanism_controllers::TrajectoryStart::Request  send_traj_start_req;
		    pr2_mechanism_controllers::TrajectoryStart::Response send_traj_start_res;
		    
		    pr2_mechanism_controllers::TrajectoryQuery::Request  send_traj_query_req;
		    pr2_mechanism_controllers::TrajectoryQuery::Response send_traj_query_res;
		    
		    send_traj_start_req.traj = traj;
		    int traj_done = -1;
		    if (ros::service::call("right_arm_trajectory_controller/TrajectoryStart", send_traj_start_req, send_traj_start_res))
		    {
			ROS_INFO("Sent trajectory to controller");
			
			send_traj_query_req.trajectoryid =  send_traj_start_res.trajectoryid;
			while(!(traj_done == send_traj_query_res.State_Done || traj_done == send_traj_query_res.State_Failed))
			{
			    if(ros::service::call("right_arm_trajectory_controller/TrajectoryQuery",  send_traj_query_req,  send_traj_query_res))  
				traj_done = send_traj_query_res.done;
			    else
			    {
				ROS_ERROR("Trajectory query failed");
			    }
			}
			ROS_INFO("Trajectory execution is complete");	    
		    }
		}
		
		printf("Sent: ");
		for (unsigned int i = 0 ;  i < traj.points[0].get_positions_size() ; ++i)
		    printf("%f ", traj.points[0].positions[i]);
		printf("\n");
		
		m_robotState->copyParamsGroup(to_send, groupName);
                printf("Achieved: ");
                for (unsigned int i = 0 ;  i < traj.points[0].get_positions_size() ; ++i)
                    printf("%f ", to_send[i]);
		printf("\n\n");

		fprintf(stderr, "%f %f\n", traj.points[0].positions[index], to_send[index]);
		break;
		
            }

            delete sp;
	}
	
    }
    
    void run(void)
    {
	loadRobotDescription();
	if (loadedRobot())
	{
	    sleep(1);
	    testJointLimitsRightArm();
	}
	sleep(1);
    }
    
protected:
    
    int  sleep_duration_;
    bool use_topic_;
    
};


int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    ros::Node node("test_kinematic_path");
    TestExecutionPath plan(&node);
    plan.run();
    
    return 0;    
}
