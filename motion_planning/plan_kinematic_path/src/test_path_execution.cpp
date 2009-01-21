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
#include <pr2_mechanism_controllers/JointTraj.h>
#include <cassert>
    
class TestExecutionPath : public ros::Node,
			  public kinematic_planning::KinematicStateMonitor
{
public:
    
    TestExecutionPath(const std::string& robot_model) : ros::Node("plan_kinematic_path"),
							kinematic_planning::KinematicStateMonitor(dynamic_cast<ros::Node*>(this), robot_model)
    {
	advertise<pr2_mechanism_controllers::JointTraj>("arm_trajectory_command", 1);	
	sleep_duration_ = 4;
    }
    
    void testJointLimitsRightArm(void)
    {
	// we send a trajectory with one state
	pr2_mechanism_controllers::JointTraj traj;
	const int controllerDim = 8;
	std::string groupName = "pr2::right_arm";
	traj.set_points_size(1);
        traj.points[0].set_positions_size(controllerDim);

	// get the joints in the group
        std::vector<std::string> joints;
        m_kmodel->getJointsInGroup(joints, m_kmodel->getGroupID(groupName));

        for (unsigned int j = 0 ;  j < joints.size() ; ++j)
        {
	    // we only test revolute joints
            planning_models::KinematicModel::RevoluteJoint *joint = dynamic_cast<planning_models::KinematicModel::RevoluteJoint*>(m_kmodel->getJoint(joints[j]));
            if (!joint) continue;
            
	    printf("Testing '%s': [%f, %f]\n", joint->name.c_str(), joint->limit[0], joint->limit[1]);
	    fprintf(stderr, "%s:\n", joint->name.c_str());
	    
	    // check the value of the joint at small increments
            planning_models::KinematicModel::StateParams *sp = m_kmodel->newStateParams();
            for (double val = joint->limit[0] ; val <= joint->limit[1] ; val += 0.05)
            {
             	double to_send[controllerDim];
                for (int i = 0 ;  i < controllerDim ; ++i)
	            to_send[i] = 0.0;
                sp->setParams(&val, joints[j]);
                sp->copyParams(to_send, m_kmodel->getGroupID(groupName));
		int index = sp->getPos(joints[j], m_kmodel->getGroupID(groupName));
		
                for (unsigned int i = 0 ;  i < traj.points[0].get_positions_size() ; ++i)
                    traj.points[0].positions[i] = to_send[i];
		
                publish("arm_trajectory_command", traj);
	        sleep(sleep_duration_);

		printf("Sending: ");
		for (unsigned int i = 0 ;  i < traj.points[0].get_positions_size() ; ++i)
		    printf("%f ", traj.points[0].positions[i]);
		printf("\n");
		
		m_robotState->copyParams(to_send, m_kmodel->getGroupID(groupName));
                printf("Achieved: ");
                for (unsigned int i = 0 ;  i < traj.points[0].get_positions_size() ; ++i)
                    printf("%f ", to_send[i]);
		printf("\n\n");

		fprintf(stderr, "%f %f\n", traj.points[0].positions[index], to_send[index]);
            }

            delete sp;
        }
    }
    
protected:
    
    int sleep_duration_;
    
};


void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used when building the map.\n");
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv);
    
    if (argc >= 2)
    {
	TestExecutionPath *plan = new TestExecutionPath(argv[1]);
	plan->loadRobotDescription();
	if (plan->loadedRobot())
	{
	    sleep(2);
	    plan->testJointLimitsRightArm();
	}
	sleep(1);
	
	plan->shutdown();
	delete plan;
    }
    else
	usage(argv[0]);
    
    return 0;    
}
