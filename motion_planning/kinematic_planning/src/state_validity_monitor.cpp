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

/**

@mainpage

@htmlinclude ../manifest.html

@b StateValidityMonitor is a node capable of verifying if the current
state of the robot is valid or not.

<hr>

@section usage Usage
@verbatim
$ state_validity_monitor robot_model [standard ROS args]
@endverbatim

@par Example

@verbatim
$ state_validity_monitor robotdesc/pr2
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- None

Additional subscriptions due to inheritance from CollisionSpaceMonitor:

Publishes to (name/type):
- @b "state_validity"/Byte : 1 if state is valid, 0 if it is invalid

<hr>

@section services ROS services

Uses (name/type):
- None

Provides (name/type):
- None


<hr>

@section parameters ROS parameters
- None

**/

#include "kinematic_planning/CollisionSpaceMonitor.h"

#include <std_msgs/Byte.h>

#include <iostream>
#include <sstream>
#include <string>
#include <map>
using namespace kinematic_planning;

class StateValidityMonitor : public ros::Node,
			     public CollisionSpaceMonitor
{
public:
    
    StateValidityMonitor(const std::string &robot_model) : ros::Node("state_validity_monitor"),
							   CollisionSpaceMonitor(dynamic_cast<ros::Node*>(this),
										 robot_model)
    {
	advertise<std_msgs::Byte>("state_validity", 1);	
    }
    
    /** Free the memory */
    virtual ~StateValidityMonitor(void)
    {
    }
    
    void stateUpdate(void)
    {
	CollisionSpaceMonitor::stateUpdate();

	if (m_collisionSpace && m_collisionSpace->getModelCount() == 1)
	{
	    m_collisionSpace->lock();
	    m_kmodel->computeTransforms(m_robotState->getParams());
	    m_collisionSpace->updateRobotModel(0);
	    bool invalid = m_collisionSpace->isCollision(0);
	    m_collisionSpace->unlock();
	    std_msgs::Byte msg;
	    msg.data = invalid ? 0 : 1;
	    publish("state_validity", msg);
	    if (invalid)
		ROS_WARN("State is in collision");
	    else
		ROS_INFO("State is valid");
	}
    }
    
};

void usage(const char *progname)
{
    printf("\nUsage: %s robot_model [standard ROS args]\n", progname);
    printf("       \"robot_model\" is the name (string) of a robot description to be used for path validation.\n");
}

int main(int argc, char **argv)
{ 
    if (argc == 2)
    { 
	ros::init(argc, argv);
	
	StateValidityMonitor *validator = new StateValidityMonitor(argv[1]);
	validator->loadRobotDescription();
	validator->spin();
	validator->shutdown();
	
	delete validator;	
    }
    else
	usage(argv[0]);
        
    return 0;    
}
