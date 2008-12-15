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
#include <collision_space/environmentODE.h>
#include <algorithm>
#include <std_msgs/VisualizationMarker.h>
#include <rostools/Time.h>
#include "tf/transform_broadcaster.h"

using namespace collision_space;

class TestVM : public ros::node
{
public:
    
    TestVM(void) : ros::node("TVM")
    {
	advertise<std_msgs::VisualizationMarker>("visualizationMarker", 1);	
	advertise<rostools::Time>("time", 1);
	m_tfServer = new tf::TransformBroadcaster(*this);	
    }

    virtual ~TestVM(void)
    { 
	if (m_tfServer)
	    delete m_tfServer; 
    }
    
    void test1(void)
    {

	// Set ROS time:
	rostools::Time tm;
	tm.rostime.sec  = 1;
	tm.rostime.nsec = 0;
	//	tm.header.frame_id = "base"; 
	// Do I need to set this?
	publish("time", tm);
		

	// Send a transform
	// 'map' is assumed to be the fixed frame in ROS, right?
	tf::Transform t;
	t.setIdentity();
	
	//	m_tfServer->sendTransform(tf::Stamped<tf::Transform>(t, tm.rostime, "base", "map"));
	m_tfServer->sendTransform(tf::Stamped<tf::Transform>(t, tm.rostime, "base", "map"));
	sleep(1);
	

	// send my marker:
	std_msgs::VisualizationMarker mk;

	mk.header.stamp = tm.rostime;
	
	mk.header.frame_id = "map";

	mk.id = 52;
	mk.type = std_msgs::VisualizationMarker::SPHERE;
	mk.action = std_msgs::VisualizationMarker::ADD;
	mk.x = 0;
	mk.y = 1;
	mk.z = 0;

	mk.roll = 0;
	mk.pitch = 0;
	mk.yaw = 0;
	
	mk.xScale = 10;
	mk.yScale = 10;
	mk.zScale = 10;
		
	mk.alpha = 0.1;
	mk.r = 100;
	mk.g = 100;
	mk.b = 250;
	
	publish("visualizationMarker", mk);
	
    }

protected:

    tf::TransformBroadcaster *m_tfServer;
    
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv);
    
    TestVM tvm;
    sleep(1);
    
    tvm.test1();    

    sleep(3);
    tvm.shutdown();

    return 0;    
}

