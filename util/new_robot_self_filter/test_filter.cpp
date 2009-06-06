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

#include <ros/ros.h>
#include "self_see_filter.h"
#include <visualization_msgs/Marker.h>

class TestSelfFilter
{
public:

    TestSelfFilter(void)
    {
	id_ = 1;
	sf_.configure();
	vmPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
    }
    
    void sendPoint(double x, double y, double z)
    {
	visualization_msgs::Marker mk;

	mk.header.stamp = ros::Time::now();

	mk.header.frame_id = "base_link";

	mk.ns = "test_self_filter";
	mk.id = id_++;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;
	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = 0.02;

	mk.color.a = 1.0;
	mk.color.r = 1.0;
	mk.color.g = 0.04;
	mk.color.b = 0.04;

	vmPub_.publish(mk);
    }
    void run(void)
    {
	robot_msgs::PointCloud in;
	robot_msgs::PointCloud out;
	
	in.header.stamp = ros::Time::now();
	in.header.frame_id = "base_link";
	
	const unsigned int N = 100000;	
	in.pts.resize(N);
	for (unsigned int i = 0 ; i < N ; ++i)
	{
	    in.pts[i].x = uniform(1);
	    in.pts[i].y = uniform(1);
	    in.pts[i].z = uniform(1);
	}
	
	ros::WallTime tm = ros::WallTime::now();
	sf_.update(in, out);
	printf("%f points per second\n", (double)N/(ros::WallTime::now() - tm).toSec());
	
	for (unsigned int i = 0 ; i < out.pts.size() ; ++i)
	{
	    sendPoint(out.pts[i].x, out.pts[i].y, out.pts[i].z);
	}
	
	ros::spin();	
    }
protected:
    
    double uniform(double magnitude)
    {
	return (2.0 * drand48() - 1.0) * magnitude;
    }

    ros::Publisher                              vmPub_;
    ros::NodeHandle                             nodeHandle_;        
    filters::SelfFilter<robot_msgs::PointCloud> sf_;
    int                                         id_;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_self_filter");

    TestSelfFilter t;
    sleep(1);
    t.run();
    
    return 0;
}

    
    
