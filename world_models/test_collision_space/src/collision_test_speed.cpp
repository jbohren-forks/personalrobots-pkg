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
#include <planning_environment/collision_space_monitor.h>
#include <visualization_msgs/Marker.h>

class CollisionTestSpeed
{
public:

    CollisionTestSpeed(void) 
    {
	vmPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
	cm_ = new planning_environment::CollisionModels("robot_description", 1.0, 0.0);
	id_ = 1;
    }

    virtual ~CollisionTestSpeed(void)
    {
	delete cm_;
    }

    void testPointCloud(void)
    {
	if (!cm_->loadedModels())
	    return;	
	collision_space::EnvironmentModel *em = cm_->getODECollisionModel().get();
	em->setSelfCollision(false);
	
	em->updateRobotModel();
	ROS_INFO("Collision (should be 0): %d", em->isCollision());
	
	const int n = 10000;
	double *data = new double[n * 4];
	
	for (int i = 0 ; i < n ; ++i)
	{
	    int i4 = i * 4;
	    do 
	    {
		data[i4 + 0] = uniform(1.5);
		data[i4 + 1] = uniform(1.5);
		data[i4 + 2] = uniform(1.5);
		data[i4 + 3] = 0.02;
		em->clearObstacles();
		em->addPointCloud(1, data + i4);
	    }
	    while(em->isCollision());
	}
	
	em->clearObstacles();
	em->addPointCloud(n, data);
	ROS_INFO("Added %d points", n);
	
	delete[] data;

	ROS_INFO("Collision (should be 0): %d", em->isCollision());


	const unsigned int K = 10000;
	
	ros::WallTime tm = ros::WallTime::now();
	for (unsigned int i = 0 ; i < K ; ++i)
	    em->isCollision();
	ROS_INFO("%f collision tests per second (without self collision checking)", (double)K/(ros::WallTime::now() - tm).toSec());
	
	em->setSelfCollision(true);
	tm = ros::WallTime::now();
	for (unsigned int i = 0 ; i < K ; ++i)
	    em->isCollision();
	ROS_INFO("%f collision tests per second (with self collision checking)", (double)K/(ros::WallTime::now() - tm).toSec());

	tm = ros::WallTime::now();
	for (unsigned int i = 0 ; i < K ; ++i)
	    em->isSelfCollision();
	ROS_INFO("%f collision tests per second (only self collision checking)", (double)K/(ros::WallTime::now() - tm).toSec());
    }

    void testCollision(void)
    {
	if (!cm_->loadedModels())
	    return;	
	collision_space::EnvironmentModel *em = cm_->getODECollisionModel().get();
	em->setSelfCollision(false);
	
	em->updateRobotModel();
	ROS_INFO("Collision (should be 0): %d", em->isCollision());
	
	const int n = 10000;
	double *data = new double[n * 4];
	
	for (int i = 0 ; i < n ; ++i)
	{
	    int i4 = i * 4;
	    do 
	    {
		data[i4 + 0] = uniform(1.5);
		data[i4 + 1] = uniform(1.5);
		data[i4 + 2] = uniform(1.5);
		data[i4 + 3] = 0.02;
		em->clearObstacles();
		em->addPointCloud(1, data + i4);
	    }
	    while(!em->isCollision());
	    sendPoint(data[i4], data[i4 + 1], data[i4 + 2]);
	}
	
	ROS_INFO("Added %d points", n);
	
	delete[] data;
    }

protected:

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

	mk.scale.x = mk.scale.y = mk.scale.z = 0.01;

	mk.color.a = 1.0;
	mk.color.r = 1.0;
	mk.color.g = 0.04;
	mk.color.b = 0.04;

	vmPub_.publish(mk);
    }

    // return a random number (uniform distribution)
    // between -magnitude and magnitude
    double uniform(double magnitude)
    {
	return (2.0 * drand48() - 1.0) * magnitude;
    }

    ros::Publisher                         vmPub_;
    ros::NodeHandle                        nh_;
    planning_environment::CollisionModels *cm_;
    int                                    id_;
    
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "CollisionTestSpeed");

    CollisionTestSpeed cts;
    
    cts.testPointCloud();
    cts.testCollision();
    
    ros::spin();
    
    return 0;
}
