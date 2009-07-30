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
#include <planning_environment/monitors/collision_space_monitor.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread.hpp>

class CollisionTestSpeed
{
public:

    CollisionTestSpeed(void) 
    {
	vmPub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
	cm_ = new planning_environment::CollisionModels("robot_description", 1.0, 0.02);
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
	
	std::vector<shapes::Shape*> spheres;
	std::vector<btTransform>    poses;
	
	for (int i = 0 ; i < n ; ++i)
	{
	    btTransform pos;
	    pos.setIdentity();
	    do 
	    {
		shapes::Sphere* obj = new shapes::Sphere(0.02);
		em->clearObjects();
		pos.setOrigin(btVector3(uniform(1.5), uniform(1.5), uniform(1.5)));
		em->addObject("points", obj, pos);
		if (n < 100)
		{
		    collision_space::EnvironmentModel *clone = em->clone();
		    clone->updateRobotModel();
		    if (clone->isCollision() != em->isCollision())
			ROS_ERROR("Error in cloning");
		    delete clone;
		}
	    }
	    while(em->isCollision());
	    spheres.push_back(new shapes::Sphere(0.02));
	    poses.push_back(pos);
	}
	
	em->clearObjects();
	em->addObjects("points", spheres, poses);
	ROS_INFO("Added %d spheres", n);
	
	ROS_INFO("Collision (should be 0): %d", em->isCollision());
	collision_space::EnvironmentModel *clone = em->clone();
	clone->updateRobotModel();
	clone->setVerbose(true);
	ROS_INFO("Collision of clone (should be 0): %d", clone->isCollision());
	delete clone;
	
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

	const unsigned int C = 100;
	tm = ros::WallTime::now();
	for (unsigned int i = 0 ; i < C ; ++i)
	{
	    collision_space::EnvironmentModel *clone = em->clone();
	    clone->updateRobotModel();
	    if (em->isCollision() != clone->isCollision())
		ROS_ERROR("Cloning is problematic");
	    delete clone;
	}	
	ROS_INFO("%f collision tests + clones per second", (double)C/(ros::WallTime::now() - tm).toSec());
    }
    
    void testCollision(void)
    {
	if (!cm_->loadedModels())
	    return;	
	collision_space::EnvironmentModel *em = cm_->getODECollisionModel()->clone();
	em->setSelfCollision(false);
	
	em->updateRobotModel();
	ROS_INFO("Collision (should be 0): %d", em->isCollision());
	
	const int n = 10000;
	
	for (int i = 0 ; i < n ; ++i)
	{
	    btTransform pos;
	    pos.setIdentity();
	    do 
	    {	shapes::Sphere* obj = new shapes::Sphere(0.02);
		em->clearObjects();
		pos.setOrigin(btVector3(uniform(1.5), uniform(1.5), uniform(1.5)));
		em->addObject("points", obj, pos);
	    }
	    while(!em->isCollision());
	    sendPoint(pos.getOrigin().x(), pos.getOrigin().y(), pos.getOrigin().z());
	}
	
	ROS_INFO("Added %d points", n);
	
	delete em;
    }

    void collisionThread(int tid, collision_space::EnvironmentModel *emx)
    {
	collision_space::EnvironmentModel *em = emx->clone();
	em->updateRobotModel();
	
	ROS_INFO("Thread %d using instance %p, collision = %d (should be 0)", tid, em, em->isCollision());
	
	const unsigned int K = 10000;
	
	sleep(1);
	
	ros::WallTime tm = ros::WallTime::now();
	for (unsigned int i = 0 ; i < K ; ++i)
	    em->isCollision();
	ROS_INFO("Thread %d: %f collision tests per second (with self collision checking)", tid, (double)K/(ros::WallTime::now() - tm).toSec());
	
	delete em;
    }

    void collisionSetupThread(collision_space::EnvironmentModel *em)
    {
	const int n = 10000;
	
	std::vector<shapes::Shape*> spheres;
	std::vector<btTransform>    poses;
	
	for (int i = 0 ; i < n ; ++i)
	{
	    btTransform pos;
	    pos.setIdentity();
	    do 
	    {	shapes::Sphere* obj = new shapes::Sphere(0.02);
		em->clearObjects();
		pos.setOrigin(btVector3(uniform(1.5), uniform(1.5), uniform(1.5)));
		em->addObject("points", obj, pos);
	    }
	    while(em->isCollision());
	    spheres.push_back(new shapes::Sphere(0.02));
	    poses.push_back(pos);
	}
	
	em->clearObjects();
	em->addObjects("points", spheres, poses);
	ROS_INFO("Added %d points", n);	
	ROS_INFO("Collision after thread setup (should be 0): %d", em->isCollision());

    }
    
    void testThreads(void)
    {
	if (!cm_->loadedModels())
	    return;
	
	collision_space::EnvironmentModel *em = cm_->getODECollisionModel().get();
	em->setSelfCollision(true);
	em->updateRobotModel();
	
	boost::thread t(boost::bind(&CollisionTestSpeed::collisionSetupThread, this, em));
	t.join();
	
	int nt = 2;
	std::vector<boost::thread*> th(nt);
	for (int i = 0 ; i < nt ; ++i)
	    th[i] = new boost::thread(boost::bind(&CollisionTestSpeed::collisionThread, this, i, em));
	for (int i = 0 ; i < nt ; ++i)
	{
	    th[i]->join();
	    delete th[i];
	}
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
    cts.testThreads();
    
    ROS_INFO("Done");
    ros::spin();
    
    return 0;
}
