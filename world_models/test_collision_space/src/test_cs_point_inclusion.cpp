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

#include <cstdlib>
#include <ros/node.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <collision_space/environmentODE.h>
#include <algorithm>
#include <robot_msgs/VisualizationMarker.h>
#include <roslib/Time.h>
#include <collision_space/point_inclusion.h>
using namespace collision_space;

const int TEST_TIMES  = 3;
const int TEST_POINTS = 50000;

namespace planning_models
{
    shapes::Mesh* create_mesh_from_vertices(const std::vector<btVector3> &source);
}

class TestVM
{
public:
    
    TestVM(void) : m_node("TVM")
    {
	m_node.advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 10240);
	m_id = 1;
    }

    virtual ~TestVM(void)
    { 
    }
    
    void sendPoint(double x, double y, double z)
    {
	robot_msgs::VisualizationMarker mk;

	mk.header.stamp = m_tm;
	
	mk.header.frame_id = "base_link";

	mk.id = m_id++;
	mk.type = robot_msgs::VisualizationMarker::SPHERE;
	mk.action = robot_msgs::VisualizationMarker::ADD;
	mk.x = x;
	mk.y = y;
	mk.z = z;

	mk.roll = 0;
	mk.pitch = 0;
	mk.yaw = 0;
	
	mk.xScale = 0.2;
	mk.yScale = 0.2;
	mk.zScale = 0.2;
		
	mk.alpha = 255;
	mk.r = 255;
	mk.g = 10;
	mk.b = 10;
	
	m_node.publish("visualizationMarker", mk);
    }

    void testShape(collision_space::bodies::Body *s)
    {
	for (int i = 0 ; i < TEST_POINTS ; ++i)
	{
	    double x = uniform(5.0);
	    double y = uniform(5.0);
	    double z = uniform(5.0);
	    if (!s->containsPoint(x, y, z))
		continue;
	    sendPoint(x, y, z);
	}
    }
    
    void setShapeTransformAndMarker(collision_space::bodies::Body *s,
				    robot_msgs::VisualizationMarker &mk)
    {
	btTransform t;
	
	double yaw   = uniform(M_PI);
	double pitch = uniform(M_PI);
	double roll  = uniform(M_PI);
	
	double x = uniform(3.0);
	double y = uniform(3.0);
	double z = uniform(3.0);
	
	t.setRotation(btQuaternion(yaw, pitch, roll));
	t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
	
	s->setPose(t);
	s->setScale(0.99);
	
	mk.header.stamp = m_tm;	
	mk.header.frame_id = "base_link";
	mk.id = m_id++;
	
	mk.action = robot_msgs::VisualizationMarker::ADD;
	
	mk.x = x;
	mk.y = y;
	mk.z = z;
	
	mk.roll = roll;
	mk.pitch = pitch;
	mk.yaw = yaw;
	
	mk.alpha = 150;
	mk.r = 0;
	mk.g = 100;
	mk.b = 255;
    }
    
    void testSphere(void)
    {
	planning_models::shapes::Sphere shape(2.0);	
	collision_space::bodies::Body *s = new collision_space::bodies::Sphere(&shape);

	for (int i = 0 ; i < TEST_TIMES ; ++i)
	{
	    robot_msgs::VisualizationMarker mk;
	    setShapeTransformAndMarker(s, mk);	
	    
	    mk.type = robot_msgs::VisualizationMarker::SPHERE;
	    mk.xScale = shape.radius*2.0;
	    mk.yScale = shape.radius*2.0;
	    mk.zScale = shape.radius*2.0;
	    
	    m_node.publish("visualizationMarker", mk);
	
	    testShape(s);
	}
	
	delete s;
    }

    void testBox(void)
    {
	planning_models::shapes::Box shape(2.0, 1.33, 1.5);
	collision_space::bodies::Body *s = new collision_space::bodies::Box(&shape);
	
	for (int i = 0 ; i < TEST_TIMES ; ++i)
	{
	    robot_msgs::VisualizationMarker mk;
	    setShapeTransformAndMarker(s, mk);	
	    
	    mk.type = robot_msgs::VisualizationMarker::CUBE;
	    mk.xScale = shape.size[0]; // length
	    mk.yScale = shape.size[1]; // width
	    mk.zScale = shape.size[2]; // height
	    
	    m_node.publish("visualizationMarker", mk);
	    
	    testShape(s);
	}
	
	delete s;
    }
    
    void testCylinder(void)
    {
	planning_models::shapes::Cylinder shape(0.5, 2.5);
	collision_space::bodies::Body *s = new collision_space::bodies::Cylinder(&shape);
	
	for (int i = 0 ; i < TEST_TIMES ; ++i)
	{
	    robot_msgs::VisualizationMarker mk;
	    setShapeTransformAndMarker(s, mk);	
	    
	    mk.type = robot_msgs::VisualizationMarker::CUBE;
	    mk.xScale = shape.radius * 2.0; // radius
	    mk.yScale = shape.radius * 2.0; // radius
	    mk.zScale = shape.length; //length
	    
	    m_node.publish("visualizationMarker", mk);
	    
	    testShape(s);
	}
	
	delete s;
    }
    
    void testConvexMesh(void)
    {
	std::vector<btVector3> pts(4);
	pts[0] = btVector3(0,0,1);
	pts[1] = btVector3(1,0,-0.3);
	pts[2] = btVector3(-0.5,0.8,-0.3);
	pts[3] = btVector3(-0.5,-0.8,-0.3);

	planning_models::shapes::Mesh *shape = planning_models::create_mesh_from_vertices(pts);
	collision_space::bodies::Body *s = new collision_space::bodies::ConvexMesh(shape);

	robot_msgs::VisualizationMarker mk;
	setShapeTransformAndMarker(s, mk);	
	testShape(s);
	
	delete s;
	delete shape;
    }
    
    void run()
    {
	ros::Duration d;
	d.fromSec(2);
	d.sleep();
	
	m_tm = ros::Time::now();
	
	testSphere();
	testBox();
	testCylinder();
	
	m_node.spin();    
	m_node.shutdown();
	
    }
    
protected:
    
    // return a random number (uniform distribution)
    // between -magnitude and magnitude
    double uniform(double magnitude)
    {
	return (2.0 * drand48() - 1.0) * magnitude;	
    }
    
    ros::Node                 m_node;
    ros::Time                 m_tm;  
    int                       m_id;
    
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv);
    
    TestVM tvm;
    tvm.run();
    
    return 0;    
}
