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

#include <ros/node.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <collision_space/environmentODE.h>
#include <algorithm>
#include <std_msgs/VisualizationMarker.h>
#include <rostools/Time.h>
#include <tf/transform_broadcaster.h>
#include <collision_space/util.h>
#include <random_utils/random_utils.h>
using namespace collision_space;

const int TEST_TIMES  = 3;
const int TEST_POINTS = 50000;

class TestVM : public ros::node
{
public:
    
    TestVM(void) : ros::node("TVM")
    {
        std_msgs::VisualizationMarker mk;
	advertise("visualizationMarker", 
                  mk,
                  &TestVM::subCb,
                  10240);	
	m_tfServer = new tf::TransformBroadcaster(*this);	
	m_id = 1;
        m_connected = false;
    }

    virtual ~TestVM(void)
    { 
	if (m_tfServer)
	    delete m_tfServer; 
    }
    
    void setupTransforms(void)
    {
	m_tm = ros::Time::now();
	tf::Transform t;
	t.setIdentity();
	m_tfServer->sendTransform(tf::Stamped<tf::Transform>(t, m_tm, "base", "map"));
    }

  void subCb(const ros::PublisherPtr&)
    {
      m_connected = true;
    }
    
    void sendPoint(double x, double y, double z)
    {
	std_msgs::VisualizationMarker mk;

	mk.header.stamp = m_tm;
	
	mk.header.frame_id = "map";

	mk.id = m_id++;
	mk.type = std_msgs::VisualizationMarker::SPHERE;
	mk.action = std_msgs::VisualizationMarker::ADD;
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
	
	publish("visualizationMarker", mk);
    }

    void testShape(collision_space::bodies::Shape *s)
    {
	for (int i = 0 ; i < TEST_POINTS ; ++i)
	{
	    double x = random_utils::uniform(-5.0, 5.0);
	    double y = random_utils::uniform(-5.0, 5.0);
	    double z = random_utils::uniform(-5.0, 5.0);
	    if (!s->containsPoint(x, y, z))
		continue;
	    sendPoint(x, y, z);
	}
    }
    
    void setShapeTransformAndMarker(collision_space::bodies::Shape *s,
				    std_msgs::VisualizationMarker &mk)
    {
	btTransform t;
	
	double yaw   = random_utils::uniform(-M_PI, M_PI);
	double pitch = random_utils::uniform(-M_PI, M_PI);
	double roll  = random_utils::uniform(-M_PI, M_PI);
	
	double x = random_utils::uniform(-3.0, 3.0);
	double y = random_utils::uniform(-3.0, 3.0);
	double z = random_utils::uniform(-3.0, 3.0);
	
	t.setRotation(btQuaternion(yaw, pitch, roll));
	t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
	
	s->setPose(t);
	s->setScale(0.99);
	
	mk.header.stamp = m_tm;	
	mk.header.frame_id = "map";
	mk.id = m_id++;
	
	mk.action = std_msgs::VisualizationMarker::ADD;
	
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
	collision_space::bodies::Shape *s = new collision_space::bodies::Sphere();
	double radius[1] = {2.0};
	s->setDimensions(radius);

	for (int i = 0 ; i < TEST_TIMES ; ++i)
	{
	    std_msgs::VisualizationMarker mk;
	    setShapeTransformAndMarker(s, mk);	
	    
	    mk.type = std_msgs::VisualizationMarker::SPHERE;
	    mk.xScale = radius[0]*2.0;
	    mk.yScale = radius[0]*2.0;
	    mk.zScale = radius[0]*2.0;
	    
	    publish("visualizationMarker", mk);
	
	    testShape(s);
	}
	
	delete s;
    }

    void testBox(void)
    {
	collision_space::bodies::Shape *s = new collision_space::bodies::Box();
	double dims[3] = {2.0, 1.33, 1.5};
	s->setDimensions(dims);
	
	for (int i = 0 ; i < TEST_TIMES ; ++i)
	{
	    std_msgs::VisualizationMarker mk;
	    setShapeTransformAndMarker(s, mk);	
	    
	    mk.type = std_msgs::VisualizationMarker::CUBE;
	    mk.xScale = dims[0]; // length
	    mk.yScale = dims[1]; // width
	    mk.zScale = dims[2]; // height
	    
	    publish("visualizationMarker", mk);
	    
	    testShape(s);
	}
	
	delete s;
    }
    
    void testCylinder(void)
    {
	collision_space::bodies::Shape *s = new collision_space::bodies::Cylinder();
	double dims[2] = {2.5, 0.5};
	s->setDimensions(dims);
	
	for (int i = 0 ; i < TEST_TIMES ; ++i)
	{
	    std_msgs::VisualizationMarker mk;
	    setShapeTransformAndMarker(s, mk);	
	    
	    mk.type = std_msgs::VisualizationMarker::CUBE;
	    mk.xScale = dims[1] * 2.0; // radius
	    mk.yScale = dims[1] * 2.0; // radius
	    mk.zScale = dims[0]; //length
	    
	    publish("visualizationMarker", mk);
	    
	    testShape(s);
	}
	
	delete s;
    }

    bool isConnected()
    {
      return m_connected;
    }
    
protected:

    tf::TransformBroadcaster *m_tfServer;
    ros::Time                 m_tm;  
    int                       m_id;
    bool                      m_connected;
};

    
int main(int argc, char **argv)
{
    ros::init(argc, argv);
    
    TestVM tvm;
    ros::Duration d;
    d.fromSec(0.1);

    while(!tvm.isConnected())
      d.sleep();

    tvm.setupTransforms();    
    
    tvm.testSphere();
    tvm.testBox();
    tvm.testCylinder();
    
    tvm.spin();    
    tvm.shutdown();

    return 0;    
}
