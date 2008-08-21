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

/** \Author Ioan Sucan */

/**

@mainpage

@htmlinclude manifest.html

@b odom_localization is simply forwards the odometry information 

<hr>

@section usage Usage
@verbatim
$ odom_localization
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "odom"/RobotBase2DOdom : robot's odometric pose.  Only the position information is used (velocity is ignored).

Publishes to (name / type):
- @b "localizedpose"/RobotBase2DOdom : robot's localized map pose.  Only the position information is set (no velocity).
- @b "particlecloud"/ParticleCloud2D : fake set of particles being maintained by the filter (one paricle only).

<hr>

@section parameters ROS parameters

- None

 **/

#include <ros/node.h>
#include <ros/time.h>

#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/ParticleCloud2D.h>
#include <std_msgs/Pose2DFloat32.h>

#include <math_utils/angles.h>
#include <rosTF/rosTF.h>


class FakeOdomNode: public ros::node
{
public:
    FakeOdomNode(void) : ros::node("fake_localization")
    {
	advertise<std_msgs::RobotBase2DOdom>("localizedpose");
	advertise<std_msgs::ParticleCloud2D>("particlecloud");
	
	m_tfServer = new rosTFServer(*this);	

	m_lastUpdate = ros::Time::now();
		
	m_iniPos.x = m_iniPos.y = m_iniPos.th = 0.0;
	m_particleCloud.set_particles_size(1);

	param("max_publish_frequency", m_maxPublishFrequency, 0.5);
	subscribe("odom", m_odomMsg, &FakeOdomNode::odomReceived);
	subscribe("initialpose", m_iniPos, &FakeOdomNode::initialPoseReceived);
    }
    
    ~FakeOdomNode(void)
    {
	if (m_tfServer)
	    delete m_tfServer; 
    }
    
    
private:
    
    rosTFServer              *m_tfServer;
    ros::Time                 m_lastUpdate;
    double                    m_maxPublishFrequency;
    
    std_msgs::RobotBase2DOdom m_odomMsg;
    std_msgs::ParticleCloud2D m_particleCloud;
    std_msgs::RobotBase2DOdom m_currentOdom;
    std_msgs::Pose2DFloat32   m_iniPos;

    
    void initialPoseReceived(void)
    {
	update();
    }
    
    void odomReceived(void)
    {
	update();
    }

    void update(void)
    {
	if ((ros::Time::now() - m_lastUpdate).to_double() < 1.0/m_maxPublishFrequency)
	    return;
	
	m_lastUpdate = ros::Time::now();
	
	m_currentOdom = m_odomMsg;
	
	m_currentOdom.pos.x += m_iniPos.x;
	m_currentOdom.pos.y += m_iniPos.y;
	m_currentOdom.pos.th = math_utils::normalize_angle(m_currentOdom.pos.th + m_iniPos.th);
	m_currentOdom.header.frame_id = "FRAMEID_MAP";
	
	m_tfServer->sendEuler("FRAMEID_ROBOT",
			      "FRAMEID_MAP",
			      m_currentOdom.pos.x,
			      m_currentOdom.pos.y,
			      0.0,
			      m_currentOdom.pos.th,
			      0.0,
			      0.0,
			      m_currentOdom.header.stamp); 
			      
	publish("localizedpose", m_currentOdom);
	
	m_particleCloud.particles[0] = m_currentOdom.pos;
	publish("particlecloud", m_particleCloud);
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv);
    
    FakeOdomNode odom;
    odom.spin();
    odom.shutdown();    
    
    return 0;
}
