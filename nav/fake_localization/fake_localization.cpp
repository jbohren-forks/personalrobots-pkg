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
- @b "base_pose_ground_truth"/std_msgs::PoseWithRatesStamped : robot's odometric pose.  Only the position information is used (velocity is ignored).
- @b "initialpose"/Pose2DFloat32 : robot's odometric pose.  Only the position information is used (velocity is ignored).

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
#include <std_msgs/PoseWithRatesStamped.h>
#include <std_msgs/Pose3DStamped.h>
#include <std_msgs/ParticleCloud2D.h>
#include <std_msgs/Pose2DFloat32.h>

#include <angles/angles.h>

#include "rosconsole/rosconsole.h"

#include "tf/transform_broadcaster.h"


class FakeOdomNode: public ros::node
{
public:
    FakeOdomNode(void) : ros::node("fake_localization")
    {
      advertise<std_msgs::RobotBase2DOdom>("localizedpose",1);
      advertise<std_msgs::ParticleCloud2D>("particlecloud",1);

      m_tfServer = new tf::TransformBroadcaster(*this);	

      m_lastUpdate = ros::Time::now();
      
      m_base_pos_received = false;

      m_iniPos.x = m_iniPos.y = m_iniPos.th = 0.0;
      m_particleCloud.set_particles_size(1);

      param("max_publish_frequency", m_maxPublishFrequency, 0.5);

      subscribe("base_pose_ground_truth", m_basePosMsg, &FakeOdomNode::basePosReceived,2);
      subscribe("initialpose", m_iniPos, &FakeOdomNode::initialPoseReceived,2);
    }
    
    ~FakeOdomNode(void)
    {
      if (m_tfServer)
        delete m_tfServer; 
    }
    
    
private:
    
    tf::TransformBroadcaster       *m_tfServer;
    ros::Time                      m_lastUpdate;
    double                         m_maxPublishFrequency;
    bool                           m_base_pos_received;
    
    std_msgs::PoseWithRatesStamped  m_basePosMsg;
    std_msgs::ParticleCloud2D      m_particleCloud;
    std_msgs::RobotBase2DOdom      m_currentPos;
    std_msgs::Pose2DFloat32        m_iniPos;

    
    void initialPoseReceived(void)
    {
      if(!m_base_pos_received)
        ROS_WARN("Waiting to receive current pose on topic base_pose_ground_truth");
      else
	update();
    }
    
    void basePosReceived(void)
    {
        m_base_pos_received = true;
	update();
    }

    void update(void)
    {
      if ((ros::Time::now() - m_lastUpdate).to_double() < 1.0/m_maxPublishFrequency)
        return;

      m_lastUpdate = ros::Time::now();

      tf::Transform txi(tf::Quaternion(m_basePosMsg.pos.orientation.x,
                                       m_basePosMsg.pos.orientation.y, 
                                       m_basePosMsg.pos.orientation.z, 
                                       m_basePosMsg.pos.orientation.w),
                             tf::Point(m_basePosMsg.pos.position.x,
                                       m_basePosMsg.pos.position.y, 0.0));

      double x = txi.getOrigin().x() + m_iniPos.x;
      double y = txi.getOrigin().y() + m_iniPos.y;
      double z = txi.getOrigin().z();
      double yaw, pitch, roll;
      txi.getBasis().getEulerZYX(yaw, pitch, roll);
      yaw = angles::normalize_angle(yaw + m_iniPos.th);

      tf::Transform txo(tf::Quaternion(yaw, pitch, roll),
                        tf::Point(x, y, z));

      m_tfServer->sendTransform(tf::Stamped<tf::Transform>
                                (txo.inverse(),
                                 m_basePosMsg.header.stamp,
                                 "map", "base_link"));

      m_currentPos.header = m_basePosMsg.header;
      m_currentPos.pos.x = x;
      m_currentPos.pos.y = y;
      m_currentPos.pos.th = yaw;

      publish("localizedpose", m_currentPos);

      m_particleCloud.particles[0] = m_currentPos.pos;
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
