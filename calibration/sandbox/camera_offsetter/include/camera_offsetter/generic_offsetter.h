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

//! \author Vijay Pradeep

#ifndef CAMERA_OFFSETTER_GENERIC_OFFSETTER_H_
#define CAMERA_OFFSETTER_GENERIC_OFFSETTER_H_

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <fstream>

namespace camera_offsetter
{

class GenericOffsetter
{
public:
  GenericOffsetter() : virtual_offset_( btQuaternion(0,0,0,1))
  {
    pose_sub_ = nh_.subscribe("~virtual_pose", 1, &GenericOffsetter::poseCb, this);
    twist_sub_ = nh_.subscribe("~virtual_twist", 1, &GenericOffsetter::twistCb, this);

    nh_.param("~position_scaling", position_scaling_, 1.0);
    nh_.param("~angular_scaling",  angular_scaling_,  1.0);

    nh_.param("~config_file", config_file_, std::string("N/A"));

    readConfig();
  }

  void readConfig()
  {
    if(config_file_==std::string("N/A"))
    {
      ROS_WARN("No config file is set");
      return;
    }

    geometry_msgs::PosePtr p(new geometry_msgs::Pose());
    std::fstream f (config_file_.c_str(), std::fstream::in);
    if(!f.is_open())
    {
      ROS_WARN("Couldn't open config file");
      return;
    }
    f >> p->position.x;
    f >> p->position.y;
    f >> p->position.z;
    f >> p->orientation.x;
    f >> p->orientation.y;
    f >> p->orientation.z;
    f >> p->orientation.w;
    ROS_INFO_STREAM(p->orientation.x);
    ROS_INFO_STREAM(p);
    poseCb(p);
  }
  void saveConfig()
  {
    if(config_file_==std::string("N/A"))
    {
      ROS_WARN("No config file is set");
      return;
    }
    std::fstream f (config_file_.c_str(), std::fstream::out);
    f << virtual_offset_.getOrigin().x() << " ";
    f << virtual_offset_.getOrigin().y() << " ";
    f << virtual_offset_.getOrigin().z() << std::endl;
    f << virtual_offset_.getRotation().x()<< " ";
    f << virtual_offset_.getRotation().y()<< " ";
    f << virtual_offset_.getRotation().z()<< " ";
    f << virtual_offset_.getRotation().w();
  }

  void poseCb(const geometry_msgs::PoseConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(offset_mutex_);
    tf::poseMsgToTF(*msg, virtual_offset_);
  }

  void twistCb(const geometry_msgs::TwistConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(offset_mutex_);

    btVector3 rot_vector(msg->angular.x, msg->angular.y, msg->angular.z);
    double angle = rot_vector.length();

    btQuaternion rot;
    if (angle < 1e-6)
    {
      rot = btQuaternion(0,0,0,1);
    }
    else
      rot = btQuaternion( rot_vector, angle);

    btVector3 trans( msg->linear.x, msg->linear.y, msg->linear.z);

    btTransform incrementalT(rot, trans);

    virtual_offset_ = virtual_offset_ * incrementalT;

    saveConfig();
  }

  void publishTransform(const ros::Time& time, const std::string frame_id, const std::string parent_id)
  {
    boost::mutex::scoped_lock lock(offset_mutex_);
    printf("Sending Transform: Trans:(%.3f, %.3f, %.3f)  Q:(%.3f, %.3f , %.3f, %.3f)\n",
              virtual_offset_.getOrigin().x(),
              virtual_offset_.getOrigin().y(),
              virtual_offset_.getOrigin().z(),
              virtual_offset_.getRotation().x(),
              virtual_offset_.getRotation().y(),
              virtual_offset_.getRotation().z(),
              virtual_offset_.getRotation().w());
    tf_.sendTransform(virtual_offset_, time, frame_id, parent_id);
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;

  tf::TransformBroadcaster tf_;

  double position_scaling_;
  double angular_scaling_;

  std::string config_file_;

  boost::mutex offset_mutex_;
  btTransform virtual_offset_;
} ;


}

#endif
