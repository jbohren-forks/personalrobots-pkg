/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <boost/shared_ptr.hpp>
#include <ros/node.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <ros/time.h>
#include <robot_msgs/PositionMeasurement.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/PointStamped.h>
#include <robot_msgs/PointCloud.h>
#include <deprecated_msgs/Pose2DFloat32.h>
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

using robot_msgs::Point;
using robot_msgs::PointStamped;
using robot_msgs::PositionMeasurement;
using robot_msgs::PointCloud;
using deprecated_msgs::Pose2DFloat32;
using ros::Node;
using ros::Time;
using ros::Duration;
using tf::TransformException;
using std::string;

typedef tf::Stamped<tf::Pose> StampedPose;
typedef tf::MessageNotifier<PointCloud> Notifier;
typedef boost::shared_ptr<Notifier> NotifierPtr;

double getYaw(const StampedPose& tf_pose)
{
  double pitch, roll, yaw;
  tf_pose.getBasis().getEulerZYX(yaw, pitch, roll);
  return yaw;
}


class PersonPosSender
{
public:
  PersonPosSender() : node_("person_pos_sender"), tf_(node_)
  {

    node_.param("/global_frame_id", global_frame_, string("/map"));

    node_.subscribe("people_tracker_measurements", person_message_, &PersonPosSender::posCallback, this, 1);
    hallway_cloud_notifier_ = NotifierPtr(new Notifier(&tf_, &node_, bind(&PersonPosSender::hallwayCallback, this, _1),
                                                       "parallel_lines_model", global_frame_, 50));
                                                       

    node_.advertise<Point>("person_position", 1);
    node_.advertise<PointCloud>("hallway_points", 1);
    node_.advertise<Pose2DFloat32>("robot_pose", 1);
  }

  void posCallback ()
  {
    try {
    PointStamped point, transformedPoint;
    point.point = person_message_.pos;
    point.header = person_message_.header;
    tf_.transformPoint (global_frame_, Time(), point, global_frame_, transformedPoint);
    node_.publish("person_position", transformedPoint.point);
    }
    catch (TransformException& e) {
      ROS_INFO_STREAM ("Tf exception when transforming person position: " << e.what());
    }
  }

  void hallwayCallback (const Notifier::MessagePtr& hallway_message)
  {
    PointCloud msg;
    try {
      tf_.transformPointCloud(global_frame_, *hallway_message, msg);
      node_.publish("hallway_points", msg);
    }
    catch (TransformException& e)
    {
      ROS_INFO_STREAM ("TF exception transforming hallway " << e.what());
    }
  }

  void spin () 
  {
    Duration d(.5);


    while (node_.ok()) 
    {
      StampedPose id, odom_pose;
      id.setIdentity();
      d.sleep();
      id.frame_id_ = "base_footprint";
      id.stamp_ = Time();
      try {
        tf_.transformPose(global_frame_, id, odom_pose);
        Pose2DFloat32 pose;
        pose.x = odom_pose.getOrigin().x();
        pose.y = odom_pose.getOrigin().y();
        pose.th = getYaw(odom_pose);
        node_.publish("robot_pose", pose);
      }
      catch (TransformException& e)
      {
        ROS_INFO_STREAM ("TF exception transforming odom pose " << e.what());
      }
    }
  }


private:
  Node node_;
  string global_frame_;
  PositionMeasurement person_message_;
  PointCloud hallway_points_;
  tf::TransformListener tf_;
  NotifierPtr hallway_cloud_notifier_;
};
  

int main (int argc, char** argv)
{
  ros::init(argc, argv);
  PersonPosSender sender;
  sender.spin();
}


  

  
