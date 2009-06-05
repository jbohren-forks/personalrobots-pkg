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

// Author: Stuart Glaser

#include <cstdio>
#include <algorithm>
#include "ros/ros.h"
#include "ros/console.h"
#include "ros/node_handle.h"

#include "tf/transform_datatypes.h"
#include "robot_msgs/Twist.h"
#include "robot_msgs/PoseDot.h"

const char *TIP_FRAME = "";
const char *ROOT_FRAME = "base_link";
const tf::Vector3 OFFSET(0.2, 0, 0);
const std::string CONTROLLER("/r_arm_cartesian_pose_controller");
const tf::Vector3 POSITION(0.7, -0.2, 1.4);
const std::string DRIVE_TOPIC("/cmd_vel");
const double K = 10;

double apply_dead_zone(double x, double dead_zone)
{
  if (x >= 0)
    return std::max(0.0, x - dead_zone);
  else
    return std::min(0.0, x + dead_zone);
}

class X {
public:
  X(ros::NodeHandle &node_) : node(node_)
  {
    pub_drive = node.advertise<robot_msgs::PoseDot>(DRIVE_TOPIC, 1);
    sub_error = node.subscribe(CONTROLLER + "/state/error", 1, &X::stateCB, this);

    node.param("~dead_zone", dead_zone, 0.01);
    node.param("~k_trans", k_trans, 10.0);
    node.param("~k_rot", k_rot, 3.0);
  }

  ~X()
  {
    pub_drive.shutdown();
    sub_error.shutdown();
  }

  void stateCB(const robot_msgs::TwistConstPtr &msg)
  {
    robot_msgs::PoseDot base_vel;
    //ROS_INFO("%lf  %lf  %lf -- %lf  %lf", msg->vel.x, dead_zone, apply_dead_zone(msg->vel.x, dead_zone), k_trans, k_trans * apply_dead_zone(msg->vel.x, dead_zone));
    base_vel.vel.vx = k_trans * apply_dead_zone(msg->vel.x, dead_zone);
    base_vel.vel.vy = k_trans * apply_dead_zone(msg->vel.y, dead_zone);
    base_vel.ang_vel.vz = k_rot * apply_dead_zone(msg->rot.z, dead_zone);
    pub_drive.publish(base_vel);
  }

  ros::NodeHandle node;
  ros::Publisher pub_drive;
  ros::Subscriber sub_error;

  robot_msgs::Twist error_msg;

  double dead_zone;
  double k_trans;
  double k_rot;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handhold");
  ros::NodeHandle node("handhold");

  robot_msgs::PoseStamped cmd;
  node.param("~pose_frame", cmd.header.frame_id, std::string("base_link"));
  node.param("~pose_x", cmd.pose.position.x, 0.4);
  node.param("~pose_y", cmd.pose.position.y, -0.3);
  node.param("~pose_z", cmd.pose.position.z, 0.7);
  node.param("~pose_qx", cmd.pose.orientation.x, 0.0);
  node.param("~pose_qy", cmd.pose.orientation.y, 0.0);
  node.param("~pose_qz", cmd.pose.orientation.z, 0.0);
  node.param("~pose_qw", cmd.pose.orientation.w, 1.0);
  ros::Publisher pub_cmd = node.advertise<robot_msgs::PoseStamped>(CONTROLLER + "/command", 2);
  for (int i = 0; i < 30; ++i) {
    pub_cmd.publish(cmd);
    ros::Duration(0.1).sleep();
  }
  pub_cmd.shutdown();

  X x(node);
  ros::spin();

  return 0;
}
