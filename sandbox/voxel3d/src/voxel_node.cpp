/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "boost/scoped_ptr.hpp"
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "tf/transform_listener.h"
#include "voxel3d/voxel3d.h"

#include "robot_msgs/PointCloud.h"

int world_to_voxel(double x)
{
  return (int)(x * 100.0) + 100;
}

class VoxelNode
{
public:
  VoxelNode(const ros::NodeHandle &node)
    : node_(node), TF(*node_.getNode())
  {
    node_.param("~frame", frame_, std::string("torso_lift_link"));

    bool visualize;
    node_.param("~visualize", visualize, false);
    voxel_.reset(new Voxel3d(200,200,200, 0.01, tf::Vector3(-1,-1,-1), visualize));

    sub_cloud_ = node_.subscribe("cloud", 1, &VoxelNode::cloudCB, this);
  }

  ~VoxelNode()
  {
    sub_cloud_.shutdown();
  }

private:
  ros::NodeHandle node_;
  std::string frame_;
  tf::TransformListener TF;
  ros::Subscriber sub_cloud_;

  boost::scoped_ptr<Voxel3d> voxel_;

  void cloudCB(const robot_msgs::PointCloudConstPtr &msg_orig)
  {
    try
    {
      ros::Time start = ros::Time::now();

      robot_msgs::PointCloud msg;
      TF.transformPointCloud(frame_, *msg_orig, msg);
      ROS_INFO("Transform took %lf seconds", (ros::Time::now() - start).toSec());
      start = ros::Time::now();

      voxel_->reset();
      voxel_->updateWorld(msg);

      ros::Duration d = ros::Time::now() - start;
      ROS_INFO("Took %lf seconds", (ros::Time::now() - start).toSec());
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Transform exception: %s", ex.what());
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "voxel");
  ros::NodeHandle node;
  VoxelNode v(node);
  ros::spin();
  return 0;
}
