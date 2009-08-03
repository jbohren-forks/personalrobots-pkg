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

#include "sensor_msgs/PointCloud.h"

#include <voxel3d/GetDistanceField.h>

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

    node_.param("~size_x", size_[0], 2.0);
    node_.param("~size_y", size_[1], 2.0);
    node_.param("~size_z", size_[2], 2.0);
    node_.param("~resolution", resolution_, 0.01);

    for (int i=0; i<3; i++)
      num_cells_[i] = (int)(size_[i]/resolution_);

    double origin[3];
    node_.param("~origin_x", origin[0], -1.0);
    node_.param("~origin_y", origin[1], -1.0);
    node_.param("~origin_z", origin[2], -1.0);

    voxel_.reset(new Voxel3d(num_cells_[0], num_cells_[1], num_cells_[2], resolution_,
        tf::Vector3(origin[0], origin[1], origin[2]), visualize));

    sub_cloud_ = node_.subscribe("point_cloud", 1, &VoxelNode::cloudCB, this);

    // advertise the GetDistanceField service:
    get_distance_field_service_ = node_.advertiseService("get_distance_field", &VoxelNode::getDistanceField, this);

  }

  ~VoxelNode()
  {
    sub_cloud_.shutdown();
  }

  /**
   * \brief Callback for the GetDistanceField service
   */
  bool getDistanceField(voxel3d::GetDistanceField::Request &req, voxel3d::GetDistanceField::Response &res)
  {
    ros::WallTime start = ros::WallTime::now();

    voxel_->toDistanceFieldMsg(res.field);
    res.field.header.frame_id = frame_;

    ROS_INFO("Service call response took %lf seconds", (ros::WallTime::now() - start).toSec());
    return true;
  }

private:
  ros::NodeHandle node_;
  std::string frame_;
  tf::TransformListener TF;
  ros::Subscriber sub_cloud_;
  ros::ServiceServer get_distance_field_service_;      /**< The distance field service */

  int num_cells_[3];
  double size_[3];
  double resolution_;
  tf::Vector3 origin_;

  boost::scoped_ptr<Voxel3d> voxel_;

  void cloudCB(const sensor_msgs::PointCloudConstPtr &msg_orig)
  {
    try
    {
      ros::WallTime start = ros::WallTime::now();

      sensor_msgs::PointCloud msg;
      TF.transformPointCloud(frame_, *msg_orig, msg);
      ROS_INFO("TF Transform took %lf seconds", (ros::WallTime::now() - start).toSec());
      printf("%d points\n", msg.pts.size());
      start = ros::WallTime::now();

      voxel_->reset();
      voxel_->updateWorld(msg);

      ROS_INFO("Distance transform took %lf seconds", (ros::WallTime::now() - start).toSec());
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
