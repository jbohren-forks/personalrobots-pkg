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

#include <vector>
#include "ros/node_handle.h"
#include "tf/transform_datatypes.h"
#include "robot_msgs/PointCloud.h"

class Voxel3d
{
public:
  const static unsigned char CLEAR;

  Voxel3d(int size1, int size2, int size3, double resolution, const tf::Vector3 &origin,
          bool visualize = false);
  ~Voxel3d();

  void updateWorld(const robot_msgs::PointCloud &cloud);

  unsigned char &operator()(int i, int j, int k) {
    return data_[ref(i,j,k)];
  }
  const unsigned char &operator()(int i, int j, int k) const {
    return data_[ref(i,j,k)];
  }

  void reset();

  void putObstacle(int i, int j, int k);

  void putWorldObstacle(double i, double j, double k);

private:
  std::vector<unsigned char> data_;
  int size1_, size2_, size3_;
  int stride1_, stride2_;
  double resolution_;  // meters/cell
  tf::Vector3 origin_;

  bool visualize_;
  ros::Publisher pub_viz_;
  ros::Time last_visualized_;

  void worldToGrid(double wx, double wy, double wz, int &gx, int &gy, int &gz) const {
    gx = (int)((wx - origin_.x()) / resolution_);
    gy = (int)((wy - origin_.y()) / resolution_);
    gz = (int)((wz - origin_.z()) / resolution_);
  }
  void gridToWorld(int gx, int gy, int gz, double &wx, double &wy, double &wz) const {
    wx = gx * resolution_ + origin_.x();
    wy = gy * resolution_ + origin_.y();
    wz = gz * resolution_ + origin_.z();
  }
  inline int ref(int i, int j, int k) const {
    return k * stride2_ + j * stride1_ + i;
  }

  //std::vector<unsigned char> kernel_;
  unsigned char *kernel_;
};
