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

#include "voxel3d/voxel3d.h"
#define USE_SSE 1

//#if USE_SSE
#include <emmintrin.h>
#include <xmmintrin.h>
//#endif

#include <cmath>
#include <malloc.h>
#include <string.h> // for memset(3)

#include "visualization_msgs/MarkerArray.h"


const unsigned char Voxel3d::CLEAR = 0xff;

Voxel3d::Voxel3d(int size1, int size2, int size3, double resolution, const tf::Vector3 &origin,
                 bool visualize)
  : size1_(size1), size2_(size2), size3_(size3),
    stride1_(size1_), stride2_(size1_*size2_),
    resolution_(resolution), origin_(origin),
    visualize_(visualize)
{
  inv_resolution_ = 1.0 / resolution_;
  max_distance_ = 8.0 * resolution_;

  data_.resize(size1_*size2_*size3_);
  reset();

  // Constructs the kernel
  //kernel_.resize(16*17*17);
  kernel_ = (unsigned char*)memalign(16, 16*17*17);
  for (int k = 0; k < 17; ++k) {
    for (int j = 0; j < 17; ++j) {
      for (int i = 0; i < 16; ++i) {
        kernel_[i + 16*(j + 17*k)] =
          (char)round(sqrt(pow(i-8,2) + pow(j-8,2) + pow(k-8,2)));
      }
    }
  }

  if (visualize_)
  {
    ros::NodeHandle node;
    //pub_viz_ = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 3);
    pub_viz_ = node.advertise<visualization_msgs::Marker>("visualization_marker", 3);
  }
}

Voxel3d::~Voxel3d()
{
  //delete [] kernel_;
  free(kernel_);
  if (visualize_)
    pub_viz_.shutdown();
}

void Voxel3d::reset()
{
  memset(&data_[0], CLEAR, data_.size());
}

void Voxel3d::updateWorld(const robot_msgs::PointCloud &cloud)
{
  int x, y, z;
  for (size_t i = 0; i < cloud.pts.size(); ++i)
  {
    worldToGrid(cloud.pts[i].x, cloud.pts[i].y, cloud.pts[i].z, x, y, z);
    putObstacle(x, y, z);
  }

  if (visualize_ && cloud.header.stamp - last_visualized_ > ros::Duration(3.0))
  {
    last_visualized_ = cloud.header.stamp;
#if 1
    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = cloud.header.frame_id;
    obs_marker.header.stamp = cloud.header.stamp;
    obs_marker.ns = "voxel3d";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obs_marker.action = 0;
    obs_marker.scale.x = resolution_;
    obs_marker.scale.y = resolution_;
    obs_marker.scale.z = resolution_;
    obs_marker.color.r = 1.0;
    obs_marker.color.g = 0.0;
    obs_marker.color.b = 0.5;
    obs_marker.color.a = 0.5;
    obs_marker.lifetime = ros::Duration(30.0);

    visualization_msgs::Marker inf_marker; // Marker for the inflation
    inf_marker.header.frame_id = cloud.header.frame_id;
    inf_marker.header.stamp = cloud.header.stamp;
    inf_marker.ns = "voxel3d";
    inf_marker.id = 1;
    inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
    inf_marker.action = 0;
    inf_marker.scale.x = resolution_;
    inf_marker.scale.y = resolution_;
    inf_marker.scale.z = resolution_;
    inf_marker.color.r = 1.0;
    inf_marker.color.g = 0.0;
    inf_marker.color.b = 0.0;
    inf_marker.color.a = 0.1;
    inf_marker.lifetime = ros::Duration(30.0);

    obs_marker.points.reserve(50000);
    inf_marker.points.reserve(100000);
    for (int k = 0; k < size3_; ++k) {
      for (int j = 0; j < size2_; ++j) {
        for (int i = 0; i < size1_; ++i) {
          unsigned char dist = data_[ref(i, j, k)];
          if (dist == 0)
          {
            int last = obs_marker.points.size();
            obs_marker.points.resize(last + 1);
            gridToWorld(i, j, k,
                        obs_marker.points[last].x,
                        obs_marker.points[last].y,
                        obs_marker.points[last].z);

          }
          if (dist == 8)
          {
            int last = inf_marker.points.size();
            inf_marker.points.resize(last + 1);
            gridToWorld(i, j, k,
                        inf_marker.points[last].x,
                        inf_marker.points[last].y,
                        inf_marker.points[last].z);

          }
        }
      }
    }
    ROS_INFO("Publishing a markers: %d obstacles, %d inflated",
              obs_marker.points.size(), inf_marker.points.size());
    pub_viz_.publish(obs_marker);
    pub_viz_.publish(inf_marker);
#else
    visualization_msgs::MarkerArray msg;
    msg.markers.reserve(300000);
    for (int k = 0; k < size3_; ++k) {
      for (int j = 0; j < size2_; ++j) {
        for (int i = 0; i < size1_; ++i) {
          unsigned char dist = data_[ref(i, j, k)];
          if (dist == 8)
          {
            int last = msg.markers.size();
            msg.markers.resize(last + 1);
            msg.markers[last].header.frame_id = cloud.header.frame_id;
            msg.markers[last].header.stamp = cloud.header.stamp;
            msg.markers[last].ns = "voxel3d";
            msg.markers[last].id = last;
            msg.markers[last].type = visualization_msgs::Marker::CUBE;
            msg.markers[last].action = 0;
            gridToWorld(i, j, k,
                        msg.markers[last].pose.position.x,
                        msg.markers[last].pose.position.y,
                        msg.markers[last].pose.position.z);
            msg.markers[last].scale.x = resolution_;
            msg.markers[last].scale.y = resolution_;
            msg.markers[last].scale.z = resolution_;
            msg.markers[last].color.r = 1.0;
            msg.markers[last].color.g = 0.0;
            msg.markers[last].color.b = 0.0;
            msg.markers[last].color.a = 0.5;
            msg.markers[last].lifetime = ros::Duration(30.0);
          }
        }
      }
    }
    ROS_INFO("Publishing a marker array with %d elements", msg.markers.size());
    pub_viz_.publish(msg);
#endif
  }
}

void Voxel3d::updateWorld(const mapping_msgs::CollisionMap &collision_map)
{
  int x, y, z;
  ROS_INFO("Warning: Updating the voxel3d from a collisionMap only supports 1cm cells for now");

  for (size_t i = 0; i < collision_map.boxes.size(); ++i)
  {
    worldToGrid(collision_map.boxes[i].center.x, collision_map.boxes[i].center.y, collision_map.boxes[i].center.z, x, y, z);
    putObstacle(x, y, z);
  }

  if (visualize_ && collision_map.header.stamp - last_visualized_ > ros::Duration(3.0))
  {
    last_visualized_ = collision_map.header.stamp;
#if 1
    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = collision_map.header.frame_id;
    obs_marker.header.stamp = collision_map.header.stamp;
    obs_marker.ns = "voxel3d";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obs_marker.action = 0;
    obs_marker.scale.x = resolution_;
    obs_marker.scale.y = resolution_;
    obs_marker.scale.z = resolution_;
    obs_marker.color.r = 1.0;
    obs_marker.color.g = 0.0;
    obs_marker.color.b = 0.5;
    obs_marker.color.a = 0.5;
    obs_marker.lifetime = ros::Duration(30.0);

    visualization_msgs::Marker inf_marker; // Marker for the inflation
    inf_marker.header.frame_id = collision_map.header.frame_id;
    inf_marker.header.stamp = collision_map.header.stamp;
    inf_marker.ns = "voxel3d";
    inf_marker.id = 1;
    inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
    inf_marker.action = 0;
    inf_marker.scale.x = resolution_;
    inf_marker.scale.y = resolution_;
    inf_marker.scale.z = resolution_;
    inf_marker.color.r = 1.0;
    inf_marker.color.g = 0.0;
    inf_marker.color.b = 0.0;
    inf_marker.color.a = 0.1;
    inf_marker.lifetime = ros::Duration(30.0);

    obs_marker.points.reserve(50000);
    inf_marker.points.reserve(100000);
    for (int k = 0; k < size3_; ++k) {
      for (int j = 0; j < size2_; ++j) {
	for (int i = 0; i < size1_; ++i) {
	  unsigned char dist = data_[ref(i, j, k)];
	  if (dist == 0)
	  {
	    int last = obs_marker.points.size();
	    obs_marker.points.resize(last + 1);
	    gridToWorld(i, j, k,
			obs_marker.points[last].x,
   obs_marker.points[last].y,
   obs_marker.points[last].z);

	  }
	  if (dist == 8)
	  {
	    int last = inf_marker.points.size();
	    inf_marker.points.resize(last + 1);
	    gridToWorld(i, j, k,
			inf_marker.points[last].x,
   inf_marker.points[last].y,
   inf_marker.points[last].z);

	  }
	}
      }
    }
    ROS_INFO("Publishing a markers: %d obstacles, %d inflated",
	     obs_marker.points.size(), inf_marker.points.size());
    pub_viz_.publish(obs_marker);
    pub_viz_.publish(inf_marker);
#else
    visualization_msgs::MarkerArray msg;
    msg.markers.reserve(300000);
    for (int k = 0; k < size3_; ++k) {
      for (int j = 0; j < size2_; ++j) {
	for (int i = 0; i < size1_; ++i) {
	  unsigned char dist = data_[ref(i, j, k)];
	  if (dist == 8)
	  {
	    int last = msg.markers.size();
	    msg.markers.resize(last + 1);
	    msg.markers[last].header.frame_id = collision_map.header.frame_id;
	    msg.markers[last].header.stamp = collision_map.header.stamp;
	    msg.markers[last].ns = "voxel3d";
	    msg.markers[last].id = last;
	    msg.markers[last].type = visualization_msgs::Marker::CUBE;
	    msg.markers[last].action = 0;
	    gridToWorld(i, j, k,
			msg.markers[last].pose.position.x,
   msg.markers[last].pose.position.y,
   msg.markers[last].pose.position.z);
	    msg.markers[last].scale.x = resolution_;
	    msg.markers[last].scale.y = resolution_;
	    msg.markers[last].scale.z = resolution_;
	    msg.markers[last].color.r = 1.0;
	    msg.markers[last].color.g = 0.0;
	    msg.markers[last].color.b = 0.0;
	    msg.markers[last].color.a = 0.5;
	    msg.markers[last].lifetime = ros::Duration(30.0);
	  }
	}
      }
    }
    ROS_INFO("Publishing a marker array with %d elements", msg.markers.size());
    pub_viz_.publish(msg);
#endif
  }
}

void Voxel3d::putObstacle(int i, int j, int k)
{
  // Doesn't do points near the edges
  if (i < 8 || i >= size1_ - 8 ||
      j < 8 || j >= size2_ - 8 ||
      k < 8 || k >= size3_ - 8)
    return;

  if ((*this)(i,j,k) == 0)
    return;

  // (i,j,k) corresponds to (8,8,8) in the kernel
  unsigned char *p = &data_[ref(i-8,j-8,k-8)];
  unsigned char *r = &kernel_[0];
  for (int kk = 0; kk < 17; ++kk)
  {
    unsigned char *jj_start = p;

#if USE_SSE
    for (int jj = 0; jj < 17; ++jj)
    {
      unsigned char *ii_start = p;
      __m128i vp, vr;
      vp = _mm_loadu_si128((__m128i*)p);
      vr = _mm_load_si128((__m128i*)r);
      _mm_storeu_si128((__m128i*)p, _mm_min_epu8(vp, vr));
      r += 16;
      p = ii_start + stride1_;
    }
//     for (int jj = 0; jj < 16; jj += 2)
//     {
//       __m128i vp1, vr1, vp2, vr2;
//       vp1 = _mm_loadu_si128((__m128i*)p);
//       vr1 = _mm_load_si128((__m128i*)r);
//       vp2 = _mm_loadu_si128((__m128i*)(p+stride1_));
//       vr2 = _mm_load_si128((__m128i*)(r+16));
//       vp1 = _mm_min_epu8(vp1, vr1);
//       vp2 = _mm_min_epu8(vp2, vr2);
//       _mm_storeu_si128((__m128i*)p, vp1);
//       _mm_storeu_si128((__m128i*)(p+stride1_), vp2);

//       r += 32;
//       p += 2*stride1_;// + stride1_;
//     }
//     INCOMPLETE!!!
#else
    for (int jj = 0; jj < 17; ++jj)
    {
      unsigned char *ii_start = p;
      for (int ii = 0; ii < 16; ++ii)
      {
        if (*p > *r)
          *p = *r;
        ++r;
        ++p;
      }
      p = ii_start + stride1_;
    }
#endif

    p = jj_start + stride2_;
  }
}

void Voxel3d::putWorldObstacle(double i, double j, double k)
{
  int x,y,z;
  worldToGrid(i, j, k, x, y, z);
  putObstacle(x, y, z);
}

void Voxel3d::updateVisualizations()
{
//   if (visualize_ && cloud.header.stamp - last_visualized_ > ros::Duration(3.0))
//   {
//     last_visualized_ = cloud.header.stamp;
// #if 1
    std::string frame_id = "torso_lift_link";
    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = frame_id;
    obs_marker.header.stamp = ros::Time::now();
    obs_marker.ns = "voxel3d";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::CUBE_LIST;
    obs_marker.action = 0;
    obs_marker.scale.x = resolution_;
    obs_marker.scale.y = resolution_;
    obs_marker.scale.z = resolution_;
    obs_marker.color.r = 1.0;
    obs_marker.color.g = 0.0;
    obs_marker.color.b = 0.5;
    obs_marker.color.a = 0.5;
    obs_marker.lifetime = ros::Duration(30.0);

    visualization_msgs::Marker inf_marker; // Marker for the inflation
    inf_marker.header.frame_id = frame_id;
    inf_marker.header.stamp = ros::Time::now();
    inf_marker.ns = "voxel3d";
    inf_marker.id = 1;
    inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
    inf_marker.action = 0;
    inf_marker.scale.x = resolution_;
    inf_marker.scale.y = resolution_;
    inf_marker.scale.z = resolution_;
    inf_marker.color.r = 1.0;
    inf_marker.color.g = 0.0;
    inf_marker.color.b = 0.0;
    inf_marker.color.a = 0.1;
    inf_marker.lifetime = ros::Duration(30.0);

    obs_marker.points.reserve(50000);
    inf_marker.points.reserve(100000);
    for (int k = 0; k < size3_; ++k) {
      for (int j = 0; j < size2_; ++j) {
	for (int i = 0; i < size1_; ++i) {
	  unsigned char dist = data_[ref(i, j, k)];
	  if (dist == 0)
	  {
	    int last = obs_marker.points.size();
	    obs_marker.points.resize(last + 1);
	    gridToWorld(i, j, k,
			obs_marker.points[last].x,
			obs_marker.points[last].y,
			obs_marker.points[last].z);

	  }
	  if (dist == 8)
	  {
	    int last = inf_marker.points.size();
	    inf_marker.points.resize(last + 1);
	    gridToWorld(i, j, k,
			inf_marker.points[last].x,
			inf_marker.points[last].y,
			inf_marker.points[last].z);

	  }
	}
      }
    }
    ROS_INFO("Publishing a markers: %d obstacles, %d inflated",
	     obs_marker.points.size(), inf_marker.points.size());
    pub_viz_.publish(obs_marker);
    pub_viz_.publish(inf_marker);
// #else
//     visualization_msgs::MarkerArray msg;
//     msg.markers.reserve(300000);
//     for (int k = 0; k < size3_; ++k) {
//       for (int j = 0; j < size2_; ++j) {
// 	for (int i = 0; i < size1_; ++i) {
// 	  unsigned char dist = data_[ref(i, j, k)];
// 	  if (dist == 8)
// 	  {
// 	    int last = msg.markers.size();
// 	    msg.markers.resize(last + 1);
// 	    msg.markers[last].header.frame_id = cloud.header.frame_id;
// 	    msg.markers[last].header.stamp = cloud.header.stamp;
// 	    msg.markers[last].ns = "voxel3d";
// 	    msg.markers[last].id = last;
// 	    msg.markers[last].type = visualization_msgs::Marker::CUBE;
// 	    msg.markers[last].action = 0;
// 	    gridToWorld(i, j, k,
// 			msg.markers[last].pose.position.x,
// 			msg.markers[last].pose.position.y,
// 			msg.markers[last].pose.position.z);
// 	    msg.markers[last].scale.x = resolution_;
// 	    msg.markers[last].scale.y = resolution_;
// 	    msg.markers[last].scale.z = resolution_;
// 	    msg.markers[last].color.r = 1.0;
// 	    msg.markers[last].color.g = 0.0;
// 	    msg.markers[last].color.b = 0.0;
// 	    msg.markers[last].color.a = 0.5;
// 	    msg.markers[last].lifetime = ros::Duration(30.0);
// 	  }
// 	}
//       }
//     }
//     ROS_INFO("Publishing a marker array with %d elements", msg.markers.size());
//     pub_viz_.publish(msg);
// #endif
//   }
}

void Voxel3d::toDistanceFieldMsg(voxel3d::DistanceField& msg)
{
  msg.max_expansion = 8*resolution_;
  msg.resolution = resolution_;
  msg.size.resize(3);
  msg.size[0] = size1_;
  msg.size[1] = size2_;
  msg.size[2] = size3_;
  msg.origin.x = origin_.x();
  msg.origin.y = origin_.y();
  msg.origin.z = origin_.z();

  // count the number of non-free voxels in a first pass, to avoid memory reallocations
  unsigned int num_voxels = 0;
  ROS_INFO("Number of total voxels is %d", data_.size());
  for (unsigned int i=0; i<data_.size(); ++i)
  {
    if (data_[i] < Voxel3d::CLEAR)
      ++num_voxels;
  }

  ROS_INFO("Number of non-free voxels is %d", num_voxels);

  // allocate the memory and assign the voxels:
  msg.voxels.clear();
  msg.voxels.resize(num_voxels);

  int size=0;
  for (int k = 0; k < size3_; ++k) {
    for (int j = 0; j < size2_; ++j) {
      for (int i = 0; i < size1_; ++i) {
        unsigned char dist = data_[ref(i, j, k)];
        if (dist == Voxel3d::CLEAR)
          continue;
        msg.voxels[size].x = i;
        msg.voxels[size].y = j;
        msg.voxels[size].z = k;
        msg.voxels[size].distance = resolution_*dist;
        ++size;
      }
    }
  }

}
