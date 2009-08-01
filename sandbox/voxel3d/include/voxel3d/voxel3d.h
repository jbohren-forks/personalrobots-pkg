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

/** \author Stuart Glaser */

#include <vector>
#include "ros/node_handle.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/PointCloud.h"
#include "mapping_msgs/CollisionMap.h"
#include "voxel3d/DistanceField.h"
#include <limits>

class Voxel3d
{
public:
  const static unsigned char CLEAR;

  Voxel3d(int size1, int size2, int size3, double resolution, const tf::Vector3 &origin,
          bool visualize = false);

  ~Voxel3d();

  /** \brief add the points in the point cloud to the voxel */
  void updateWorld(const sensor_msgs::PointCloud &cloud);

  /** \brief adds the points in a collision map to the voxel - only works on 1cm cells! (temporarily) */
  void updateWorld(const mapping_msgs::CollisionMap &collision_map);

  unsigned char &operator()(int i, int j, int k) {
    return data_[ref(i,j,k)];
  }
  const unsigned char &operator()(int i, int j, int k) const {
    return data_[ref(i,j,k)];
  }

  /** \brief clear the voxel (set all cells to 0)*/
  void reset();

  /** \brief publish the visualization markers for a specified period of time (hardcoded right now) */
  void updateVisualizations();

  /** \brief retrieve the dimensions of the voxel and the resolution (meters) */
  void getDimensions(int *size1, int *size2, int *size3, double *resolution)  {
    *size1 = size1_;
    *size2 = size2_;
    *size3 = size3_;
    *resolution = resolution_;
  }

  /** \brief place an obstacle in the voxel defined in voxel coordinates */
  void putObstacle(int i, int j, int k);

  /** \brief add an obstacle to the voxel defined in world coordinates */
  void putWorldObstacle(double i, double j, double k);

  /** \brief Converts this voxel3d to a DistanceField message */
  void toDistanceFieldMsg(voxel3d::DistanceField& msg);

  /**
   * \brief Gets the distance field value at a point (in m) and its gradient (unit-less)
   * \return the value of the distance field
   */
  double getDistanceGradient(double x, double y, double z,
      double& gradient_x, double& gradient_y, double& gradient_z) const;

private:
  std::vector<unsigned char> data_;
  int size1_, size2_, size3_;
  int stride1_, stride2_;
  double resolution_;  // meters/cell
  tf::Vector3 origin_;

  bool visualize_;
  ros::Publisher pub_viz_;
  ros::Time last_visualized_;

  double inv_resolution_; /**< 1/resolution_ pre-calculated */
  double max_distance_; /** max achievable distance in the distance field*/

  /** \brief convert coordinates from world to grid */
  void worldToGrid(double wx, double wy, double wz, int &gx, int &gy, int &gz) const {
    gx = (int)((wx - origin_.x()) * inv_resolution_);
    gy = (int)((wy - origin_.y()) * inv_resolution_);
    gz = (int)((wz - origin_.z()) * inv_resolution_);
  }

  /** \brief convert coordinates from grid to world */
  void gridToWorld(int gx, int gy, int gz, double &wx, double &wy, double &wz) const {
    wx = gx * resolution_ + origin_.x();
    wy = gy * resolution_ + origin_.y();
    wz = gz * resolution_ + origin_.z();
  }
  inline int ref(int i, int j, int k) const {
    return k * stride2_ + j * stride1_ + i;
  }

  /** \brief place an obstacle in the voxel defined in voxel coordinates */
//   void putObstacle(int i, int j, int k);

  //std::vector<unsigned char> kernel_;
  unsigned char *kernel_;
};

//////////////////////////////////// inline functions follow ///////////////////////////////

inline double Voxel3d::getDistanceGradient(double x, double y, double z,
    double& gradient_x, double& gradient_y, double& gradient_z) const
{
  int gx, gy, gz;
  int int_grad[3];

  worldToGrid(x, y, z, gx, gy, gz);

  // if out of bounds, return max distance, and 0 gradient
  // we need extra padding of 1 to get gradients
  if (gx<1 || gy<1 || gz<1 || gx>=size1_-1 || gy>=size2_-1 || gz>=size3_-1)
  {
    gradient_x = 0.0;
    gradient_y = 0.0;
    gradient_z = 0.0;
    return max_distance_;
  }

  int_grad[0] = ((int)((*this)(gx+1,gy,gz)) - (int)((*this)(gx-1,gy,gz)));
  int_grad[1] = ((int)((*this)(gx,gy+1,gz)) - (int)((*this)(gx,gy-1,gz)));
  int_grad[2] = ((int)((*this)(gx,gy,gz+1)) - (int)((*this)(gx,gy,gz-1)));

  // clamp the gradients:
  for (int i=0; i<3; i++)
  {
    if (int_grad[i]<-2)
      int_grad[i] = -2;
    else if (int_grad[i]>2)
      int_grad[i] = 2;
  }

  gradient_x = 0.5 * int_grad[0];
  gradient_y = 0.5 * int_grad[1];
  gradient_z = 0.5 * int_grad[2];

  unsigned char dist = (*this)(gx,gy,gz);
  return (dist > 8) ? max_distance_ : dist*resolution_;
}
