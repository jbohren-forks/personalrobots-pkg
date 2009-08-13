/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <distance_field/distance_field.h>
#include <visualization_msgs/Marker.h>

namespace distance_field
{

DistanceField::~DistanceField()
{
}

DistanceField::DistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      VoxelGrid<DistanceFieldVoxel>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, DistanceFieldVoxel(max_distance))
{
  max_distance_ = max_distance;
  int max_dist_int = int(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  //reset();
  ros::NodeHandle node;
  pub_viz_ = node.advertise<visualization_msgs::Marker>("visualization_marker", 3);
}

int DistanceField::eucDistSq(int* point1, int* point2)
{
  int dx = point1[DIM_X] - point2[DIM_X];
  int dy = point1[DIM_Y] - point2[DIM_Y];
  int dz = point1[DIM_Z] - point2[DIM_Z];
  return dx*dx + dy*dy + dz*dz;
}

void DistanceField::addPointsToField(std::vector<btVector3> points)
{
  // initialize the bucket queue
  bucket_queue_.resize(max_distance_sq_);

  bucket_queue_[0].reserve(points.size());
  // first mark all the points as distance=0, and add them to the queue
  int x, y, z, nx, ny, nz;
  int loc[3];
  for (unsigned int i=0; i<points.size(); ++i)
  {
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
    if (!valid)
      continue;
    DistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.distance_square_ = 0.0;
    voxel.closest_point_[DIM_X] = x;
    voxel.closest_point_[DIM_Y] = y;
    voxel.closest_point_[DIM_Z] = z;
    voxel.location_[DIM_X] = x;
    voxel.location_[DIM_Y] = y;
    voxel.location_[DIM_Z] = z;
    bucket_queue_[0].push_back(&voxel);
  }

  // now process the queue:
  for (unsigned int i=0; i<bucket_queue_.size(); ++i)
  {
    std::vector<DistanceFieldVoxel*>::iterator list_it = bucket_queue_[i].begin();
    while(list_it!=bucket_queue_[i].end())
    {
      DistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_[DIM_X];
      y = vptr->location_[DIM_Y];
      z = vptr->location_[DIM_Z];
      // generate all its neighbors
      for (int dx=-1; dx<=1; ++dx)
      {
        nx = x + dx;
        if (!isCellValid(DIM_X, nx))
          continue;
        for (int dy=-1; dy<=1; ++dy)
        {
          ny = y + dy;
          if (!isCellValid(DIM_Y, ny))
            continue;
          for (int dz=-1; dz<=1; ++dz)
          {
            nz = z + dz;
            if (!isCellValid(DIM_Z, nz))
              continue;

            if (dx==0 && dy==0 && dz==0)
              continue;

            // the real update code:
            // calculate the neighbor's new distance based on my closest filled voxel:
            DistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
            loc[DIM_X] = nx;
            loc[DIM_Y] = ny;
            loc[DIM_Z] = nz;
            int new_distance_sq = eucDistSq(vptr->closest_point_, loc);
            if (new_distance_sq > max_distance_sq_)
              continue;
            if (new_distance_sq < neighbor->distance_square_)
            {
              // update the neighboring voxel
              neighbor->distance_square_ = new_distance_sq;
              neighbor->closest_point_[DIM_X] = vptr->closest_point_[DIM_X];
              neighbor->closest_point_[DIM_Y] = vptr->closest_point_[DIM_Y];
              neighbor->closest_point_[DIM_Z] = vptr->closest_point_[DIM_Z];
              neighbor->location_[DIM_X] = loc[DIM_X];
              neighbor->location_[DIM_Y] = loc[DIM_Y];
              neighbor->location_[DIM_Z] = loc[DIM_Z];

              // and put it in the queue:
              bucket_queue_[new_distance_sq].push_back(neighbor);
            }

          }
        }
      }
      ++list_it;
    }
    bucket_queue_[i].clear();
  }

}

void DistanceField::visualize(double radius, std::string frame_id, ros::Time stamp)
{
  visualization_msgs::Marker inf_marker; // Marker for the inflation
  inf_marker.header.frame_id = frame_id;
  inf_marker.header.stamp = stamp;
  inf_marker.ns = "distance_field";
  inf_marker.id = 1;
  inf_marker.type = visualization_msgs::Marker::CUBE_LIST;
  inf_marker.action = 0;
  inf_marker.scale.x = resolution_[DIM_X];
  inf_marker.scale.y = resolution_[DIM_Y];
  inf_marker.scale.z = resolution_[DIM_Z];
  inf_marker.color.r = 1.0;
  inf_marker.color.g = 0.0;
  inf_marker.color.b = 0.0;
  inf_marker.color.a = 0.1;
  inf_marker.lifetime = ros::Duration(30.0);

  int rad_int_low = int(radius/resolution_[DIM_X]);
  int rad_int_high = int(ceil(radius/resolution_[DIM_X]));
  int dist_sq_required_low = int(rad_int_low*rad_int_low);
  int dist_sq_required_high = int(rad_int_high*rad_int_high);
  if (dist_sq_required_high == max_distance_sq_)
    dist_sq_required_high--;

  inf_marker.points.reserve(100000);
  for (int x = 0; x < num_cells_[DIM_X]; ++x)
  {
    for (int y = 0; y < num_cells_[DIM_Y]; ++y)
    {
      for (int z = 0; z < num_cells_[DIM_Z]; ++z)
      {
        int dist_sq = getCell(x,y,z).distance_square_;
        if (dist_sq >= dist_sq_required_low && dist_sq <=dist_sq_required_high)
        {
          int last = inf_marker.points.size();
          inf_marker.points.resize(last + 1);
          gridToWorld(x,y,z,
                      inf_marker.points[last].x,
                      inf_marker.points[last].y,
                      inf_marker.points[last].z);
        }
      }
    }
  }
  ROS_DEBUG("Publishing markers: %d inflated", inf_marker.points.size());
  pub_viz_.publish(inf_marker);
}

void DistanceField::reset()
{
  VoxelGrid<DistanceFieldVoxel>::reset(DistanceFieldVoxel(max_distance_sq_));
}

}
