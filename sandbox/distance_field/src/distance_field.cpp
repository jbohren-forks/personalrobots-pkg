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
  max_distance_sq_ = int((max_distance_*max_distance_)+1);
  reset();
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
    std::list<DistanceFieldVoxel*>::iterator list_it = bucket_queue_[i].begin();
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

void DistanceField::reset()
{
  VoxelGrid<DistanceFieldVoxel>::reset(DistanceFieldVoxel(max_distance_sq_));
}

}
