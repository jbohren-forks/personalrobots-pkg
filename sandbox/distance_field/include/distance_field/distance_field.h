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

#ifndef DISTANCE_FIELD_H_
#define DISTANCE_FIELD_H_

#include <distance_field/voxel_grid.h>
#include <LinearMath/btVector3.h>
#include <vector>
#include <list>
#include <ros/ros.h>

namespace distance_field
{

struct DistanceFieldVoxel
{
  DistanceFieldVoxel();
  DistanceFieldVoxel(int distance_sq);

  int distance_square_;
  int location_[3];
  int closest_point_[3];

  static const int UNINITIALIZED=-1;
};

class DistanceField: public VoxelGrid<DistanceFieldVoxel>
{
public:
  //DistanceField();
  DistanceField(double size_x, double size_y, double size_z, double resolution,
      double origin_x, double origin_y, double origin_z, double max_distance);

  virtual ~DistanceField();

  void addPointsToField(std::vector<btVector3> points);
  void reset();

  void visualize(double radius, std::string frame_id, ros::Time stamp);

private:
  std::vector<std::vector<DistanceFieldVoxel*> > bucket_queue_;
  double max_distance_;
  int max_distance_sq_;
  ros::Publisher pub_viz_;

  int eucDistSq(int* point1, int* point2);
};

////////////////////////// inline functions follow ////////////////////////////////////////

inline DistanceFieldVoxel::DistanceFieldVoxel(int distance_sq):
  distance_square_(distance_sq)
{
    for (int i=0; i<3; i++)
      closest_point_[i] = DistanceFieldVoxel::UNINITIALIZED;
}

inline DistanceFieldVoxel::DistanceFieldVoxel()
{
}

}

#endif /* DISTANCE_FIELD_H_ */
