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

#ifndef VOXEL_GRID_H_
#define VOXEL_GRID_H_

namespace voxel_grid
{

/**
 * \brief Container for a discretized 3D voxel grid
 */
template <typename T>
class VoxelGrid
{
  VoxelGrid(double size_x, double size_y, double size_z, double resolution,
      double origin_x, double origin_y, double origin_z, T& default_object);
  virtual ~VoxelGrid();

  const typename T& operator()(double x, double y, double z) const;

  enum Dimension
  {
    DIM_X = 0,
    DIM_Y = 1,
    DIM_Z = 2
  };

  double getSize(Dimension dim) const;
  double getResolution(Dimension dim) const;
  double getOrigin(Dimension dim) const;

private:
  typename T* data_;
  typename T& default_object_;
  double size_[3];
  double resolution_[3];
  double origin_[3];
  int num_cells_[3];
  int num_cells_total_;
};

//////////////////////////// template function definitions follow //////////////////

template<typename T>
VoxelGrid::VoxelGrid(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, T& default_object)
{
  size_[DIM_X] = size_x;
  size_[DIM_Y] = size_y;
  size_[DIM_Z] = size_z;
  origin_[DIM_X] = origin_x;
  origin_[DIM_Y] = origin_y;
  origin_[DIM_Z] = origin_z;
  num_cells_total_ = 1;
  for (int i=DIM_X; i<=DIM_Z; ++i)
  {
    resolution_[i] = resolution;
    num_cells_[i] = size_[i] / resolution_[i];
    num_cells_total_ *= num_cells_[i];
  }
  default_object_ = default_object;
}

template<typename T>
VoxelGrid::~VoxelGrid()
{
}

template<typename T>
double VoxelGrid::getSize(Dimension dim) const
{
  return size_[dim];
}

template<typename T>
double VoxelGrid::getResolution(Dimension dim) const
{
  return resolution_[dim];
}

template<typename T>
double VoxelGrid::getOrigin(Dimension dim) const
{
  return origin_[dim];
}

} // namespace voxel_grid
#endif /* VOXEL_GRID_H_ */
