/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

// Author: mariusm

#ifndef VOXEL_GRID_INDEX_H_
#define VOXEL_GRID_INDEX_H_


#include "index.h"


namespace neighborhood_index
{
// forward declaration
struct Cell;

class VoxelGridIndex : public Index {
public:
	VoxelGridIndex(double x_res, double y_res, double z_res) :  x_res_(x_res), y_res_(y_res), z_res_(z_res) {}

	void buildIndex(const sensor_msgs::PointCloud& points);

	void knnSearch(const geometry_msgs::Point32 &point, int k,
			std::vector<int> &indices, std::vector<float> &distances);

	bool radiusSearch (const geometry_msgs::Point32 &point, double radius,
			std::vector<int> &indices, std::vector<float> &istances, int max_nn = INT_MAX);

private:

	double x_res_, y_res_, z_res_;
	int x_size_, y_size_, z_size_;
	// strides
	int x_str_, y_str_, z_str_;

	geometry_msgs::Point32 center_;
	geometry_msgs::Point32 min_, max_;
	sensor_msgs::PointCloud cloud_;

	Cell* grid_;
};

}

#endif /* VOXEL_GRID_INDEX_H_ */
