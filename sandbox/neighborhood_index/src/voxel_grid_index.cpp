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

#include <vector>
#include <point_cloud_mapping/geometry/statistics.h>

#include "neighborhood_index/voxel_grid_index.h"
#include "neighborhood_index/index_utils.h"





const size_t MAX_CELL_SIZE = 64;

namespace neighborhood_index {

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std;


Point32 computeCentroid(const sensor_msgs::PointCloud& cloud)
{
	Point32 centroid;

	for (size_t i=0;i<cloud.get_points_size();++i) {
		centroid.x += cloud.points[i].x;
		centroid.y += cloud.points[i].y;
		centroid.z += cloud.points[i].z;
	}

	centroid.x /= cloud.points.size();
	centroid.y /= cloud.points.size();
	centroid.z /= cloud.points.size();

	return centroid;
}


int logTwoFloor(int size)
{
	if (size==0) return 0;

	int val = 1;
	int k = 1;
	while (val<size) {
		val <<= 1;
		k ++;
	}

	return k;
}


struct Cell {
	Cell() : cnt(0), vec(NULL) {};
	~Cell()
	{
		if (vec!=NULL) {
			delete[] vec;
		}
	}

	bool add (int ind) {
		if (vec==NULL) {
			vec = new int[MAX_CELL_SIZE];
		}
		if (cnt==MAX_CELL_SIZE) {
			return false;
		}
		vec[cnt++] = ind;

		return true;
	}

	void search(ResultSet& resultSet, const PointCloud& pc)
	{
		for(size_t i=0; i<cnt;++i) {
			resultSet.add(pc.points[vec[i]],vec[i]);
		}
	}

	size_t cnt;
	int *vec;
};



void VoxelGridIndex::buildIndex(const sensor_msgs::PointCloud& cloud)
{
	cloud_ = cloud;

	center_ = computeCentroid(cloud_);

	cloud_geometry::statistics::getMinMax (cloud_, min_, max_);

	x_size_ = logTwoFloor(int((max_.x-min_.x)/x_res_+1));
	y_size_ = logTwoFloor(int((max_.y-min_.y)/y_res_+1));
	z_size_ = logTwoFloor(int((max_.z-min_.z)/z_res_+1));

	x_str_ = y_size_ + z_size_;
	y_str_ = z_size_;
	z_str_ = 0;

	size_t mem_size = 1<<(x_size_+y_size_+z_size_);
	// allocate memory
	try {
		grid_ = new Cell[mem_size];
	}
	catch (const std::bad_alloc& e) {
		// memory allocation failure
		return;
	}

	// fill with point cloud indices
	for (size_t i=0; i<cloud_.get_points_size(); ++i) {
		int xind = int((cloud_.points[i].x-min_.x)/x_res_);
		int yind = int((cloud_.points[i].y-min_.y)/y_res_);
		int zind = int((cloud_.points[i].z-min_.z)/z_res_);

		Cell& loc = grid_[(xind<<x_str_ +yind<<y_str_ + zind)];
		loc.add(i);
	}
}



void VoxelGridIndex::knnSearch(const geometry_msgs::Point32 &point, int k,
		std::vector<int> &indices, std::vector<float> &distances)
{
	indices.resize(k);
	distances.resize(k);

	int xind = int((point.x-min_.x)/x_res_);
	int yind = int((point.y-min_.y)/y_res_);
	int zind = int((point.z-min_.z)/z_res_);
	Cell& loc = grid_[(xind<<x_str_ +yind<<y_str_ + zind)];

	KNNResultSet resultSet(indices, distances);
	resultSet.init(point);

	loc.search(resultSet, cloud_);
}

bool VoxelGridIndex::radiusSearch (const geometry_msgs::Point32 &point, double radius,
		std::vector<int> &indices, std::vector<float> &distances, int max_nn)
{
	int xind = int((point.x-min_.x)/x_res_);
	int yind = int((point.y-min_.y)/y_res_);
	int zind = int((point.z-min_.z)/z_res_);
	Cell& loc = grid_[(xind<<x_str_ +yind<<y_str_ + zind)];

	RadiusResultSet resultSet(indices, distances, radius, max_nn);
	resultSet.init(point);

	loc.search(resultSet, cloud_);

	return indices.size()!=0;
}

}
