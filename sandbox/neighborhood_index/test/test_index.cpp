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

#include "neighborhood_index/linear_index.h"
#include "neighborhood_index/voxel_grid_index.h"

#include <vector>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>

using namespace neighborhood_index;
using namespace std;

class NeighborhoodIndexTester
{
public:
	ros::Subscriber cloud_sub_;
	ros::NodeHandle nh_;

	NeighborhoodIndexTester()
	{
		cloud_sub_ = nh_.subscribe("~cloud", 1, &NeighborhoodIndexTester::cloudCallback, this);
	}



	void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& point_cloud)
	{
		ROS_INFO("I got a point cloud with: %d points\n", point_cloud->get_points_size());

		Index* index;
		vector<vector<int> > indices;
		vector<vector<float> >distances;

		ROS_INFO("Linear index\n");
		index = new LinearIndex();
		index->buildIndex(*point_cloud);

		clock_t start_time = clock();
		index->knnSearch(*point_cloud, 10, indices, distances);
		ROS_INFO("Searching took: %g seconds", double(clock()-start_time)/CLOCKS_PER_SEC);
		delete index;

		ROS_INFO("Voxel gridindex\n");
		index = new VoxelGridIndex(0.1,0.1,0.1); // 10 cm cells
		index->buildIndex(*point_cloud);

		start_time = clock();
		index->knnSearch(*point_cloud, 10, indices, distances);
		ROS_INFO("Searching took: %g seconds", double(clock()-start_time)/CLOCKS_PER_SEC);
		delete index;
	}


};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "neighborhood_index_tester");

	NeighborhoodIndexTester tester;

	ros::spin();
}
