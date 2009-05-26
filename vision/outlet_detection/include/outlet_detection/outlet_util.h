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

// Author: Marius Muja

#ifndef OUTLET_UTIL_H_
#define OUTLET_UTIL_H_

#include <vector>

#include "robot_msgs/PointCloud.h"
#include "robot_msgs/PointStamped.h"
#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/Point32.h"


template <typename T, typename U>
double squaredPointDistance(T p1, U p2)
{
	return (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
}



/**
* Compute the wall pose using the base laser scan. The pose origin will be the point from the laser scan nearest to a given point.
*
* @param pc Base scan point cloud.
* @param near_point Only the laser scan point near this point are used
* @param distance The distance from the near_point  at which to consider the points from the base laser scan when doing a line fit
* @param wall_pose
* @return
*
* PRECONDITION: pc and near_point are in the same frame
*/
bool getWallPoseFromBaseLaser(const robot_msgs::PointCloud& pc, const robot_msgs::PointStamped& near_point, double distance, robot_msgs::PoseStamped& wall_pose);



/**
*
* Fits a line in a point cloud
*
* @param points The point cloud
* @param indices The indices of the points in the point cloud to use.
* @param coeff The coefficients of the line
* @param dist_thresh Distance threshold for a point to be considered inlier
* @param min_pts Minimum points that have to fit the model
* @param line_segment The extremities of the segment
* @return True if a line can be fitted in the point cloud
*/
bool fitSACLine(robot_msgs::PointCloud& points, std::vector<int> indices, std::vector<double> &coeff, double dist_thresh, int min_pts, std::vector<robot_msgs::Point32> &line_segment);


/** \brief Find a plane model in a point cloud with SAmple Consensus method
* \param points the point cloud message
* \param indices a subset of point indices to use
* \param inliers the resultant planar model inliers
* \param coeff the resultant plane model coefficients
* \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
* \param dist_thresh the maximum allowed distance threshold of an inlier to the model
* \param min_points_per_model the minimum number of points allowed for a planar model (default: 100)
*/
bool fitSACOrientedPlane (const robot_msgs::PointCloud& points, const std::vector<int> &indices,
		std::vector<int> &inliers, std::vector<double> &coeff,
		const robot_msgs::Point32& orientation, double dist_thresh, double eps_angle, int min_points_per_model);


#endif /* OUTLET_UTIL_H_ */
