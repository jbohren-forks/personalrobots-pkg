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

#include <vector>
#include <map>
#include <list>
#include <fstream>
#include <sstream>
#include <time.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <limits>


#include "outlet_detection/outlet_util.h"
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_oriented_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;

PointCloud pointCloudVicinity(PointCloud laser_cloud, PointStamped ps_cloud, double distance, Point32& closest)
{
	PointCloud result;
	double dist = 1e10;
	result.header.frame_id = laser_cloud.header.frame_id;
	result.header.stamp = laser_cloud.header.stamp;

	double d = distance*distance;
	for (size_t i=0; i<laser_cloud.get_pts_size(); ++i) {
		double crt_dist =squaredPointDistance(laser_cloud.pts[i],ps_cloud.point);
		if (crt_dist<dist) {
			closest = laser_cloud.pts[i];
			dist = crt_dist;
		}
		if (crt_dist<d) {
			result.pts.push_back(laser_cloud.pts[i]);
		}
	}

	return result;
}



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
bool fitSACLine(PointCloud& points, vector<int> indices, vector<double> &coeff, double dist_thresh, int min_pts, vector<Point32> &line_segment)
{
	ROS_INFO("OutletSpotting: Trying to fit a line in %d points", indices.size());
	Point32 minP, maxP;

	if ((int)indices.size()<min_pts) {
		coeff.resize (0);
		return false;
	}

	// Create and initialize the SAC model
	sample_consensus::SACModelLine *model = new sample_consensus::SACModelLine ();
	sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
	sac->setMaxIterations (100);
	model->setDataSet (&points, indices);

	if(sac->computeModel())
	{
		if((int) sac->getInliers().size() < min_pts) {
			coeff.resize(0);
			return false;
		}
		sac->computeCoefficients (coeff);

		Point32 minP, maxP;
		cloud_geometry::statistics::getLargestDiagonalPoints(points, sac->getInliers(), minP, maxP);
		line_segment.push_back(minP);
		line_segment.push_back(maxP);

		ROS_INFO("OutletSpotting: Found a model supported by %d inliers: [%g, %g, %g, %g, %g, %g]", (int)sac->getInliers ().size (), coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5] );
	}

	delete sac;
	delete model;
	return true;
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
bool getWallPoseFromBaseLaser(const PointCloud& pc, const PointStamped& near_point, double distance, PoseStamped& wall_pose)
{
	assert(pc.header.frame_id==near_point.header.frame_id);

	Point32 closest_base_laser; // the closest point in
	PointCloud filtered_pc = pointCloudVicinity(pc, near_point, distance, closest_base_laser);
	// if no points from base laser scan are found in the vicinity of the outlet center
	// then ignore the current blob
	if (filtered_pc.get_pts_size()==0) {
		return false;
	}
	// fit a line in the outlet cloud
	vector<int> indices(filtered_pc.get_pts_size());
	for (size_t j=0;j<filtered_pc.get_pts_size();++j) {
		indices[j] = j;
	}
	vector<double> coeff(6);	// line coefficients
	double dist_thresh = 0.02;
	int min_pts = 10;
	vector<Point32> line_segment;
	if ( !fitSACLine(filtered_pc, indices, coeff, dist_thresh, min_pts, line_segment) ) {
		ROS_ERROR ("Cannot find line in laser scan, aborting...");
		return false;
	}

	// fill the wall pose
	wall_pose.header.frame_id = pc.header.frame_id;
	wall_pose.header.stamp = pc.header.stamp;

	btVector3 position(closest_base_laser.x,closest_base_laser.y,closest_base_laser.z);
	btVector3 up(0,0,1);
	btVector3 left(line_segment[1].x-line_segment[0].x,line_segment[1].y-line_segment[0].y,line_segment[1].z-line_segment[0].z);
	left = left.normalized();
	btVector3 normal = left.cross(up).normalized();

	btMatrix3x3 rotation;
	rotation[0] = normal; // x
	rotation[1] = left; // y
	rotation[2] = up;     // z
	rotation = rotation.transpose();
	btQuaternion orientation;
	rotation.getRotation(orientation);
	tf::Transform tf_pose(orientation, position);
	tf::poseTFToMsg(tf_pose, wall_pose.pose);

	return true;
}



/** \brief Find a plane model in a point cloud with SAmple Consensus method
* \param points the point cloud message
* \param indices a subset of point indices to use
* \param inliers the resultant planar model inliers
* \param coeff the resultant plane model coefficients
* \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
* \param dist_thresh the maximum allowed distance threshold of an inlier to the model
* \param min_points_per_model the minimum number of points allowed for a planar model (default: 100)
*/
bool fitSACOrientedPlane (const PointCloud& points, const vector<int> &indices,  // input
		vector<int> &inliers, vector<double> &coeff,  // output
		/*const geometry_msgs::Point32 &viewpoint_cloud,*/ const Point32& orientation, double dist_thresh, double eps_angle, int min_points_per_model // constraints
		)
{
	// Create and initialize the SAC model
	sample_consensus::SACModelOrientedPlane *model = new sample_consensus::SACModelOrientedPlane ();
	sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
	sac->setMaxIterations (100);
	model->setDataSet ((PointCloud*)&points, indices);
	model->setAxis(orientation);
	model->setEpsAngle(eps_angle);

	// Search for the best plane
	if (sac->computeModel ()) {
		sac->computeCoefficients (coeff);                              // Compute the model coefficients
		sac->refineCoefficients (coeff);                             // Refine them using least-squares

		// Get the list of inliers
		model->selectWithinDistance (coeff, dist_thresh, inliers);

		if ((int)inliers.size()<min_points_per_model) {
			return false;
		}

//    		// Flip the plane normal towards the viewpoint
//    		cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.pts.at(inliers[0]), viewpoint_cloud);
//
		ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (), coeff[0], coeff[1], coeff[2], coeff[3]);
	}
	else {
		ROS_ERROR ("Could not compute a planar model for %d points.", indices.size());
		return false;
	}

	delete sac;
	delete model;
	return true;
}
