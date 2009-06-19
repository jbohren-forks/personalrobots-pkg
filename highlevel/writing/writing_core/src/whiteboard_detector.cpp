/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
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
 *
 */

// ROS core
#include <ros/ros.h>
// ROS messages
#include <robot_msgs/PointCloud.h>


// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/geometry/projections.h>

#include <angles/angles.h>

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace robot_msgs;




class WhiteboardDetector
{
	ros::NodeHandle nh_;

	PointCloudConstPtr cloud_;

	ros::Subscriber cloud_sub_;
	ros::Publisher cloud_pub_;

	tf::TransformListener tf_;
	tf::TransformBroadcaster broadcaster_;

	double min_height_;

public:
	WhiteboardDetector()
	{
		string topic_name;
		nh_.param<string>("~full_cloud_topic", topic_name, "full_cloud");
		nh_.param("~min_height", min_height_, 0.9);
		ROS_INFO("Topic name: %s", topic_name.c_str());
		cloud_sub_ = nh_.subscribe(topic_name, 1, &WhiteboardDetector::cloudCallback, this);

		cloud_pub_ = nh_.advertise<PointCloud>("whiteboard_cloud",1);
	}

private:

	void cloudCallback(const PointCloudConstPtr& the_cloud)
	{
		ROS_INFO("Received point cloud, starting detection");
		cloud_ = the_cloud;
		detectWhiteboard(*cloud_);
	}


	void filterPointCloud(const PointCloud& in, PointCloud& out)
	{
		out.header.stamp = in.header.stamp;
		out.header.frame_id = in.header.frame_id;

		out.chan.resize(in.chan.size());
		for (size_t c=0;c<in.chan.size();++c) {
			out.chan[c].name = in.chan[c].name;
		}

		for (size_t i=0;i<in.get_pts_size();++i) {
			if (in.pts[i].z>min_height_) {
				out.pts.push_back(in.pts[i]);
				for (size_t c=0;c<in.chan.size();++c) {
					out.chan[c].vals.push_back(in.chan[c].vals[i]);
				}
			}
		}
	}

	void detectWhiteboard(const PointCloud& point_cloud)
	{
		PointCloud odom_cloud;
		tf_.transformPointCloud("odom_combined",point_cloud,odom_cloud);

		PointCloud cloud;
		filterPointCloud(odom_cloud,cloud);

		cloud_pub_.publish(cloud);

		vector<int> indices(cloud.get_pts_size());
		for (size_t i=0;i<cloud.get_pts_size();++i) {
			indices[i] = i;
		}
		Point32 viewpoint;
		viewpoint.x = viewpoint.y = viewpoint.z = 0;

		// Use the entire data to estimate the plane equation.
		vector<int> inliers;
		inliers.clear(); //Points that are in plane

		vector<double> model;
		model.clear();  //Plane equation

		fitSACPlane(cloud, indices, viewpoint, inliers, model, 0.005, 1000);


		PointStamped laser_origin;
		laser_origin.header.frame_id = point_cloud.header.frame_id;
		laser_origin.header.stamp = point_cloud.header.stamp;
		laser_origin.point.x = 0;
		laser_origin.point.y = 0;
		laser_origin.point.z = 0;


		PointStamped odom_laser_origin;
		tf_.transformPoint("odom_combined", laser_origin, odom_laser_origin);

		Point32 before_projection;
		before_projection.x = odom_laser_origin.point.x;
		before_projection.y = odom_laser_origin.point.y;
		before_projection.z = odom_laser_origin.point.z;
		Point32 origin_projection;

		cloud_geometry::projections::pointToPlane(before_projection, origin_projection, model);

		PointStamped origin;
		origin.header.stamp = odom_laser_origin.header.stamp;
		origin.header.frame_id = odom_laser_origin.header.frame_id;
		origin.point.x = origin_projection.x;
		origin.point.y = origin_projection.y;
		origin.point.z = origin_projection.z;

		addWhiteboardFrame(origin, model);

	}


	void addWhiteboardFrame(PointStamped origin, const vector<double>& plane)
	{

		btVector3 position(origin.point.x,origin.point.y,origin.point.z);

		btQuaternion orientation;
		btMatrix3x3 rotation;
		btVector3 z(plane[0],plane[1],plane[2]);
		btVector3 y(0,0,1);
		btVector3 x = y.cross(z).normalized();
		rotation[0] = x; 	// x
		rotation[1] = y; 	// y
		rotation[2] = z; 	// z
		rotation = rotation.transpose();
		rotation.getRotation(orientation);

		tf::Transform tf_pose(orientation, position);

		// add wall_frame to tf
		tf::Stamped<tf::Pose> table_pose_frame(tf_pose, origin.header.stamp, "whiteboard_frame", origin.header.frame_id);

		tf_.setTransform(table_pose_frame);

		broadcaster_.sendTransform(table_pose_frame);
	}


	bool fitSACPlane (const PointCloud& points, const vector<int> &indices, Point32 viewpoint, // input
			vector<int> &inliers, vector<double> &coeff,  // output
			double dist_thresh, int min_points_per_model)
	{
		// Create and initialize the SAC model
		sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
		sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
		sac->setMaxIterations (100);
		model->setDataSet ((PointCloud*)&points, indices);

		// Search for the best plane
		if (sac->computeModel ()) {
			sac->computeCoefficients (coeff);                              // Compute the model coefficients
			sac->refineCoefficients (coeff);                             // Refine them using least-squares

			// Get the list of inliers
			model->selectWithinDistance (coeff, dist_thresh, inliers);

			if ((int)inliers.size()<min_points_per_model) {
				return false;
			}

			// Flip the plane normal towards the viewpoint
			cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.pts.at(inliers[0]), viewpoint);

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


};




/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "whiteboard_detector");
  WhiteboardDetector d;
  ros::spin();

  return (0);
}
/* ]--- */

