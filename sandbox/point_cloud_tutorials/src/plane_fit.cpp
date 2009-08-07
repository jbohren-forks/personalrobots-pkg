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

#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>


// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

using namespace std;

class PlanarFit
{
	ros::NodeHandle nh_;

	ros::Subscriber cloud_sub_;
	ros::Publisher pm_pub_;
	ros::Publisher plane_pub_;

	// parameters
	string cloud_topic_name_;
	int sac_maximum_iterations_;
	double sac_distance_threshold_;

	sensor_msgs::PointCloud cloud_;

public:
	PlanarFit()
	{
		nh_.param<string>("~cloud_topic", cloud_topic_name_, "full_cloud_annotated");  // the cloud topic name
		nh_.param("~sac_distance_threshold", sac_distance_threshold_, 0.02);   // 2 cm threshold
		nh_.param("~sac_maximum_iterations", sac_maximum_iterations_, 500);    // maximum 500 SAC iterations

		cloud_sub_ = nh_.subscribe(cloud_topic_name_, 1, &PlanarFit::cloudCallback, this);
		pm_pub_ = nh_.advertise<mapping_msgs::PolygonalMap>("~convex_hull",1);
		plane_pub_ = nh_.advertise<sensor_msgs::PointCloud>("~plane",1);
	}


	/**
	 * Cloud topic callback
	 *
	 * @param cloud
	 */
	void cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud)
	{
		cloud_ = *cloud;
		ROS_INFO("Received new point cloud with %d points", cloud_.get_points_size());

		vector<double> coeff(4);
		fitSACPlane(cloud_, coeff);
	}


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
	* \param dist_thresh the maximum allowed distance threshold of an inlier to the model
	* \param min_pts the minimum number of points allowed as inliers for a plane model
	*/
	bool fitSACPlane (sensor_msgs::PointCloud &points, vector<double> &coeff)
	{
		vector<int> inliers;
		// Create and initialize the SAC model
		sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
		sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);

		sac->setMaxIterations (sac_maximum_iterations_);
		model->setDataSet (&points);

		// Search for the best plane
		if (sac->computeModel (0))
		{
			sac->computeCoefficients (coeff);     // Compute the model coefficients
			sac->refineCoefficients (coeff);      // Refine them using least-squares
			model->selectWithinDistance (coeff, sac_distance_threshold_, inliers);

			// make plane normal point towards origin
			geometry_msgs::Point32 viewpoint;
			viewpoint.x = 0; viewpoint.y = 0; viewpoint.z = 0;
			cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.points.at (inliers[0]), viewpoint);

			ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
					coeff[0], coeff[1], coeff[2], coeff[3]);

			// Project the inliers onto the model
			model->projectPointsInPlace (inliers, coeff);

			// find and publish convex hull
			mapping_msgs::PolygonalMap pmap;
			pmap.header.stamp = points.header.stamp;
			pmap.header.frame_id = points.header.frame_id;
			pmap.polygons.resize (1);
			cloud_geometry::areas::convexHull2D (points, inliers, coeff, pmap.polygons[0]);
			pm_pub_.publish(pmap);

                        sensor_msgs::PointCloud cloud_plane;
			cloud_geometry::getPointCloud (cloud_, inliers, cloud_plane);
			plane_pub_.publish(cloud_plane);

		}
		else
		{
			ROS_ERROR ("Could not compute a plane model.");
			return (false);
		}

		delete sac;
		delete model;
		return (true);
	}


};




int main(int argc, char** argv)
{
	ros::init(argc, argv, "plane_fit");
	PlanarFit pf;

	ros::spin(); // ROS spin loop

	return 0;
}
