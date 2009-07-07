/*
 * Copyright (c) 2009 Juergen Sturm <sturm -=- informatik.uni-freiburg.de>
 *
 */

#include "rectangular_fit.h"
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace robot_msgs;

#define MIN(a,b) ((a<b)?a:b)
#define MAX(a,b) ((a>b)?a:b)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
PlanarFit::PlanarFit(ros::Node& anode) :
	node_(anode) {
	node_.param("~z_min", z_min_, 0.5);
	node_.param("~z_max", z_max_, 1.5);
	node_.param("~support", support_, 0.1);
	node_.param("~min_area", min_area_, 0.2);
	node_.param("~n_max", n_max_, 1);

	string cloud_topic("/stereo/cloud");

	vector<pair<string, string> > t_list;
	node_.getPublishedTopics(&t_list);
	bool topic_found = false;
	for (vector<pair<string, string> >::iterator it = t_list.begin(); it
			!= t_list.end(); it++) {
		if (it->first.find(node_.mapName(cloud_topic)) != string::npos) {
			topic_found = true;
			break;
		}
	}
	if (!topic_found)
		ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", node_.mapName (cloud_topic).c_str ());

	node_.subscribe(cloud_topic, cloud_, &PlanarFit::cloud_cb, this, 1);
	node_.advertise<PointCloud> ("~plane", 1);
	node_.advertise<PointCloud> ("~outliers", 1);
	node_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback
void PlanarFit::cloud_cb() {
	ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_.pts.size (), cloud_.header.frame_id.c_str (),
			(int)cloud_.chan.size (), cloud_geometry::getAvailableChannels (cloud_).c_str ());
	if (cloud_.pts.size() == 0) {
		ROS_ERROR ("No data points found. Exiting...");
		return;
	}

	ros::Time ts = ros::Time::now();

	vector<vector<int> > indices;
	vector<vector<double> > models;
	segmentPlanes(cloud_, z_min_, z_max_, support_, min_area_, n_max_, indices,
			models,1);

	if ((int) indices.size() > 0) {
		cloud_geometry::getPointCloud(cloud_, indices[0], cloud_plane_);
		cloud_geometry::getPointCloudOutside(cloud_, indices[0],
				cloud_outliers_);

		segmentPlanes(cloud_outliers_, z_min_, z_max_, support_, min_area_, n_max_, indices,
				models,2);


		ROS_INFO("coeff[0] %f, %f, %f, %f; %d, %d ",models[0][0],models[0][1],models[0][2],models[0][3],models[0].size(),models.size());
		vector<robot_msgs::Vector3> normals;
		normals.resize(cloud_plane_.get_pts_size());

//		for (size_t i=0;i<cloud_plane_.get_pts_size();++i) {
//			normals[i].x = models[0][0];
//			normals[i].y = models[0][1];
//			normals[i].z = models[0][2];
//		}
//		publishNormals(cloud_plane_,normals,0.1);

		ROS_INFO ("Planar model found with %d / %d inliers in %g seconds.\n", (int)indices[0].size (), (int)cloud_.pts.size (), (ros::Time::now () - ts).toSec ());
	}

	node_.publish("~plane", cloud_plane_);
	node_.publish("~outliers", cloud_outliers_);
}


void PlanarFit::segmentPlanes(PointCloud &points, double z_min, double z_max,
		double support, double min_area, int n_max,
		vector<vector<int> > &indices, vector<vector<double> > &models,int number) {
	// This should be given as a parameter as well, or set global, etc
	double sac_distance_threshold_ = 0.02; // 2cm distance threshold for inliers (point-to-plane distance)

	vector<int> indices_in_bounds;
	// Get the point indices within z_min <-> z_max
	getPointIndicesInZBounds(points, z_min, z_max, indices_in_bounds);
	ROS_INFO("segmentPlanes #%d running on %d/%d points",number,points.get_pts_size(),indices_in_bounds.size());

	// We need to know the viewpoint where the data was acquired
	// For simplicity, assuming 0,0,0 for stereo data in the stereo frame - however if this is not true, use TF to get
	//the point in a different frame !
	Point32 viewpoint;
	viewpoint.x = viewpoint.y = viewpoint.z = 0;

	// Use the entire data to estimate the plane equation.
	// NOTE: if this is slow, we can downsample first, then fit (check mapping/point_cloud_mapping/src/planar_fit.cpp)
	//   vector<vector<int> > inliers;
	indices.clear(); //Points that are in plane
	models.clear(); //Plane equations
	//    vector<vector<double> > models;
	fitSACPlanes(&points, indices_in_bounds, indices, models, viewpoint,
			sac_distance_threshold_, n_max);

	// Check the list of planar areas found against the minimally imposed area
	for (unsigned int i = 0; i < models.size(); i++) {
		// Compute the convex hull of the area
		// NOTE: this is faster than computing the concave (alpha) hull, so let's see how this works out
		Polygon3D polygon;
		cloud_geometry::areas::convexHull2D(points, indices[i], models[i],
				polygon);
		publishPolygon(points,polygon,number);

		// Compute the area of the polygon
		double area = cloud_geometry::areas::compute2DPolygonalArea(polygon,
				models[i]);

		// If the area is smaller, reset this planar model
//		if (area < min_area) {
//			models[i].resize(0);
//			indices[i].resize(0);
//			continue;
//		}
	}

	//    // Copy all the planar models inliers to indices
	//    for (unsigned int i = 0; i < inliers.size (); i++)
	//    {
	//      if (inliers[i].size () == 0) continue;
	//
	//      int old_indices_size = indices.size ();
	//      indices.resize (old_indices_size + inliers[i].size ());
	//      for (unsigned int j = 0; j < inliers[i].size (); j++)
	//        indices[old_indices_size + j] = inliers[i][j];
	//    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Obtain a subset of the point cloud indices between two given Z (min, max) values
 * \param points the point cloud message
 * \param z_min the minimum Z value
 * \param z_max the maximum Z value
 */
void PlanarFit::getPointIndicesInZBounds(const PointCloud &points,
		double z_min, double z_max, vector<int> &indices) {
	indices.resize(points.pts.size());
	int nr_p = 0;
	for (unsigned int i = 0; i < points.pts.size(); i++) {
		if ((points.pts[i].z >= z_min && points.pts[i].z <= z_max)) {
			indices[nr_p] = i;
			nr_p++;
		}
	}
	indices.resize(nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Find a list of plane models in a point cloud with SAmple Consensus methods
 * \param points the point cloud message
 * \param indices a subset of point indices to use
 * \param inliers the resultant planar model inliers
 * \param coeff the resultant plane model coefficients
 * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
 * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
 * \param n_max maximum number of planar models to search for
 * \param min_points_per_model the minimum number of points allowed for a planar model (default: 100)
 */
bool PlanarFit::fitSACPlanes(PointCloud *points, vector<int> &indices, vector<
		vector<int> > &inliers, vector<vector<double> > &coeff,
		const robot_msgs::Point32 &viewpoint_cloud, double dist_thresh,
		int n_max, int min_points_per_model) {
	// Create and initialize the SAC model
	sample_consensus::SACModelPlane *model =
			new sample_consensus::SACModelPlane();
	sample_consensus::SAC *sac = new sample_consensus::RANSAC(model,
			dist_thresh);
	sac->setMaxIterations(100);
	model->setDataSet(points, indices);

	int nr_models = 0, nr_points_left = indices.size();
	while (nr_models < n_max && nr_points_left > min_points_per_model) {
		// Search for the best plane
		if (sac->computeModel()) {
			vector<double> model_coeff;
			sac->computeCoefficients(model_coeff); // Compute the model coefficients
			sac->refineCoefficients(model_coeff); // Refine them using least-squares
			coeff.push_back(model_coeff);

			// Get the list of inliers
			vector<int> model_inliers;
			model->selectWithinDistance(model_coeff, dist_thresh, model_inliers);
			inliers.push_back(model_inliers);

			// Flip the plane normal towards the viewpoint
			cloud_geometry::angles::flipNormalTowardsViewpoint(model_coeff,
					points->pts.at(model_inliers[0]), viewpoint_cloud);

			ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)model_inliers.size (),
					model_coeff[0], model_coeff[1], model_coeff[2], model_coeff[3]);

			// Remove the current inliers from the global list of points
			nr_points_left = sac->removeInliers();
			nr_models++;
		} else {
			ROS_ERROR ("Could not compute a planar model for %d points.", nr_points_left);
			break;
		}
	}

	delete sac;
	delete model;
	return (true);
}

void PlanarFit::publishNormals(robot_msgs::PointCloud points, vector<robot_msgs::Vector3> coeff, float length)
{

	visualization_msgs::Marker marker;
	marker.header.frame_id = points.header.frame_id;
	marker.header.stamp = ros::Time((uint64_t)0ULL);
	marker.ns = "normals";
	marker.id = 1;
	marker.type = visualization_msgs::Marker::LINE_LIST;
//	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.002;
	marker.color.a = 1.0;
	marker.color.g = 0.2;
	marker.color.b = 1.0;

	size_t N = MIN(100,points.get_pts_size());
	if(N==0) return;
	size_t STEP = points.get_pts_size()/N;
//	N = points.get_pts_size();
	marker.set_points_size(2*N);
//	marker.set_points_size(2);

	for (size_t i=0;i<N;++i) {

		marker.points[2*i].x = points.pts[i*STEP].x;
		marker.points[2*i].y = points.pts[i*STEP].y;
		marker.points[2*i].z = points.pts[i*STEP].z;

		marker.points[2*i+1].x = points.pts[i*STEP].x+length*coeff[i*STEP].x;
		marker.points[2*i+1].y = points.pts[i*STEP].y+length*coeff[i*STEP].y;
		marker.points[2*i+1].z = points.pts[i*STEP].z+length*coeff[i*STEP].z;
//		ROS_INFO("line %f,%f,%f --> %f,%f,%f",marker.points[2*i].x,marker.points[2*i].y,marker.points[2*i].z,marker.points[2*i+1].x,marker.points[2*i+1].y,marker.points[2*i+1].z);
	}
	ROS_INFO("visualization_marker normals with n=%d points",N);
	node_.publish( "visualization_marker", marker );

}



void PlanarFit::publishPolygon(robot_msgs::PointCloud pointcloud,Polygon3D points,int number)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = pointcloud.header.frame_id;
	marker.header.stamp = ros::Time((uint64_t)0ULL);
	marker.ns = "polygon";
	marker.id = number+1;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.002;
	marker.color.a = 1.0;
	marker.color.r = (number==1)?1.0:0.2;
	marker.color.g = (number==2)?1.0:0.2;

	size_t N = MIN(100,points.get_points_size());
	if(N==0) return;
	size_t STEP = points.get_points_size()/N;
	marker.set_points_size(N);

	for (size_t i=0;i<N;++i) {

		marker.points[i].x = points.points[i*STEP].x;
		marker.points[i].y = points.points[i*STEP].y;
		marker.points[i].z = points.points[i*STEP].z;
//		ROS_INFO("line %f,%f,%f --> %f,%f,%f",marker.points[2*i].x,marker.points[2*i].y,marker.points[2*i].z,marker.points[2*i+1].x,marker.points[2*i+1].y,marker.points[2*i+1].z);
	}
	ROS_INFO("visualization_marker polygon with n=%d points",N);
	node_.publish( "visualization_marker", marker );

}


/* ---[ */
int main(int argc, char** argv) {
	ros::init(argc, argv);

	ros::Node ros_node("planar_objects");

	PlanarFit p(ros_node);
	ros_node.spin();

	return (0);
}
/* ]--- */

