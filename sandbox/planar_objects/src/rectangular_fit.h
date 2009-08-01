#ifndef rectangular_fit
#define rectangluar_fit

// ROS core
#include <ros/node.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>

class PlanarFit {
	void getPointIndicesInZBounds(const sensor_msgs::PointCloud &points,
			double z_min, double z_max, std::vector<int> &indices);

	bool fitSACPlanes(sensor_msgs::PointCloud *points,
			std::vector<int> &indices, std::vector<std::vector<int> > &inliers,
			std::vector<std::vector<double> > &coeff,
			const geometry_msgs::Point32 &viewpoint_cloud, double dist_thresh,
			int n_max, int min_points_per_model = 100);
	/**
	 * Input
	 o list of X,Y,Z,x,y points
	 o Z min, Z max (only process points within min to max range
	 o Support  where "Support" is a vertical distance above the plane where an object not on that plane will be considered to be supported by that plane  (OK, if this takes time, don't do it right now)
	 o A where A is the minimal area a plane should have (again, don't bother with for now unless its already there)
	 o N max number of planes to find in order of size
	 * Output
	 o list of indices in order of input list. Each indices is the tuple x,y,A where x,y is the pixel and A is "computed attribute"
	 + 0 is illegal range or othersize "no data"
	 + 1-N where this numbers the planes in terms of area. 1 is largest plane, 2 is next largest ...
	 + -1, -2, -3 means supported object by cluster (or for now for speed, all points that are not a 0 or not a plane are -1).
	 **/

	void segmentPlanes(sensor_msgs::PointCloud &points, double z_min,
			double z_max, double support, double min_area, int n_max,
			std::vector<std::vector<int> > &indices, std::vector<std::vector<
					double> > &models,int number);
	void publishNormals(sensor_msgs::PointCloud points, std::vector<geometry_msgs::Vector3> coeff, float length=0.1);
	void publishPolygon(sensor_msgs::PointCloud pointcloud,robot_msgs::Polygon3D points,int number);
protected:
	ros::Node& node_;

public:
	// ROS messages
	sensor_msgs::PointCloud cloud_, cloud_plane_, cloud_outliers_;

	double z_min_, z_max_, support_, min_area_;
	int n_max_;

	PlanarFit(ros::Node& anode);

	void cloud_cb(); // callback
};

#endif
