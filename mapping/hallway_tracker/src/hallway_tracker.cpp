//Software License Agreement (BSD License)

//Copyright (c) 2009, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

/**
   @mainpage

   @htmlinclude manifest.html

   \author Caroline Pantofaru

   @b hallway_tracker Given that the robot is in a hallway, find the two dominant walls.

**/

#include <sys/time.h>
#include <float.h>

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/PolygonalMap.h>

#include <robot_msgs/Point32.h>
//#include <robot_msgs/Hallway.h>
#include <robot_msgs/VisualizationMarker.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
//#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
//#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/sac_model_parallel_lines.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/statistics.h>

// Transformations
#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

// Clouds and scans
#include <robot_msgs/PointCloud.h>
#include <laser_scan/LaserScan.h>
#include <laser_scan/laser_scan.h>



using namespace std;
using namespace robot_msgs;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Assuming that the robot is in a hallway, find the two dominant lines.
 * \note The walls are defined as two parallel lines, appropriately spaced, in the 2D base scan.
 * \note Each scan is processed independently. A possible extension is to add a time filter for more stable approximation.
 * \note Walls may be found at any angle. This could be extended to look for walls only within a certain angle range.
 */
class HallwayTracker
{
public:

  ros::Node *node_;

  PointCloud cloud_;                 
  laser_scan::LaserProjection projector_; // Used to project laser scans into point clouds

  tf::TransformListener *tf_;
  tf::MessageNotifier<laser_scan::LaserScan>* message_notifier_;

  /********** Parameters from the param server *******/
  std::string base_laser_topic_; // Topic for the laser scan message.
  int sac_min_points_per_model_; // Minimum number of points in the scan within max_point_dist_m_ of the robot needed for a model.
  double sac_distance_threshold_; // The distance threshold used by the SAC methods for declaring a point an inlier of the model.
  //double eps_angle_; // For possible later use if the method is extended to find oriented parallel lines.
  std::string fixed_frame_; // Frame to work in.
  double min_hallway_width_m_, max_hallway_width_m_; // Minimum and maximum hallway widths.
  double max_point_dist_m_; // Max distance from the robot of points in the model.

  HallwayTracker():message_notifier_(NULL)
  {
    node_ = ros::Node::instance();
    tf_ = new tf::TransformListener(*node_);          
           
    // Get params from the param server. 
    node_->param("~p_base_laser_topic", base_laser_topic_, string("base_scan")); 
    node_->param("~p_sac_min_points_per_model", sac_min_points_per_model_, 50);  
    node_->param("~p_sac_distance_threshold", sac_distance_threshold_, 0.03);     // 3 cm 
    //node_->param("~p_eps_angle", eps_angle_, 10.0);                              // 10 degrees
    node_->param("~p_fixed_frame", fixed_frame_, string("odom_combined"));
    node_->param("~p_min_hallway_width_m", min_hallway_width_m_, 1.0);
    node_->param("~p_max_hallway_width_m", max_hallway_width_m_, 5.0);
    node_->param("~p_max_point_dist_m", max_point_dist_m_, 5.0);

    //eps_angle_ = cloud_geometry::deg2rad (eps_angle_);                      // convert to radians

    // Visualization: 
    // The visualization markers are the two lines. The start/end points are arbitrary.
    node_->advertise<robot_msgs::VisualizationMarker>( "visualizationMarker", 0 );
    // A point cloud of model inliers.
    node_->advertise<robot_msgs::PointCloud>("parallel_lines_inliers",10);

    // Output: a point cloud with 3 points. The first two points lie on the first line, and the third point lies on the second line.
    node_->advertise<robot_msgs::PointCloud>("parallel_lines_model",0);

    // Subscribe to the scans.
    message_notifier_ = new tf::MessageNotifier<laser_scan::LaserScan> (tf_, node_, boost::bind(&HallwayTracker::laserCallBack, this, _1), base_laser_topic_.c_str(), fixed_frame_, 1);
  };

  ~HallwayTracker()
  {
    node_->unadvertise("visualizationMarker");
    node_->unadvertise("parallel_lines_inliers");
    delete message_notifier_;
  }

  /**
   * \brief Laser callback. Processes a laser scan by converting it into a point cloud, removing points that are too far away, finding two parallel lines and publishing the results.
   */
  void laserCallBack(const tf::MessageNotifier<laser_scan::LaserScan>::MessagePtr& scan_msg)
  {

    // Transform into a PointCloud message
    int mask = laser_scan::MASK_INTENSITY | laser_scan::MASK_DISTANCE | laser_scan::MASK_INDEX | laser_scan::MASK_TIMESTAMP;
    projector_.transformLaserScanToPointCloud(fixed_frame_, cloud_, *scan_msg, *tf_, mask);

    // Check that the cloud is large enough.
    if(cloud_.pts.empty())
    {
      ROS_WARN("Received an empty point cloud");
      return;
    }
    if ((int)cloud_.pts.size() < sac_min_points_per_model_)
    {
      ROS_WARN("Insufficient points in the scan for the parallel lines model.");
      return;
    }

    int cloud_size = cloud_.pts.size();
    vector<int> possible_hallway_points;
    possible_hallway_points.resize(cloud_size);  

    // Keep only points that are within max_point_dist_m_ of the robot.
    int iind = 0;
    tf::Stamped<tf::Point> tmp_pt_in, tmp_pt_out;
    tmp_pt_in.stamp_ = cloud_.header.stamp;
    tmp_pt_in.frame_id_ = cloud_.header.frame_id;
    for (int i=0; i<cloud_size; ++i) {
      tmp_pt_in[0] = cloud_.pts[i].x;
      tmp_pt_in[1] = cloud_.pts[i].y;
      tmp_pt_in[2] = cloud_.pts[i].z;
      tf_->transformPoint("base_laser", tmp_pt_in, tmp_pt_out); // Get distance from the robot.
      if (tmp_pt_out.length() < max_point_dist_m_) {
	possible_hallway_points[iind] = i;
	iind++;
      }
    }
    possible_hallway_points.resize(iind);

    //// Find the dominant lines ////
    vector<int> inliers;
    vector<double> coeffs;

    // Create and initialize the SAC model
    sample_consensus::SACModelParallelLines *model = new sample_consensus::SACModelParallelLines(min_hallway_width_m_,max_hallway_width_m_);
    // RANSAC works best! For choosing the best model, a strict inlier count is better than the average point->model distance.
    sample_consensus::SAC *sac = new sample_consensus::RANSAC(model, sac_distance_threshold_);
    sac->setMaxIterations (500);
    sac->setProbability (0.98);
    model->setDataSet(&cloud_, possible_hallway_points);


    // Now find the best fit parallel lines to this set of points and a corresponding set of inliers
    if (sac->computeModel()) {
      inliers = sac->getInliers();
      coeffs = sac->computeCoefficients();
      visualization(coeffs, inliers);
      // Publish the result
      robot_msgs::PointCloud model_cloud;
      model_cloud.pts.resize(3);
      model_cloud.header.stamp = cloud_.header.stamp;
      model_cloud.header.frame_id = fixed_frame_;
      model_cloud.pts[0].x = coeffs[0];
      model_cloud.pts[0].y = coeffs[1];
      model_cloud.pts[0].z = coeffs[2];
      model_cloud.pts[1].x = coeffs[3];
      model_cloud.pts[1].y = coeffs[4];
      model_cloud.pts[1].z = coeffs[5];
      model_cloud.pts[2].x = coeffs[6];
      model_cloud.pts[2].y = coeffs[7];
      model_cloud.pts[2].z = coeffs[8];
      node_->publish("parallel_lines_model", model_cloud);
    }
    else {
      // No parallel lines were found.
    }  

  }


  /**
   * \brief Compute and publish the visualization info, including the two lines and the model inliers point cloud.
   */
  void visualization(std::vector<double> coeffs, std::vector<int> inliers) {
     // First line:
    robot_msgs::VisualizationMarker marker;
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = cloud_.header.stamp;
    marker.id = 0;
    marker.type = robot_msgs::VisualizationMarker::LINE_STRIP;
    marker.action = robot_msgs::VisualizationMarker::ADD;
    marker.x = 0.0;
    marker.y = 0.0;
    marker.z = 0.0;
    marker.yaw = 0.0;
    marker.pitch = 0.0;
    marker.roll = 0.0;
    marker.xScale = 0.05;//0.01;
    marker.yScale = 0.05;//0.1;
    marker.zScale = 0.05;//0.1;
    marker.alpha = 255;
    marker.r = 0;
    marker.g = 255;
    marker.b = 0;
    marker.set_points_size(2);

    robot_msgs::Point32 d;
    d.x = coeffs[3] - coeffs[0];
    d.y = coeffs[4] - coeffs[1];
    d.z = coeffs[5] - coeffs[2];
    double l = sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
    d.x = d.x/l;
    d.y = d.y/l;
    d.z = d.z/l;

    marker.points[0].x = coeffs[0] - 6*d.x;
    marker.points[0].y = coeffs[1] - 6*d.y;
    marker.points[0].z = coeffs[2] - 6*d.z;

    marker.points[1].x = coeffs[0] + 6*d.x;
    marker.points[1].y = coeffs[1] + 6*d.y;
    marker.points[1].z = coeffs[2] + 6*d.z;

    node_->publish( "visualizationMarker", marker );

    // Second line:
    marker.id = 1;

    marker.points[0].x = coeffs[6] - 6*d.x;
    marker.points[0].y = coeffs[7] - 6*d.y;
    marker.points[0].z = coeffs[8] - 6*d.z;

    marker.points[1].x = coeffs[6] + 6*d.x;
    marker.points[1].y = coeffs[7] + 6*d.y;
    marker.points[1].z = coeffs[8] + 6*d.z;

    node_->publish( "visualizationMarker", marker );

    // Inlier cloud
    robot_msgs::PointCloud  inlier_cloud; 
    inlier_cloud.header.frame_id = fixed_frame_;
    inlier_cloud.header.stamp = cloud_.header.stamp;
    inlier_cloud.pts.resize(inliers.size());
    for (unsigned int i=0; i<inliers.size(); ++i) {
      inlier_cloud.pts[i]  = cloud_.pts[inliers[i]];
    }
    node_->publish("parallel_lines_inliers", inlier_cloud);
  }   

};


/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node n("hallway_tracker");

  HallwayTracker ht;

  n.spin ();
  return (0);
}
/* ]--- */

