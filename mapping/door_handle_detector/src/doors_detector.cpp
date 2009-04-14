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
 * $Id$
 *
 */

#include "door_handle_detector/doors_detector.h"
#include "door_handle_detector/door_functions.h"


using namespace std;
using namespace robot_msgs;
using namespace ros;
using namespace door_handle_detector;

#define DEBUG_FAILURES 0


DoorDetector::DoorDetector (ros::Node* anode)
  : node_ (anode), tf_ (*anode)
{
  // ---[ Parameters regarding geometric constraints for the door/handle
  {
    node_->param ("~parameter_frame", parameter_frame_, string ("base_footprint"));
    node_->param ("~fixed_frame", fixed_frame_, string ("odom_combined"));

    node_->param ("~door_min_height", door_min_height_, 1.2);                  // minimum height of a door: 1.2m
    node_->param ("~door_max_height", door_max_height_, 3.0);                  // maximum height of a door: 3m
    node_->param ("~door_min_width", door_min_width_, 0.7);                    // minimum width of a door: 0.7m
    node_->param ("~door_max_width", door_max_width_, 1.4);                    // maximum width of a door: 1.4m
    ROS_DEBUG ("Using the following thresholds for door detection [min-max height / min-max width]: %f-%f / %f-%f.",
               door_min_height_, door_max_height_, door_min_width_, door_max_width_);

    // NOTE: it makes _absolutely_ no sense to search for information far away from the robot as the data is incredibly sparse !
    node_->param ("~maximum_search_radius", maximum_search_radius_, 8.0);     // only consider points closer than this value to the robot
    maximum_search_radius_ *= maximum_search_radius_;                        // squared for faster processing

    // Heuristically discard all door candidates which have their polygonal bounds at angles larger than this value with the viewpoint
    node_->param ("~maximum_scan_angle", maximum_scan_angle_limit_, 70.0);
    maximum_scan_angle_limit_ = angles::from_degrees (maximum_scan_angle_limit_);

    // This parameter constrains the door polygon to resamble a rectangle,
    // that is: the two lines parallel (with some angular threshold) to the Z-axis must have some minimal length
    // each side of the door has to be at least 40 cm
    node_->param ("~rectangle_constrain_edge_height", rectangle_constrain_edge_height_, 0.4);
    // maximum angle threshold between a side and the Z-axis: 10 degrees
    node_->param ("~rectangle_constrain_edge_angle", rectangle_constrain_edge_angle_, 10.0);
    rectangle_constrain_edge_angle_ = angles::from_degrees (rectangle_constrain_edge_angle_);
  }

  // ---[ Parameters regarding optimizations / real-time computations
  leaf_width_ = 0.02;              // 2cm box size by default
  k_search_   = 10;                // 10 k-neighbors by default
  z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;

  minimum_z_ = 0.05;               // We don't care about points 5 cm below the ground
  maximum_z_ = 2.7;                // We don't care about points 2.7m above the ground

  normal_angle_tolerance_ = angles::from_degrees (15.0); // Maximum angular difference in normal space for inliers wrt the Z-axis
  // Parameters regarding the thresholds for Euclidean region growing/clustering
  euclidean_cluster_angle_tolerance_    = angles::from_degrees (25.0);
  euclidean_cluster_min_pts_            = 1000;               // 1000 points
  euclidean_cluster_distance_tolerance_ = 0.05;               // 5 cm

  // This should be set to whatever the leaf_width * 2 factor is in the downsampler
  sac_distance_threshold_ = leaf_width_ * 2;                  // 4 cm by default

  // The minimum Z point on the door must be lower than this value
  door_min_z_ = 0.35;

  // Heuristically discard all door candidates which have their polygonal bounds at distances more than this value
  maximum_search_radius_limit_ = maximum_search_radius_ - 0.2;


  // Estimate the minimum region point density in 1 square meter
  minimum_region_density_ = (1 / leaf_width_) * (1 / leaf_width_) * 2.0 / 3.0;


  // advertise services
  node_->param ("~input_cloud_topic", input_cloud_topic_, string ("/snapshot_cloud"));
  node_->advertiseService ("doors_detector", &DoorDetector::detectDoorSrv, this);
  node_->advertiseService ("doors_detector_cloud", &DoorDetector::detectDoorCloudSrv, this);
  node_->advertise<robot_msgs::VisualizationMarker> ("visualizationMarker", 100);
  node_->advertise<PolygonalMap> ("~door_frames", 1);
  node_->advertise<PointCloud> ("~door_regions", 1);

  global_marker_id_ = 1;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief This is the main door detection function */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
  DoorDetector::detectDoors(const robot_msgs::Door& door, PointCloud pointcloud, std::vector<robot_msgs::Door>& result) const
{
  ROS_INFO ("DoorDetector: Start detecting doors in a point cloud of size %i", (int)pointcloud.pts.size ());

  Time ts = Time::now();
  Duration duration;
  Duration timeout = Duration().fromSec(5.0);

  // transform the PCD (Point Cloud Data) into the parameter_frame, and work there
  if (!tf_.canTransform(parameter_frame_, pointcloud.header.frame_id, pointcloud.header.stamp, timeout)){
    ROS_ERROR ("DoorDetector: Could not transform point cloud from frame '%s' to frame '%s'.",
               pointcloud.header.frame_id.c_str (), parameter_frame_.c_str ());
    return false;
  }
  tf_.transformPointCloud (parameter_frame_, pointcloud, pointcloud);
  ROS_INFO("DoorDetector: Pointcloud transformed to parameter frame");

  // transform the door message into the parameter_frame, and work there
  Door door_tr;
  if (!transformTo(tf_, parameter_frame_, door, door_tr)){
     ROS_ERROR ("DoorDetector: Could not transform door message from frame %s to frame %s.",
                door.header.frame_id.c_str (), parameter_frame_.c_str ());
     return false;
   }
   ROS_INFO("DoorDetector: door message transformed to parameter frame");

  // Get the cloud viewpoint in the parameter frame
  PointStamped viewpoint_cloud_;
  getCloudViewPoint (parameter_frame_, viewpoint_cloud_, &tf_);

  // ---[ Optimization: select a subset of the points for faster processing
  // Select points whose distances are up to Xm from the robot
  int nr_p = 0;
  vector<int> indices_in_bounds (pointcloud.pts.size ());
  for (unsigned int i = 0; i < pointcloud.pts.size (); i++)
  {
    double dist = cloud_geometry::distances::pointToPointDistanceSqr (viewpoint_cloud_.point, pointcloud.pts[i]);
    if (dist < maximum_search_radius_ && pointcloud.pts[i].z > minimum_z_ && pointcloud.pts[i].z < maximum_z_)
      indices_in_bounds[nr_p++] = i;
  }
  indices_in_bounds.resize (nr_p);

  // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
  PointCloud cloud_down;
  vector<cloud_geometry::Leaf> leaves;
  try
  {
    Point leaf_width_xyz;
    leaf_width_xyz.x = leaf_width_xyz.y = leaf_width_xyz.z = leaf_width_;
    cloud_geometry::downsamplePointCloud (pointcloud, indices_in_bounds, cloud_down, leaf_width_xyz, leaves, -1);
    ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_xyz.x, leaf_width_xyz.y, leaf_width_xyz.z, (int)cloud_down.pts.size ());
  }
  catch (std::bad_alloc)
  {
    // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
    return (false);
  }
  leaves.resize (0);    // dealloc memory used for the downsampling process

#if DEBUG_FAILURES
  sendMarker (viewpoint_cloud_.point.x, viewpoint_cloud_.point.y, viewpoint_cloud_.point.z, parameter_frame_, &node_, global_marker_id_, 0, 0, 0);
#endif
  // Create Kd-Tree and estimate the point normals in the original point cloud
  estimatePointNormals (pointcloud, indices_in_bounds, cloud_down, k_search_, viewpoint_cloud_);

  // Select points whose normals are perpendicular to the Z-axis
  vector<int> indices_xy;
  cloud_geometry::getPointIndicesAxisPerpendicularNormals (cloud_down, 0, 1, 2, normal_angle_tolerance_, z_axis_, indices_xy);

  // Split the Z-perpendicular points into clusters
  vector<vector<int> > clusters;
  findClusters (cloud_down, indices_xy, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2,
                euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
  sort (clusters.begin (), clusters.end (), compareRegions);
  reverse (clusters.begin (), clusters.end ());

  PolygonalMap pmap_;
  if (clusters.size () == 0)
  {
    ROS_ERROR ("did not find a door");
    node_->publish ("~door_frames", pmap_);
    return (false);
  }

  // Output the point regions
  PointCloud cloud_regions;
  cloud_regions.chan.resize (1);
  cloud_regions.chan[0].name = "rgb";
  cloud_regions.header = cloud_down.header;
  cloud_regions.pts.resize (0);
  cloud_regions.chan[0].vals.resize (0);
  for (unsigned int cc = 0; cc < clusters.size (); cc++)
  {
    float r = rand () / (RAND_MAX + 1.0);
    float g = rand () / (RAND_MAX + 1.0);
    float b = rand () / (RAND_MAX + 1.0);
    for (unsigned int j = 0; j < clusters[cc].size (); j++)
    {
      cloud_regions.pts.push_back (cloud_down.pts[clusters[cc][j]]);
      cloud_regions.chan[0].vals.push_back (getRGB (r, g, b));
    }
  }
  node_->publish ("~door_regions", cloud_regions);


  vector<vector<double> > coeff (clusters.size ()); // Need to save all coefficients for all models
  pmap_.header = pointcloud.header;
  pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map

  ROS_INFO (" - Process all clusters (%i)", (int)clusters.size ());
  vector<double> goodness_factor (clusters.size());

#pragma omp parallel for schedule(dynamic)
  // Process all clusters in parallel
  for (int cc = 0; cc < (int)clusters.size (); cc++)
  {
    bool bad_candidate = false;
    // initial goodness factor
    goodness_factor[cc] = 1;

    // Find the best plane in this cluster
    vector<int> inliers;
    if (!fitSACPlane (cloud_down, clusters[cc], inliers, coeff[cc], viewpoint_cloud_, sac_distance_threshold_, euclidean_cluster_min_pts_))
    {
      goodness_factor[cc] = 0;
      ROS_DEBUG ("R: Could not find planar model for cluster %d (%d hull points, %d points).", cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size ());
      continue;
    }

    if (inliers.size () == 0)
    {
      ROS_DEBUG ("R: Could not find planar model for cluster %d (%d hull points, %d points).", cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size ());
      goodness_factor[cc] = 0;
      continue;
    }

    // Compute the convex hull
    cloud_geometry::areas::convexHull2D (cloud_down, inliers, coeff[cc], pmap_.polygons[cc]);

    // ---[ If any of the points on the hull are near/outside the imposed "maximum search radius to viewpoint" limit, discard the candidate
    for (int i = 0; i < (int)pmap_.polygons[cc].points.size (); i++)
    {
      double dist = cloud_geometry::distances::pointToPointDistanceSqr (viewpoint_cloud_.point, pmap_.polygons[cc].points[i]);
      if (dist < maximum_search_radius_limit_)
        continue;

      goodness_factor[cc] = 0;
      ROS_DEBUG ("R: Door candidate (%d, %d hull points, %d points) rejected because the bounds (distance = %g) are near the edges of the cloud!",
                 cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), dist);
      bad_candidate = true;
      break;
    }
    if (bad_candidate) continue;

    // ---[ If any of the points on the hull are near/outside the imposed "maximum scan angle to viewpoint" limit, discard the candidate
    // Select points which are acquired at in a horizontal (Z perpendicular) angle of 140 degrees (-70; +70) by default
    for (int i = 0; i < (int)pmap_.polygons[cc].points.size (); i++)
    {
      // Get the angle with the X axis (0->2pi)
      double angle = cloud_geometry::angles::getAngle2D (pmap_.polygons[cc].points[i].x, pmap_.polygons[cc].points[i].y);
      if (angle > M_PI / 2.0) angle -= 2*M_PI;
      if (fabs (angle) < maximum_scan_angle_limit_)
        continue;

      goodness_factor[cc] = 0;
#if DEBUG_FAILURES
      Point32 centroid;
      cloud_geometry::nearest::computeCentroid (&pmap_.polygons[cc], centroid);
      sendMarker (centroid.x, centroid.y, centroid.z, parameter_frame_, &node_, global_marker_id_, 255, 0, 0);
#endif
      ROS_DEBUG ("R: Door candidate (%d, %d hull points, %d points) rejected because the bounds (angle = %g, %d, <%g,%g>) are near the edges of the cloud!",
                 cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), angles::to_degrees (angle), i, pmap_.polygons[cc].points[i].x, pmap_.polygons[cc].points[i].y);
      bad_candidate = true;
      break;
    }
    if (bad_candidate) continue;

    // Filter the region based on its height and width
    robot_msgs::Point32 min_p, max_p;
    cloud_geometry::statistics::getMinMax (pmap_.polygons[cc], min_p, max_p);

    // ---[ Quick test for min_p.z (!)
    if (min_p.z > door_min_z_)
    {
      goodness_factor[cc] = 0;
#if DEBUG_FAILURES
      Point32 centroid;
      cloud_geometry::nearest::computeCentroid (&pmap_.polygons[cc], centroid);
      sendMarker (centroid.x, centroid.y, centroid.z, parameter_frame_, &node_, global_marker_id_, 0, 255, 0);
#endif
      ROS_DEBUG ("R: Door candidate (%d, %d hull points, %d points) rejected because min.Z (%g) > than (%g)!",
                 cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), min_p.z, door_min_z_);
      continue;
    }

    // ---[ Get the limits of the two "parallel to the Z-axis" lines defining the door
    double height_df1 = 0.0, height_df2 = 0.0;
    if (!checkDoorEdges (pmap_.polygons[cc], z_axis_, rectangle_constrain_edge_height_, rectangle_constrain_edge_angle_,
                         height_df1, height_df2))
    {
      goodness_factor[cc] = 0;
      ROS_DEBUG ("R: Door candidate (%d, %d hull points, %d points) rejected because the length of the door edges (%g / %g) < than %g!",
                 cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), height_df1, height_df2, rectangle_constrain_edge_height_);
      continue;
    }

    // ---[ Compute the door width and height
    double door_frame = sqrt ( (max_p.x - min_p.x) * (max_p.x - min_p.x) + (max_p.y - min_p.y) * (max_p.y - min_p.y) );
    double door_height = fabs (max_p.z - min_p.z);
    // Adapt the goodness factor for each cluster
    if (door_frame < door_min_width_ || door_height < door_min_height_ || door_frame > door_max_width_ || door_height > door_max_height_)
    {
      goodness_factor[cc] = 0;
#if DEBUG_FAILURES
      Point32 centroid;
      cloud_geometry::nearest::computeCentroid (&pmap_.polygons[cc], centroid);
      sendMarker (centroid.x, centroid.y, centroid.z, parameter_frame_, &node_, global_marker_id_, 0, 0, 255);
#endif
      ROS_DEBUG ("R: Door candidate (%d, %d hull points, %d points) rejected because of bad width/height (%g < %g < %g / %g < %g < %g).",
                 cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), door_min_width_, door_frame, door_min_height_, door_max_width_, door_height, door_max_height_);
      continue;
    }
    double area = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);

    double density = (double)inliers.size () / area;        // Compute the average point density

    // ---[ Check the average point density
    if (density < minimum_region_density_)
    {
      goodness_factor[cc] = 0;
#if DEBUG_FAILURES
      Point32 centroid;
      cloud_geometry::nearest::computeCentroid (&pmap_.polygons[cc], centroid);
      sendMarker (centroid.x, centroid.y, centroid.z, parameter_frame_, &node_, global_marker_id_, 255, 255, 255);
#endif
      ROS_DEBUG ("R: Door candidate (%d, %d hull points, %d points) rejected because the average point density (%g) < than %g.",
                 cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), density, minimum_region_density_);
      continue;
    }

    goodness_factor[cc] *= (area / (door_frame * door_height));

    // ---[ Compute the distance from the door to the prior of the door
    double door_distance = fmax (0.001,
                                 fmin (cloud_geometry::distances::pointToPointXYDistance (door_tr.door_p1, min_p),
                                       fmin (cloud_geometry::distances::pointToPointXYDistance (door_tr.door_p1, max_p),
                                             fmin (cloud_geometry::distances::pointToPointXYDistance (door_tr.door_p2, min_p),
                                                   cloud_geometry::distances::pointToPointXYDistance (door_tr.door_p2, max_p)))));
    goodness_factor[cc] /= door_distance;
    ROS_WARN ("A: Door candidate (%d, %d hull points, %d points) accepted with: average point density (%g), area (%g), width (%g), height (%g).\n Planar coefficients: [%g %g %g %g]",
              cc, (int)pmap_.polygons[cc].points.size (), (int)inliers.size (), density, area, door_frame, door_height, coeff[cc][0], coeff[cc][1], coeff[cc][2], coeff[cc][3]);
  } // loop over clusters


  // Merge clusters with the same or similar door frame
  double similar_door_eps = 0.1;
  Point32 min_pcc, max_pcc, min_pdd, max_pdd;
  for (unsigned int cc = 0; cc < clusters.size (); cc++)
  {
    if (goodness_factor[cc] == 0)
      continue;
    cloud_geometry::statistics::getMinMax (pmap_.polygons[cc], min_pcc, max_pcc);
    double area_cc = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);

    for (unsigned int dd = cc; dd < clusters.size (); dd++)
    {
      if (cc == dd || goodness_factor[dd] == 0)
        continue;
      cloud_geometry::statistics::getMinMax (pmap_.polygons[dd], min_pdd, max_pdd);

      // Check if any of the doors have at least a common frame side
      if ((fabs (min_pcc.x - min_pdd.x) < similar_door_eps && fabs (min_pcc.y - min_pdd.y) < similar_door_eps) ||
          (fabs (max_pcc.x - max_pdd.x) < similar_door_eps && fabs (max_pcc.y - max_pdd.y) < similar_door_eps)
          )
      {
        // Check if the normals of the planes are parallel
        double angle = acos ( cloud_geometry::dot (coeff[cc], coeff[dd]) );
        if ( (angle < normal_angle_tolerance_) || ( (M_PI - angle) < normal_angle_tolerance_ ) )
        {
          // Check which area is bigger
          double area_dd = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[dd], coeff[dd]);
          if (area_dd < area_cc)
            goodness_factor[dd] = 0;
          else
            goodness_factor[cc] = 0;
        }
      }
    }
  }

  // Count the number of remaining clusters with non-null goodness factor
  int doors_found_cnt = 0;
  for (int cc = 0; cc < (int)clusters.size (); cc++)
  {
    if (goodness_factor[cc] != 0)
      doors_found_cnt++;
    else
      pmap_.polygons[cc].points.resize (0);
  }

  ROS_INFO (" - Found %d / %d potential door candidates.", doors_found_cnt, (int)clusters.size ());
  result.resize(doors_found_cnt);

  // Copy all clusters
  int nr_d = 0;
  robot_msgs::Point32 min_p, max_p;
  for (int cc = 0; cc < (int)clusters.size (); cc++)
  {
    if (goodness_factor[cc] == 0)
      continue;

    // initialize with original door message
    result[nr_d] = door_tr;

    // set the timestamp to the stamp of the pontcloud
    result[nr_d].header.stamp = pointcloud.header.stamp;

    // Save the weight (we need to reorder at the end)
    result[nr_d].weight = goodness_factor[cc];

    // Get the min_p and max_p of selected cluster
    cloud_geometry::statistics::getLargestXYPoints (pmap_.polygons[cc], min_p, max_p);

    // Reply doors message in same frame as request doors message
    result[nr_d].door_p1 = min_p;
    result[nr_d].door_p2 = max_p;
    result[nr_d].door_p2.z = min_p.z;
    result[nr_d].normal.x = -coeff[cc][0];
    result[nr_d].normal.y = -coeff[cc][1];
    result[nr_d].normal.z = -coeff[cc][2];

    // Need min/max Z
    cloud_geometry::statistics::getMinMax (pmap_.polygons[cc], min_p, max_p);
    result[nr_d].height = fabs (max_p.z - min_p.z);

    cout << "transform door to " << fixed_frame_ << endl;
    if (!transformTo(tf_, fixed_frame_, result[nr_d], result[nr_d])){
      ROS_ERROR ("DoorsDetector: could not tranform door from '%s' frame to '%s' frame", 
		 result[nr_d].header.frame_id.c_str(), fixed_frame_.c_str());
      return false;
    }
    cout << "found door " << result[nr_d] << endl;

    nr_d++;
  }

  // Check if any cluster respected all our constraints (i.e., has a goodness_factor > 0)
  if (nr_d == 0)
  {
    ROS_ERROR ("did not find a door");
    node_->publish ("~door_frames", pmap_);
    return (false);
  }

  // Order the results based on the weight (e.g. goodness factor)
  sort (result.begin (), result.end (), compareDoorsWeight);
  reverse (result.begin (), result.end ());

  duration = ros::Time::now () - ts;
  ROS_INFO ("Door(s) found and ordered by weight. Result in frame %s", result[0].header.frame_id.c_str ());
  for (int cd = 0; cd < nr_d; cd++)
  {
    ROS_INFO ("  %d -> P1 = [%g, %g, %g]. P2 = [%g, %g, %g]. Width = %g. Height = %g. Weight = %g.", cd,
              result[cd].door_p1.x, result[cd].door_p1.y, result[cd].door_p1.z, result[cd].door_p2.x, result[cd].door_p2.y, result[cd].door_p2.z,
              sqrt ((result[cd].door_p1.x - result[cd].door_p2.x) *
                    (result[cd].door_p1.x - result[cd].door_p2.x) +
                    (result[cd].door_p1.y - result[cd].door_p2.y) *
                    (result[cd].door_p1.y - result[cd].door_p2.y)
                    ), result[cd].height, result[cd].weight);
  }
  ROS_INFO ("  Total time: %g.", duration.toSec ());

  node_->publish ("~door_frames", pmap_);

  ROS_INFO ("Finished detecting door.");

  return true;
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Service call to detect doors*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool  DoorDetector::detectDoorSrv (door_handle_detector::DoorsDetector::Request &req,
                                   door_handle_detector::DoorsDetector::Response &resp)
{
  // receive a new laser scan
  num_clouds_received_ = 0;
  tf::MessageNotifier<robot_msgs::PointCloud>* message_notifier =
    new tf::MessageNotifier<robot_msgs::PointCloud> (&tf_, node_,  boost::bind (&DoorDetector::cloud_cb, this, _1),
                                                     input_cloud_topic_, parameter_frame_, 1);
  ros::Duration tictoc = ros::Duration ().fromSec (0.1);
  while ((int)num_clouds_received_ < 1)
    tictoc.sleep ();
  delete message_notifier;

  return detectDoors(req.door, pointcloud_, resp.doors);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Service call to detect doors*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool DoorDetector::detectDoorCloudSrv (door_handle_detector::DoorsDetectorCloud::Request &req,
                                       door_handle_detector::DoorsDetectorCloud::Response &resp)
{
  return detectDoors(req.door, req.cloud, resp.doors);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main point cloud callback.                                                                           */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DoorDetector::cloud_cb (const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& cloud)
{
  pointcloud_ = *cloud;
  ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)pointcloud_.pts.size (), pointcloud_.header.frame_id.c_str (),
            (int)pointcloud_.chan.size (), cloud_geometry::getAvailableChannels (pointcloud_).c_str ());
  num_clouds_received_++;
}






/* ---[ */
int main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("doors_detector_node");

  door_handle_detector::DoorDetector p(&ros_node);

  ros_node.spin ();


  return (0);
}
/* ]--- */

