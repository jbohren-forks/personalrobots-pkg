/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: doors_detector.cpp 9895 2009-01-21 21:05:27Z tfoote $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b doors_detector detects and returns a list of doors from 3D point cloud data (laser).

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

// ROS core
#include <ros/node.h>
#include <roslib/Header.h>
#include <robot_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include "geometric_helper.h"

// Include the service call type
#include "door_handle_detector/DoorDetector.h"

#include <tf/message_notifier.h>


using namespace std;
using namespace robot_msgs;

class DoorDetector : public ros::Node
{
public:

    // ROS messages
    PointCloud cloud_in_, cloud_down_;
    PointCloud cloud_annotated_;
    Point32 z_axis_;
    PolygonalMap pmap_;
    tf::MessageNotifier<robot_msgs::PointCloud>*  message_notifier_;

    PointStamped viewpoint_cloud_;

    tf::TransformListener tf_;

    string input_cloud_topic_, parameter_frame_, door_frame_, cloud_frame_;
    roslib::Header cloud_header_;
    ros::Time cloud_time_;
    bool publish_debug_;
    unsigned int num_clouds_received_;


    // Parameters regarding geometric constraints for the door/handle
    double door_min_height_, door_min_width_, door_max_height_, door_max_width_, door_min_z_;
    double handle_distance_door_max_threshold_, handle_max_height_, handle_min_height_;

    // Parameters regarding the size of the 3D bounding box where we will conduct the search for a door/handle
    double door_min_z_bounds_, door_max_z_bounds_;
    int door_frame_multiplier_;

    // Parameters regarding the _fast_ normals/plane computation using a lower quality (downsampled) dataset
    double leaf_width_;
    double sac_distance_threshold_;
    double normal_angle_tolerance_;
    int k_search_;

    // Parameters for the euclidean clustering/cluster rejection
    double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
    int euclidean_cluster_min_pts_;

    // Parameters for the intensity clustering/cluster rejection
    double intensity_cluster_perpendicular_angle_tolerance_;
    int intensity_cluster_min_pts_;

    // Parameters for "rectangularity" constraints
    double rectangle_constrain_edge_height_;
    double rectangle_constrain_edge_angle_;

    // Intensity threshold
    double intensity_threshold_;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DoorDetector () : ros::Node ("doors_detector"), message_notifier_ (NULL), tf_ (*this)
    {
      // ---[ Parameters regarding geometric constraints for the door/handle
      {
        param ("~parameter_frame", parameter_frame_, string ("base_footprint"));

        // Frame _independent_ parameters (absolute values)
          param ("~door_min_height", door_min_height_, 1.2);                  // minimum height of a door: 1.2m
          param ("~door_max_height", door_max_height_, 3.0);                  // maximum height of a door: 3m
          param ("~door_min_width", door_min_width_, 0.8);                    // minimum width of a door: 0.8m
          param ("~door_max_width", door_max_width_, 1.4);                    // maximum width of a door: 1.4m
          ROS_DEBUG ("Using the following thresholds for door detection [min-max height / min-max width]: %f-%f / %f-%f.",
                     door_min_height_, door_max_height_, door_min_width_, door_max_width_);

          param ("~handle_distance_door_max_threshold", handle_distance_door_max_threshold_, 0.25); // maximum distance between the handle and the door

          // This parameter constrains the door polygon to resamble a rectangle,
          // that is: the two lines parallel (with some angular threshold) to the Z-axis must have some minimal length
          param ("~rectangle_constrain_edge_height", rectangle_constrain_edge_height_, 0.2);        // each side of the door has to be at least 20 cm
          param ("~rectangle_constrain_edge_angle", rectangle_constrain_edge_angle_, 20.0);         // maximum angle threshold between a side and the Z-axis: 20 degrees
          rectangle_constrain_edge_angle_ = cloud_geometry::deg2rad (rectangle_constrain_edge_angle_);

	  // Frame _dependent_ parameters (need to be converted!)
          param ("~door_min_z", door_min_z_, 0.1);                            // the minimum Z point on the door must be lower than this value

          param ("~handle_min_height", handle_min_height_, 0.41);            // minimum height for a door handle: 0.41m
          param ("~handle_max_height", handle_max_height_, 1.41);            // maximum height for a door handle: 1.41m
          ROS_DEBUG ("Using the following thresholds for handle detection [min height / max height]: %f / %f.", handle_min_height_, handle_max_height_);

	  // TODO: remove this parameters
          //param ("~door_min_z_bounds", door_min_z_bounds_, -0.5);              
          //param ("~door_max_z_bounds", door_max_z_bounds_, 3.5);               
	  door_min_z_bounds_ = -1.0;
	  door_max_z_bounds_ = 4.0;
      }

      // ---[ Parameters regarding optimizations / real-time computations
      {
        // Frame _independent_ parameters
          // Parameters regarding the _fast_ normals/plane computation using a lower quality (downsampled) dataset
          param ("~downsample_leaf_width", leaf_width_, 0.025);             // 2.5cm radius by default
          param ("~search_k_closest", k_search_, 10);                       // 10 k-neighbors by default
          // This should be set to whatever the leaf_width factor is in the downsampler
          param ("~sac_distance_threshold", sac_distance_threshold_, 0.03); // 3 cm

          // Parameters regarding the maximum allowed angular difference in normal space for inlier considerations
          z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
          param ("~normal_angle_tolerance", normal_angle_tolerance_, 15.0); // 15 degrees, wrt the Z-axis
          normal_angle_tolerance_ = cloud_geometry::deg2rad (normal_angle_tolerance_);

          // Parameters regarding the thresholds for Euclidean region growing/clustering
          param ("~euclidean_cluster_min_pts", euclidean_cluster_min_pts_, 200);                         // 200 points
          // Difference between normals in degrees for cluster/region growing
          param ("~euclidean_cluster_angle_tolerance", euclidean_cluster_angle_tolerance_, 30.0);
          euclidean_cluster_angle_tolerance_ = cloud_geometry::deg2rad (euclidean_cluster_angle_tolerance_);
          param ("~euclidean_cluster_distance_tolerance", euclidean_cluster_distance_tolerance_, 0.04);  // 4 cm

          // Parameters regarding the thresholds for intensity region growing/clustering
          param ("~intensity_threshold", intensity_threshold_, 2000.0);                                                 // 2 points
          param ("~intensity_cluster_min_pts", intensity_cluster_min_pts_, 2);                                                 // 2 points
          param ("~intensity_cluster_perpendicular_angle_tolerance", intensity_cluster_perpendicular_angle_tolerance_, 5.0);   // 5 degrees
          intensity_cluster_perpendicular_angle_tolerance_ = cloud_geometry::deg2rad (intensity_cluster_perpendicular_angle_tolerance_);

          // This describes the size of our 3D bounding box (basically the space where we search for doors),
          // as a multiplier of the door frame (computed using the two points from the service call) in both X and Y directions
          // -1 means the multiplier is not used
	  door_frame_multiplier_ = -1;
      }


      // Temporary parameters
      param ("~publish_debug", publish_debug_, true);

      param ("~input_cloud_topic", input_cloud_topic_, string ("full_cloud"));
      advertiseService("doors_detector", &DoorDetector::detectDoor, this);

      advertise<PointCloud>("handle_visualization", 10);

      /** Sachin **/
      advertise<PointCloud>("intensity_visualization", 10);


      advertise<PolygonalMap>("pmap", 10);
      if (publish_debug_)
      {
        advertise<PolygonalMap> ("semantic_polygonal_map", 1);
        advertise<PointCloud> ("cloud_annotated", 1);

        cloud_annotated_.chan.resize (1);
        cloud_annotated_.chan[0].name = "rgb";
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~DoorDetector () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (hasParam ("~input_cloud_topic"))
        getParam ("~input_cloud_topic", input_cloud_topic_);
      // \NOTE to Wim : Perhaps we need to read the other parameters from the server here ?
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief This is the main service callback: it gets called whenever a request to find a new door is given     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detectDoor (door_handle_detector::DoorDetector::Request &req, door_handle_detector::DoorDetector::Response &resp)
    {
      ros::Time start_time = ros::Time::now();
      ros::Duration delay = ros::Duration().fromSec(25);
      cout << "start time " << start_time.toSec() << endl;
      cout << "Waiting for laser scan to come in newer than " << (start_time + delay).toSec() << endl;

      updateParametersFromServer ();

      // door frame
      door_frame_ = req.door.header.frame_id;

      // receive a new laser scan
      num_clouds_received_ = 0;
      message_notifier_ = new tf::MessageNotifier<robot_msgs::PointCloud> (&tf_, this,  boost::bind(&DoorDetector::cloud_cb, this, _1),
                                                                           input_cloud_topic_.c_str (), door_frame_, 1);
      ros::Duration tictoc = ros::Duration().fromSec(1.0);
      while ((int)num_clouds_received_ < 1 || (cloud_in_.header.stamp < (start_time + delay))){
        tictoc.sleep ();
      }
      delete message_notifier_;
      ROS_INFO("Try to detect door from %i points", cloud_in_.pts.size());

      // cloud frame
      cloud_frame_ = cloud_in_.header.frame_id;
      cloud_time_ = cloud_in_.header.stamp;
      cloud_header_ = cloud_in_.header;

      ros::Time ts;
      ros::Duration duration;
      ts = ros::Time::now ();

      // ---[ Optimization: obtain a set of indices from the entire point cloud that we will work with
      vector<int> indices_in_bounds;
      obtainCloudIndicesSet (&cloud_in_, indices_in_bounds, req, &tf_, parameter_frame_, door_min_z_bounds_, door_max_z_bounds_, door_frame_multiplier_);

      // ---[ Optimization: downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        Point leaf_width_xyz;
        leaf_width_xyz.x = leaf_width_xyz.y = leaf_width_xyz.z = leaf_width_;
        cloud_geometry::downsamplePointCloud (&cloud_in_, &indices_in_bounds, cloud_down_, leaf_width_xyz, leaves, -1);
        ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_xyz.x, leaf_width_xyz.y, leaf_width_xyz.z, cloud_down_.pts.size ());
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }
      leaves.resize (0);    // dealloc memory used for the downsampling process


      // Get the cloud viewpoint
      getCloudViewPoint (cloud_frame_, viewpoint_cloud_, &tf_);

      // Create Kd-Tree and estimate the point normals in the original point cloud
      estimatePointNormals (&cloud_in_, &indices_in_bounds, &cloud_down_, k_search_, &viewpoint_cloud_);

      // Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_xy;
      cloud_geometry::getPointIndicesAxisPerpendicularNormals (&cloud_down_, 0, 1, 2, normal_angle_tolerance_, &z_axis_, indices_xy);

      // Split the Z-perpendicular points into clusters
      vector<vector<int> > clusters;
      findClusters (&cloud_down_, &indices_xy, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2,
                    euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
      sort (clusters.begin (), clusters.end (), compareRegions);
      reverse (clusters.begin (), clusters.end ());

      // Reserve enough space
      cloud_annotated_.header = cloud_header_;
      cloud_annotated_.pts.resize (cloud_in_.pts.size ());
      cloud_annotated_.chan[0].vals.resize (cloud_in_.pts.size ());

      pmap_.header = cloud_header_;
      pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map

      vector<vector<double> > coeff (clusters.size ()); // Need to save all coefficients for all models

      ROS_INFO(" - Process all clusters (%i)", clusters.size ());
      vector<int> inliers;
      vector<int> handle_indices;
      vector<double> goodness_factor (clusters.size());
      robot_msgs::Point32 minP, maxP, handle_center;

      // Transform door_min_z_ from the parameter parameter frame (parameter_frame_) into the point cloud frame
      door_min_z_ = transformDoubleValueTF (door_min_z_, parameter_frame_, cloud_frame_, cloud_time_, &tf_);

      // Transform min/max height values for the door handle (ADA requirements!)
      handle_min_height_ = transformDoubleValueTF (handle_min_height_, parameter_frame_, cloud_frame_, cloud_time_, &tf_);
      handle_max_height_ = transformDoubleValueTF (handle_max_height_, parameter_frame_, cloud_frame_, cloud_time_, &tf_);

#pragma omp parallel for schedule(dynamic)
      // Process all clusters
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // initial goodness factor
        goodness_factor[cc] = 1;

        // Find the best plane in this cluster
        fitSACPlane (cloud_down_, clusters[cc], inliers, coeff[cc], &viewpoint_cloud_, euclidean_cluster_min_pts_, sac_distance_threshold_);

        // Compute the convex hull
        cloud_geometry::areas::convexHull2D (&cloud_down_, &inliers, &coeff[cc], pmap_.polygons[cc]);

        // Filter the region based on its height and width
        cloud_geometry::statistics::getMinMax (&pmap_.polygons[cc], minP, maxP);

        // ---[ Quick test for minP.z (!)
        if (minP.z > door_min_z_)
        {
          goodness_factor[cc] = 0;
          ROS_WARN ("Door candidate rejected because min.Z (%g) is higher than the user specified threshold (%g)!", minP.z, door_min_z_);
          continue;
        }

        // ---[ Get the limits of the two "parallel to the Z-axis" lines defining the door
        double height_df1 = 0.0, height_df2 = 0.0;
        if (!checkDoorEdges (&pmap_.polygons[cc], &z_axis_, rectangle_constrain_edge_height_, rectangle_constrain_edge_angle_,
                             height_df1, height_df2))
        {
          goodness_factor[cc] = 0;
          ROS_WARN ("Door candidate rejected because the length of the door edges (%g / %g) is smaller than the user specified threshold (%g)!",
                    height_df1, height_df2, rectangle_constrain_edge_height_);
          continue;
        }

        // ---[ Compute the door width and height
        double door_frame = sqrt ( (maxP.x - minP.x) * (maxP.x - minP.x) + (maxP.y - minP.y) * (maxP.y - minP.y) );
        double door_height = fabs (maxP.z - minP.z);
        // Adapt the goodness factor for each cluster
        if (door_frame < door_min_width_ || door_height < door_min_height_ || door_frame > door_max_width_ || door_height > door_max_height_)
        {
          goodness_factor[cc] = 0;
          ROS_WARN ("Door candidate rejected because its width/height (%g / %g) is smaller than the user specified threshold (%g / %g -> %g / %g)!",
                    door_frame, door_height, door_min_width_, door_min_height_, door_max_width_, door_max_height_);
          continue;
        }
        double area = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);
        goodness_factor[cc] *= (area / (door_frame * door_height));


	// ---[ Compute the distance from the door to the prior of the door
	double door_distance = fmax(0.001, fmin(dist_xy(req.door.door_p1, minP), 
						fmin(dist_xy(req.door.door_p1, maxP), 
						     fmin(dist_xy(req.door.door_p2, minP), dist_xy(req.door.door_p2, maxP)))));
	goodness_factor[cc] /= door_distance;

      } // loop over clusters



      // Find the best cluster
      double best_goodness_factor = 0;
      int best_cluster = -1;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        if (goodness_factor[cc] > best_goodness_factor)
        {
          best_goodness_factor = goodness_factor[cc];
          best_cluster = cc;
        }
      }
      // Resize all the "bad" polygons
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        if (cc != best_cluster)
          pmap_.polygons[cc].points.resize (0);
      }

      // Check if any cluster respected all our constraints
      if (best_cluster == -1)
      {
        ROS_ERROR ("did not find a door");
        return false;
      }
      ROS_ERROR ("found a door");
      ROS_INFO ("Number of points selected: %d.", handle_indices.size ());
      if (publish_debug_)
	publish ("semantic_polygonal_map", pmap_);

      // Find the handle by performing a composite segmentation in distance and intensity space
      if (!findDoorHandleIntensity (&cloud_in_, &indices_in_bounds, &coeff[best_cluster], &pmap_.polygons[best_cluster], &viewpoint_cloud_,
                                    handle_indices, handle_center))
      {
        ROS_ERROR ("did not find a handle");
        return (false);
      }
      ROS_ERROR ("found a handle");


      // Get the minP and maxP of selected cluster
      cloud_geometry::statistics::getLargestXYPoints (&pmap_.polygons[best_cluster], minP, maxP);

      // reply door message in same frame as request door message
      resp.door = req.door;
      tf::Stamped<Point32> door_p1 (minP, cloud_time_, cloud_frame_);
      tf::Stamped<Point32> door_p2 (maxP, cloud_time_, cloud_frame_);
      tf::Stamped<Point32> handle (handle_center, cloud_time_, cloud_frame_);
      door_p2.z = door_p1.z;
      transformPoint (&tf_, door_frame_, door_p1, door_p1);
      transformPoint (&tf_, door_frame_, door_p2, door_p2);
      transformPoint (&tf_, door_frame_, handle, handle);
      resp.door.height = fabs (maxP.z - minP.z);
      resp.door.header.stamp = cloud_time_;
      resp.door.header.frame_id = door_frame_;
      resp.door.door_p1 = door_p1;
      resp.door.door_p2 = door_p2;
      resp.door.handle = handle;
      resp.door.height = fabs (maxP.z - minP.z);

      duration = ros::Time::now () - ts;
      ROS_INFO ("Door found. Result in frame %s \n  P1 = [%f, %f, %f]. P2 = [%f, %f, %f]. \n  Height = %f. \n  Handle = [%f, %f, %f]. \n  Total time: %f.",
                resp.door.header.frame_id.c_str (),
                resp.door.door_p1.x, resp.door.door_p1.y, resp.door.door_p1.z, resp.door.door_p2.x, resp.door.door_p2.y, resp.door.door_p2.z,
                resp.door.height, resp.door.handle.x, resp.door.handle.y, resp.door.handle.z,
                duration.toSec ());


      ROS_INFO("Finished detecting door");
      return (true);
    }




    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Given a cloud and set of indices potentially containing the handle, get the actual handle center and
      * indices
      * \param points a pointer to the point cloud message
      * \param indices a pointer to a set of point cloud indices to test
      * \param door_coeff the planar coefficients of the door
      * \param door_poly the polygonal convex hull describing the door
      * \param viewpoint the viewpoint of the laser when the cloud was acquired
      * \param handle_indices the resultant set of indices describing the door handle
      * \param handle_center the resultant handle center
      */
    bool
      findDoorHandleIntensity (PointCloud *points, vector<int> *indices, vector<double> *door_coeff, Polygon3D *door_poly,
                               PointStamped *viewpoint, vector<int> &handle_indices, Point32 &handle_center)
    {
      // Transform the polygon to lie on the X-Y plane (makes all isPointIn2DPolygon computations easier)
      Polygon3D poly_tr;
      Point32 p_tr;
      Eigen::Matrix4d transformation;
      vector<double> z_axis (3, 0); z_axis[2] = 1.0;
      cloud_geometry::transforms::getPlaneToPlaneTransformation (*door_coeff, z_axis, 0, 0, 0, transformation);
      cloud_geometry::transforms::transformPoints (&door_poly->points, poly_tr.points, transformation);

      // Install the basis for a viewpoint -> point line
      vector<double> viewpoint_pt_line (6);
      viewpoint_pt_line[0] = viewpoint->point.x;
      viewpoint_pt_line[1] = viewpoint->point.y;
      viewpoint_pt_line[2] = viewpoint->point.z;

      // min and max point of door
      Point32 min_d, max_d;
      cloud_geometry::statistics::getLargestXYPoints (door_poly, min_d, max_d);

      Point32 pt;
      vector<robot_msgs::Point32> handle_visualize;

      vector<int> possible_handle_indices (indices->size ());
      int nr_phi = 0;
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        // ---[ First test (geometric)
        // Remove all points that are too close to the edge of the door
        double d1 = sqrt ( pow(points->pts.at (indices->at (i)).x - min_d.x,2) + 
                           pow(points->pts.at (indices->at (i)).y - min_d.y,2));
        double d2 = sqrt ( pow(points->pts.at (indices->at (i)).x - max_d.x,2) + 
                           pow(points->pts.at (indices->at (i)).y - max_d.y,2));
        if (d1 < 0.02 || d2 < 0.02)
          continue;

        // ---[ Second test (geometric)
        // Calculate the distance from the point to the plane
        double distance_to_plane;
        cloud_geometry::projections::pointToPlane (&points->pts.at (indices->at (i)), pt, door_coeff, distance_to_plane);
        // Is the point close to the door?
        if (fabs (distance_to_plane) > handle_distance_door_max_threshold_)
          continue;

        // ---[ Third test (geometric)
        // Transform the point onto X-Y for faster checking inside the polygonal bounds
        cloud_geometry::transforms::transformPoint (&pt, p_tr, transformation);
        if (!cloud_geometry::areas::isPointIn2DPolygon (p_tr, &poly_tr))        // Is the point's projection inside the door ?
          continue;

        // ---[ Fourth test (geometric)
        // Check whether the line viewpoint->point intersects the polygon
        viewpoint_pt_line[3] = points->pts.at (indices->at (i)).x - viewpoint->point.x;
        viewpoint_pt_line[4] = points->pts.at (indices->at (i)).y - viewpoint->point.y;
        viewpoint_pt_line[5] = points->pts.at (indices->at (i)).z - viewpoint->point.z;
        // Normalize direction
        double n_norm = sqrt (viewpoint_pt_line[3] * viewpoint_pt_line[3] +
                              viewpoint_pt_line[4] * viewpoint_pt_line[4] +
                              viewpoint_pt_line[5] * viewpoint_pt_line[5]);
        viewpoint_pt_line[3] /= n_norm;
        viewpoint_pt_line[4] /= n_norm;
        viewpoint_pt_line[5] /= n_norm;

        // Check for the actual intersection
        Point32 viewpoint_door_intersection;
        if (!cloud_geometry::intersections::lineWithPlaneIntersection (door_coeff, &viewpoint_pt_line, viewpoint_door_intersection))
        {
          ROS_WARN ("Line and plane are parallel (no intersections found between the line and the plane).");
          continue;
        }

        // Transform the point onto X-Y for faster checking inside the polygonal bounds
        cloud_geometry::projections::pointToPlane (&viewpoint_door_intersection, pt, door_coeff);
        cloud_geometry::transforms::transformPoint (&pt, p_tr, transformation);
        if (!cloud_geometry::areas::isPointIn2DPolygon (p_tr, &poly_tr)
            && distance_to_plane < 0
            )        // Is the viewpoint<->point intersection inside the door ?
          continue;


        // project the point into the door plane
        //projectOntoPlane(door_coeff, points->pts.at (indices->at (i)));
        cloud_geometry::projections::pointToPlane (&points->pts.at (indices->at (i)), points->pts.at (indices->at (i)), door_coeff);

        // Save the point indices which satisfied all the geometric criteria so far
        possible_handle_indices[nr_phi] = indices->at (i);
        nr_phi++;
      } // end loop over points
      possible_handle_indices.resize (nr_phi);    // Resize to the actual value



      // Compute the mean and standard deviation in intensity space
      int d_idx = cloud_geometry::getChannelIndex (points, "intensities");
      vector<int> handle_indices_clusters;
      if (d_idx != -1)
      {
        // ---[ Fifth test (intensity)
        // Select points outside the (mean +/- \alpha_ * stddev) distribution
        selectBestDistributionStatistics (points, &possible_handle_indices, d_idx, handle_indices_clusters);
      }
      cout << "found " << handle_indices_clusters.size() << " candidate points for clustering" << endl;

      // Check if any clusters were found
      if (handle_indices_clusters.size () == 0)
      {
        ROS_WARN ("No handle indices found because: a) no intensities channel present in the data (%d); or b) wrong statistics computed!", d_idx);
        return false;
      }

      // ---[ Sixth test (intensity)
      // Split points into clusters
      vector<vector<int> > clusters;
      findClusters (points, &handle_indices_clusters, euclidean_cluster_distance_tolerance_, clusters,
                    -1, -1, -1, 0, intensity_cluster_min_pts_);
      sort (clusters.begin (), clusters.end (), compareRegions);
      reverse (clusters.begin (), clusters.end ());
      cout << "found " << clusters.size() << " clusters" << endl;

      // visualize clusters when door handle is not found
      handle_visualize.resize (clusters.size () + 2);
      handle_visualize[0] = min_d;
      handle_visualize[1] = max_d;
      for (unsigned int i = 0; i < clusters.size (); i++){
        //cloud_geometry::nearest::computeCentroid (points, &clusters[i], handle_visualize[i+2]);
        Point32 min_h, max_h;
        cloud_geometry::statistics::getLargestDiagonalPoints (points, &clusters[i], min_h, max_h);
        handle_visualize[i+2].x = (min_h.x + max_h.x) / 2.0;
        handle_visualize[i+2].y = (min_h.y + max_h.y) / 2.0;
        handle_visualize[i+2].z = (min_h.z + max_h.z) / 2.0;
      }
      robot_msgs::PointCloud handle_cloud;
      handle_cloud.header = cloud_header_;
      handle_cloud.pts = handle_visualize;

      // ---[ Seventh test (geometric)
      // check if the center of the cluster is close to the edges of the door
      std::cout << " - distance to side of door" << std::endl;
      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        if (clusters[i].size () == 0)
          continue;
        // Compute the center for the remaining handle indices
        Point32 min_h, max_h, center;
        cloud_geometry::statistics::getLargestDiagonalPoints (points, &clusters[i], min_h, max_h);
        center.x = (min_h.x + max_h.x) / 2.0;
        center.y = (min_h.y + max_h.y) / 2.0;
        center.z = (min_h.z + max_h.z) / 2.0;

        double d1 = sqrt ( (center.x - min_d.x) * (center.x - min_d.x) + (center.y - min_d.y) * (center.y - min_d.y) );
        double d2 = sqrt ( (center.x - max_d.x) * (center.x - max_d.x) + (center.y - max_d.y) * (center.y - max_d.y) );
        // @TODO: set these threasholds as params
        if ((d1 > 0.25 || d1 < 0.03) &&  (d2 > 0.25 || d2 < 0.03))
        {
          clusters[i].resize(0);
          cout << "  reject cluster " << i << " because min distance to edge of door = " << d1 << "  " << d2 << endl;
        }
        else if (center.z < handle_min_height_ || center.z > handle_max_height_)
        {
          clusters[i].resize(0);
          cout << "  reject cluster " << i << " because z height = " << center.z << endl;
        }
        else
          cout << "  accept cluster " << i << " because min distance to edge of door = " << d1 << "  " << d2 << endl;
      }



      // ---[ Eight test (geometric)
      // Fit the best horizontal line through each cluster
      // Check the elongation of the clusters -- Filter clusters based on min/max Z
      std::cout << " - lines fit" << std::endl;
      robot_msgs::Point32 door_axis = cloud_geometry::cross (door_coeff, &z_axis_);
      vector<vector<int> > line_inliers (clusters.size ());
#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)clusters.size (); i++)
      {
        if (clusters[i].size () == 0) continue;
        fitSACOrientedLine (points, clusters[i], 0.025, &door_axis, normal_angle_tolerance_, line_inliers[i]);
      }

      // find the best fitting cluster
      double best_score = -9999.0;
      int best_i = -1;
      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        if (line_inliers[i].size () == 0) continue;
        robot_msgs::Point32 min_h, max_h;
        cloud_geometry::statistics::getLargestXYPoints (points, &line_inliers[i], min_h, max_h);
        double length = sqrt ( (min_h.x - max_h.x) * (min_h.x - max_h.x) + (min_h.y - max_h.y) * (min_h.y - max_h.y) );
        double fit = ((double)(line_inliers[i].size())) / ((double)(clusters[i].size()));
        double score = fit - 3.0 * fabs(length - 0.15);
        cout << "  cluster " << i << " has fit " << fit << " and length " << length << " --> " << score << endl;
        if (score > best_score)
        {
          best_score = score;
          best_i = i;
        }
      }
      if (best_i == -1)
      {
        publish ("handle_visualization", handle_cloud);
        ROS_ERROR ("All clusters rejected! Should exit here.");
        return (false);
      }
      else
        ROS_INFO("Selecting cluster %i", best_i);
      handle_indices.resize (line_inliers[best_i].size ());
      for (unsigned int j = 0; j < line_inliers[best_i].size (); j++)
        handle_indices[j] = line_inliers[best_i][j];


      // compute min, max, and center of best cluster
      //cloud_geometry::nearest::computeCentroid (points, &line_inliers[best_i], handle_center);
      robot_msgs::Point32 min_h, max_h;
      cloud_geometry::statistics::getLargestDiagonalPoints (points, &line_inliers[best_i], min_h, max_h);
      handle_center.x = (min_h.x + max_h.x) / 2.0;
      handle_center.y = (min_h.y + max_h.y) / 2.0;
      handle_center.z = (min_h.z + max_h.z) / 2.0;
      cout << "min_h = " << min_h.x << " " << min_h.y << " "<< min_h.z << endl;
      cout << "max_h = " << max_h.x << " " << max_h.y << " "<< max_h.z << endl;
      cout << "handle_center = " << handle_center.x << " " << handle_center.y << " "<< handle_center.z << endl;

      // Calculate the unsigned distance from the point to the plane
      double distance_to_plane = door_coeff->at (0) * handle_center.x + door_coeff->at (1) * handle_center.y +
                                 door_coeff->at (2) * handle_center.z + door_coeff->at (3) * 1;
      cout << "distance to plane = " << distance_to_plane << endl;

      bool show_cluster = true;
      if (show_cluster)
      {
        handle_visualize.resize (line_inliers[best_i].size());
        for (unsigned int i=0; i<line_inliers[best_i].size(); i++)
        {
          handle_visualize[i].x = points->pts.at (line_inliers[best_i][i]).x;
          handle_visualize[i].y = points->pts.at (line_inliers[best_i][i]).y;
          handle_visualize[i].z = points->pts.at (line_inliers[best_i][i]).z;
        }
      }
      else
      {
        handle_visualize.resize (1);
        handle_visualize[0].x = handle_center.x;
        handle_visualize[0].y = handle_center.y;
        handle_visualize[0].z = handle_center.z;
      }
      handle_cloud.pts = handle_visualize;
      handle_cloud.header = cloud_header_;
      publish ("handle_visualization", handle_cloud);

      return (true);
    }




    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Main point cloud callback.                                                                           */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloud_cb (const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& cloud)
    {
      cout << "scan received" << endl;
      cloud_in_ = *cloud;
      num_clouds_received_++;
    }


  double dist_xy(const Point32& p1, const Point32& p2)
  {
    return sqrt( ((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y- p2.y) * (p1.y- p2.y)) );
  }


};



/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  DoorDetector p;

  p.spin ();

  p.shutdown ();
  return (0);
}
/* ]--- */

