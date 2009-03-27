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

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b handle_detector detects a handle on a given door.

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

// ROS core
#include <ros/node.h>
#include <roslib/Header.h>
#include <robot_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include <door_handle_detector/geometric_helper.h>

// Include the service call type
#include "door_handle_detector/DoorsDetector.h"

#include <tf/message_notifier.h>
#include <angles/angles.h>

using namespace std;
using namespace robot_msgs;

class HandleDetector
{
  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_orig_, cloud_tr_, cloud_regions_;
    Point32 z_axis_;
    PolygonalMap pmap_;
    tf::MessageNotifier<robot_msgs::PointCloud>*  message_notifier_;

    PointStamped viewpoint_cloud_;

    tf::TransformListener tf_;

    string input_cloud_topic_, parameter_frame_, door_frame_;
    unsigned int num_clouds_received_;

    int k_search_;

    // Parameters regarding geometric constraints for the door/handle
    double handle_distance_door_max_threshold_;

    // ADA requirements with respect to the handle
    double handle_max_height_, handle_min_height_;

    // Parameters for the euclidean clustering/cluster rejection
    double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
    int euclidean_cluster_min_pts_;

    double distance_from_door_margin_, min_plane_pts_, sac_distance_threshold_;

    int global_marker_id_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    HandleDetector (ros::Node& anode) : node_ (anode), message_notifier_ (NULL), tf_ (anode)
    {
      // ---[ Parameters regarding geometric constraints for the door/handle
      {
        node_.param ("~parameter_frame", parameter_frame_, string ("base_footprint"));

        node_.param ("~handle_distance_door_max_threshold", handle_distance_door_max_threshold_, 0.3); // maximum handle distance from the door plane

        node_.param ("~handle_min_height", handle_min_height_, 0.5);            // minimum height for a door handle: 0.5m
        node_.param ("~handle_max_height", handle_max_height_, 1.22);           // [ADA] maximum height for a door handle: 1.22m
        ROS_DEBUG ("Using the following thresholds for handle detection [min height / max height]: %f / %f.", handle_min_height_, handle_max_height_);
      }

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      k_search_ = 10;                // 10 k-neighbors by default

      distance_from_door_margin_ = 0.02 * 0.02;
      min_plane_pts_             = 1000;               // 1000 points
      sac_distance_threshold_    = 0.015;              // 1.5 cm by default

      // Parameters regarding the thresholds for Euclidean region growing/clustering
      euclidean_cluster_angle_tolerance_    = angles::from_degrees (15.0);
      euclidean_cluster_min_pts_            = 4;                  // 4 points
      euclidean_cluster_distance_tolerance_ = 0.04;               // 4 cm

      node_.param ("~input_cloud_topic", input_cloud_topic_, string ("/snapshot_cloud"));
      node_.advertiseService ("handle_detector", &HandleDetector::detectHandle, this);
      node_.advertise<robot_msgs::VisualizationMarker> ("visualizationMarker", 100);

      node_.advertise<PolygonalMap> ("~handle_polygon", 1);
      node_.advertise<PointCloud> ("~handle_regions", 1);
      node_.advertise<PointCloud> ("~door_outliers", 1);

      cloud_regions_.chan.resize (1); cloud_regions_.chan[0].name = "intensities";

      global_marker_id_ = 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief This is the main service callback: it gets called whenever a request to find a new handle is given   */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detectHandle (door_handle_detector::DoorsDetector::Request &req, door_handle_detector::DoorsDetector::Response &resp)
    {
      vector<int> tmp_indices;    // Used as a temporary indices array
      Point32 tmp_p;              // Used as a temporary point
      ROS_INFO ("Service request for handle identification received, with the following door parameters:\nP1 = [%g, %g, %g]. P2 = [%g, %g, %g]. Width = %g. Height = %g. Weight = %g.",
                req.door.door_p1.x, req.door.door_p1.y, req.door.door_p1.z, req.door.door_p2.x, req.door.door_p2.y, req.door.door_p2.z,
                sqrt ( (req.door.door_p1.x - req.door.door_p2.x) * (req.door.door_p1.x - req.door.door_p2.x) +
                       (req.door.door_p1.y - req.door.door_p2.y) * (req.door.door_p1.y - req.door.door_p2.y)
                     ), req.door.height, req.door.weight);

      ros::Time start_time = ros::Time::now ();
      ros::Duration delay = ros::Duration ().fromSec (25);
      cout << "start time " << start_time.toSec () << endl;
      ROS_INFO ("Waiting for PointCloud newer than %f on topic %s.", (start_time + delay).toSec (), node_.mapName (input_cloud_topic_).c_str ());

      // door frame
      door_frame_ = req.door.header.frame_id;

      // receive a new laser scan
      num_clouds_received_ = 0;
      message_notifier_ = new tf::MessageNotifier<robot_msgs::PointCloud> (&tf_, &node_,  boost::bind (&HandleDetector::cloud_cb, this, _1),
                                                                           input_cloud_topic_, door_frame_, 1);
      ros::Duration tictoc = ros::Duration ().fromSec (1.0);
      while ((int)num_clouds_received_ < 1)//|| (cloud_in_.header.stamp < (start_time + delay)))
        tictoc.sleep ();
      delete message_notifier_;

      ros::Time ts;
      ros::Duration duration;
      ts = ros::Time::now ();

      // New strategy: transform the PCD (Point Cloud Data) into the parameter_frame, and work there
      if (parameter_frame_ != cloud_orig_.header.frame_id)
      {
        try
        {
          tf_.transformPointCloud (parameter_frame_, cloud_orig_, cloud_tr_);
        }
        catch (tf::ConnectivityException e)
        {
          ROS_ERROR ("Could not transform point cloud from frame %s to frame %s.", cloud_orig_.header.frame_id.c_str (), parameter_frame_.c_str ());
        }
      }
      else
        cloud_tr_ = cloud_orig_;            // This can be replaced by modifying TF to copy (point to) the data

      // Get the cloud viewpoint in the parameter frame
      getCloudViewPoint (parameter_frame_, viewpoint_cloud_, &tf_);

      // Get the rough planar coefficients
      Point32 pt;
      pt.x = (req.door.door_p2.x + req.door.door_p1.x) / 2.0;
      pt.y = (req.door.door_p2.y + req.door.door_p1.y) / 2.0;
      pt.z = (req.door.door_p2.z + req.door.door_p1.z) / 2.0 + req.door.height / 2.0;
      vector<double> coeff (4);
      coeff[0] = req.door.normal.x;
      coeff[1] = req.door.normal.y;
      coeff[2] = req.door.normal.z;
      coeff[3] = - cloud_geometry::dot (req.door.normal, pt);

      // ---[ Optimization: select a subset of the points for faster processing
      // Select points close to the door plane (assumes door did not move since detection !)
      int nr_p = 0;
      vector<int> indices_in_bounds (cloud_tr_.pts.size ());
      double dist_p1p2 = cloud_geometry::distances::pointToPointXYDistanceSqr (req.door.door_p1, req.door.door_p2);
      for (int i = cloud_tr_.pts.size () - 1; i >= 0; --i)
      {
        if (cloud_tr_.pts[i].z > handle_min_height_ && cloud_tr_.pts[i].z < handle_max_height_ &&
            cloud_geometry::distances::pointToPlaneDistance (cloud_tr_.pts[i], coeff) < handle_distance_door_max_threshold_)
        {
          double dist_p1 = cloud_geometry::distances::pointToPointXYDistanceSqr (cloud_tr_.pts[i], req.door.door_p1);
          double dist_p2 = cloud_geometry::distances::pointToPointXYDistanceSqr (cloud_tr_.pts[i], req.door.door_p2);
          if (dist_p1 < dist_p1p2 && dist_p1 > distance_from_door_margin_ && dist_p2 < dist_p1p2 && dist_p2 > distance_from_door_margin_)
          {
            indices_in_bounds[nr_p] = i;
            nr_p++;
          }
        }
      }
      indices_in_bounds.resize (nr_p);
      sort (indices_in_bounds.begin (), indices_in_bounds.end ());

      vector<int> inliers, outliers;
      // Find the actual door plane. If the door moved since detection, return false/exit
      if (!fitSACPlane (cloud_tr_, indices_in_bounds, inliers, coeff, &viewpoint_cloud_, sac_distance_threshold_, min_plane_pts_) || inliers.size () == 0)
      {
        ROS_ERROR ("Could not find a door planar model in the input data (%d points)! Exiting...", nr_p);
        return (false);
      }
      // Compute the convex hull of the door
      robot_msgs::Polygon3D polygon, polygon_tr;
      cloud_geometry::areas::convexHull2D (cloud_tr_, inliers, coeff, polygon);

      // Create a polygonal representation on the X-Y plane (makes all isPointIn2DPolygon computations easier)
      Eigen::Matrix4d transformation;
      cloud_geometry::transforms::getPlaneToPlaneTransformation (coeff, z_axis_, 0, 0, 0, transformation);
      cloud_geometry::transforms::transformPoints (polygon.points, polygon_tr.points, transformation);

      // Get the outliers (we'll need them later)
      getDoorOutliers (indices_in_bounds, inliers, coeff, polygon, polygon_tr, transformation, outliers);

      // Get the handle candidates including the door itself
      vector<int> handle_indices;
      getHandleCandidates (indices_in_bounds, coeff, polygon, polygon_tr, transformation, handle_indices);

      // Create Kd-Tree and estimate the point normals in the original point cloud for the points left
      estimatePointNormals (cloud_tr_, handle_indices, k_search_, &viewpoint_cloud_);

      // Find the handle by performing a composite segmentation in distance and intensity space for all points left
      // Select points outside the (mean +/- \alpha_ * stddev) distribution
      int curvature_idx = cloud_geometry::getChannelIndex (cloud_tr_, "curvatures");
      int intensity_idx = cloud_geometry::getChannelIndex (cloud_tr_, "intensities");
      if (intensity_idx == -1 || curvature_idx == -1)
      {
        ROS_ERROR ("Intensity (and/or curvature) channels not present in the point cloud! Exiting...");
        return (false);
      }
      selectBestDualDistributionStatistics (cloud_tr_, handle_indices, curvature_idx, intensity_idx, tmp_indices);
      // Check if any points were returned
      if (tmp_indices.size () == 0)
      {
        ROS_WARN ("No handle indices found !");
        return (false);
      }
      handle_indices = tmp_indices;
      ROS_DEBUG ("Number of candidate points for clustering in the dual intensity-curvature space: %d.", (int)handle_indices.size ());

      // Refine the remaining handle indices using the door outliers
      ROS_DEBUG (" -- handle indices before refinement: %d.", (int)handle_indices.size ());
      robot_msgs::Point32 door_axis = cloud_geometry::cross (coeff, z_axis_);
      refineHandleCandidatesWithDoorOutliers (handle_indices, outliers, polygon, coeff, door_axis);
      ROS_DEBUG (" -- handle indices after refinement: %d.", (int)handle_indices.size ());

      duration = ros::Time::now () - ts;

      // Assemble the reply
      robot_msgs::Point32 min_h, max_h, handle_center;
      cloud_geometry::statistics::getLargestDiagonalPoints (cloud_tr_, handle_indices, min_h, max_h);
      handle_center.x = (min_h.x + max_h.x) / 2.0;
      handle_center.y = (min_h.y + max_h.y) / 2.0;
      handle_center.z = (min_h.z + max_h.z) / 2.0;
      //polygon.points.resize (2);
      //polygon.points[0] = min_h; polygon.points[1] = max_h;
      cout << "min_h = " << min_h.x << " " << min_h.y << " "<< min_h.z << endl;
      cout << "max_h = " << max_h.x << " " << max_h.y << " "<< max_h.z << endl;
      cout << "handle_center = " << handle_center.x << " " << handle_center.y << " "<< handle_center.z << endl;

      // Calculate the unsigned distance from the point to the plane
      double distance_to_plane = coeff[0] * handle_center.x + coeff[1] * handle_center.y + coeff[2] * handle_center.z + coeff[3] * 1;
      cout << "distance to plane = " << distance_to_plane << endl;




      // Output the point regions
      cloud_regions_.header = cloud_tr_.header;
      cloud_regions_.pts.resize (0);
      cloud_regions_.chan[0].vals.resize (0);

      bool show_cluster = true;
      if (show_cluster)
      {
        for (unsigned int i = 0; i < handle_indices.size (); i++)
        {
          cloud_regions_.pts.push_back ( cloud_tr_.pts[handle_indices[i]] );
          cloud_regions_.chan[0].vals.push_back ( cloud_tr_.chan[curvature_idx].vals[handle_indices[i]] );
        }
      }
      else
      {
        cloud_regions_.pts.push_back (handle_center);
        cloud_regions_.chan[0].vals.push_back (9999);
      }

      //cloud_regions_.pts.push_back (req.door.door_p1);
      //cloud_regions_.chan[0].vals.push_back (9999);
      //cloud_regions_.pts.push_back (req.door.door_p2);
      //cloud_regions_.chan[0].vals.push_back (9999);

      node_.publish ("~handle_regions", cloud_regions_);

      // Publish the outliers
      handle_indices = outliers;
      cloud_regions_.pts.resize (0);
      cloud_regions_.chan[0].vals.resize (0);
      for (unsigned int i = 0; i < handle_indices.size (); i++)
      {
        cloud_regions_.pts.push_back ( cloud_tr_.pts[handle_indices[i]] );
        cloud_regions_.chan[0].vals.push_back ( cloud_tr_.chan[curvature_idx].vals[handle_indices[i]] );
      }
      node_.publish ("~door_outliers", cloud_regions_);

      pmap_.header = cloud_tr_.header;
      pmap_.polygons.resize (1);         // Allocate space for the handle polygonal representation
      pmap_.polygons[0] = polygon;
      node_.publish ("~handle_polygon", pmap_);

      // Reply door message in same frame as request door message
      resp.doors.resize(1);
      resp.doors[0] = req.door;
      tf::Stamped<Point32> handle (handle_center, cloud_orig_.header.stamp, parameter_frame_);
      transformPoint (&tf_, cloud_orig_.header.frame_id, handle, handle);
      resp.doors[0].header.stamp = cloud_orig_.header.stamp;
      resp.doors[0].header.frame_id = cloud_orig_.header.frame_id;
      resp.doors[0].handle = handle;

      ROS_INFO ("Handle detected. Result in frame %s \n  Handle = [%f, %f, %f]. \n  Total time: %f.",
                resp.doors[0].header.frame_id.c_str (),
                resp.doors[0].handle.x, resp.doors[0].handle.y, resp.doors[0].handle.z,
                duration.toSec ());
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Refine the intensity/curvature handle indices with the door outliers
      *
      * \param handle_indices the handle indices
      * \param outliers the door outliers
      * \param polygon the polygonal bounds of the door
      * \param coeff the door planar coefficients
      * \param door_axis search for lines along this axis
      *
      * \note The following global parameters are used:
      *       cloud_tr_
      *       distance_from_door_margin_, euclidean_cluster_distance_tolerance_, euclidean_cluster_min_pts_
      */
    void
      refineHandleCandidatesWithDoorOutliers (vector<int> &handle_indices, vector<int> &outliers, const Polygon3D &polygon,
                                              const vector<double> &coeff, const Point32 &door_axis)
    {
      // Split the remaining candidates into into clusters and remove solitary points (< euclidean_cluster_min_pts_)
      vector<vector<int> > clusters;
      findClusters (cloud_tr_, handle_indices, euclidean_cluster_distance_tolerance_, clusters, -1, -1, -1, 0, euclidean_cluster_min_pts_);
      ROS_DEBUG ("Number of clusters for the handle candidate points: %d.", (int)clusters.size ());

      // Copy the clusters back to the indices
      handle_indices.resize (0);
      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        int old_size = handle_indices.size ();
        handle_indices.resize (old_size + clusters[i].size ());
        for (unsigned int j = 0; j < clusters[i].size (); j++)
          handle_indices.at (old_size + j) = clusters[i][j];
      }

      // Go over each cluster, fit vertical lines to get rid of the points near the door edges in an elegant manner,
      // then consider the rest as true handle candidate clusters
      vector<vector<int> > line_inliers (clusters.size ());
      vector<vector<int> > inliers (clusters.size ());
      for (int i = 0; i < (int)clusters.size (); i++)
      {
        // One method to prune point clusters would be to fit vertical lines (Z parallel) and remove their inliers
#if 0
        //fitSACOrientedLine (cloud_tr_, clusters[i], sac_distance_threshold_, &z_axis_, euclidean_cluster_angle_tolerance_, line_inliers[i]);
        //set_difference (clusters[i].begin (), clusters[i].end (), line_inliers[i].begin (), line_inliers[i].end (),
        //                inserter (remaining_clusters[i], remaining_clusters[i].begin ()));
#endif
        // Grow the current cluster using the door outliers
        growCurrentCluster (cloud_tr_, outliers, clusters[i], inliers[i], 2 * euclidean_cluster_distance_tolerance_);

        PointCloud tmp_cloud;
        cloud_geometry::projections::pointsToPlane  (cloud_tr_, inliers[i], tmp_cloud, coeff);
        // Fit the best horizontal line through each cluster
        fitSACOrientedLine (tmp_cloud, sac_distance_threshold_ * 2, door_axis, euclidean_cluster_angle_tolerance_, line_inliers[i]);
        for (unsigned int j = 0; j < line_inliers[i].size (); j++)
          line_inliers[i][j] = inliers[i][line_inliers[i][j]];
      }

      double best_score = -FLT_MAX;
      int best_i = -1;
      // Check the elongation of the clusters
      for (unsigned int i = 0; i < line_inliers.size (); i++)
      {
        if (line_inliers[i].size () == 0)
          continue;

        robot_msgs::Point32 min_h, max_h;
        cloud_geometry::statistics::getLargestXYPoints (cloud_tr_, line_inliers[i], min_h, max_h);

        double length = sqrt ( (min_h.x - max_h.x) * (min_h.x - max_h.x) + (min_h.y - max_h.y) * (min_h.y - max_h.y) );
        double fit = ((double)(line_inliers[i].size())) / ((double)(inliers[i].size()));
        double score = fit - 3.0 * fabs (length - 0.15);

        ROS_INFO ("  Handle line cluster %d with %d inliers, has fit %g and length %g  --> %g.", i, (int)line_inliers[i].size (), fit, length, score);

        if (score > best_score)
        {
          best_score = score;
          best_i = i;
        }
      }

      if (best_i == -1)
      {
        ROS_ERROR ("All clusters rejected!");
        // Copy the extra inliers to handle_indices
        for (unsigned int i = 0; i < inliers.size (); i++)
        {
          if (inliers[i].size () == 0)
            continue;
          int old_size = handle_indices.size ();
          handle_indices.resize (old_size + inliers[i].size ());
          for (unsigned int j = 0; j < inliers[i].size (); j++)
            handle_indices.at (old_size + j) = inliers[i][j];
        }
      }
      else
      {
        ROS_INFO ("Selecting cluster %d.", best_i);

        // Copy the intensity/curvature cluster
        handle_indices.resize (clusters[best_i].size ());
        for (unsigned int j = 0; j < clusters[best_i].size (); j++)
          handle_indices[j] = clusters[best_i][j];

        // Copy the inliers cluster
        int old_size = handle_indices.size ();
        handle_indices.resize (old_size + line_inliers[best_i].size ());
        for (unsigned int j = 0; j < line_inliers[best_i].size (); j++)
          handle_indices[old_size + j] = line_inliers[best_i][j];
      }
      sort (handle_indices.begin (), handle_indices.end ());
      handle_indices.erase (unique (handle_indices.begin (), handle_indices.end ()), handle_indices.end ());

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Select all points that could represent a handle, including the door
      *
      * \param indices a pointer to all the point indices
      * \param coeff the door planar coefficients
      * \param polygon the polygonal bounds of the door
      * \param polygon_tr the polygonal bounds of the door in the X-Y plane
      * \param transformation the transformation between the door planar coefficients and the X-Y plane
      *
      * \param handle_indices the resultant handle indices
      *
      * \note In principle, this method could be fused with getDoorOutliers, but we want to keep them separate for now
      * for debugging purposes
      * \note The following global parameters are used:
      *       cloud_tr_, viewpoint_cloud_
      */
    void
      getHandleCandidates (const vector<int> &indices, const vector<double> &coeff,
                           const Polygon3D &polygon, const Polygon3D &polygon_tr, Eigen::Matrix4d transformation,
                           vector<int> &handle_indices)
    {
      // Check the points in bounds for extra geometric constraints
      handle_indices.resize (indices.size ());

      // Install the basis for a viewpoint -> point line
      vector<double> viewpoint_pt_line (6);
      viewpoint_pt_line[0] = viewpoint_cloud_.point.x;
      viewpoint_pt_line[1] = viewpoint_cloud_.point.y;
      viewpoint_pt_line[2] = viewpoint_cloud_.point.z;

      // Remove outliers around the door margin
      Point32 tmp_p;              // Used as a temporary point
      int nr_p = 0;
      Point32 pt;
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        // Transform the point onto X-Y for faster checking inside the polygonal bounds
        double distance_to_plane;
        cloud_geometry::projections::pointToPlane (cloud_tr_.pts.at (indices.at (i)), pt, coeff, distance_to_plane);
        if (distance_to_plane < 0)
          continue;
        cloud_geometry::transforms::transformPoint (pt, tmp_p, transformation);
        if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))        // Is the point's projection inside the door ?
          continue;

        // Close to the edges (3D)
        if (cloud_geometry::distances::pointToPolygonDistanceSqr (tmp_p, polygon_tr) < distance_from_door_margin_)
          continue;

        // Check whether the line viewpoint->point intersects the polygon
        viewpoint_pt_line[3] = cloud_tr_.pts.at (indices.at (i)).x - viewpoint_cloud_.point.x;
        viewpoint_pt_line[4] = cloud_tr_.pts.at (indices.at (i)).y - viewpoint_cloud_.point.y;
        viewpoint_pt_line[5] = cloud_tr_.pts.at (indices.at (i)).z - viewpoint_cloud_.point.z;
        // Normalize direction
        double n_norm = sqrt (viewpoint_pt_line[3] * viewpoint_pt_line[3] +
                              viewpoint_pt_line[4] * viewpoint_pt_line[4] +
                              viewpoint_pt_line[5] * viewpoint_pt_line[5]);
        viewpoint_pt_line[3] /= n_norm;
        viewpoint_pt_line[4] /= n_norm;
        viewpoint_pt_line[5] /= n_norm;

        // Check for the actual intersection
        Point32 viewpoint_door_intersection;
        if (!cloud_geometry::intersections::lineWithPlaneIntersection (coeff, viewpoint_pt_line, viewpoint_door_intersection))
        {
          ROS_WARN ("Line and plane are parallel (no intersections found between the line and the plane).");
          continue;
        }
        // Transform the point onto X-Y for faster checking inside the polygonal bounds
        cloud_geometry::projections::pointToPlane (viewpoint_door_intersection, pt, coeff);
        cloud_geometry::transforms::transformPoint (pt, tmp_p, transformation);
        if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))    // Is the viewpoint<->point intersection inside the door ?
          continue;

        // Save the point indices which satisfied all the geometric criteria so far
        handle_indices[nr_p++] = indices.at (i);
      } // end loop over points
      handle_indices.resize (nr_p);    // Resize to the actual value
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Select the door outliers that could represent a handle
      *
      * \param indices a pointer to all the point indices
      * \param inliers a pointer to the point indices which are inliers for the door plane
      * \param coeff the door planar coefficients
      * \param polygon the polygonal bounds of the door
      * \param polygon_tr the polygonal bounds of the door in the X-Y plane
      * \param transformation the transformation between the door planar coefficients and the X-Y plane
      *
      * \param outliers the resultant outliers
      *
      * \note The following global parameters are used:
      *       cloud_tr_, viewpoint_cloud_
      *       distance_from_door_margin_, euclidean_cluster_distance_tolerance_, euclidean_cluster_min_pts_
      */
    void
      getDoorOutliers (const vector<int> &indices, const vector<int> &inliers,
                       const vector<double> &coeff, const Polygon3D &polygon, const Polygon3D &polygon_tr, Eigen::Matrix4d transformation,
                       vector<int> &outliers)
    {
      vector<int> tmp_indices;    // Used as a temporary indices array
      Point32 tmp_p;              // Used as a temporary point
      set_difference (indices.begin (), indices.end (), inliers.begin (), inliers.end (),
                      inserter (outliers, outliers.begin ()));

#if 0
      // Install the basis for a viewpoint -> point line
      vector<double> viewpoint_pt_line (6);
      viewpoint_pt_line[0] = viewpoint_cloud_.point.x;
      viewpoint_pt_line[1] = viewpoint_cloud_.point.y;
      viewpoint_pt_line[2] = viewpoint_cloud_.point.z;
#endif
      Point32 pt;
      tmp_indices.resize (outliers.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < outliers.size (); i++)
      {
        // Compute a projection on the plane
        double distance_to_plane;
        cloud_geometry::projections::pointToPlane (cloud_tr_.pts.at (outliers[i]), pt, coeff, distance_to_plane);
        if (distance_to_plane < 0)
          continue;
        cloud_geometry::transforms::transformPoint (pt, tmp_p, transformation);
        if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))        // Is the point's projection inside the door ?
          continue;

        // Remove outliers around the door margin (close to the edges)
        if (cloud_geometry::distances::pointToPolygonDistanceSqr (tmp_p, polygon_tr) < distance_from_door_margin_)
          continue;

#if 0
//        // Check whether the line viewpoint->point intersects the polygon
//        viewpoint_pt_line[3] = cloud_tr_.pts.at (outliers[i]).x - viewpoint_pt_line[0];
//        viewpoint_pt_line[4] = cloud_tr_.pts.at (outliers[i]).y - viewpoint_pt_line[1];
//        viewpoint_pt_line[5] = cloud_tr_.pts.at (outliers[i]).z - viewpoint_pt_line[2];
//        // Normalize direction
//        double n_norm = sqrt (viewpoint_pt_line[3] * viewpoint_pt_line[3] +
//                              viewpoint_pt_line[4] * viewpoint_pt_line[4] +
//                              viewpoint_pt_line[5] * viewpoint_pt_line[5]);
//        viewpoint_pt_line[3] /= n_norm;
//        viewpoint_pt_line[4] /= n_norm;
//        viewpoint_pt_line[5] /= n_norm;
//
//        // Compute the angle between the normal of the plane and the ray we cast
//        double angle = acos (viewpoint_pt_line[3] * coeff.at (0) + viewpoint_pt_line[4] * coeff.at (1) + viewpoint_pt_line[5] * coeff.at (2));
//        if (fabs (M_PI / 2.0 - angle) < euclidean_cluster_angle_tolerance_)
//          continue;                                                                   // Don't check if ~ perpendicular
//
//        // Check for the actual intersection
//        Point32 viewpoint_door_intersection;
//        if (!cloud_geometry::intersections::lineWithPlaneIntersection (coeff, &viewpoint_pt_line, viewpoint_door_intersection))
//        {
//          ROS_WARN ("Line and plane are parallel (no intersections found between the line and the plane).");
//          continue;
//        }
//
//        // Transform the point onto X-Y for faster checking inside the polygonal bounds
//        cloud_geometry::projections::pointToPlane (&viewpoint_door_intersection, pt, coeff);
//        cloud_geometry::transforms::transformPoint (&pt, tmp_p, transformation);
//        if (!cloud_geometry::areas::isPointIn2DPolygon (tmp_p, polygon_tr))    // Is the viewpoint<->point intersection inside the door ?
//          continue;
#endif

        tmp_indices[nr_p] = outliers[i];
        nr_p++;
      }
      tmp_indices.resize (nr_p);
      outliers = tmp_indices;

      // Split the remaining candidates into into clusters and remove the small clusters
      vector<vector<int> > clusters;
      if (outliers.size () > 0)
      {
        findClusters (cloud_tr_, outliers, euclidean_cluster_distance_tolerance_, clusters, -1, -1, -1, 0, euclidean_cluster_min_pts_);
        outliers.resize (0);
        for (unsigned int i = 0; i < clusters.size (); i++)
        {
          int old_size = outliers.size ();
          outliers.resize (old_size + clusters[i].size ());
          for (unsigned int j = 0; j < clusters[i].size (); j++)
            outliers[old_size + j] = clusters[i][j];
        }
      }
      else
        ROS_DEBUG ("[getDoorOutliers] No door plane outliers found.");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Main point cloud callback.                                                                           */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      cloud_cb (const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& cloud)
    {
      cloud_orig_ = *cloud;
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_orig_.pts.size (), cloud_orig_.header.frame_id.c_str (),
                (int)cloud_orig_.chan.size (), cloud_geometry::getAvailableChannels (cloud_orig_).c_str ());
      num_clouds_received_++;
    }
};



/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("handle_detector_node");

  HandleDetector p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

