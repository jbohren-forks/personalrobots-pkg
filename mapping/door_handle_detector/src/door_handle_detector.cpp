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
 * $Id: door_handle_detector_omp.cpp 9895 2009-01-21 21:05:27Z tfoote $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b door_handle_detector detects doors and handles.

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

// ROS core
#include <ros/node.h>
#include <std_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include "geometric_helper.h"

// Include the service call type
#include "door_handle_detector/Door.h"

using namespace std;
using namespace std_msgs;

class DoorHandleDetector : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_in_, cloud_down_;
    PointCloud cloud_annotated_;
    Point32 z_axis_;
    PolygonalMap pmap_;

    PointStamped viewpoint_cloud_;

    tf::TransformListener tf_;

    string input_cloud_topic_;
    bool need_cloud_data_, publish_debug_;


    // Parameters regarding geometric constraints for the door/handle
    double door_min_height_, door_min_width_, door_max_height_, door_max_width_, door_min_z_;
    double handle_distance_door_min_threshold_, handle_distance_door_max_threshold_, handle_max_height_, handle_min_height_;
    double handle_height_threshold_;

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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DoorHandleDetector () : ros::Node ("door_handle_detector"), tf_(*this)
    {
      // ---[ Parameters regarding geometric constraints for the door/handle
      {
        param ("~door_min_z", door_min_z_, 0.4);                            // the minimum Z point on the door must be lower than this value
        param ("~door_min_height", door_min_height_, 1.4);                  // minimum height of a door: 1.4m
        param ("~door_max_height", door_max_height_, 3.0);                  // maximum height of a door: 3m
        param ("~door_min_width", door_min_width_, 0.8);                    // minimum width of a door: 0.8m
        param ("~door_max_width", door_max_width_, 1.4);                    // maximum width of a door: 1.4m
        ROS_DEBUG ("Using the following thresholds for door detection [min-max height / min-max width]: %f-%f / %f-%f.",
                   door_min_height_, door_max_height_, door_min_width_, door_max_width_);

        param ("~handle_max_height", handle_max_height_, 1.41);            // maximum height for a door handle: 1.41m
        param ("~handle_min_height", handle_min_height_, 0.41);            // minimum height for a door handle: 0.41m
        param ("~handle_distance_door_max_threshold", handle_distance_door_max_threshold_, 0.15); // maximum distance between the handle and the door
        param ("~handle_distance_door_min_threshold", handle_distance_door_min_threshold_, 0.02); // minimum distance between the handle and the door
        param ("~handle_height_threshold", handle_height_threshold_, 0.1); // Additional threshold for filtering large Z clusters (potentially part of the handle)

        ROS_DEBUG ("Using the following thresholds for handle detection [min height / max height]: %f / %f.", handle_min_height_, handle_max_height_);

        // This parameter constrains the door polygon to resamble a rectangle,
        // that is: the two lines parallel (with some angular threshold) to the Z-axis must have some minimal length
        param ("~rectangle_constrain_edge_height", rectangle_constrain_edge_height_, 0.5);        // each side of the door has to be at least 50 cm
        param ("~rectangle_constrain_edge_angle", rectangle_constrain_edge_angle_, 20.0);         // maximum angle threshold between a side and the Z-axis: 20 degrees
        rectangle_constrain_edge_angle_ = cloud_geometry::deg2rad (rectangle_constrain_edge_angle_);
      }

      // ---[ Parameters regarding optimizations / real-time computations
      {
        // Parameters regarding the _fast_ normals/plane computation using a lower quality (downsampled) dataset
        param ("~downsample_leaf_width", leaf_width_, 0.025);          // 2.5cm radius by default
        param ("~search_k_closest", k_search_, 10);                    // 10 k-neighbors by default
        // This should be set to whatever the leaf_width factor is in the downsampler
        param ("~sac_distance_threshold", sac_distance_threshold_, 0.03); // 3 cm

        // Parameters regarding the maximum allowed angular difference in normal space for inlier considerations
        z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
        param ("~normal_angle_tolerance", normal_angle_tolerance_, 15.0);              // 15 degrees, wrt the Z-axis
        normal_angle_tolerance_ = cloud_geometry::deg2rad (normal_angle_tolerance_);

        // Parameters regarding the thresholds for Euclidean region growing/clustering
        param ("~euclidean_cluster_min_pts", euclidean_cluster_min_pts_, 200);                         // 200 points
        // Difference between normals in degrees for cluster/region growing
        param ("~euclidean_cluster_angle_tolerance", euclidean_cluster_angle_tolerance_, 30.0);
        euclidean_cluster_angle_tolerance_ = cloud_geometry::deg2rad (euclidean_cluster_angle_tolerance_);
        param ("~euclidean_cluster_distance_tolerance", euclidean_cluster_distance_tolerance_, 0.04);  // 4 cm

        // Parameters regarding the thresholds for intensity region growing/clustering
        param ("~intensity_cluster_min_pts", intensity_cluster_min_pts_, 2);                                                 // 2 points
        param ("~intensity_cluster_perpendicular_angle_tolerance", intensity_cluster_perpendicular_angle_tolerance_, 5.0);   // 5 degrees
        intensity_cluster_perpendicular_angle_tolerance_ = cloud_geometry::deg2rad (intensity_cluster_perpendicular_angle_tolerance_);

        // This describes the size of our 3D bounding box (basically the space where we search for doors),
        // as a multiplier of the door frame (computed using the two points from the service call) in both X and Y directions
        param ("~door_frame_multiplier", door_frame_multiplier_, 4);
        param ("~door_min_z_bounds", door_min_z_bounds_, 0.0);               // restrict the search Z dimension between 0...
        param ("~door_max_z_bounds", door_max_z_bounds_, 3.0);               // ...and 3.0 m
      }


      // Temporary parameters
      param ("~publish_debug", publish_debug_, true);

      param ("~input_cloud_topic", input_cloud_topic_, string ("full_cloud"));
      advertiseService("door_handle_detector", &DoorHandleDetector::detectDoor, this);

      advertise<PointCloud>("handle_visualization", 10);
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
    virtual ~DoorHandleDetector () { }

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
      detectDoor (door_handle_detector::Door::Request &req, door_handle_detector::Door::Response &resp)
    {
      ros::Time ts;
      ros::Duration duration;
      cout << "Start detecting door at points ";
      cout << "(" << req.door.frame_p1.x << " " <<req.door.frame_p1.y << " "<<req.door.frame_p1.z << ")   ";
      cout << "(" << req.door.frame_p2.x << " " <<req.door.frame_p2.y << " "<<req.door.frame_p2.z << ")" << endl;

      updateParametersFromServer ();

      // Obtain the bounding box information
      Point32 minB, maxB;
      get3DBounds (&req.door.frame_p1, &req.door.frame_p2, minB, maxB, door_min_z_bounds_, door_max_z_bounds_, door_frame_multiplier_);

      // Subscribe to a point cloud topic
      need_cloud_data_ = true;
      subscribe (input_cloud_topic_.c_str (), cloud_in_, &DoorHandleDetector::cloud_cb, 1);

      // Wait until the scan is ready, sleep for 10ms
      ros::Duration tictoc (0, 10000000);
      while (need_cloud_data_)
      {
        tictoc.sleep ();
      }
      // Unsubscribe from the point cloud topic
      unsubscribe (input_cloud_topic_.c_str ()) ;

      cout << " - door scan received" << endl;

      ts = ros::Time::now ();
      // We have a pointcloud, estimate the true point bounds
      vector<int> indices_in_bounds (cloud_in_.pts.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < cloud_in_.pts.size (); i++)
      {
        if ((cloud_in_.pts[i].x >= minB.x && cloud_in_.pts[i].x <= maxB.x) &&
            (cloud_in_.pts[i].y >= minB.y && cloud_in_.pts[i].y <= maxB.y) &&
            (cloud_in_.pts[i].z >= minB.z && cloud_in_.pts[i].z <= maxB.z))
        {
          indices_in_bounds[nr_p] = i;
          nr_p++;
        }
      }
      indices_in_bounds.resize (nr_p);

      //ROS_DEBUG ("Number of points in bounds [%f,%f,%f] -> [%f,%f,%f]: %d.", minB.x, minB.y, minB.z, maxB.x, maxB.y, maxB.z, indices_in_bounds.size ());

      // Downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        Point leaf_width_xyz;
        leaf_width_xyz.x = leaf_width_xyz.y = leaf_width_xyz.z = leaf_width_;
        cloud_geometry::downsamplePointCloud (&cloud_in_, &indices_in_bounds, cloud_down_, leaf_width_xyz, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }
      leaves.resize (0);    // dealloc memory used for the downsampling process

      //ROS_DEBUG ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, cloud_down_.pts.size ());

      // Get the cloud viewpoint
      getCloudViewPoint (cloud_in_.header.frame_id, viewpoint_cloud_, &tf_);

      // Create Kd-Tree and estimate the point normals
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
      cloud_annotated_.header = cloud_down_.header;
      cloud_annotated_.pts.resize (cloud_in_.pts.size ());
      cloud_annotated_.chan[0].vals.resize (cloud_in_.pts.size ());

      pmap_.header = cloud_down_.header;
      pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map

      vector<vector<double> > coeff (clusters.size ()); // Need to save all coefficients for all models

      cout << " - Process all clusters (" << clusters.size () << ")" << endl;
      nr_p = 0;
      vector<int> inliers;
      vector<int> handle_indices;
      vector<double> goodness_factor (clusters.size());
      std_msgs::Point32 minP, maxP, handle_center;

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
          //ROS_WARN ("Door candidate rejected because min.Z (%g) is higher than the user specified threshold (%g)!", minP.z, door_min_z_);
          continue;
        }

        // ---[ Get the limits of the two "parallel to the Z-axis" lines defining the door
        double height_df1 = 0.0, height_df2 = 0.0;
        rectangle_constrain_edge_height_ = 0.2;
        if (!checkDoorEdges (&pmap_.polygons[cc], &z_axis_, rectangle_constrain_edge_height_, rectangle_constrain_edge_angle_,
                             height_df1, height_df2))
        {
          goodness_factor[cc] = 0;
          //ROS_WARN ("Door candidate rejected because the length of the door edges (%g / %g) is smaller than the user specified threshold (%g)!",
          //          height_df1, height_df2, rectangle_constrain_edge_height_);
          continue;
        }

        // Compute the door width and height
        double door_frame = sqrt ( (maxP.x - minP.x) * (maxP.x - minP.x) + (maxP.y - minP.y) * (maxP.y - minP.y) );
          //sqrt (pow ((minP.x - maxP.x), 2) + pow ((minP.y - maxP.y), 2));
        double door_height = fabs (maxP.z - minP.z);
        // Adapt the goodness factor for each cluster
        if (door_frame < door_min_width_ || door_height < door_min_height_ || door_frame > door_max_width_ || door_height > door_max_height_)
        {
          goodness_factor[cc] = 0;
          //ROS_WARN ("Door candidate rejected because its width/height (%g / %g) is smaller than the user specified threshold (%g / %g -> %g / %g)!",
          //          door_frame, door_height, door_min_width_, door_min_height_, door_max_width_, door_max_height_);
          continue;
        }
        double area = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);
        goodness_factor[cc] *= (area / (door_frame * door_height));
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
        // @Wim: decide what you want to do here
        //return (false);
      }
      else
      {
        // Find the handle by performing a composite segmentation in distance and intensity space
        findDoorHandleIntensity (&cloud_in_, &indices_in_bounds, &coeff[best_cluster], &pmap_.polygons[best_cluster], &viewpoint_cloud_,
                                 handle_indices, handle_center);
        ROS_INFO ("Number of points selected: %d.", handle_indices.size ());
      }

      // This needs to be removed
#if 1
      if (publish_debug_)
      {
        double r, g, b, rgb;
        r = g = b = 1.0;
        int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
        rgb = *(float*)(&res);

        // Mark all the points inside
        for (unsigned int k = 0; k < handle_indices.size (); k++)
        {
          cloud_annotated_.pts[nr_p].x = cloud_in_.pts.at (handle_indices[k]).x;
          cloud_annotated_.pts[nr_p].y = cloud_in_.pts.at (handle_indices[k]).y;
          cloud_annotated_.pts[nr_p].z = cloud_in_.pts.at (handle_indices[k]).z;
          cloud_annotated_.chan[0].vals[nr_p] = rgb;
          nr_p++;
        }
      }
#endif

      // Prepare the reply
      resp.door.door_p1.x = minP.x; resp.door.door_p1.y = minP.y;
      resp.door.door_p2.x = maxP.x; resp.door.door_p2.y = maxP.y;
      resp.door.height = fabs (maxP.z - minP.z);
      resp.door.handle = handle_center;

      duration = ros::Time::now () - ts;
      ROS_INFO ("Door found. P1 = [%f, %f, %f]. P2 = [%f, %f, %f]. Height = %f. Handle = [%f, %f, %f]. Total time: %f.",
                resp.door.door_p1.x, resp.door.door_p1.y, resp.door.door_p1.z, resp.door.door_p2.x, resp.door.door_p2.y, resp.door.door_p2.z,
                resp.door.height, resp.door.handle.x, resp.door.handle.y, resp.door.handle.z,
                duration.toSec ());

// This needs to be removed
#if 1
      if (publish_debug_)
      {
        cloud_annotated_.pts.resize (nr_p);
        cloud_annotated_.chan[0].vals.resize (nr_p);

        publish ("cloud_annotated", cloud_down_);//cloud_annotated_);
        publish ("semantic_polygonal_map", pmap_);
      }
#endif

      cout << "Finished detecting door" << endl;
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
    void
      findDoorHandleIntensity (PointCloud *points, vector<int> *indices, vector<double> *door_coeff, Polygon3D *door_poly,
                               PointStamped *viewpoint, vector<int> &handle_indices, Point32 &handle_center)
    {
      // Transform the polygon to lie on the X-Y plane (makes all isPointIn2DPolygon computations easier)
      Polygon3D poly_tr, poly_tr_shrunk;
      Point32 p_tr;
      Eigen::Matrix4d transformation;
      vector<double> z_axis (3, 0); z_axis[2] = 1.0;
      cloud_geometry::transforms::getPlaneToPlaneTransformation (*door_coeff, z_axis, 0, 0, 0, transformation);
      cloud_geometry::transforms::transformPoints (&door_poly->points, poly_tr.points, transformation);
      //cloud_geometry::areas::shrink2DPolygon (&poly_tr, poly_tr_shrunk, 0.2);
      //cloud_geometry::areas::expand2DPolygon (&poly_tr, poly_tr_shrunk, -0.2);

      // Install the basis for a viewpoint -> point line
      vector<double> viewpoint_pt_line (6);
      viewpoint_pt_line[0] = viewpoint->point.x;
      viewpoint_pt_line[1] = viewpoint->point.y;
      viewpoint_pt_line[2] = viewpoint->point.z;

      PolygonalMap pmap;
      pmap.polygons.push_back (poly_tr);
      //pmap.polygons.push_back (poly_tr_shrunk);

      Point32 pt;
      vector<std_msgs::Point32> handle_visualize;

      vector<int> possible_handle_indices (indices->size ());
      int nr_phi = 0;
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        // ---[ First test (geometric)
        // Select all the points in the given bounds which are between the given handle min->max height
        if (points->pts.at (indices->at (i)).z < handle_min_height_ || points->pts.at (indices->at (i)).z > handle_max_height_)
          continue;

        // ---[ Second test (geometric)
        // Calculate the distance from the point to the plane
        double distance_to_plane;
        cloud_geometry::projections::pointToPlane (&points->pts.at (indices->at (i)), pt, door_coeff, distance_to_plane);
        // Is the point close to the door and on the same side as the plane normal ?
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


// This needs to be removed
#if 1
        std_msgs::Point32 pnt;
        pnt.x = points->pts.at (indices->at (i)).x;
        pnt.y = points->pts.at (indices->at (i)).y;
        pnt.z = points->pts.at (indices->at (i)).z;
        //handle_visualize.push_back (viewpoint_door_intersection);//p_tr);//pnt);
        handle_visualize.push_back (pnt);
#endif

        possible_handle_indices[nr_phi] = indices->at (i);
        nr_phi++;
      }
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

      // Check if any clusters were found
      if (handle_indices_clusters.size () == 0)
      {
        ROS_WARN ("No handle indices found because: a) no intensities channel present in the data (%d); or b) wrong statistics computed!", d_idx);
        return;
      }

      // ---[ Sixth test (intensity)
      // Split points into clusters
      vector<vector<int> > clusters;
      findClusters (points, &handle_indices_clusters, euclidean_cluster_distance_tolerance_, clusters,
                    -1, -1, -1, 0, intensity_cluster_min_pts_);
      sort (clusters.begin (), clusters.end (), compareRegions);
      reverse (clusters.begin (), clusters.end ());

      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        ROS_INFO ("Intensity cluster %d has %d points.", i, clusters[i].size ());

        // Need to remove the planar cluster on which the viewpoint <-> cluster_centroid is perpendicular to (intensity artifact)
        if (checkIfClusterPerpendicular (points, &clusters[i], viewpoint, door_coeff, intensity_cluster_perpendicular_angle_tolerance_))
        {
          ROS_WARN ("Cluster %d removed due to perpendicularity condition (viewpoint<->plane) criterion.", i);
          clusters[i].resize (0);
        }
      }

      // ---[ Seventh test (geometric)
      // Check the elongation of the clusters -- Filter clusters based on min/max Z
      std_msgs::Point32 minP, maxP;
      handle_indices.resize (handle_indices_clusters.size ());
      nr_phi = 0;
      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        if (clusters[i].size () == 0) continue;
        cloud_geometry::statistics::getMinMax (points, &clusters[i], minP, maxP);

        double dist_x = fabs (maxP.x - minP.x);
        double dist_y = fabs (maxP.y - minP.y);
        double dist_z = fabs (maxP.z - minP.z);

        if (dist_z > 2 * dist_x && dist_z > 2 * dist_y)
        {
          ROS_WARN ("Rejecting potential handle cluster (%d) because elongation on z (%g) is larger than elongation on x (%g) and y (%g)!",
                    clusters[i].size (), dist_z, dist_x, dist_y);
          continue;
        }

        if (fabs (maxP.z - minP.z) > handle_height_threshold_)
        {
          //ROS_WARN ("Rejecting potential handle cluster because height (%g) is above threshold (%g)!", fabs (maxP.z - minP.z), handle_height_threshold_);
          continue;
        }

        for (unsigned int j = 0; j < clusters[i].size (); j++)
        {
          handle_indices[nr_phi] = clusters[i][j];
          nr_phi++;
        }
        break;      // get the first cluster
      }
      handle_indices.resize (nr_phi);

      // Compute the centroid for the remaining handle indices
      cloud_geometry::nearest::computeCentroid (points, &handle_indices, handle_center);

      // Calculate the unsigned distance from the point to the plane
      double distance_to_plane = door_coeff->at (0) * handle_center.x + door_coeff->at (1) * handle_center.y +
                                 door_coeff->at (2) * handle_center.z + door_coeff->at (3) * 1;
      // Calculate the projection of the point on the plane
      handle_center.x -= distance_to_plane * door_coeff->at (0);
      handle_center.y -= distance_to_plane * door_coeff->at (1);
      handle_center.z -= distance_to_plane * door_coeff->at (2);


// This needs to be removed
#if 1
      std_msgs::PointCloud handle_cloud;
      //        handle_indices_clusters = possible_handle_indices;
      handle_cloud.chan.resize (1);
      handle_cloud.chan[0].name = "intensities";
      handle_cloud.chan[0].vals.resize (handle_indices.size ());
      handle_visualize.resize (handle_indices.size ());
      for (unsigned int i = 0; i < handle_indices.size (); i++)
      {
        handle_visualize[i].x = points->pts.at (handle_indices[i]).x;
        handle_visualize[i].y = points->pts.at (handle_indices[i]).y;
        handle_visualize[i].z = points->pts.at (handle_indices[i]).z;
        handle_cloud.chan[0].vals[i] = points->chan[d_idx].vals.at (handle_indices[i]);
      }
      cout << "added " << handle_visualize.size() << " points to visualize" << endl;
      handle_cloud.header.stamp = ros::Time::now ();
      handle_cloud.header.frame_id = "base_link";
      handle_cloud.pts = handle_visualize;

      publish ("handle_visualization", handle_cloud);

      pmap.polygons.push_back (poly_tr);
      pmap.header.stamp = ros::Time::now ();
      pmap.header.frame_id = "base_link";
      publish ("pmap", pmap);
#endif
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Main point cloud callback.                                                                           */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloud_cb ()
    {
      need_cloud_data_ = false;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  DoorHandleDetector p;

  door_handle_detector::Door::Request req;
  req.door.frame_p1.x = 1.9; req.door.frame_p1.y = 0.6; req.door.frame_p1.z = 0;
  req.door.frame_p2.x = 2.0; req.door.frame_p2.y = -0.3; req.door.frame_p2.z = 0;

  door_handle_detector::Door::Response resp;
  ros::service::call ("door_handle_detector", req, resp);

//  p.spin ();

  p.shutdown ();
  return (0);
}
/* ]--- */

