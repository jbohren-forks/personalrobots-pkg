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
// ROS messages
#include <std_msgs/PointCloud.h>
#include <std_msgs/Polygon3D.h>
#include <std_msgs/PolygonalMap.h>

// Sample Consensus
#include <sample_consensus/sac.h>
#include <sample_consensus/msac.h>
#include <sample_consensus/ransac.h>
#include <sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>

// Kd Tree
#include <cloud_kdtree/kdtree.h>

// Cloud geometry
#include <cloud_geometry/areas.h>
#include <cloud_geometry/point.h>
#include <cloud_geometry/distances.h>
#include <cloud_geometry/nearest.h>
#include <cloud_geometry/transforms.h>

#include <sys/time.h>

#include "door_handle_detector/Door.h"

using namespace std;
using namespace std_msgs;

class DoorHandleDetector : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_in_, cloud_down_;
    Point leaf_width_;
    PointCloud cloud_annotated_;
    Point32 z_axis_;
    PolygonalMap pmap_;

    tf::TransformListener tf_;

    // Parameters
    double frame_distance_eps_, min_z_bounds_, max_z_bounds_;
    string input_cloud_topic_;
    int k_;
    double clusters_growing_tolerance_;
    int clusters_min_pts_;

    bool need_cloud_data_, publish_debug_;

    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;

    double door_min_height_, door_min_width_, door_max_height_, door_max_width_;
    double handle_distance_door_min_threshold_, handle_distance_door_max_threshold_, handle_max_height_, handle_min_height_;
    double handle_height_threshold_;
    
    int door_frame_multiplier_threshold_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DoorHandleDetector () : ros::Node ("door_handle_detector"), tf_(*this)
    {
      param ("~frame_distance_eps", frame_distance_eps_, 0.2);          // Allow 20cm extra space by default

      param ("~min_z_bounds", min_z_bounds_, 0.0);                      // restrict the Z dimension between 0
      param ("~max_z_bounds", max_z_bounds_, 3.0);                      // and 3.0 m

      param ("~downsample_leaf_width_x", leaf_width_.x, 0.03);          // 3cm radius by default
      param ("~downsample_leaf_width_y", leaf_width_.y, 0.03);          // 3cm radius by default
      param ("~downsample_leaf_width_z", leaf_width_.z, 0.03);          // 3cm radius by default
      param ("~search_k_closest", k_, 6);                               // 6 k-neighbors by default

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      param ("~normal_eps_angle_", eps_angle_, 15.0);                   // 15 degrees, wrt the Z-axis
      eps_angle_ = (eps_angle_ * M_PI / 180.0);                         // convert to radians

      param ("~region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = (region_angle_threshold_ * M_PI / 180.0); // convert to radians

      param ("~clusters_growing_tolerance", clusters_growing_tolerance_, 0.05);  // 5 cm
      param ("~clusters_min_pts", clusters_min_pts_, 10);                        // 10 points

      param ("~door_min_height", door_min_height_, 1.4);                  // minimum height of a door: 1.4m
      param ("~door_max_height", door_max_height_, 3.0);                  // maximum height of a door: 3m
      param ("~door_min_width", door_min_width_, 0.8);                    // minimum width of a door: 0.8m
      param ("~door_max_width", door_max_width_, 1.2);                    // maximumwidth of a door: 1.2m
      ROS_DEBUG ("Using the following thresholds for door detection [min-max height / min-max width]: %f-%f / %f-%f.",
                 door_min_height_, door_max_height_, door_min_width_, door_max_width_);

      param ("~handle_max_height", handle_max_height_, 1.41);            // maximum height for a door handle: 1.41m
      param ("~handle_min_height", handle_min_height_, 0.41);            // minimum height for a door handle: 0.41m
      param ("~handle_distance_door_max_threshold", handle_distance_door_max_threshold_, 0.15); // maximum distance between the handle and the door
      param ("~handle_distance_door_min_threshold", handle_distance_door_min_threshold_, 0.05); // minimum distance between the handle and the door
      param ("~handle_height_threshold", handle_height_threshold_, 0.1); // Additional threshold for filtering large Z clusters (potentially part of the handle)

      ROS_DEBUG ("Using the following thresholds for handle detection [min height / max height]: %f / %f.", handle_min_height_, handle_max_height_);

      // This describes the size of our 3D bounding box (basically the space where we search for doors),
      // as a multiplier of the door frame (computed using the two points from the service call) in both X and Y directions
      param ("~door_frame_multiplier_threshold", door_frame_multiplier_threshold_, 4);
      param ("~publish_debug", publish_debug_, true);

      param ("~input_cloud_topic", input_cloud_topic_, string ("full_cloud"));
      advertiseService("door_handle_detector", &DoorHandleDetector::detectDoor, this);

      // This should be set to whatever the leaf_width factor is in the downsampler
      param ("~sac_distance_threshold", sac_distance_threshold_, 0.03);     // 5 cm

      advertise<std_msgs::PointCloud>("handle_visualization",10);
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
      if (hasParam ("~frame_distance_eps"))
        getParam ("~frame_distance_eps", frame_distance_eps_);
      if (hasParam ("~input_cloud_topic"))
        getParam ("~input_cloud_topic", input_cloud_topic_);
      // \NOTE to Wim : Perhaps we need to read the other parameters from the server here ?
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      get3DBounds (Point32 *p1, Point32 *p2, Point32 &minB, Point32 &maxB)
    {
      // Get the door_frame distance in the X-Y plane
      float door_frame = sqrt ( (p1->x - p2->x) * (p1->x - p2->x) + (p1->y - p2->y) * (p1->y - p2->y) );

      float center[2];
      center[0] = (p1->x + p2->x) / 2.0;
      center[1] = (p1->y + p2->y) / 2.0;

      // Obtain the bounds (doesn't matter which is min and which is max at this point)
      minB.x = center[0] + (door_frame_multiplier_threshold_ * door_frame) / 2.0 + frame_distance_eps_;
      minB.y = center[1] + (door_frame_multiplier_threshold_ * door_frame) / 2.0 + frame_distance_eps_;
      minB.z = min_z_bounds_;

      maxB.x = center[0] - (door_frame_multiplier_threshold_ * door_frame) / 2.0 + frame_distance_eps_;
      maxB.y = center[1] - (door_frame_multiplier_threshold_ * door_frame) / 2.0 + frame_distance_eps_;
      maxB.z = max_z_bounds_;

      // Order min/max
      if (minB.x > maxB.x)
      {
        float tmp = minB.x;
        minB.x = maxB.x;
        maxB.x = tmp;
      }
      if (minB.y > maxB.y)
      {
        float tmp = minB.y;
        minB.y = maxB.y;
        maxB.y = tmp;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detectDoor (door_handle_detector::Door::Request &req, door_handle_detector::Door::Response &resp)
    {
      cout << "Start detecting door at points ";
      cout << "(" << req.frame_p1.x << " " <<req.frame_p1.y << " "<<req.frame_p1.z << ")   ";
      cout << "(" << req.frame_p2.x << " " <<req.frame_p2.y << " "<<req.frame_p2.z << ")" << endl;

      timeval t1, t2;
      double time_spent;

      updateParametersFromServer ();

      // Obtain the bounding box information
      Point32 minB, maxB;
      get3DBounds (&req.frame_p1, &req.frame_p2, minB, maxB);


      cout << " - receive door scan" << endl;

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

      gettimeofday (&t1, NULL);
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

      ROS_DEBUG ("Number of points in bounds [%f,%f,%f] -> [%f,%f,%f]: %d.", minB.x, minB.y, minB.z, maxB.x, maxB.y, maxB.z, indices_in_bounds.size ());

      // Downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        cloud_geometry::downsamplePointCloud (&cloud_in_, &indices_in_bounds, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }
      leaves.resize (0);

      ROS_DEBUG ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, cloud_down_.pts.size ());

      // Reserve space for 4 channels: nx, ny, nz, curvature
      cloud_down_.chan.resize (4);     // Allocate 7 more channels
      cloud_down_.chan[0].name = "nx";
      cloud_down_.chan[1].name = "ny";
      cloud_down_.chan[2].name = "nz";
      cloud_down_.chan[3].name = "curvature";
      for (unsigned int d = 0; d < cloud_down_.chan.size (); d++)
        cloud_down_.chan[d].vals.resize (cloud_down_.pts.size ());

      // Create Kd-Tree
      estimatePointNormals (&cloud_down_);

      cout << " - Select points whose normals are perpendicular to the Z-axis" << endl;

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_xy;
      // Check all points
      for (unsigned int i = 0; i < cloud_down_.pts.size (); i++)
      {
        std_msgs::Point32 p;
        p.x = cloud_down_.chan[0].vals[i];
        p.y = cloud_down_.chan[1].vals[i];
        p.z = cloud_down_.chan[2].vals[i];
        // Compute the angle between their normal and the given axis
        double angle = acos (cloud_geometry::dot (p, z_axis_));
        if ( fabs (M_PI / 2.0 - angle) < eps_angle_)
          indices_xy.push_back (i);
      }
      vector<vector<int> > clusters;
      // Split the Z-perpendicular points into clusters
      findClusters (&cloud_down_, &indices_xy, clusters_growing_tolerance_, clusters, 0, 1, 2, clusters_min_pts_);

      // Compute the total number of points in all clusters
      int total_p = 0;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
        total_p += clusters[cc].size ();

      ROS_DEBUG ("Number of clusters found: %d, total points: %d.", clusters.size (), total_p);

      // Reserve enough space
      cloud_annotated_.header = cloud_down_.header;
      cloud_annotated_.pts.resize (cloud_in_.pts.size ());//total_p);
      cloud_annotated_.chan[0].vals.resize (cloud_in_.pts.size ());//total_p);

      pmap_.header = cloud_down_.header;
      pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map

      vector<vector<double> > coeff (clusters.size ()); // Need to save all coefficients for all models

      cout << " - Process all clusters" << endl;
      nr_p = 0;
      vector<int> inliers;
      vector<int> handle_indices;
      vector<double> goodness_factor (clusters.size());
      std_msgs::Point32 minP, maxP, handle_center;

      // Process all clusters
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // initial goodness factor
        goodness_factor[cc] = 1;

        // Find the best plane in this cluster
        fitSACPlane (&cloud_down_, &clusters[cc], inliers, coeff[cc]);
        // Compute the convex hull
        cloud_geometry::areas::convexHull2D (&cloud_down_, &inliers, &coeff[cc], pmap_.polygons[cc]);

        // Filter the region based on its height and width
        cloud_geometry::getMinMax (&pmap_.polygons[cc], minP, maxP);

        // adapt the goodness factor for each cluster
        double door_frame = sqrt (pow ((minP.x - maxP.x), 2) + pow ((minP.y - maxP.y), 2));
        double door_height = fabs (maxP.z - minP.z);
        if (door_frame < door_min_width_ || door_height < door_min_height_ || door_frame > door_max_width_ || door_height > door_max_height_)
          goodness_factor[cc] = 0;
        double area = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);
        goodness_factor[cc] *= (area / (door_frame * door_height));

        // Do a segmentation of all inlier points on the plane +/- door_dist_thresh in intensity space

//        cout << "minP and maxP (" << minP.x << " " << minP.y <<" "<< minP.z <<") ("<< maxP.x <<" " << maxP.y <<" "<< maxP.z << ")" << endl;
//        cout << "width = " << door_frame << endl;
//        cout << "height = " << door_height << endl;
//        cout << "area = " << cloud_geometry::areas::compute2DPolygonalArea(pmap_.polygons[cc], coeff[cc]) << endl;
      } // loop over clusters

      // find best cluster
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
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        if (cc != best_cluster)
          pmap_.polygons[cc].points.resize (0);
      }

      if (best_cluster == -1)
      {
        ROS_ERROR("did not find a door");
        return false;
      }

      // Find the handle by performing a segmentation
      findDoorHandle (&cloud_in_, &indices_in_bounds, &coeff[best_cluster], &pmap_.polygons[best_cluster], handle_indices, handle_center);
      ROS_INFO ("Number of points selected: %d.", handle_indices.size ());

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

      resp.door_p1.x = minP.x; resp.door_p1.y = minP.y;
      resp.door_p2.x = maxP.x; resp.door_p2.y = maxP.y;
      resp.height = fabs (maxP.z - minP.z);
      resp.handle = handle_center;

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Door found. P1 = [%f, %f, %f]. P2 = [%f, %f, %f]. Height = %f. Handle = [%f, %f, %f]. Total time: %f.",
                resp.door_p1.x, resp.door_p1.y, resp.door_p1.z, resp.door_p2.x, resp.door_p2.y, resp.door_p2.z,
                resp.height, resp.handle.x, resp.handle.y, resp.handle.z,
                time_spent);

      if (publish_debug_)
      {
        cloud_annotated_.pts.resize (nr_p);
        cloud_annotated_.chan[0].vals.resize (nr_p);

        publish ("cloud_annotated", cloud_annotated_);
        publish ("semantic_polygonal_map", pmap_);
      }


      cout << "Finished detecting door" << endl;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 24-bit RGB coded value from 3 independent <r, g, b> channel values
      * \param r the red channel value
      * \param g the green channel value
      * \param b the blue channel value
      */
    double
      getRGB (float r, float g, float b)
    {
      int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
      double rgb = *(float*)(&res);
      return (rgb);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      findDoorHandle (PointCloud *points, vector<int> *indices, vector<double> *coeff, Polygon3D *poly,
                      vector<int> &handle_indices, Point32 &handle_center)
    {
      // Transform the polygon to lie on the X-Y plane (makes all isPointIn2DPolygon computations easier)
      Polygon3D poly_tr;
      Point32 p_tr;
      Eigen::Matrix4d transformation;
      vector<double> z_axis (3, 0); z_axis[2] = 1.0;
      cloud_geometry::transforms::getPlaneToPlaneTransformation (*coeff, z_axis, 0, 0, 0, transformation);
      cloud_geometry::transforms::transformPoints (&poly->points, poly_tr.points, transformation);

      cout << "size cluster for handle detection = " << poly->points.size() << endl;

      vector<int> possible_handle_indices;
      possible_handle_indices.resize (indices->size ());
      int nr_p = 0;
      Point32 pt;
      vector<std_msgs::Point32> handle_visualize;
      vector<float> colors;

      for (unsigned int i = 0; i < indices->size (); i++)
      {
        // Select all the points in the given bounds which are between the given handle min->max height
        if (points->pts.at (indices->at (i)).z > handle_min_height_ &&
            points->pts.at (indices->at (i)).z < handle_max_height_)
        {

          // Calculate the distance from the point to the plane
          double distance_to_plane = coeff->at (0) * points->pts.at (indices->at (i)).x +
                                     coeff->at (1) * points->pts.at (indices->at (i)).y +
                                     coeff->at (2) * points->pts.at (indices->at (i)).z +
                                     coeff->at (3);
          // Calculate the projection of the point on the plane
          pt.x = points->pts.at (indices->at (i)).x - distance_to_plane * coeff->at (0);
          pt.y = points->pts.at (indices->at (i)).y - distance_to_plane * coeff->at (1);
          pt.z = points->pts.at (indices->at (i)).z - distance_to_plane * coeff->at (2);

          std_msgs::Point32 pnt;
          pnt.x = points->pts.at (indices->at (i)).x;
          pnt.y = points->pts.at (indices->at (i)).y;
          pnt.z = points->pts.at (indices->at (i)).z;
          handle_visualize.push_back (pnt);

          if (fabs (distance_to_plane) < handle_distance_door_max_threshold_ && fabs (distance_to_plane) > handle_distance_door_min_threshold_)
          {
            // Transform the point onto X-Y for faster checking inside the polygonal bounds
            cloud_geometry::transforms::transformPoint (&pt, p_tr, transformation);
            if (cloud_geometry::areas::isPointIn2DPolygon (p_tr, &poly_tr))
            {
              possible_handle_indices[nr_p] = indices->at (i);
              nr_p++;
              colors.push_back (getRGB (0, 1, 0));              // Mark with green if inside the polygon and close to the door
            }
            else
              colors.push_back (getRGB (0, 0, 1));              // Mark with blue if close to the door but outside the polygon
          }
          else
            colors.push_back (getRGB (1, 0, 0));                // Mark with red otherwise
        }
      }

      cout << "added " << handle_visualize.size() << " points to visualize" << endl;
      std_msgs::ChannelFloat32 channel;
      channel.name = "rgb";
      channel.vals = colors;
      std_msgs::PointCloud handle_cloud;
      handle_cloud.header.stamp = ros::Time::now ();
      handle_cloud.header.frame_id = "base_link";
      handle_cloud.pts = handle_visualize;
      handle_cloud.chan.push_back (channel);

      publish ("handle_visualization", handle_cloud);

      possible_handle_indices.resize (nr_p);
      cout << "found " << nr_p << " point candidates for door handle" << endl;

      // Project the handle on the door plane (just for kicks)
      for (int i = 0; i < nr_p; i++)
      {
        // Calculate the distance from the point to the plane
        double distance_to_plane = coeff->at (0) * points->pts.at (possible_handle_indices[i]).x +
                                   coeff->at (1) * points->pts.at (possible_handle_indices[i]).y +
                                   coeff->at (2) * points->pts.at (possible_handle_indices[i]).z +
                                   coeff->at (3) * 1;
        // Calculate the projection of the point on the plane
        points->pts.at (possible_handle_indices[i]).x -= distance_to_plane * coeff->at (0);
        points->pts.at (possible_handle_indices[i]).y -= distance_to_plane * coeff->at (1);
        points->pts.at (possible_handle_indices[i]).z -= distance_to_plane * coeff->at (2);
      }

      // Split points into clusters
      vector<vector<int> > clusters;
      findClusters (points, &possible_handle_indices, clusters_growing_tolerance_, clusters, -1, -1, -1, 2);
      for (unsigned int i=0; i<clusters.size(); i++)
        cout << "cluster " << i << " has size " << clusters[i].size() << endl;

      // \NOTE to Wim => need to think about how to filter the edges near the
      // door.  We could check for min/max in Z and if they are too big then
      // we discard the cluster. Below is an example:
      std_msgs::Point32 minP, maxP;
      handle_indices.resize (possible_handle_indices.size ());
      nr_p = 0;
      for (unsigned int i = 0; i < clusters.size (); i++)
      {
        cloud_geometry::getMinMax (points, &clusters[i], minP, maxP);
        if (fabs (maxP.z - minP.z) < handle_height_threshold_)
        {
          for (unsigned int j = 0; j < clusters[i].size (); j++)
          {
            handle_indices[nr_p] = clusters[i][j];
            nr_p++;
          }
        }
        else
          ROS_ERROR ("Rejecting potential handle cluster because height (%g) is above threshold (%g)!", fabs (maxP.z - minP.z), handle_height_threshold_);
      }
      handle_indices.resize (nr_p);

      // Compute the centroid for the remaining handle indices
      cloud_geometry::nearest::computeCentroid (points, &handle_indices, handle_center);

      // Calculate the distance from the point to the plane
      double distance_to_plane = coeff->at (0) * handle_center.x +
                                 coeff->at (1) * handle_center.y +
                                 coeff->at (2) * handle_center.z +
                                 coeff->at (3) * 1;
      // Calculate the projection of the point on the plane
      handle_center.x -= distance_to_plane * coeff->at (0);
      handle_center.y -= distance_to_plane * coeff->at (1);
      handle_center.z -= distance_to_plane * coeff->at (2);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      need_cloud_data_ = false;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points
      * \param points pointer to the point cloud message
      * \param indices pointer to a list of point indices
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud *points, vector<int> *indices, double tolerance, vector<vector<int> > &clusters,
                    int nx_idx, int ny_idx, int nz_idx,
                    unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTree (points, indices);

      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (indices->size (), false);

      vector<int> nn_indices;
      // Process all points in the indices vector
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        if (processed[i])
          continue;

        vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        double norm_a = 0.0;
        if (nx_idx != -1)         // If we use normal indices...
          norm_a = sqrt (points->chan[nx_idx].vals[indices->at (i)] * points->chan[nx_idx].vals[indices->at (i)] +
                         points->chan[ny_idx].vals[indices->at (i)] * points->chan[ny_idx].vals[indices->at (i)] +
                         points->chan[nz_idx].vals[indices->at (i)] * points->chan[nz_idx].vals[indices->at (i)]);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance);
          tree->getNeighborsIndices (nn_indices);

          for (unsigned int j = 1; j < nn_indices.size (); j++)
          {
            if (!processed.at (nn_indices[j]))
            {
              if (nx_idx != -1)         // If we use normal indices...
              {
                double norm_b = sqrt (points->chan[nx_idx].vals[indices->at (nn_indices[j])] * points->chan[nx_idx].vals[indices->at (nn_indices[j])] +
                                      points->chan[ny_idx].vals[indices->at (nn_indices[j])] * points->chan[ny_idx].vals[indices->at (nn_indices[j])] +
                                      points->chan[nz_idx].vals[indices->at (nn_indices[j])] * points->chan[nz_idx].vals[indices->at (nn_indices[j])]);
                // [-1;1]
                double dot_p = points->chan[nx_idx].vals[indices->at (i)] * points->chan[nx_idx].vals[indices->at (nn_indices[j])] +
                               points->chan[ny_idx].vals[indices->at (i)] * points->chan[ny_idx].vals[indices->at (nn_indices[j])] +
                               points->chan[nz_idx].vals[indices->at (i)] * points->chan[nz_idx].vals[indices->at (nn_indices[j])];
                if ( acos (dot_p / (norm_a * norm_b)) < region_angle_threshold_)
                {
                  processed[nn_indices[j]] = true;
                  seed_queue.push_back (nn_indices[j]);
                }
              }
              else
              {
                processed[nn_indices[j]] = true;
                seed_queue.push_back (nn_indices[j]);
              }
            }
          }

          sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster)
        {
          vector<int> r;
          //r.indices = seed_queue;
          r.resize (seed_queue.size ());
          for (unsigned int j = 0; j < r.size (); j++)
            r[j] = indices->at (seed_queue[j]);
          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int
      fitSACPlane (PointCloud *points, vector<int> *indices, vector<int> &inliers, vector<double> &coeff)
    {
      if ((int)indices->size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (-1);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (500);
      sac->setProbability (0.99);
      model->setDataSet (points, *indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
        {
          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
          inliers.resize (0);
          coeff.resize (0);
          return (0);
        }
        inliers = sac->getInliers ();
        coeff   = sac->computeCoefficients ();

        //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
        //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      estimatePointNormals (PointCloud *cloud)
    {
      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTree (cloud);
      vector<vector<int> > points_k_indices;
      // Allocate enough space for point indices
      points_k_indices.resize (cloud->pts.size ());
      for (int i = 0; i < (int)cloud->pts.size (); i++)
        points_k_indices[i].resize (k_);
      // Get the nerest neighbors for all the point indices in the bounds
      for (int i = 0; i < (int)cloud->pts.size (); i++)
      {
        vector<double> distances (k_);
        kdtree->nearestKSearch (i, k_, points_k_indices[i], distances);
      }

      // Figure out the viewpoint value in the point cloud frame
      PointStamped viewpoint_laser, viewpoint_cloud;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0,0,0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf_.transformPoint (cloud->header.frame_id, viewpoint_laser, viewpoint_cloud);
      }
      catch (tf::ConnectivityException)
      {
        viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
      }

      #pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)cloud->pts.size (); i++)
      {
        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        Eigen::Vector4d plane_parameters;
        double curvature;
        cloud_geometry::nearest::computeSurfaceNormalCurvature (cloud, &points_k_indices[i], plane_parameters, curvature);

        // See if we need to flip any plane normals
        Point32 vp_m;
        vp_m.x = viewpoint_cloud.point.x - cloud_down_.pts[i].x;
        vp_m.y = viewpoint_cloud.point.y - cloud_down_.pts[i].y;
        vp_m.z = viewpoint_cloud.point.z - cloud_down_.pts[i].z;

        // Dot product between the (viewpoint - point) and the plane normal
        double cos_theta = (vp_m.x * plane_parameters (0) + vp_m.y * plane_parameters (1) + vp_m.z * plane_parameters (2));// / norm;

        // Flip the plane normal
        if (cos_theta < 0)
        {
          for (int d = 0; d < 3; d++)
            plane_parameters (d) *= -1;
        }
        cloud->chan[0].vals[i] = plane_parameters (0);
        cloud->chan[1].vals[i] = plane_parameters (1);
        cloud->chan[2].vals[i] = plane_parameters (2);
        cloud->chan[3].vals[i] = fabs (plane_parameters (3));
      }
      // Delete the kd-tree
      delete kdtree;
    }


};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  DoorHandleDetector p;

  door_handle_detector::Door::Request req;
//  req.frame_p1.x = 1.2; req.frame_p1.y = 0.6; req.frame_p1.z = 0;
//  req.frame_p2.x = 1.4; req.frame_p2.y = -0.5; req.frame_p2.z = 0;
//  req.frame_p1.x = 0.9; req.frame_p1.y = 0.7; req.frame_p1.z = 0;
//  req.frame_p2.x = 1.0; req.frame_p2.y = -0.1; req.frame_p2.z = 0;
  req.frame_p1.x = 1.9; req.frame_p1.y = 0.6; req.frame_p1.z = 0;
  req.frame_p2.x = 2.0; req.frame_p2.y = -0.3; req.frame_p2.z = 0;
//Picked Point with Index: 88693 [1.9313, 0.64552, 0.025881]
//Picked Point with Index: 86094 [2.06, -0.37576, 0.024952]
    
  door_handle_detector::Door::Response resp;
  ros::service::call ("door_handle_detector", req, resp);

//  p.spin ();

  p.shutdown ();
  return (0);
}
/* ]--- */

