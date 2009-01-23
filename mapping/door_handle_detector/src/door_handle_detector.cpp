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

@b door_handle_detector_omp annotates 3D point clouds with semantic labels.

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
#include <sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>

// Kd Tree
#include <cloud_kdtree/kdtree.h>

// Cloud geometry
#include <cloud_geometry/areas.h>
#include <cloud_geometry/point.h>
#include <cloud_geometry/distances.h>
#include <cloud_geometry/nearest.h>

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

    bool need_cloud_data_;

    int sac_min_points_per_model_, sac_min_points_left_;
    double sac_distance_threshold_, eps_angle_, region_angle_threshold_, boundary_angle_threshold_;

    double rule_wall_;

    bool polygonal_map_, concave_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DoorHandleDetector () : ros::Node ("door_handle_detector"), tf_(*this)
    {
      param ("~frame_distance_eps", frame_distance_eps_, 0.2);          // Allow 20cm extra space by default

      param ("~min_z_bounds", min_z_bounds_, 0.0);                      // restrict the Z dimension between 0
      param ("~max_z_bounds", max_z_bounds_, 3.0);                      // and 3.0 m

      param ("~downsample_leaf_width_x", leaf_width_.x, 0.03);          // 3cm radius by default
      param ("~downsample_leaf_width_y", leaf_width_.y, 0.03);          // 3cm radius by default
      param ("~downsample_leaf_width_z", leaf_width_.z, 0.03);          // 3cm radius by default
      param ("~search_k_closest", k_, 10);                              // 10 k-neighbors by default

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      param ("~normal_eps_angle_", eps_angle_, 15.0);                   // 15 degrees
      eps_angle_ = (eps_angle_ * M_PI / 180.0);                         // convert to radians

      param ("~region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = (region_angle_threshold_ * M_PI / 180.0); // convert to radians

      param ("~clusters_growing_tolerance", clusters_growing_tolerance_, 0.25);  // 25 cm
      param ("~clusters_min_pts", clusters_min_pts_, 10);                        // 10 points

      param ("~input_cloud_topic", input_cloud_topic_, string ("full_cloud"));
      advertiseService("door_handle_detector", &DoorHandleDetector::detectDoor, this);

      param ("~p_sac_min_points_left", sac_min_points_left_, 10);
      param ("~p_sac_min_points_per_model", sac_min_points_per_model_, 10);   // 50 points at high resolution

      // This should be set to whatever the leaf_width factor is in the downsampler
      param ("~p_sac_distance_threshold", sac_distance_threshold_, 0.05);     // 5 cm

      param ("~create_polygonal_map", polygonal_map_, true);            // Create a polygonal map ?

      param ("~boundary_angle_threshold", boundary_angle_threshold_, 120.0); // Boundary angle threshold
      boundary_angle_threshold_ = (boundary_angle_threshold_ * M_PI / 180.0);   // convert to radians

      if (polygonal_map_)
        advertise<PolygonalMap> ("semantic_polygonal_map", 1);

      advertise<PointCloud> ("cloud_annotated", 1);

      cloud_annotated_.chan.resize (1);
      cloud_annotated_.chan[0].name = "rgb";
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
      minB.x = center[0] + (3 * door_frame) / 2.0 + frame_distance_eps_;
      minB.y = center[1] + (3 * door_frame) / 2.0 + frame_distance_eps_;
      minB.z = min_z_bounds_;

      maxB.x = center[0] - (3 * door_frame) / 2.0 + frame_distance_eps_;
      maxB.y = center[1] - (3 * door_frame) / 2.0 + frame_distance_eps_;
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
      detectDoor (door_handle_detector::Door::request &req, door_handle_detector::Door::response &resp)
    {
      timeval t1, t2;
      double time_spent;

      updateParametersFromServer ();

      // Obtain the bounding box information
      Point32 minB, maxB;
      get3DBounds (&req.frame_p1, &req.frame_p2, minB, maxB);

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

      ROS_INFO ("Number of points in bounds [%f,%f,%f] -> [%f,%f,%f]: %d.", minB.x, minB.y, minB.z, maxB.x, maxB.y, maxB.z, indices_in_bounds.size ());

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

      ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, cloud_down_.pts.size ());

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

      ROS_INFO ("Number of clusters found: %d, total points: %d.", clusters.size (), total_p);

      // Reserve enough space
      cloud_annotated_.header = cloud_down_.header;
      cloud_annotated_.pts.resize (total_p);
      cloud_annotated_.chan[0].vals.resize (total_p);

      nr_p = 0;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        double r, g, b, rgb;
        r = g = b = 1.0;
        //int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
        int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
        rgb = *(float*)(&res);

        // Mark all the points inside
        for (unsigned int k = 0; k < clusters[cc].size (); k++)
        {
          cloud_annotated_.pts[nr_p].x = cloud_down_.pts.at (clusters[cc][k]).x;
          cloud_annotated_.pts[nr_p].y = cloud_down_.pts.at (clusters[cc][k]).y;
          cloud_annotated_.pts[nr_p].z = cloud_down_.pts.at (clusters[cc][k]).z;
          cloud_annotated_.chan[0].vals[nr_p] = rgb;
          nr_p++;
        }
      }

      cloud_annotated_.pts.resize (nr_p);
      cloud_annotated_.chan[0].vals.resize (nr_p);

      publish ("cloud_annotated", cloud_annotated_);

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Door found. Total time: %f.", time_spent);
      return (true);
   }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      need_cloud_data_ = false;

/*
      vector<vector<vector<int> > > all_cluster_inliers (clusters.size ());
      vector<vector<vector<double> > > all_cluster_coeff (clusters.size ());

      // Reserve enough space
      cloud_annotated_.pts.resize (total_p);
      cloud_annotated_.chan[0].vals.resize (total_p);

      int nr_p = 0;

      if (polygonal_map_)
      {
        pmap_.header = cloud_.header;
        pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map
      }
      
      // Process all clusters in parallel
      #pragma omp parallel for schedule(dynamic)
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // Find all planes in this cluster
        int ret = fitSACPlane (&cloud_, &clusters[cc].indices, all_cluster_inliers[cc], all_cluster_coeff[cc]);
        if (ret != 0 || all_cluster_inliers[cc].size () == 0)
          continue;
      }
      
      // Bummer - no omp here
      // cloud_kdtree is not thread safe because we rely on ANN/FL-ANN, so get the neighbors here
      vector<vector<vector<int> > > neighbors (clusters.size ());
      if (concave_)
      {
        for (int cc = 0; cc < (int)clusters.size (); cc++)
        {
          if (all_cluster_inliers[cc].size () == 0 || all_cluster_coeff[cc].size () == 0)
            continue;
            
          vector<vector<int> > *cluster_neighbors = &neighbors[cc];
          vector<int> *indices = &all_cluster_inliers[cc][0];
          
          if (indices->size () == 0)
            continue;
          cluster_neighbors->resize (indices->size ());

          // Create a tree for these points
          cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTree (&cloud_, indices);
          for (unsigned int i = 0; i < indices->size (); i++)
          {
            tree->radiusSearch (i, 0.3);                      // 30cm radius search
//            tree->nearestKSearch (i, 30);                      // 30cm radius search
            // Note: the neighbors below are in the 0->indices.size () spectrum and need to be
            // transformed into global point indices (!)
            tree->getNeighborsIndices (cluster_neighbors->at (i));
            for (unsigned int j = 0; j < cluster_neighbors->at (i).size (); j++)
              cluster_neighbors->at(i).at(j) = indices->at (cluster_neighbors->at(i).at(j));
          }
          // Destroy the tree
          delete tree;
        }
        gettimeofday (&t2, NULL);
        time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
        ROS_INFO ("Nearest neighbors (concave hull) estimated in %g seconds.", time_spent);
        gettimeofday (&t1, NULL);
      }
      
      if (polygonal_map_)
      {
        // Process all clusters in parallel
        //#pragma omp parallel for schedule(dynamic)
        // Fit convex hulls to the inliers (points should be projected!)
        for (int cc = 0; cc < (int)clusters.size (); cc++)
        {      
          if (all_cluster_inliers[cc].size () == 0 || all_cluster_coeff[cc].size () == 0)
            continue;

          vector<int> *indices  = &all_cluster_inliers[cc][0];
          vector<double> *coeff = &all_cluster_coeff[cc][0];
          if (indices->size () == 0 || coeff->size () == 0)
            continue;

          if (concave_)
            computeConcaveHull (&cloud_, indices, coeff, &neighbors[cc], pmap_.polygons[cc]);
          else
            cloud_geometry::areas::convexHull2D (&cloud_, indices, coeff, pmap_.polygons[cc]);
        }
      }
      
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        double r, g, b, rgb;
        r = g = b = 1.0;
        int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
        rgb = *(float*)(&res);

        // Get the planes in this cluster
        vector<vector<int> > *planes_inliers   = &all_cluster_inliers[cc];
        vector<vector<double> > *planes_coeffs = &all_cluster_coeff[cc];
        
        if (planes_inliers->size () == 0 || planes_coeffs->size () == 0 || planes_inliers->size () != planes_coeffs->size ())
          continue;
          
        // For every plane in this cluster
        for (unsigned int j = 0; j < planes_inliers->size (); j++)
        {
          vector<int> *plane_inliers   = &planes_inliers->at (j);
          vector<double> *plane_coeffs = &planes_coeffs->at (j);
          
          if (plane_inliers->size () == 0 || plane_coeffs->size () == 0)
            continue;
            
          // Mark all the points inside
          getObjectClassForPerpendicular (&cloud_, plane_inliers, rgb);
          for (unsigned int k = 0; k < plane_inliers->size (); k++)
          {
            cloud_annotated_.pts[nr_p].x = cloud_.pts.at (plane_inliers->at (k)).x;
            cloud_annotated_.pts[nr_p].y = cloud_.pts.at (plane_inliers->at (k)).y;
            cloud_annotated_.pts[nr_p].z = cloud_.pts.at (plane_inliers->at (k)).z;
            cloud_annotated_.chan[0].vals[nr_p] = rgb;
            nr_p++;
          }
        }
      }

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Cloud annotated in: %g seconds.", time_spent);
      gettimeofday (&t1, NULL); 

      if (polygonal_map_)
        publish ("semantic_polygonal_map", pmap_);
      return;*/
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

        double norm_a = sqrt (points->chan[nx_idx].vals[indices->at (i)] * points->chan[nx_idx].vals[indices->at (i)] +
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
      fitSACPlane (PointCloud *points, vector<int> *indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff)
    {
      vector<int> empty_inliers;
      vector<double> empty_coeffs;
      if ((int)indices->size () < sac_min_points_per_model_)
      {
        //ROS_ERROR ("fitSACPlane: Indices.size (%d) < sac_min_points_per_model (%d)!", indices->size (), sac_min_points_per_model_);
        inliers.push_back (empty_inliers);
        coeff.push_back (empty_coeffs);
        return (-1);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (120);
      sac->setProbability (0.99);
      model->setDataSet (points, *indices);

      PointCloud pts (*points);
      int nr_points_left = indices->size ();
      int nr_models = 0;
      while (nr_points_left > sac_min_points_left_)
      {
        // Search for the best plane
        if (sac->computeModel ())
        {
          // Obtain the inliers and the planar model coefficients
          if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
          {
            //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
            inliers.push_back (empty_inliers);
            coeff.push_back (empty_coeffs);
            //return (-1);
            break;
          }
          inliers.push_back (sac->getInliers ());
          coeff.push_back (sac->computeCoefficients ());

          //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
          //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

          // Project the inliers onto the model
          model->projectPointsInPlace (sac->getInliers (), coeff[coeff.size () - 1]);

          // Remove the current inliers in the model
          nr_points_left = sac->removeInliers ();
          nr_models++;
        }
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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      computeConcaveHull (PointCloud *points, vector<int> *indices, vector<double> *coeff, 
                          vector<vector<int> > *neighbors, Polygon3D &poly)
    {
      Eigen::Vector3d u, v;
      cloud_geometry::getCoordinateSystemOnPlane (coeff, u, v);
      
      vector<int> inliers (indices->size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        vector<int> *point_neighbors = &neighbors->at (i);
        if (cloud_geometry::nearest::isBoundaryPoint (points, indices->at (i), point_neighbors, u, v, boundary_angle_threshold_))
        {
          inliers[nr_p] = indices->at (i);
          nr_p++;
        }
      }
      inliers.resize (nr_p);
    }
    

};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  DoorHandleDetector p;

  door_handle_detector::Door::request req;
  req.frame_p1.x = 1.2; req.frame_p1.y = 0.6; req.frame_p1.z = 0;
  req.frame_p2.x = 1.4; req.frame_p2.y = -0.5; req.frame_p2.z = 0;
  door_handle_detector::Door::response resp;
  ros::service::call ("door_handle_detector", req, resp);

//  p.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */

