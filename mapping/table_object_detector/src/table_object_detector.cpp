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
 * $Id$
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b table_object_detector detects tables and objects.

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>
#include <tf/message_notifier.h>
#include <angles/angles.h>

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <sys/time.h>

#include <robot_srvs/FindTable.h>
#include <robot_msgs/Table.h>
#include <robot_msgs/ObjectOnTable.h>

using namespace std;
using namespace std_msgs;
using namespace robot_msgs;
using namespace robot_srvs;

// Comparison operator for a vector of vectors
bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class TableObjectDetector
{
  protected:
    ros::Node& node_;
  public:

    // ROS messages
    PointCloud cloud_in_, cloud_down_;
    Point leaf_width_;
    PointCloud cloud_annotated_;
    Point32 z_axis_;
    PolygonalMap pmap_;

    tf::TransformListener tf_;

    // Parameters
    string global_frame_;
    double frame_distance_eps_, min_z_bounds_, max_z_bounds_;
    string input_cloud_topic_;
    int k_;
    double clusters_growing_tolerance_;
    int clusters_min_pts_;

    int min_points_per_object_;
    double object_cluster_tolerance_;

    bool need_cloud_data_, publish_debug_;

    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;

    double table_min_height_, table_max_height_, delta_z_, object_min_distance_from_table_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TableObjectDetector (ros::Node& anode) : node_ (anode), tf_(anode)
    {
      node_.param ("/global_frame_id", global_frame_, std::string("/base_link"));
      node_.param ("~frame_distance_eps", frame_distance_eps_, 0.2);          // Allow 20cm extra space by default

      node_.param ("~min_z_bounds", min_z_bounds_, 0.0);                      // restrict the Z dimension between 0
      node_.param ("~max_z_bounds", max_z_bounds_, 3.0);                      // and 3.0 m

      node_.param ("~downsample_leaf_width_x", leaf_width_.x, 0.03);          // 3cm radius by default
      node_.param ("~downsample_leaf_width_y", leaf_width_.y, 0.03);          // 3cm radius by default
      node_.param ("~downsample_leaf_width_z", leaf_width_.z, 0.03);          // 3cm radius by default
      node_.param ("~search_k_closest", k_, 10);                              // 10 k-neighbors by default

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;
      node_.param ("~normal_eps_angle_", eps_angle_, 15.0);                   // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);                         // convert to radians

      node_.param ("~region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = angles::from_degrees (region_angle_threshold_); // convert to radians

      node_.param ("~clusters_growing_tolerance", clusters_growing_tolerance_, 0.5);   // 0.5 m
      node_.param ("~clusters_min_pts", clusters_min_pts_, 10);                        // 10 points

      node_.param ("~object_cluster_tolerance", object_cluster_tolerance_, 0.05);   // 5cm between two objects
      node_.param ("~min_points_per_object", min_points_per_object_, 30);           // 30 points per object cluster

      node_.param ("~table_min_height", table_min_height_, 0.5);              // minimum height of a table : 0.5m
      node_.param ("~table_max_height", table_max_height_, 1.5);              // maximum height of a table : 1.5m
      node_.param ("~table_delta_z", delta_z_, 0.03);                         // consider objects starting at 3cm from the table
      node_.param ("~object_min_distance_from_table", object_min_distance_from_table_, 0.10); // objects which have their support more 10cm from the table will not be considered
      ROS_DEBUG ("Using the following thresholds for table detection [min / max height]: %f / %f.", table_min_height_, table_max_height_);

      node_.param ("~publish_debug", publish_debug_, true);

      node_.param ("~input_cloud_topic", input_cloud_topic_, string ("tilt_laser_cloud"));
      node_.advertiseService("table_object_detector", &TableObjectDetector::detectTable, this);

      // This should be set to whatever the leaf_width factor is in the downsampler
      node_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.03);     // 5 cm

      if (publish_debug_)
      {
        node_.advertise<PolygonalMap> ("semantic_polygonal_map", 1);
        node_.advertise<PointCloud> ("cloud_annotated", 1);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (node_.hasParam ("~frame_distance_eps"))
        node_.getParam ("~frame_distance_eps", frame_distance_eps_);
      if (node_.hasParam ("~input_cloud_topic"))
        node_.getParam ("~input_cloud_topic", input_cloud_topic_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 24-bit RGB coded value from 3 independent <r, g, b> channel values
      * \param r the red channel value
      * \param g the green channel value
      * \param b the blue channel value
      */
    inline double
      getRGB (float r, float g, float b)
    {
      int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
      double rgb = *(float*)(&res);
      return (rgb);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detectTable (FindTable::Request &req, FindTable::Response &resp)
    {
      updateParametersFromServer ();

      // Subscribe to a point cloud topic
      need_cloud_data_ = true;
      tf::MessageNotifier<robot_msgs::PointCloud> _pointcloudnotifier (&tf_, ros::Node::instance (),
                                                                       boost::bind (&TableObjectDetector::cloud_cb, this, _1),
                                                                       input_cloud_topic_, global_frame_, 50);
      //node_.subscribe (input_cloud_topic_, cloud_in_, &TableObjectDetector::cloud_cb, this, 1);

      // Wait until the scan is ready, sleep for 10ms
      ros::Duration tictoc (0, 10000000);
      while (need_cloud_data_)
      {
        tictoc.sleep ();
      }
      // Unsubscribe from the point cloud topic
      node_.unsubscribe (input_cloud_topic_.c_str ()) ;

      ros::Time ts = ros::Time::now ();
      // We have a pointcloud, estimate the true point bounds
      vector<int> indices_in_bounds (cloud_in_.pts.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < cloud_in_.pts.size (); i++)
      {
        if (cloud_in_.pts[i].z >= table_min_height_ && cloud_in_.pts[i].z <= table_max_height_)
        {
          indices_in_bounds[nr_p] = i;
          nr_p++;
        }
      }
      indices_in_bounds.resize (nr_p);

      // Downsample the cloud in the bounding box for faster processing
      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        cloud_geometry::downsamplePointCloud (cloud_in_, indices_in_bounds, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }

      ROS_DEBUG ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_.x, leaf_width_.y, leaf_width_.z, (int)cloud_down_.pts.size ());

      // Reserve space for 3 channels: nx, ny, nz
      cloud_down_.chan.resize (3);
      cloud_down_.chan[0].name = "nx";
      cloud_down_.chan[1].name = "ny";
      cloud_down_.chan[2].name = "nz";
      for (unsigned int d = 0; d < cloud_down_.chan.size (); d++)
        cloud_down_.chan[d].vals.resize (cloud_down_.pts.size ());

      // Create Kd-Tree
      estimatePointNormals (cloud_down_);

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_, z_axis_, indices_z);
      ROS_DEBUG ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      findClusters (cloud_down_, indices_z, clusters_growing_tolerance_, clusters, 0, 1, 2, region_angle_threshold_, clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);

      vector<int> inliers;
      vector<double> coeff, z_coeff (3);
      z_coeff[0] = z_axis_.x; z_coeff[1] = z_axis_.y; z_coeff[2] = z_axis_.z;
      int c_good = -1;
      double eps_angle_deg = angles::to_degrees (eps_angle_);
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        // Find the best plane in this cluster
        fitSACPlane (&cloud_down_, &clusters[i], inliers, coeff);
        double angle = angles::to_degrees (cloud_geometry::angles::getAngleBetweenPlanes (coeff, z_coeff));
        if ( fabs (angle) < eps_angle_deg || fabs (180.0 - angle) < eps_angle_deg )
        {
          c_good = i;
          break;
        }
      }

      if (c_good == -1)
      {
        ROS_WARN ("No table found");
        return (false);
      }
      ROS_INFO ("Number of clusters found: %d, largest cluster: %d.", (int)clusters.size (), (int)clusters[c_good].size ());

      // Fill in the header
      resp.table.header.frame_id = global_frame_;
      resp.table.header.stamp = cloud_in_.header.stamp;

      // Get the table bounds
      robot_msgs::Point32 minP, maxP;
      cloud_geometry::statistics::getMinMax (cloud_down_, inliers, minP, maxP);
      // Transform to the global frame
      PointStamped minPstamped_local, maxPstamped_local;
      minPstamped_local.point.x = minP.x;
      minPstamped_local.point.y = minP.y;
      minPstamped_local.header = cloud_in_.header;
      maxPstamped_local.point.x = maxP.x;
      maxPstamped_local.point.y = maxP.y;
      maxPstamped_local.header = cloud_in_.header;
      PointStamped minPstamped_global, maxPstamped_global;
      try
      {
        tf_.transformPoint (global_frame_, minPstamped_local, minPstamped_global);
        tf_.transformPoint (global_frame_, maxPstamped_local, maxPstamped_global);
        resp.table.table_min.x = minPstamped_global.point.x;
        resp.table.table_min.y = minPstamped_global.point.y;
        resp.table.table_max.x = maxPstamped_global.point.x;
        resp.table.table_max.y = maxPstamped_global.point.y;
      }
      catch (tf::TransformException)
      {
        ROS_ERROR ("Failed to transform table bounds from frame %s to frame %s",
                   cloud_in_.header.frame_id.c_str(), global_frame_.c_str());
        return (false);
      }

      // Compute the convex hull
      pmap_.header.stamp = cloud_down_.header.stamp;
      pmap_.header.frame_id = global_frame_;
      pmap_.polygons.resize (1);
      cloud_geometry::areas::convexHull2D (cloud_down_, inliers, coeff, pmap_.polygons[0]);

      // Find the object clusters supported by the table
      inliers.clear ();
      findObjectClusters (cloud_in_, coeff, pmap_.polygons[0], minP, maxP, inliers, resp.table);

      // Transform into the global frame
      try
      {
        PointStamped local, global;
        local.header = cloud_down_.header;
        for (unsigned int i = 0; i < pmap_.polygons.size (); i++)
        {
          for (unsigned int j = 0; j < pmap_.polygons[i].points.size (); j++)
          {
            local.point.x = pmap_.polygons[i].points[j].x;
            local.point.y = pmap_.polygons[i].points[j].y;
            tf_.transformPoint (global_frame_, local, global);
            pmap_.polygons[i].points[j].x = global.point.x;
            pmap_.polygons[i].points[j].y = global.point.y;
          }
        }
      }
      catch (tf::TransformException)
      {
        ROS_ERROR ("Failed to PolygonalMap from frame %s to frame %s", cloud_down_.header.frame_id.c_str(), global_frame_.c_str());
        return (false);
      }

      resp.table.table = pmap_.polygons[0];


      ROS_INFO ("Table found. Bounds: [%f, %f] -> [%f, %f]. Number of objects: %d. Total time: %f.",
                resp.table.table_min.x, resp.table.table_min.y, resp.table.table_max.x, resp.table.table_max.y, (int)resp.table.objects.size (), (ros::Time::now () - ts).toSec ());

      // Should only used for debugging purposes (on screen visualization)
      if (publish_debug_)
      {
        // Break the object inliers into clusters in an Euclidean sense
        vector<vector<int> > objects;
        findClusters (cloud_in_, inliers, object_cluster_tolerance_, objects, -1, -1, -1, region_angle_threshold_, clusters_min_pts_);

        int total_nr_pts = 0;
        for (unsigned int i = 0; i < objects.size (); i++)
          total_nr_pts += objects[i].size ();

        cloud_annotated_.header = cloud_down_.header;
        cloud_annotated_.pts.resize (total_nr_pts);

        // Copy all the channels from the original pointcloud
        cloud_annotated_.chan.resize (cloud_in_.chan.size () + 1);
        for (unsigned int d = 0; d < cloud_in_.chan.size (); d++)
        {
          cloud_annotated_.chan[d].name = cloud_in_.chan[d].name;
          cloud_annotated_.chan[d].vals.resize (total_nr_pts);
        }
        cloud_annotated_.chan[cloud_in_.chan.size ()].name = "rgb";
        cloud_annotated_.chan[cloud_in_.chan.size ()].vals.resize (total_nr_pts);

        // For each object in the set
        int nr_p = 0;
        for (unsigned int i = 0; i < objects.size (); i++)
        {
          float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));
          // Get its points
          for (unsigned int j = 0; j < objects[i].size (); j++)
          {
            cloud_annotated_.pts[nr_p] = cloud_in_.pts.at (objects[i][j]);
            for (unsigned int d = 0; d < cloud_in_.chan.size (); d++)
              cloud_annotated_.chan[d].vals[nr_p] = cloud_in_.chan[d].vals.at (objects[i][j]);
            cloud_annotated_.chan[cloud_in_.chan.size ()].vals[nr_p] = rgb;
            nr_p++;
          }
        }
        node_.publish ("cloud_annotated", cloud_annotated_);
        node_.publish ("semantic_polygonal_map", pmap_);
      }
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      findObjectClusters (PointCloud &points, const vector<double> &coeff, const Polygon3D &poly,
                          const Point32 &minP, const Point32 &maxP,
                          vector<int> &object_indices, Table &table)
    {
      int nr_p = 0;
      Point32 pt;
      object_indices.resize (points.pts.size ());
      for (unsigned int i = 0; i < points.pts.size (); i++)
      {
        // Select all the points in the given bounds
        if ( points.pts.at (i).x > minP.x &&
             points.pts.at (i).x < maxP.x &&
             points.pts.at (i).y > minP.y &&
             points.pts.at (i).y < maxP.y &&
             points.pts.at (i).z > (maxP.z + delta_z_)
           )
        {
          // Calculate the distance from the point to the plane
          double distance_to_plane = coeff.at (0) * points.pts.at (i).x +
                                     coeff.at (1) * points.pts.at (i).y +
                                     coeff.at (2) * points.pts.at (i).z +
                                     coeff.at (3) * 1;
          // Calculate the projection of the point on the plane
          pt.x = points.pts.at (i).x - distance_to_plane * coeff.at (0);
          pt.y = points.pts.at (i).y - distance_to_plane * coeff.at (1);
          pt.z = points.pts.at (i).z - distance_to_plane * coeff.at (2);

          if (cloud_geometry::areas::isPointIn2DPolygon (pt, poly))
          {
            object_indices[nr_p] = i;
            nr_p++;
          }
        }
      }
      object_indices.resize (nr_p);

      // Find the clusters
      nr_p = 0;
      vector<vector<int> > object_clusters;
      findClusters (points, object_indices, object_cluster_tolerance_, object_clusters, min_points_per_object_);

      robot_msgs::Point32 minPCluster, maxPCluster;
      table.objects.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (points, object_idx, minPCluster, maxPCluster);
        if (minPCluster.z > (maxP.z + object_min_distance_from_table_) )
            continue;

        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          object_indices[nr_p] = object_idx.at (j);
          nr_p++;
        }
        cloud_geometry::statistics::getMinMax (points, object_idx, table.objects[i].min_bound, table.objects[i].max_bound);
        cloud_geometry::nearest::computeCentroid (points, object_idx, table.objects[i].center);
      }
      object_indices.resize (nr_p);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& pc)
    {
      cloud_in_ = *pc;
      need_cloud_data_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
      * angular deviation
      * \NOTE: assumes normalized point normals !
      * \param points pointer to the point cloud message
      * \param indices pointer to a list of point indices
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param nx_idx the index of the channel containing the X component of the normal
      * \param ny_idx the index of the channel containing the Y component of the normal
      * \param nz_idx the index of the channel containing the Z component of the normal
      * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (const PointCloud &points, const vector<int> &indices, double tolerance, vector<vector<int> > &clusters,
                    int nx_idx, int ny_idx, int nz_idx,
                    double eps_angle, unsigned int min_pts_per_cluster)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points, indices);

      int nr_points = indices.size ();
      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (nr_points, false);

      vector<int> nn_indices;
      vector<float> nn_distances;
      // Process all points in the indices vector
      for (int i = 0; i < nr_points; i++)
      {
        if (processed[i])
          continue;

        vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          // Search for sq_idx
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

          for (unsigned int j = 1; j < nn_indices.size (); j++)       // nn_indices[0] should be sq_idx
          {
            if (processed.at (nn_indices[j]))                         // Has this point been processed before ?
              continue;

            processed[nn_indices[j]] = true;
            if (nx_idx != -1)                                         // Are point normals present ?
            {
              // [-1;1]
              double dot_p = points.chan[nx_idx].vals[indices.at (i)] * points.chan[nx_idx].vals[indices.at (nn_indices[j])] +
                             points.chan[ny_idx].vals[indices.at (i)] * points.chan[ny_idx].vals[indices.at (nn_indices[j])] +
                             points.chan[nz_idx].vals[indices.at (i)] * points.chan[nz_idx].vals[indices.at (nn_indices[j])];
              if ( fabs (acos (dot_p)) < eps_angle )
              {
                processed[nn_indices[j]] = true;
                seed_queue.push_back (nn_indices[j]);
              }
            }
            // If normal information is not present, perform a simple Euclidean clustering
            else
            {
              processed[nn_indices[j]] = true;
              seed_queue.push_back (nn_indices[j]);
            }
          }

          sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster)
        {
          vector<int> r (seed_queue.size ());
          for (unsigned int j = 0; j < seed_queue.size (); j++)
            r[j] = indices.at (seed_queue[j]);

          sort (r.begin (), r.end ());
          r.erase (unique (r.begin (), r.end ()), r.end ());

          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
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
      findClusters (const PointCloud &points, const vector<int> &indices, double tolerance, vector<vector<int> > &clusters,
                    unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points, indices);

      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (indices.size (), false);

      vector<int> nn_indices;
      vector<float> nn_distances;
      // Process all points in the indices vector
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        if (processed[i])
          continue;

        vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

          for (unsigned int j = 1; j < nn_indices.size (); j++)
          {
            if (!processed.at (nn_indices[j]))
            {
              processed[nn_indices[j]] = true;
              seed_queue.push_back (nn_indices[j]);
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
            r[j] = indices.at (seed_queue[j]);
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
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
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
        sac->computeCoefficients ();          // Compute the model coefficients
        coeff   = sac->refineCoefficients (); // Refine them using least-squares
        model->selectWithinDistance (coeff, sac_distance_threshold_, inliers);

        //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
        //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      estimatePointNormals (PointCloud &cloud)
    {
      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (cloud);
      vector<vector<int> > points_k_indices;
      // Allocate enough space for point indices
      points_k_indices.resize (cloud.pts.size ());
      for (int i = 0; i < (int)cloud.pts.size (); i++)
        points_k_indices[i].resize (k_);
      // Get the nerest neighbors for all the point indices in the bounds
      vector<float> distances;
      for (int i = 0; i < (int)cloud.pts.size (); i++)
        kdtree->nearestKSearch (i, k_, points_k_indices[i], distances);

      // Figure out the viewpoint value in the point cloud frame
      PointStamped viewpoint_laser, viewpoint_cloud;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0,0,0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf_.transformPoint (cloud.header.frame_id, viewpoint_laser, viewpoint_cloud);
      }
      catch (tf::TransformException)
      {
        viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
      }

#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)cloud.pts.size (); i++)
      {
        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        Eigen::Vector4d plane_parameters;
        double curvature;
        cloud_geometry::nearest::computePointNormal (cloud, points_k_indices[i], plane_parameters, curvature);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud_down_.pts[i], viewpoint_cloud);

        cloud.chan[0].vals[i] = plane_parameters (0);
        cloud.chan[1].vals[i] = plane_parameters (1);
        cloud.chan[2].vals[i] = plane_parameters (2);
      }
      // Delete the kd-tree
      delete kdtree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      spin ()
    {
      ros::Duration tictoc (0, 10000000);
      while (node_.ok ())
      {
        tictoc.sleep ();

/*        FindTable::Request req;
        FindTable::Response resp;
        ros::service::call ("table_object_detector", req, resp);*/
      }

      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("table_object_detector");

  TableObjectDetector p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

