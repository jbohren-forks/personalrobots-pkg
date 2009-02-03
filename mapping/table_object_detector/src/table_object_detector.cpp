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

class TableObjectDetector : public ros::Node
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

    int min_points_per_object_;
    double object_cluster_tolerance_;

    bool need_cloud_data_, publish_debug_;

    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;

    double table_min_height_, table_max_height_, delta_z_, object_min_distance_from_table_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TableObjectDetector () : ros::Node ("table_object_detector"), tf_(*this)
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

      param ("~clusters_growing_tolerance", clusters_growing_tolerance_, 0.5);   // 0.5 m
      param ("~clusters_min_pts", clusters_min_pts_, 10);                        // 10 points

      param ("~object_cluster_tolerance", object_cluster_tolerance_, 0.05);   // 5cm between two objects
      param ("~min_points_per_object", min_points_per_object_, 30);           // 30 points per object cluster

      param ("~table_min_height", table_min_height_, 0.5);              // minimum height of a table : 0.5m
      param ("~table_max_height", table_max_height_, 1.5);              // maximum height of a table : 1.5m
      param ("~table_delta_z", delta_z_, 0.03);                         // consider objects starting at 3cm from the table
      param ("~object_min_distance_from_table", object_min_distance_from_table_, 0.10); // objects which have their support more 10cm from the table will not be considered
      ROS_DEBUG ("Using the following thresholds for table detection [min / max height]: %f / %f.", table_min_height_, table_max_height_);

      param ("~publish_debug", publish_debug_, true);

      param ("~input_cloud_topic", input_cloud_topic_, string ("full_cloud"));
      advertiseService("table_object_detector", &TableObjectDetector::detectTable, this);

      // This should be set to whatever the leaf_width factor is in the downsampler
      param ("~sac_distance_threshold", sac_distance_threshold_, 0.03);     // 5 cm

      if (publish_debug_)
      {
        advertise<PolygonalMap> ("semantic_polygonal_map", 1);
        advertise<PointCloud> ("cloud_annotated", 1);

        cloud_annotated_.chan.resize (1);
        cloud_annotated_.chan[0].name = "rgb";
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~TableObjectDetector () { }

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
    bool
      detectTable (FindTable::Request &req, FindTable::Response &resp)
    {
      timeval t1, t2;
      double time_spent;

      updateParametersFromServer ();

      // Subscribe to a point cloud topic
      need_cloud_data_ = true;
      subscribe (input_cloud_topic_.c_str (), cloud_in_, &TableObjectDetector::cloud_cb, 1);

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
        cloud_geometry::downsamplePointCloud (&cloud_in_, &indices_in_bounds, cloud_down_, leaf_width_, leaves, -1);
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }

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

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (&cloud_down_, 0, 1, 2, eps_angle_, z_axis_, indices_z);

/*      indices_z.resize (cloud_down_.pts.size ());
      for (unsigned int i = 0; i < cloud_down_.pts.size (); i++)
      {
        indices_z[i] = i;
      }
      
        cloud_annotated_.header = cloud_down_.header;
        cloud_annotated_.pts.resize (indices_z.size ());
        cloud_annotated_.chan[0].vals.resize (indices_z.size ());
        for (unsigned int i = 0; i < indices_z.size (); i++)
        {
          cloud_annotated_.pts[i] = cloud_down_.pts.at (indices_z[i]);
          cloud_annotated_.chan[0].vals[i] = cloud_down_.chan[0].vals.at (indices_z[i]);
        }
        publish ("cloud_annotated", cloud_annotated_);

        return (true);*/
      ROS_DEBUG ("Number of points with normals parallel to Z: %d.", indices_z.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      findClusters (&cloud_down_, &indices_z, clusters_growing_tolerance_, clusters, 0, 1, 2, clusters_min_pts_);

      sort (clusters.begin (), clusters.end (), compareRegions);

      vector<int> inliers;
      vector<double> coeff, z_coeff (3);
      z_coeff[0] = z_axis_.x; z_coeff[1] = z_axis_.y; z_coeff[2] = z_axis_.z;
      int c_good = -1;
      for (int i = clusters.size () - 1; i >= 0; i--)
      {
        // Find the best plane in this cluster
        fitSACPlane (&cloud_down_, &clusters[i], inliers, coeff);
        double angle = cloud_geometry::transforms::getAngleBetweenPlanes (coeff, z_coeff) * 180.0 / M_PI;
        if ( fabs (angle) < (eps_angle_  * 180.0 / M_PI) || fabs (180.0 - angle) < (eps_angle_  * 180.0 / M_PI) )
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
      ROS_DEBUG ("Number of clusters found: %d, largest cluster: %d.", clusters.size (), clusters[c_good].size ());

      // Get the table bounds
      std_msgs::Point32 minP, maxP;
      cloud_geometry::getMinMax (&cloud_down_, &inliers, minP, maxP);
      resp.table.min_x = minP.x; resp.table.min_y = minP.y;
      resp.table.max_x = maxP.x; resp.table.max_y = maxP.y;
      
      // Get the goal position for the robot base
//      resp.base_target_pose.x  = ;
//      resp.base_target_pose.y  = ;
//      resp.base_target_pose.th = ;

      // Compute the convex hull
      pmap_.header = cloud_down_.header;
      pmap_.polygons.resize (1);
      cloud_geometry::areas::convexHull2D (&cloud_down_, &inliers, &coeff, pmap_.polygons[0]);

      // Find the object clusters supported by the table
      inliers.clear ();
      findObjectClusters (&cloud_in_, &coeff, &pmap_.polygons[0], &minP, &maxP, inliers, resp.table);

      // Reserve enough space
      if (publish_debug_)
      {
        cloud_annotated_.header = cloud_down_.header;
        cloud_annotated_.pts.resize (inliers.size ());
        cloud_annotated_.chan[0].vals.resize (inliers.size ());
        for (unsigned int i = 0; i < inliers.size (); i++)
        {
          cloud_annotated_.pts[i] = cloud_in_.pts.at (inliers[i]);
          cloud_annotated_.chan[0].vals[i] = cloud_in_.chan[0].vals.at (inliers[i]);
/*          cloud_annotated_.pts[i] = cloud_down_.pts.at (inliers[i]);
          cloud_annotated_.chan[0].vals[i] = cloud_down_.chan[0].vals.at (inliers[i]);*/
        }
        publish ("cloud_annotated", cloud_annotated_);
        publish ("semantic_polygonal_map", pmap_);
      }

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Table found. Bounds: [%f, %f] -> [%f, %f]. Number of objects: %d. Total time: %f.",
                resp.table.min_x, resp.table.min_y, resp.table.max_x, resp.table.max_y, resp.table.objects.size (), time_spent);
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      findObjectClusters (PointCloud *points, vector<double> *coeff, Polygon3D *poly, Point32 *minP, Point32 *maxP,
                          vector<int> &object_indices, Table &table)
    {
      int nr_p = 0;
      Point32 pt;
      object_indices.resize (points->pts.size ());
      for (unsigned int i = 0; i < points->pts.size (); i++)
      {
        // Select all the points in the given bounds
        if ( points->pts.at (i).x > minP->x &&
             points->pts.at (i).x < maxP->x &&
             points->pts.at (i).y > minP->y &&
             points->pts.at (i).y < maxP->y &&
             points->pts.at (i).z > (maxP->z + delta_z_)
           )
        {
          // Calculate the distance from the point to the plane
          double distance_to_plane = coeff->at (0) * points->pts.at (i).x +
                                     coeff->at (1) * points->pts.at (i).y +
                                     coeff->at (2) * points->pts.at (i).z +
                                     coeff->at (3) * 1;
          // Calculate the projection of the point on the plane
          pt.x = points->pts.at (i).x - distance_to_plane * coeff->at (0);
          pt.y = points->pts.at (i).y - distance_to_plane * coeff->at (1);
          pt.z = points->pts.at (i).z - distance_to_plane * coeff->at (2);

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
      findClusters (points, &object_indices, object_cluster_tolerance_, object_clusters, min_points_per_object_);

      std_msgs::Point32 minPCluster, maxPCluster;
      table.objects.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::getMinMax (points, &object_idx, minPCluster, maxPCluster);
        if (minPCluster.z > (maxP->z + object_min_distance_from_table_) )
            continue;

        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          object_indices[nr_p] = object_idx.at (j);
          nr_p++;
        }
        cloud_geometry::getMinMax (points, &object_idx, table.objects[i].min_bound, table.objects[i].max_bound);
        cloud_geometry::nearest::computeCentroid (points, &object_idx, table.objects[i].center);
      }
      object_indices.resize (nr_p);
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
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points
      * \param points pointer to the point cloud message
      * \param indices pointer to a list of point indices
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud *points, vector<int> *indices, double tolerance, vector<vector<int> > &clusters,
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

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance);
          tree->getNeighborsIndices (nn_indices);

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

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      spin ()
    {
      ros::Duration tictoc (0, 10000000);
      while (ok ())
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

  TableObjectDetector p;
  p.spin ();

  

  return (0);
}
/* ]--- */

