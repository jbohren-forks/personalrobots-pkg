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

@b planar_fit attempts to fit the best plane to a given PointCloud message, using several different techniques.
This node should be used for testing different planar segmentation strategies.

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/PointStamped.h>

#include <tf/transform_listener.h>

// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/rransac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/rmsac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>

using namespace std;
using namespace robot_msgs;

class PlanarFit
{
  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_down_, cloud_plane_, cloud_outliers_;

    tf::TransformListener tf_;

    // Kd-tree stuff
    cloud_kdtree::KdTree *kdtree_;
    vector<vector<int> > points_indices_;

    // Parameters
    double radius_;
    int k_;

    // Additional downsampling parameters
    int downsample_;
    Point leaf_width_;

    vector<cloud_geometry::Leaf> leaves_;

    // Do not use normals (0), use radius search (0) or use K-nearest (1) for point neighbors
    int radius_or_knn_;

    // If normals_fidelity_, then use the original cloud data to estimate the k-neighborhood and thus the normals
    int normals_fidelity_;

    double sac_distance_threshold_;
    int sac_maximum_iterations_;

    // Euclidean clustering
    int use_clustering_;      // use clustering (1) or not (0)
    double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
    int euclidean_cluster_min_pts_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlanarFit (ros::Node& anode) : node_ (anode),  tf_ (anode)
    {
      node_.param ("~search_radius_or_knn", radius_or_knn_, 1);   // none (0), radius (1) or k (2) nearest neighbor search
      node_.param ("~search_radius", radius_, 0.01);              // 1cm radius by default
      node_.param ("~search_k_closest", k_, 20);                  // 20 k-neighbors by default

      node_.param ("~downsample", downsample_, 1);                        // downsample cloud before normal estimation
      node_.param ("~downsample_leaf_width_x", leaf_width_.x, 0.02);      // 2cm radius by default
      node_.param ("~downsample_leaf_width_y", leaf_width_.y, 0.02);      // 2cm radius by default
      node_.param ("~downsample_leaf_width_z", leaf_width_.z, 0.02);      // 2cm radius by default

      node_.param ("~normals_high_fidelity", normals_fidelity_, 1);       // compute the downsampled normals from the original data

      node_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.02);   // 2 cm threshold
      node_.param ("~sac_maximum_iterations", sac_maximum_iterations_, 500);    // maximum 500 SAC iterations

      node_.param ("~use_clustering", use_clustering_, 0);
      node_.param ("~euclidean_cluster_angle_tolerance", euclidean_cluster_angle_tolerance_, 15.0);
      euclidean_cluster_angle_tolerance_ = angles::from_degrees (euclidean_cluster_angle_tolerance_);
      node_.param ("~euclidean_cluster_min_pts", euclidean_cluster_min_pts_, 500);
      node_.param ("~euclidean_cluster_distance_tolerance", euclidean_cluster_distance_tolerance_, 0.05);

      string cloud_topic ("tilt_laser_cloud");

      vector<pair<string, string> > t_list;
      node_.getPublishedTopics (&t_list);
      bool topic_found = false;
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (node_.mapName (cloud_topic)) != string::npos)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", node_.mapName (cloud_topic).c_str ());

      node_.subscribe (cloud_topic, cloud_, &PlanarFit::cloud_cb, this, 1);
      node_.advertise<PointCloud> ("~normals", 1);
      node_.advertise<PointCloud> ("~plane", 1);
      node_.advertise<PointCloud> ("~outliers", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PlanarFit () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the view point from where the scans were taken in the incoming PointCloud message frame
      * \param cloud_frame the point cloud message TF frame
      * \param viewpoint_cloud the resultant view point in the incoming cloud frame
      * \param tf a pointer to a TransformListener object
      */
    void
      getCloudViewPoint (const string &cloud_frame, PointStamped &viewpoint_cloud, tf::TransformListener &tf)
    {
      // Figure out the viewpoint value in the point cloud frame
      PointStamped viewpoint_laser;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0, 0, 0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf.transformPoint (cloud_frame, viewpoint_laser, viewpoint_cloud);
        ROS_INFO ("Cloud view point in frame %s is: %g, %g, %g.", cloud_frame.c_str (),
                  viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
      }
      catch (...)
      {
        // Default to 0.05, 0, 0.942768 (base_link, ~base_footprint)
        viewpoint_cloud.point.x = 0.05; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.942768;
        ROS_WARN ("Could not transform a point from frame %s to frame %s! Defaulting to <%f, %f, %f>",
                  viewpoint_laser.header.frame_id.c_str (), cloud_frame.c_str (),
                  viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (node_.hasParam ("~search_radius_or_knn")) node_.getParam ("~search_radius_or_knn", radius_or_knn_);
      if (node_.hasParam ("~search_radius")) node_.getParam ("~search_radius", radius_);
      if (node_.hasParam ("~search_k_closest")) node_.getParam ("~search_k_closest", k_);

      if (node_.hasParam ("~downsample")) node_.getParam ("~downsample", downsample_);
      if (node_.hasParam ("~downsample_leaf_width_x")) node_.getParam ("~downsample_leaf_width_x", leaf_width_.x);
      if (node_.hasParam ("~downsample_leaf_width_y")) node_.getParam ("~downsample_leaf_width_y", leaf_width_.y);
      if (node_.hasParam ("~downsample_leaf_width_z")) node_.getParam ("~downsample_leaf_width_z", leaf_width_.z);

      if (node_.hasParam ("~normals_high_fidelity")) node_.getParam ("~normals_high_fidelity", normals_fidelity_);

      if (node_.hasParam ("~sac_distance_threshold")) node_.getParam ("~sac_distance_threshold", sac_distance_threshold_);
      if (node_.hasParam ("~sac_maximum_iterations")) node_.getParam ("~sac_maximum_iterations", sac_maximum_iterations_);

      if (node_.hasParam ("~use_clustering")) node_.getParam ("~use_clustering", use_clustering_);
      if (node_.hasParam ("~euclidean_cluster_min_pts")) node_.getParam ("~euclidean_cluster_min_pts", euclidean_cluster_min_pts_);
      if (node_.hasParam ("~euclidean_cluster_distance_tolerance")) node_.getParam ("~euclidean_cluster_distance_tolerance", euclidean_cluster_distance_tolerance_);
      if (node_.hasParam ("~euclidean_cluster_angle_tolerance"))
      {
        node_.getParam ("~euclidean_cluster_angle_tolerance", euclidean_cluster_angle_tolerance_);
        euclidean_cluster_angle_tolerance_ = angles::from_degrees (euclidean_cluster_angle_tolerance_);
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      updateParametersFromServer ();

      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_.pts.size (), cloud_.header.frame_id.c_str (),
                (int)cloud_.chan.size (), cloud_geometry::getAvailableChannels (cloud_).c_str ());
      if (cloud_.pts.size () == 0)
      {
        ROS_ERROR ("No data points found. Exiting...");
        return;
      }

      cloud_down_.header = cloud_.header;

      ros::Time ts = ros::Time::now ();
      // Figure out the viewpoint value in the cloud_frame frame
      PointStamped viewpoint_cloud;
      getCloudViewPoint (cloud_.header.frame_id, viewpoint_cloud, tf_);

      // ---------------------------------------------------------------------------------------------------------------
      // ---[ Downsample and/or estimate point normals
      if (downsample_ != 0)
      {
        int d_idx = cloud_geometry::getChannelIndex (cloud_, "distances");
        ros::Time ts1 = ros::Time::now ();
        try
        {
          // We sacrifice functionality for speed. Use a fixed 3D grid to downsample the data instead of an octree structure
          cloud_geometry::downsamplePointCloud (cloud_, cloud_down_, leaf_width_, leaves_, d_idx, -1);  // -1 means use all data
        }
        catch (std::bad_alloc)
        {
          ROS_ERROR ("Not enough memory to create a simple downsampled representation. Change the downsample_leaf_width parameters.");
          return;
        }
        ROS_INFO ("Downsampling enabled. Number of points left: %d / %d in %g seconds.", (int)cloud_down_.pts.size (), (int)cloud_.pts.size (), (ros::Time::now () - ts1).toSec ());

        if (radius_or_knn_ == 1)             // Use a radius search
        {
          if (normals_fidelity_)
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, cloud_, radius_, viewpoint_cloud);  // Estimate point normals in the original point cloud using a fixed radius search
          else
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, radius_, viewpoint_cloud);          // Estimate point normals in the downsampled point cloud using a fixed radius search
        }
        else if (radius_or_knn_ == 2)        // Use a k-nearest neighbors search
        {
          if (normals_fidelity_)
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, cloud_, k_, viewpoint_cloud);       // Estimate point normals in the original point cloud using a K-nearest search
          else
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, k_, viewpoint_cloud);               // Estimate point normals in the downsampled point cloud using a K-nearest search
        }
        node_.publish ("~normals", cloud_down_);
      } // if (downsample)
      else
      {
        if (radius_or_knn_ == 1)             // Use a radius search
          cloud_geometry::nearest::computePointCloudNormals (cloud_, radius_, viewpoint_cloud);    // Estimate point normals using a fixed radius search
        else if (radius_or_knn_ == 2)        // Use a k-nearest neighbors search
          cloud_geometry::nearest::computePointCloudNormals (cloud_, k_, viewpoint_cloud);         // Estimate point normals using a K-nearest search
        node_.publish ("~normals", cloud_);
      }
      ROS_INFO ("Normals estimated in %g seconds.", (ros::Time::now () - ts).toSec ());


      // ---------------------------------------------------------------------------------------------------------------
      // ---[ Fit a planar model
      vector<int> inliers;
      vector<double> coeff;
      int total_nr_points;
      ts = ros::Time::now ();
      if (downsample_ != 0)
      {
        total_nr_points = cloud_down_.pts.size ();

        // Break into clusters
        if (use_clustering_)
        {
          ros::Time ts1 = ros::Time::now ();
          vector<vector<int> > clusters;

          if (radius_or_knn_ != 0)                  // did we estimate normals ?
            findClusters (cloud_down_, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          else                                      // if not, set nx_idx to -1 and perform a pure Euclidean clustering
            findClusters (cloud_down_, euclidean_cluster_distance_tolerance_, clusters, -1, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          ROS_INFO ("Clustering done. Number of clusters extracted: %d in %g seconds.", (int)clusters.size (), (ros::Time::now () - ts1).toSec ());

          ts = ros::Time::now ();
          fitSACPlane (&cloud_down_, clusters[clusters.size () - 1], inliers, coeff, viewpoint_cloud, sac_distance_threshold_);    // Fit the plane model
        }
        else
          fitSACPlane (&cloud_down_, inliers, coeff, viewpoint_cloud, sac_distance_threshold_);    // Fit the plane model

        cloud_geometry::getPointCloud (cloud_down_, inliers, cloud_plane_);
        cloud_geometry::getPointCloudOutside (cloud_down_, inliers, cloud_outliers_);
      }
      else
      {
        total_nr_points = cloud_.pts.size ();

        // Break into clusters
        if (use_clustering_)
        {
          ros::Time ts1 = ros::Time::now ();
          vector<vector<int> > clusters;

          if (radius_or_knn_ != 0)                  // did we estimate normals ?
            findClusters (cloud_, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          else                                      // if not, set nx_idx to -1 and perform a pure Euclidean clustering
            findClusters (cloud_, euclidean_cluster_distance_tolerance_, clusters, -1, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          ROS_INFO ("Clustering done. Number of clusters extracted: %d in %g seconds.", (int)clusters.size (), (ros::Time::now () - ts1).toSec ());

          ts = ros::Time::now ();
          fitSACPlane (&cloud_, clusters[clusters.size () - 1], inliers, coeff, viewpoint_cloud, sac_distance_threshold_);    // Fit the plane model
        }
        else
          fitSACPlane (&cloud_, inliers, coeff, viewpoint_cloud, sac_distance_threshold_);       // Fit the plane model

        cloud_geometry::getPointCloud (cloud_, inliers, cloud_plane_);
        cloud_geometry::getPointCloudOutside (cloud_, inliers, cloud_outliers_);
      }
      ROS_INFO ("Planar model found with %d / %d inliers in %g seconds.\n", (int)inliers.size (), total_nr_points, (ros::Time::now () - ts).toSec ());

      node_.publish ("~plane", cloud_plane_);
      node_.publish ("~outliers", cloud_outliers_);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    bool
      fitSACPlane (PointCloud *points, vector<int> &inliers, vector<double> &coeff,
                   const robot_msgs::PointStamped &viewpoint_cloud, double dist_thresh)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();

      //      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
      //      sample_consensus::SAC *sac             = new sample_consensus::RRANSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RRANSAC*>(sac)->setFractionNrPretest (10);
      //      sample_consensus::SAC *sac             = new sample_consensus::RMSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RMSAC*>(sac)->setFractionNrPretest (10);

      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, dist_thresh);
      sac->setMaxIterations (sac_maximum_iterations_);
      model->setDataSet (points);

      // Search for the best plane
      if (sac->computeModel (0))
      {
        sac->computeCoefficients ();          // Compute the model coefficients
        coeff   = sac->refineCoefficients (); // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->pts.at (inliers[0]), viewpoint_cloud);

        //ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
        //           coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (false);
      }

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param indices a subset of point indices to use
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    bool
      fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   const robot_msgs::PointStamped &viewpoint_cloud, double dist_thresh)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();

      //      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
      //      sample_consensus::SAC *sac             = new sample_consensus::RRANSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RRANSAC*>(sac)->setFractionNrPretest (10);
      //      sample_consensus::SAC *sac             = new sample_consensus::RMSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RMSAC*>(sac)->setFractionNrPretest (10);

      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, dist_thresh);
      sac->setMaxIterations (sac_maximum_iterations_);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel (0))
      {
        sac->computeCoefficients ();          // Compute the model coefficients
        coeff   = sac->refineCoefficients (); // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->pts.at (inliers[0]), viewpoint_cloud);

        //ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
        //           coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (false);
      }

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
      * angular deviation
      * \NOTE: assumes normalized point normals !
      * \param points pointer to the point cloud message
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param nx_idx the index of the channel containing the X component of the normal
      * \param ny_idx the index of the channel containing the Y component of the normal
      * \param nz_idx the index of the channel containing the Z component of the normal
      * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud &points, double tolerance, vector<vector<int> > &clusters,
                    int nx_idx, int ny_idx, int nz_idx,
                    double eps_angle, unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points);

      int nr_points = points.pts.size ();
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
              double dot_p = points.chan[nx_idx].vals[i] * points.chan[nx_idx].vals[nn_indices[j]] +
                             points.chan[ny_idx].vals[i] * points.chan[ny_idx].vals[nn_indices[j]] +
                             points.chan[nz_idx].vals[i] * points.chan[nz_idx].vals[nn_indices[j]];
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
            r[j] = seed_queue[j];

          // Remove duplicates
          sort (r.begin (), r.end ());
          r.erase (unique (r.begin (), r.end ()), r.end ());

          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("planar_fit");

  PlanarFit p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

