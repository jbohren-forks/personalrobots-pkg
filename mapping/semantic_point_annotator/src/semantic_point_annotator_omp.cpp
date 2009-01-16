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

@b semantic_point_annotator_omp annotates 3D point clouds with semantic labels.

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

using namespace std;
using namespace std_msgs;

struct Region
{
  char region_type;
  vector<int> indices;
};

class SemanticPointAnnotator : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_annotated_;
    Point32 z_axis_;

    tf::TransformListener tf_;

    // Parameters
    int sac_min_points_per_model_, sac_min_points_left_;
    double sac_distance_threshold_, eps_angle_, region_angle_threshold_;

    double rule_floor_, rule_ceiling_, rule_wall_;
    double rule_table_min_, rule_table_max_;

    double region_growing_tolerance_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SemanticPointAnnotator () : ros::Node ("semantic_point_annotator_omp"), tf_(*this)
    {
      param ("~rule_floor", rule_floor_, 0.2);          // Rule for FLOOR
      param ("~rule_ceiling", rule_ceiling_, 2.0);       // Rule for CEILING
      param ("~rule_table_min", rule_table_min_, 0.5);   // Rule for MIN TABLE
      param ("~rule_table_max", rule_table_max_, 1.5);   // Rule for MIN TABLE
      param ("~rule_wall", rule_wall_, 2.0);             // Rule for WALL

      param ("~region_growing_tolerance", region_growing_tolerance_, 0.25);  // 10 cm
      
      param ("~region_angle_threshold", region_angle_threshold_, 30.0);   // Difference between normals in degrees for cluster/region growing
      region_angle_threshold_ = (region_angle_threshold_ * M_PI / 180.0); // convert to radians

      param ("~p_sac_min_points_left", sac_min_points_left_, 10);
      param ("~p_sac_min_points_per_model", sac_min_points_per_model_, 10);   // 50 points at high resolution

      // This should be set to whatever the leaf_width factor is in the downsampler
      param ("~p_sac_distance_threshold", sac_distance_threshold_, 0.06);     // 6 cm
      
      param ("~p_eps_angle_", eps_angle_, 15.0);                              // 15 degrees

      eps_angle_ = (eps_angle_ * M_PI / 180.0);           // convert to radians

      string cloud_topic ("cloud_normals");

      vector<pair<string, string> > t_list;
      getPublishedTopics (&t_list);
      bool topic_found = false;
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (cloud_topic) == string::npos)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());

      subscribe (cloud_topic.c_str (), cloud_, &SemanticPointAnnotator::cloud_cb, 1);

      advertise<PointCloud> ("cloud_annotated", 1);

      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;

      cloud_annotated_.chan.resize (3);
      //cloud_annotated_.chan[0].name = "intensities";
      cloud_annotated_.chan[0].name = "r";
      cloud_annotated_.chan[1].name = "g";
      cloud_annotated_.chan[2].name = "b";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~SemanticPointAnnotator () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points
      * \param points pointer to the point cloud message
      * \param indices pointer to a list of point indices
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param nx_idx
      * \param ny_idx
      * \param nz_idx
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud *points, vector<int> *indices, double tolerance, vector<Region> &clusters,
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
          Region r;
          //r.indices = seed_queue;
          r.indices.resize (seed_queue.size ());
          for (unsigned int j = 0; j < r.indices.size (); j++)
            r.indices[j] = indices->at (seed_queue[j]);
          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      fitSACPlane (PointCloud *points, vector<int> *indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff)
    {
      if ((int)indices->size () < sac_min_points_per_model_)
        return;

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (120);
      sac->setProbability (0.99);
      model->setDataSet (points, *indices);

      PointCloud pts (*points);
      int nr_points_left = indices->size ();
      while (nr_points_left > sac_min_points_left_)
      {
        // Search for the best plane
        if (sac->computeModel ())
        {
          // Obtain the inliers and the planar model coefficients
          if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
            break;
          inliers.push_back (sac->getInliers ());
          coeff.push_back (sac->computeCoefficients ());

          //fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
          //         coeff[coeff.size () - 1][0], coeff[coeff.size () - 1][1], coeff[coeff.size () - 1][2], coeff[coeff.size () - 1][3]);

          // Project the inliers onto the model
          model->projectPointsInPlace (sac->getInliers (), coeff[coeff.size () - 1]);

          // Remove the current inliers in the model
          nr_points_left = sac->removeInliers ();
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      getObjectClassForParallel (vector<double> *coeff, PointStamped map_origin,
                                 double &r, double &g, double &b)
    {
      // Get all planes parallel to the floor (perpendicular to Z)
      Point32 robot_origin;
      robot_origin.x = map_origin.point.x;
      robot_origin.y = map_origin.point.y;
      robot_origin.z = map_origin.point.z;

      // Compute a distance from 0,0,0 to the plane
      double distance = cloud_geometry::distances::pointToPlaneDistance (robot_origin, *coeff);

      // Test for floor
      if (distance < rule_floor_)
      {
        r = 0.6; g = 0.67; b = 0.01;
      }
      // Test for ceiling
      if (distance > rule_ceiling_)
      {
        r = 0.8; g = 0.63; b = 0.33;
      }
      // Test for tables
      if (distance > rule_table_min_ && distance < rule_table_max_)
      {
        r = 0.0; g = 1.0; b = 0.0;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      getObjectClassForPerpendicular (PointCloud *points, vector<int> *indices,
                                      double &r, double &g, double &b)
    {
      // Get the minimum and maximum bounds of the plane
      Point32 minP, maxP;
      cloud_geometry::getMinMax (points, indices, minP, maxP);
      // Test for wall
      if (maxP.z > rule_wall_)
      {
        r = rand () / (RAND_MAX + 1.0);
        g = rand () / (RAND_MAX + 1.0);
        b = rand () / (RAND_MAX + 1.0);
        r = r * .3;
        b = b * .3 + .7;
        g = g * .3;
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      PointStamped base_link_origin, map_origin;
      base_link_origin.point.x = base_link_origin.point.y = base_link_origin.point.z = 0.0;
      base_link_origin.header.frame_id = "base_link";
      base_link_origin.header.stamp = 0;

      tf_.transformPoint ("base_link", base_link_origin, map_origin);

      ROS_INFO ("Received %d data points. Current robot pose is %g, %g, %g", cloud_.pts.size (), map_origin.point.x, map_origin.point.y, map_origin.point.z);

      cloud_annotated_.header = cloud_.header;

      int nx = cloud_geometry::getChannelIndex (&cloud_, "nx");
      int ny = cloud_geometry::getChannelIndex (&cloud_, "ny");
      int nz = cloud_geometry::getChannelIndex (&cloud_, "nz");

      if ( (cloud_.chan.size () < 3) || (nx == -1) || (ny == -1) || (nz == -1) )
      {
        ROS_ERROR ("This PointCloud message does not contain normal information!");
        return;
      }

      timeval t1, t2;
      double time_spent;
      gettimeofday (&t1, NULL);

      // ---[ Select points whose normals are parallel with the Z-axis
      vector<int> indices_z;
      cloud_geometry::getPointIndicesAxisParallelNormals (&cloud_, nx, ny, nz, eps_angle_, z_axis_, indices_z);

      // ---[ Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_xy;
      cloud_geometry::getPointIndicesAxisPerpendicularNormals (&cloud_, nx, ny, nz, eps_angle_, z_axis_, indices_xy);

      vector<Region> clusters;
      // Split the Z-parallel points into clusters
      findClusters (&cloud_, &indices_z, region_growing_tolerance_, clusters, nx, ny, nz, 10);
      int z_c = clusters.size ();
      for (int i = 0; i < z_c; i++)
        clusters[i].region_type = 0;

      // Split the Z-perpendicular points into clusters
      findClusters (&cloud_, &indices_xy, region_growing_tolerance_, clusters, nx, ny, nz, 10);
      for (unsigned int i = z_c; i < clusters.size (); i++)
        clusters[i].region_type = 1;

      // Compute the total number of points in all clusters
      int total_p = 0;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
        total_p += clusters[cc].indices.size ();

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Found %d clusters with %d points in %g seconds.", clusters.size (), total_p, time_spent);
      gettimeofday (&t1, NULL);

      vector<vector<vector<int> > > all_cluster_inliers (clusters.size ());
      vector<vector<vector<double> > > all_cluster_coeff (clusters.size ());

      // Reserve enough space
      cloud_annotated_.pts.resize (total_p);
      cloud_annotated_.chan[0].vals.resize (total_p);
      cloud_annotated_.chan[1].vals.resize (total_p);
      cloud_annotated_.chan[2].vals.resize (total_p);

      int nr_p = 0;

      // Process all clusters in parallel
      #pragma omp parallel for schedule(dynamic)
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // Find all planes in this cluster
        fitSACPlane (&cloud_, &clusters[cc].indices, all_cluster_inliers[cc], all_cluster_coeff[cc]);
      }

      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        double r, g, b;
        r = g = b = 1.0;

        // Get the planes in this cluster
        vector<vector<int> > *planes_inliers  = &all_cluster_inliers[cc];
        vector<vector<double> > *planes_coeff = &all_cluster_coeff[cc];

        // For every plane in this cluster
        for (unsigned int j = 0; j < planes_inliers->size (); j++)
        {
//           r = rand () / (RAND_MAX + 1.0);
//           g = rand () / (RAND_MAX + 1.0);
//           b = rand () / (RAND_MAX + 1.0);

          vector<int> *plane_inliers   = &planes_inliers->at (j);
          vector<double> *plane_coeffs = &planes_coeff->at (j);
          // Mark all the points inside
          switch (clusters[cc].region_type)
          {
            case 0:     // Z-parallel
            {
              getObjectClassForParallel (plane_coeffs, map_origin, r, g, b);
              break;
            }
            case 1:     // Z-perpendicular
            {
              getObjectClassForPerpendicular (&cloud_, plane_inliers, r, g, b);
              break;
            }
          }
          for (unsigned int k = 0; k < plane_inliers->size (); k++)
          {
            cloud_annotated_.pts[nr_p].x = cloud_.pts.at (plane_inliers->at (k)).x;
            cloud_annotated_.pts[nr_p].y = cloud_.pts.at (plane_inliers->at (k)).y;
            cloud_annotated_.pts[nr_p].z = cloud_.pts.at (plane_inliers->at (k)).z;
            cloud_annotated_.chan[0].vals[nr_p] = r;
            cloud_annotated_.chan[1].vals[nr_p] = g;
            cloud_annotated_.chan[2].vals[nr_p] = b;
            nr_p++;
          }
        }
/*        r = rand () / (RAND_MAX + 1.0);
        g = rand () / (RAND_MAX + 1.0);
        b = rand () / (RAND_MAX + 1.0);
        for (unsigned int j = 0; j < clusters[cc].indices.size (); j++)
        {
          cloud_annotated_.pts[nr_p].x = cloud_.pts.at (clusters[cc].indices.at (j)).x;
          cloud_annotated_.pts[nr_p].y = cloud_.pts.at (clusters[cc].indices.at (j)).y;
          cloud_annotated_.pts[nr_p].z = cloud_.pts.at (clusters[cc].indices.at (j)).z;
          cloud_annotated_.chan[0].vals[nr_p] = r;
          cloud_annotated_.chan[1].vals[nr_p] = g;
          cloud_annotated_.chan[2].vals[nr_p] = b;
          nr_p++;
        }*/
      }

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Cloud annotated in: %g seconds.", time_spent);
      gettimeofday (&t1, NULL); 
      cloud_annotated_.pts.resize (nr_p);
      cloud_annotated_.chan[0].vals.resize (nr_p);
      cloud_annotated_.chan[1].vals.resize (nr_p);
      cloud_annotated_.chan[2].vals.resize (nr_p);

      publish ("cloud_annotated", cloud_annotated_);

      return;


/**       ROS_INFO ("Found %d clusters.", clusters.size ());

      // note: the clusters are in the z_axis_indices space so instead of points[clusters[j]] we need to do points[z_axis_indices[clusters[j]]
      nr_p = 0;
      for (unsigned int cc = 0; cc < clusters.size (); cc++)
      {
        double r, g, b;
        r = rand () / (RAND_MAX + 1.0);
        g = rand () / (RAND_MAX + 1.0);
        b = rand () / (RAND_MAX + 1.0);
        for (unsigned int j = 0; j < clusters[cc].size (); j++)
        {
          cloud_annotated_.pts[nr_p].x = cloud_.pts[xy_axis_indices[clusters[cc].at (j)]].x;
          cloud_annotated_.pts[nr_p].y = cloud_.pts[xy_axis_indices[clusters[cc].at (j)]].y;
          cloud_annotated_.pts[nr_p].z = cloud_.pts[xy_axis_indices[clusters[cc].at (j)]].z;
          //cloud_annotated_.chan[0].vals[i] = intensity_value;
          cloud_annotated_.chan[0].vals[nr_p] = r;
          cloud_annotated_.chan[1].vals[nr_p] = g;
          cloud_annotated_.chan[2].vals[nr_p] = b;
          nr_p++;
        }
      }*/

    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  SemanticPointAnnotator p;
  p.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */

