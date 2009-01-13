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

class SemanticPointAnnotator : public ros::node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_annotated_;
    Point32 z_axis_;

    tf::TransformListener tf_;

    // Parameters
    int sac_min_points_per_model_, sac_min_points_left_;
    double sac_distance_threshold_, eps_angle_;

    double rule_floor_, rule_ceiling_, rule_wall_;
    double rule_table_min_, rule_table_max_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SemanticPointAnnotator () : ros::node ("semantic_point_annotator_omp"), tf_(*this)
    {
      param ("~rule_floor", rule_floor_, 0.15);          // Rule for FLOOR
      param ("~rule_ceiling", rule_ceiling_, 2.0);       // Rule for CEILING
      param ("~rule_table_min", rule_table_min_, 0.5);   // Rule for MIN TABLE
      param ("~rule_table_max", rule_table_max_, 1.5);   // Rule for MIN TABLE
      param ("~rule_wall", rule_wall_, 2.0);             // Rule for WALL

      param ("~p_sac_min_points_left", sac_min_points_left_, 100);
      param ("~p_sac_min_points_per_model", sac_min_points_per_model_, 50);  // 50 points at high resolution
      param ("~p_sac_distance_threshold", sac_distance_threshold_, 0.06);     // 3 cm
      param ("~p_eps_angle_", eps_angle_, 15.0);                              // 15 degrees

      eps_angle_ = (eps_angle_ * M_PI / 180.0);           // convert to radians

      string cloud_topic ("cloud_normals");

      vector<pair<string, string> > t_list;
      get_published_topics (&t_list);
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (cloud_topic) == string::npos)
          ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());
      }

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
      * \param tolerace the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud *points, vector<int> *indices, double tolerance, vector<vector<int> > &clusters, unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTree (points, indices);

      vector<bool> processed;
      processed.resize (indices->size (), false);

      vector<int> nn_indices;
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

        if (seed_queue.size () >= min_pts_per_cluster)
          clusters.push_back (seed_queue);
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
      sac->setMaxIterations (100);
      sac->setProbability (0.95);
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
          //model->projectPointsInPlace (sac->getInliers (), coeff[coeff.size () - 1]);

          // Remove the current inliers in the model
          nr_points_left = sac->removeInliers ();
        }
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
      gettimeofday (&t1, NULL);

      int total_p1 = 0, total_p2 = 0, nr_p = 0;
      vector<vector<int> > inliers_parallel, inliers_perpendicular;
      vector<vector<double> > coeff_parallel, coeff_perpendicular;

      #pragma omp parallel
      {
        #pragma omp sections
        {
          // ---[ Select points whose normals are parallel with the Z-axis
          #pragma omp section
          {
            vector<int> indices;
            cloud_geometry::getPointIndicesAxisParallelNormals (&cloud_, nx, ny, nz, eps_angle_, z_axis_, indices);

            vector<vector<int> > clusters;
            findClusters (&cloud_, &indices, 0.075, clusters, 10);

            vector<vector<vector<int> > > all_cluster_inliers (clusters.size ());
            vector<vector<vector<double> > > all_cluster_coeff (clusters.size ());
            #pragma omp parallel for schedule(dynamic)
            for (int cc = 0; cc < (int)clusters.size (); cc++)
            {
              // Find all planes parallel with XY
              fitSACPlane (&cloud_, &clusters[cc], all_cluster_inliers[cc], all_cluster_coeff[cc]);
              // Mark points in the output cloud
/*              for (unsigned int i = 0; i < inliers_parallel.size (); i++)
                total_p1 += inliers_parallel[i].size ();*/
            }

          }

          // ---[ Select points whose normals are perpendicular to the Z-axis
          #pragma omp section
          {
            vector<int> indices;
            cloud_geometry::getPointIndicesAxisPerpendicularNormals (&cloud_, nx, ny, nz, eps_angle_, z_axis_, indices);

            vector<vector<int> > clusters;
            findClusters (&cloud_, &indices, 0.075, clusters, 10);

            // Find all planes perpendicular to XY
            fitSACPlane (&cloud_, &indices, inliers_perpendicular, coeff_perpendicular);

            // Mark points in the output cloud
            for (unsigned int i = 0; i < inliers_perpendicular.size (); i++)
              total_p2 += inliers_perpendicular[i].size ();
          }
        }
      }

      int total_p = total_p1 + total_p2;
      cloud_annotated_.pts.resize (total_p);
      cloud_annotated_.chan[0].vals.resize (total_p);
      cloud_annotated_.chan[1].vals.resize (total_p);
      cloud_annotated_.chan[2].vals.resize (total_p);

      // Get all planes parallel to the floor (perpendicular to Z)
      Point32 robot_origin;
      robot_origin.x = map_origin.point.x;
      robot_origin.y = map_origin.point.y;
      robot_origin.z = map_origin.point.z;
      for (unsigned int i = 0; i < inliers_parallel.size (); i++)
      {
        // Compute a distance from 0,0,0 to the plane
        double distance = cloud_geometry::distances::pointToPlaneDistance (robot_origin, coeff_parallel[i]);

        double r = 1.0, g = 1.0, b = 1.0;
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

        for (unsigned int j = 0; j < inliers_parallel[i].size (); j++)
        {
          cloud_annotated_.pts[nr_p].x = cloud_.pts.at (inliers_parallel[i].at (j)).x;
          cloud_annotated_.pts[nr_p].y = cloud_.pts.at (inliers_parallel[i].at (j)).y;
          cloud_annotated_.pts[nr_p].z = cloud_.pts.at (inliers_parallel[i].at (j)).z;
          //cloud_annotated_.chan[0].vals[i] = intensity_value;
          cloud_annotated_.chan[0].vals[nr_p] = r;
          cloud_annotated_.chan[1].vals[nr_p] = g;
          cloud_annotated_.chan[2].vals[nr_p] = b;
          nr_p++;
        }
      }

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

      /**
      // Get all planes perpendicular to the floor (parallel to XY)
      for (unsigned int i = 0; i < inliers_perpendicular.size (); i++)
      {
//        float intensity_value = rand () / (RAND_MAX + 1.0);    // Get a random value for the intensity
        // Get the minimum and maximum bounds of the plane
        Point32 minP, maxP;
        cloud_geometry::getMinMax (cloud_, inliers_perpendicular[i], minP, maxP);

        double r, g, b;
        r = g = b = 1.0;

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
        for (unsigned int j = 0; j < inliers_perpendicular[i].size (); j++)
        {
          cloud_annotated_.pts[nr_p].x = cloud_.pts.at (inliers_perpendicular[i].at (j)).x;
          cloud_annotated_.pts[nr_p].y = cloud_.pts.at (inliers_perpendicular[i].at (j)).y;
          cloud_annotated_.pts[nr_p].z = cloud_.pts.at (inliers_perpendicular[i].at (j)).z;
          //cloud_annotated_.chan[0].vals[i] = intensity_value;
          cloud_annotated_.chan[0].vals[nr_p] = r;
          cloud_annotated_.chan[1].vals[nr_p] = g;
          cloud_annotated_.chan[2].vals[nr_p] = b;
          nr_p++;
        }
      }*/

      cloud_annotated_.pts.resize (nr_p);
      cloud_annotated_.chan[0].vals.resize (nr_p);
      cloud_annotated_.chan[1].vals.resize (nr_p);
      cloud_annotated_.chan[2].vals.resize (nr_p);

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Semantic annotations done in: %g seconds.", time_spent);

      publish ("cloud_annotated", cloud_annotated_);
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

