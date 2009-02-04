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

@b sac_ground_removal returns a cloud with the ground plane extracted.

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

// Cloud geometry
#include <cloud_geometry/areas.h>
#include <cloud_geometry/point.h>
#include <cloud_geometry/distances.h>
#include <cloud_geometry/nearest.h>

// Kd Tree
#include <cloud_kdtree/kdtree.h>

#include <sys/time.h>

using namespace std;
using namespace std_msgs;

class GroundRemoval : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_noground_;

    // Parameters
    double z_threshold_;
    int sac_min_points_per_model_, sac_max_iterations_;
    double sac_distance_threshold_, sac_probability_;

    double clusters_growing_tolerance_;
    int clusters_min_pts_;

    // additional downsampling parameters
    double cut_distance_;

    vector<cloud_geometry::Leaf> leaves_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    GroundRemoval () : ros::Node ("sac_ground_removal")
    {
      param ("~cut_distance", cut_distance_, 10.0);   // 10m by default

      param ("~z_threshold", z_threshold_, 0.2);         // 20cm threshold for ground removal

      param ("~sac_min_points_per_model", sac_min_points_per_model_, 5);  // 5 points minimum per plane
      param ("~sac_distance_threshold", sac_distance_threshold_, 0.03);   // 4 cm threshold
      param ("~sac_max_iterations", sac_max_iterations_, 100);            // maximum 500 iterations
      param ("~sac_probability", sac_probability_, 0.99);                 // 0.99 probability

      param ("~clusters_growing_tolerance", clusters_growing_tolerance_, 0.2);   // 0.2 m
      param ("~clusters_min_pts", clusters_min_pts_, 5);                         // 5 points

      string cloud_topic ("full_cloud");

      vector<pair<string, string> > t_list;
      getPublishedTopics (&t_list);
      bool topic_found = false;
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (cloud_topic) != string::npos)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());

      subscribe (cloud_topic.c_str (), cloud_, &GroundRemoval::cloud_cb, 1);
      advertise<PointCloud> ("cloud_ground_filtered", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~GroundRemoval () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (hasParam ("~z_threshold")) getParam ("~z_threshold", z_threshold_);
      if (hasParam ("~sac_min_points_per_model")) getParam ("~sac_min_points_per_model", sac_min_points_per_model_);
      if (hasParam ("~sac_distance_threshold"))  getParam ("~sac_distance_threshold", sac_distance_threshold_);
      if (hasParam ("~sac_max_iterations")) getParam ("~sac_max_iterations", sac_max_iterations_);
      if (hasParam ("~sac_probability"))  getParam ("~sac_probability", sac_probability_);

      if (hasParam ("~cut_distance"))
      {
        getParam ("~cut_distance", cut_distance_);
        leaves_.resize (0);
        ROS_INFO ("Done clearing leaves.");
      }
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
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points
      * \param points pointer to the point cloud message
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      findClusters (PointCloud *points, double tolerance, vector<vector<int> > &clusters,
                    unsigned int min_pts_per_cluster = 1)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTree (points);

      // Create a bool vector of processed point indices, and initialize it to false
      vector<bool> processed;
      processed.resize (points->pts.size (), false);

      vector<int> nn_indices;
      // Process all points in the indices vector
      for (unsigned int i = 0; i < points->pts.size (); i++)
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
            r[j] = seed_queue[j];
          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      fitSACPlane (PointCloud *points, vector<int> *indices, vector<int> &inliers, vector<double> &coeff)
    {
      updateParametersFromServer ();
      if ((int)indices->size () < sac_min_points_per_model_)
        return;

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (sac_max_iterations_);
      sac->setProbability (sac_probability_);
      model->setDataSet (points, *indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
          return;
        inliers = sac->getInliers ();
        coeff   = sac->computeCoefficients ();
//        fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", inliers.size (),
//                 coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        //model->projectPointsInPlace (sac->getInliers (), coeff[1]);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      fitSACPlane (PointCloud *points, vector<int> &inliers, vector<double> &coeff)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (sac_max_iterations_);
      sac->setProbability (sac_probability_);
      model->setDataSet (points);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
          return;
        inliers = sac->getInliers ();
        coeff   = sac->computeCoefficients ();
        fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", inliers.size (),
                 coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        //model->projectPointsInPlace (sac->getInliers (), coeff[1]);
      }
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
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points.", cloud_.pts.size ());
      if (cloud_.pts.size () == 0)
        return;

      // Copy the header
      cloud_noground_.header = cloud_.header;

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Select points whose Z dimension is close to the ground (0,0,0 in base_link)
      vector<int> possible_ground_indices (cloud_.pts.size ());
      vector<int> all_indices (cloud_.pts.size ());
      int nr_p = 0;
      for (unsigned int cp = 0; cp < cloud_.pts.size (); cp++)
      {
        all_indices[cp] = cp;
        if (fabs (cloud_.pts[cp].z) < z_threshold_)
        {
          possible_ground_indices[nr_p] = cp;
          nr_p++;
        }
      }
      possible_ground_indices.resize (nr_p);

      ROS_INFO ("Number of possible ground indices: %d.", possible_ground_indices.size ());

      PointCloud cloud_down;
      Point leaf_width;
      leaf_width.x = leaf_width.y = leaf_width.z = 0.1;
      vector<cloud_geometry::Leaf> leaves;
      cloud_geometry::downsamplePointCloud (&cloud_, &possible_ground_indices, cloud_down, leaf_width, leaves, -1);

      ROS_INFO ("Number of points left after downsampling: %d.", cloud_down.pts.size ());

      vector<vector<int> > clusters;
      // Split the Z-parallel points into clusters
      //findClusters (&cloud_, &possible_ground_indices, clusters_growing_tolerance_, clusters, clusters_min_pts_);
      findClusters (&cloud_down, clusters_growing_tolerance_, clusters, clusters_min_pts_);

      ROS_INFO ("Number of clusters: %d", clusters.size ());

      vector<int> ground_inliers;
      vector<double> ground_coeff;
      nr_p = 0;

      // Prepare new arrays
//      cloud_noground_.pts.resize (possible_ground_indices.size ());
//      cloud_noground_.chan.resize (1);
//      cloud_noground_.chan[0].name = "rgb";
//      cloud_noground_.chan[0].vals.resize (possible_ground_indices.size ());

      for (unsigned int cc = 0; cc < clusters.size (); cc++)
      {
        ROS_INFO ("Cluster size: %d.", clusters[cc].size ());
        vector<int> cluster_ground_inliers;
        // Find the dominant plane in the space of possible ground indices
        fitSACPlane (&cloud_down, &clusters[cc], cluster_ground_inliers, ground_coeff);

        nr_p = 0;
        cluster_ground_inliers.resize (possible_ground_indices.size ());
        for (unsigned int i = 0; i < possible_ground_indices.size (); i++)
        {
          double dist = cloud_geometry::distances::pointToPlaneDistance (&cloud_.pts.at (possible_ground_indices[i]), ground_coeff);
          if (dist < sac_distance_threshold_)
          {
            cluster_ground_inliers[nr_p] = possible_ground_indices[i];
            nr_p++;
          }
        }
        cluster_ground_inliers.resize (nr_p);

//        float r = rand () / (RAND_MAX + 1.0);
//        float g = rand () / (RAND_MAX + 1.0);
//        float b = rand () / (RAND_MAX + 1.0);

//        int cur_size = ground_inliers.size ();
//        ground_inliers.resize (cur_size + cluster_ground_inliers.size ());
        for (unsigned int i = 0; i < cluster_ground_inliers.size (); i++)
        {
//          ground_inliers[cur_size + i] = cluster_ground_inliers[i];
          ground_inliers.push_back (cluster_ground_inliers[i]);
//          cloud_noground_.pts[nr_p].x = cloud_.pts.at (ground_inliers[i]).x;
//          cloud_noground_.pts[nr_p].y = cloud_.pts.at (ground_inliers[i]).y;
//          cloud_noground_.pts[nr_p].z = cloud_.pts.at (ground_inliers[i]).z;
//          for (unsigned int d = 0; d < cloud_.chan.size (); d++)
//            cloud_noground_.chan[d].vals[nr_p] = getRGB (r, g, b);
//
//          nr_p++;
        }
      }
      ROS_INFO ("Total number of ground inliers: %d.", ground_inliers.size ());

/*      cloud_noground_.pts.resize (nr_p);
      for (unsigned int d = 0; d < cloud_.chan.size (); d++)
        cloud_noground_.chan[d].vals.resize (nr_p);

      publish ("cloud_ground_filtered", cloud_noground_);
      return;*/

/*      PointCloud cloud_down;
      Point leaf_width;
      leaf_width.x = leaf_width.y = leaf_width.z = 0.06;
      vector<cloud_geometry::Leaf> leaves;
      cloud_geometry::downsamplePointCloud (&cloud_, &possible_ground_indices, cloud_down, leaf_width, leaves, -1);

      // Find the dominant plane in the space of possible ground indices
      vector<int> ground_inliers;
//      vector<double> ground_coeff;
      //fitSACPlane (&cloud_, &possible_ground_indices, ground_inliers, ground_coeff);
      fitSACPlane (&cloud_down, ground_inliers, ground_coeff);

      // Use the plane coefficients to see which points from the original cloud to filter
      nr_p = 0;
      ground_inliers.resize (cloud_.pts.size ());
      for (unsigned int i = 0; i < cloud_.pts.size (); i++)
      {
        double dist = cloud_geometry::distances::pointToPlaneDistance (&cloud_.pts[i], ground_coeff);
        if (dist < sac_distance_threshold_)
        {
          ground_inliers[nr_p] = i;
          nr_p++;
        }
      }
      ground_inliers.resize (nr_p);
*/
      // Get all the non-ground point indices
      vector<int> remaining_indices;
      sort (ground_inliers.begin (), ground_inliers.end ());
      set_difference (all_indices.begin (), all_indices.end (), ground_inliers.begin (), ground_inliers.end (),
                      inserter (remaining_indices, remaining_indices.begin ()));

      // Prepare new arrays
      int nr_remaining_pts = remaining_indices.size ();
      cloud_noground_.pts.resize (nr_remaining_pts);
      cloud_noground_.chan.resize (cloud_.chan.size ());
      for (unsigned int d = 0; d < cloud_.chan.size (); d++)
      {
        cloud_noground_.chan[d].name = cloud_.chan[d].name;
        cloud_noground_.chan[d].vals.resize (nr_remaining_pts);
      }

      for (unsigned int i = 0; i < remaining_indices.size (); i++)
      {
        cloud_noground_.pts[i].x = cloud_.pts.at (remaining_indices[i]).x;
        cloud_noground_.pts[i].y = cloud_.pts.at (remaining_indices[i]).y;
        cloud_noground_.pts[i].z = cloud_.pts.at (remaining_indices[i]).z;
        for (unsigned int d = 0; d < cloud_.chan.size (); d++)
          cloud_noground_.chan[d].vals[i] = cloud_.chan[d].vals.at (remaining_indices[i]);
      }

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Number of points found on ground plane: %d ; remaining: %d (%g seconds).", ground_inliers.size (),
                remaining_indices.size (), time_spent);
      publish ("cloud_ground_filtered", cloud_noground_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  GroundRemoval p;
  p.spin ();

  return (0);
}
/* ]--- */

