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

@b convex_patch_histogram is a node which gets two 3D point clouds (one textured, one normal) from a stereo dcam, fits a
plane to the textured cloud, gets the blobs lying on it from the normal cloud, computes a convex hull, creates an RGB
histogram and matches it using HIK (the Histogram Intersection Kernel) to a "database" of previously stored models.

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
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <sys/time.h>

using namespace std;
using namespace robot_msgs;

class ConvexPatchHistogram
{
  protected:
    ros::Node& node_;
  public:

    // ROS messages
    PointCloud cloud_, cloud_textured_, cloud_annotated_;

    string src_normal_, src_textured_;

    // Parameters
    double sac_distance_threshold_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ConvexPatchHistogram (ros::Node& anode) : node_ (anode)
    {
      node_.param ("~p_sac_distance_threshold", sac_distance_threshold_, 0.025);     // 3 cm

      node_.subscribe ("cloud_pcd", cloud_, &ConvexPatchHistogram::cloud_cb, this, 1);
      node_.subscribe ("cloud_textured", cloud_textured_, &ConvexPatchHistogram::cloud_cb, this, 1);

      node_.advertise<PointCloud> ("cloud_annotated", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      fitSACPlane (PointCloud *points, vector<int> &inliers, vector<double> &coeff)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
      model->setDataSet (points);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        inliers = sac->getInliers ();
        coeff = sac->computeCoefficients ();
        ROS_INFO ("The best plane model found is supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
                  coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        //model->projectPointsInPlace (sac->getInliers (), coeff);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      findClusters (const PointCloud &points, vector<double> coeff, vector<vector<int> > &clusters)
    {
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points);

      vector<bool> processed;
      processed.resize (points.pts.size (), false);

      vector<int> indices (points.pts.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        // Compute a distance from each point to the plane
        double distance = cloud_geometry::distances::pointToPlaneDistance (points.pts[i], coeff);
        if (distance < 3 * sac_distance_threshold_)
        {
          indices[nr_p] = i;
          nr_p++;
        }
      }
      indices.resize (nr_p);

      vector<int> nn_indices;
      vector<float> nn_distances;
      for (vector<int>::iterator it = indices.begin (); it != indices.end (); ++it)
      {
        if (processed[*it])
          continue;

        vector<int> q;
        int q_idx = 0;
        q.push_back (*it);

        processed[*it] = true;

        while (q_idx < (int)q.size ())
        {
          tree->radiusSearch (points, q[q_idx], 0.01, nn_indices, nn_distances, 20);

          for (unsigned int j = 1; j < nn_indices.size (); j++)
          {
            if (!processed[nn_indices[j]])
            {
              processed[nn_indices[j]] = true;
              q.push_back (nn_indices[j]);
            }
          }

          q_idx++;
        }

        if (q.size () >= 200)
          clusters.push_back (q);
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
/*    bool spin ()
    {
      cloud_io::loadPCDFile (src_normal_.c_str (), cloud_textured_);
      cloud_io::loadPCDFile (src_textured_.c_str (), cloud_);
      cloud_.header.frame_id = cloud_textured_.header.frame_id = "base_link";

      while (ok ())
      {
        usleep (1000000);

        cloud_cb ();
//        break;
      }

      return true;
    }*/

    // Assumes rgb channels are consecutive
    void
      createRGBHistogram (PointCloud *points, vector<int> *indices, int c_idx, vector<double> &histogram)
    {
      histogram.resize (256, 0);

      for (unsigned int i = 0; i < indices->size (); i++)
      {
        int mean = (points->chan[c_idx + 0].vals[indices->at (i)] +
                    points->chan[c_idx + 1].vals[indices->at (i)] +
                    points->chan[c_idx + 2].vals[indices->at (i)]) / 3;
        histogram[mean]++;
      }

      for (unsigned int i = 0; i < histogram.size (); i++)
        histogram[i] /= indices->size ();
    }


    double
      histogramIntersectionKernel (vector<double> *h1, vector <double> *h2)
    {
      assert (h1->size () == h2->size ());
      double distance = 0.0;
      for (unsigned int i = 0; i < h1->size (); i++)
      {
        distance += min (h1->at(i), h2->at (i));
      }
      return (distance);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      if (cloud_.pts.size () == 0 || cloud_textured_.pts.size () == 0)
        return;

      cloud_annotated_.header = cloud_textured_.header;
      cloud_annotated_.pts.resize (cloud_.pts.size ());

      ROS_INFO ("Received %d data points.", (int)cloud_.pts.size ());
      ROS_INFO ("Received %d data points.", (int)cloud_textured_.pts.size ());

      int c_idx = cloud_geometry::getChannelIndex (cloud_, "r");

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Find the largest plane
      vector<int> inliers;
      vector<double> coeff;
      fitSACPlane (&cloud_textured_, inliers, coeff);

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Best plane fit in %g seconds.", time_spent);

      gettimeofday (&t1, NULL);

      // Split the untextured cloud into clusters close to the plane found
      vector<vector<int> > clusters;
      findClusters (cloud_, coeff, clusters);

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Found %d cluster regions in %g seconds.", (int)clusters.size (), time_spent);

      int nr_p = 0;
      vector<double> histogram;
      // Compute the hull and area of each cluster
      Polygon3D poly;
      for (unsigned int cc = 0; cc < clusters.size (); cc++)
      {
        cloud_geometry::areas::convexHull2D (cloud_, clusters[cc], coeff, poly);
        double area = cloud_geometry::areas::compute2DPolygonalArea (poly, coeff);
        ROS_INFO ("Cluster %d has an estimated area of %g.", cc, area);

        // Check if the area is approximately what we're looking for
        if (area < 0.01)
        {
          // Construct a RGB histogram
          createRGBHistogram (&cloud_, &clusters[cc], c_idx, histogram);
          for (unsigned int d = 0; d < histogram.size (); d++)
            std::cerr << histogram[d] << " ";
          std::cerr << std::endl;

          for (unsigned int i = 0; i < clusters[cc].size (); i++)
          {
            cloud_annotated_.pts[nr_p].x = cloud_.pts[clusters[cc].at (i)].x;
            cloud_annotated_.pts[nr_p].y = cloud_.pts[clusters[cc].at (i)].y;
            cloud_annotated_.pts[nr_p].z = cloud_.pts[clusters[cc].at (i)].z;
            nr_p++;
          }
        }
      }

      cloud_annotated_.pts.resize (nr_p);
      node_.publish ("cloud_annotated", cloud_annotated_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("convex_patch_histogram_node");
  ConvexPatchHistogram p (ros_node);
  //p.src_normal_   = string (argv[1]); //string ("/work/data/WG/Stereo2/pcd_clouds/stereoImage_C22.pcd");
  //p.src_textured_ = string (argv[2]); //string ( "/work/data/WG/Stereo2/pcd_clouds/stereoImage_CnoTex22.pcd");
  ros_node.spin ();

  return (0);
}
/* ]--- */

