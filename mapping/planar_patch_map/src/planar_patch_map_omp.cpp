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

@b planar_patch_map transforms a 3D point cloud into a polygonal planar patch map using Sample Consensus techniques and Octrees.

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

// Cloud geometry
#include <cloud_geometry/point.h>
#include <cloud_geometry/areas.h>
#include <cloud_geometry/nearest.h>
#include <cloud_geometry/intersections.h>

// Cloud Octree
#include <cloud_octree/octree.h>

#include <sys/time.h>

using namespace std;
using namespace std_msgs;

class PlanarPatchMap: public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_f_;

    // Octree stuff
    cloud_octree::Octree *octree_;
    float leaf_width_;

    // Parameters
    int sac_min_points_per_cell_;
    double d_min_, d_max_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlanarPatchMap () : ros::Node ("planar_patch_map_omp")
    {
      param ("~sac_min_points_per_cell", sac_min_points_per_cell_, 10);

      param ("~distance_min", d_min_, 0.10);
      param ("~distance_max", d_max_, 10.0);

      string cloud_topic ("tilt_laser_cloud");

      vector<pair<string, string> > t_list;
      getPublishedTopics (&t_list);
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (cloud_topic) == string::npos)
          ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());
      }

      subscribe (cloud_topic.c_str (), cloud_, &PlanarPatchMap::cloud_cb, 1);

      advertise<PolygonalMap> ("planar_map", 1);

      leaf_width_ = 0.05f;
      octree_ = new cloud_octree::Octree (0.0f, 0.0f, 0.0f, leaf_width_, leaf_width_, leaf_width_, 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PlanarPatchMap () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      fitSACPlane (PointCloud *points, cloud_octree::Octree *octree, cloud_octree::Leaf* leaf, Polygon3D &poly)
    {
      vector<int> indices = leaf->getIndices ();
      if ((int)indices.size () < sac_min_points_per_cell_)
        return;

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, 0.02);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        vector<int> inliers = sac->getInliers ();
        vector<double> coeff = sac->computeCoefficients ();
        ///fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", inliers.size (), coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);

        cloud_geometry::areas::convexHull2D (model->getCloud (), inliers, coeff, poly);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      filterCloudBasedOnDistance (PointCloud *cloud_in, PointCloud &cloud_out,
                                  int d_idx, double d_min, double d_max)
    {
      cloud_out.pts.resize (cloud_in->pts.size ());
      int nr_p = 0;

      for (unsigned int i = 0; i < cloud_out.pts.size (); i++)
      {
        if (cloud_in->chan[d_idx].vals[i] >= d_min && cloud_in->chan[d_idx].vals[i] <= d_max)
        {
          cloud_out.pts[nr_p] = cloud_in->pts[i];
          nr_p++;
        }
      }
      cloud_out.pts.resize (nr_p);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points.", cloud_.pts.size ());

      int d_idx = cloud_geometry::getChannelIndex (&cloud_, "distances");
      if (d_idx != -1)
      {
        filterCloudBasedOnDistance (&cloud_, cloud_f_, d_idx, d_min_, d_max_);
        ROS_INFO ("Distance information present. Filtering points between %g and %g : %d / %d left.", d_min_, d_max_,
                  cloud_f_.pts.size (), cloud_.pts.size ());
      }
      else
        cloud_f_ = cloud_;

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      octree_ = new cloud_octree::Octree (0.0f, 0.0f, 0.0f, leaf_width_, leaf_width_, leaf_width_, 0);
      // Insert all data points into the octree
      for (unsigned int i = 0; i < cloud_f_.pts.size (); i++)
        octree_->insert (cloud_f_.pts[i].x, cloud_f_.pts[i].y, cloud_f_.pts[i].z, i);

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Created octree with %d leaves (%d ndivs) in %g seconds.", octree_->getNumLeaves (), octree_->getNumCells (), time_spent);

      // Initialize the polygonal map
      PolygonalMap pmap;
      pmap.polygons.resize (octree_->getNumLeaves ());

      std::vector<cloud_octree::Leaf*> leaves = octree_->getOccupiedLeaves ();

      gettimeofday (&t1, NULL);

      #pragma omp parallel for schedule(dynamic)
      for (int cl = 0; cl < (int)leaves.size (); cl++)
      {
        fitSACPlane (&cloud_f_, octree_, leaves[cl], pmap.polygons[cl]);
      }

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Polygonal map created in %g seconds.", time_spent);
      pmap.header = cloud_.header;
      publish ("planar_map", pmap);

      delete octree_;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  PlanarPatchMap p;
  p.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */

