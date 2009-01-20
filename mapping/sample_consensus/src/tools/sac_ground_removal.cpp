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
 * $Id: sac_ground_removal.cpp 9305 2009-01-13 02:27:20Z veedee $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b sac_ground_removal annotates 3D point clouds with semantic labels.

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

    // additional downsampling parameters
    bool downsample_;
    Point leaf_width_;
    double cut_distance_;

    vector<cloud_geometry::Leaf> leaves_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    GroundRemoval () : ros::Node ("sac_ground_removal")
    {
      param ("~downsample", downsample_, false);    // Downsample cloud before ground estimation
      param ("~downsample_leaf_width_x", leaf_width_.x, 0.025);      // 2.5cm radius by default
      param ("~downsample_leaf_width_y", leaf_width_.y, 0.025);      // 2.5cm radius by default
      param ("~downsample_leaf_width_z", leaf_width_.z, 0.025);      // 2.5cm radius by default
      param ("~cut_distance", cut_distance_, 10.0);   // 10m by default

      param ("~z_threshold", z_threshold_, 0.2);         // 20cm threshold for ground removal

      param ("~sac_min_points_per_model", sac_min_points_per_model_, 5);  // 5 points minimum per plane
      param ("~sac_distance_threshold", sac_distance_threshold_, 0.04);   // 4 cm threshold
      param ("~sac_max_iterations", sac_max_iterations_, 500);            // maximum 500 iterations
      param ("~sac_probability", sac_probability_, 0.99);                 // 0.99 probability

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
      if (hasParam ("~downsample_leaf_width_x")) getParam ("~downsample_leaf_width_x", leaf_width_.x);
      if (hasParam ("~downsample_leaf_width_y")) getParam ("~downsample_leaf_width_y", leaf_width_.y);
      if (hasParam ("~downsample_leaf_width_z")) getParam ("~downsample_leaf_width_z", leaf_width_.z);

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
        fprintf (stderr, "> Found a model supported by %d inliers: [%g, %g, %g, %g]\n", inliers.size (),
                 coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
        //model->projectPointsInPlace (sac->getInliers (), coeff[1]);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points.", cloud_.pts.size ());
      if (cloud_.pts.size () == 0)
        return;
        
      // Assuming the cloud_ should be in base_link
      if (cloud_.header.frame_id != "base_link")
      {
        ROS_ERROR ("Cloud frame_id is not base_link!");
        return;
      }
      
      // Copy the header
      cloud_noground_.header = cloud_.header;
      
      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // If a-priori downsampling is enabled...
      if (downsample_)
      {
        gettimeofday (&t1, NULL);
        int d_idx = cloud_geometry::getChannelIndex (&cloud_, "distances");
        PointCloud cloud_down;
        try
        {
          cloud_geometry::downsamplePointCloud (&cloud_, cloud_down, leaf_width_, leaves_, d_idx, cut_distance_);
          cloud_ = cloud_down;
        }
        catch (std::bad_alloc)
        {
        }

        gettimeofday (&t2, NULL);
        double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
        ROS_INFO ("Downsampling enabled. Number of points left: %d in %g seconds.", cloud_down.pts.size (), time_spent);
      }

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
      // Find the dominant plane in the space of possible ground indices
      vector<int> ground_inliers;
      vector<double> ground_coeff;
      fitSACPlane (&cloud_, &possible_ground_indices, ground_inliers, ground_coeff);

      // Prepare new arrays
      int nr_remaining_pts = cloud_.pts.size () - ground_inliers.size ();
      cloud_noground_.pts.resize (nr_remaining_pts);
      cloud_noground_.chan.resize (cloud_.chan.size ());
      for (unsigned int d = 0; d < cloud_.chan.size (); d++)
      {
        cloud_noground_.chan[d].name = cloud_.chan[d].name;
        cloud_noground_.chan[d].vals.resize (nr_remaining_pts);
      }
      
      // Get all the non-ground point indices
      vector<int> remaining_indices;
      sort (ground_inliers.begin (), ground_inliers.end ());
      set_difference (all_indices.begin (), all_indices.end (), ground_inliers.begin (), ground_inliers.end (),
                      inserter (remaining_indices, remaining_indices.begin ()));

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

  ros::fini ();

  return (0);
}
/* ]--- */

