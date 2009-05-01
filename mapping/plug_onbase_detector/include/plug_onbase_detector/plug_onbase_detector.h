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
 * $Id: plug_onbase_detector.cpp 13070 2009-03-28 00:04:21Z veedee $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b plug_onbase_detector detects the pose of the stowed plug on top of the robot base.

 **/
#ifndef PLUG_ONBASE_DETECTOR_H
#define PLUG_ONBASE_DETECTOR_H

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <tf/transform_listener.h>

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

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <sys/time.h>

#include <robot_msgs/PlugStow.h>

using namespace std;
using namespace std_msgs;
using namespace robot_msgs;

// Comparison operator for a vector of vectors
inline bool
  compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class PlugOnBaseDetector
{
  protected:
    ros::Node& node_;
    bool active_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_tr_;
    PointCloud cloud_annotated_;
    PlugStow p_stow_;

    PointStamped viewpoint_cloud_;
    tf::TransformListener tf_;

    int publish_debug_;
    double sac_distance_threshold_;
    double base_z_min_, base_xy_max_, base_plane_height_;
    void activate() {active_ = true;}
    void deactivate() {active_ = false;}

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlugOnBaseDetector (ros::Node& anode) : node_ (anode), tf_ (anode)
    {
      node_.param ("~publish_debug", publish_debug_, 1);
      node_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.02);     // 2 cm

      // Simple constraints for the robot's base on Z (minimum) and X (maximum) - These should never be changed in theory on the PR2
      // NOTE: All parameters are given in base_link. Use TF to convert them to different frames!
      node_.param ("~base_z_min_abs", base_z_min_, .2);                     // ignore all points on Z below this value
      node_.param ("~base_plane_height_rel", base_plane_height_, .15);      // ignore all points on Z above base_z_min_ + this value

      node_.param ("~base_xy_max_rel", base_xy_max_, .29);                  // specifies the maximum X-Y distance between 0,0,0 and a point
      base_xy_max_ *= base_xy_max_;

      active_ = true;

      // Check to see if the default/given topic exists in the list of published topics on the server yet, and issue a warning otherwise
      string cloud_topic ("plug_snapshot_cloud");
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
        ROS_DEBUG ("Trying to subscribe to %s, but the topic doesn't exist!", node_.mapName (cloud_topic).c_str ());

      // Subscribe to the topic
      node_.subscribe (cloud_topic, cloud_, &PlugOnBaseDetector::callback, this, 1);

      node_.advertise<PlugStow> ("~plug_stow_info", 1);

      // We want to publish an annotated point cloud for debugging purposes
      if (publish_debug_)
      {
        node_.advertise<PointCloud> ("~plug_stow_cloud_debug", 1);

        cloud_annotated_.chan.resize (1);
        cloud_annotated_.chan[0].name = "rgb";
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      callback ()
    {
      if(active_ == false)
      {
        ROS_DEBUG("plug_onbase_detector is inactive");
        return;
      }


      ROS_DEBUG ("Received %u data points in frame %s.", (unsigned int)cloud_.pts.size (), cloud_.header.frame_id.c_str ());

      if (cloud_.header.frame_id != "base_link")
      {
        ROS_DEBUG ("Transforming cloud into base_link for easier processing. This can be fixed later.");
        try
        {
          tf_.transformPointCloud ("base_link", cloud_, cloud_tr_);
        }
        catch (tf::ExtrapolationException e)
        {
          ROS_ERROR ("Cannot transform point cloud dataset from %s to base_link. Reason: %s", cloud_.header.frame_id.c_str (), e.what ());
          return;
        }
      }

      ros::Time t1, t2;
      t1 = ros::Time::now ();

      // Trim the point cloud to the base of the robot + base_plane_height
      vector<int> indices_in_bounds (cloud_tr_.pts.size ());
      int nr_p = 0;
      for (unsigned int i = 0; i < cloud_tr_.pts.size (); i++)
      {
        if (cloud_tr_.pts[i].z >= base_z_min_  && cloud_tr_.pts[i].z <= (base_z_min_ + base_plane_height_))
        {
          double dist = (cloud_.pts[i].x) * (cloud_.pts[i].x) +
                        (cloud_.pts[i].y) * (cloud_.pts[i].y);
          if (dist < base_xy_max_)
          {
            indices_in_bounds[nr_p] = i;
            nr_p++;
          }
        }
      }
      indices_in_bounds.resize (nr_p);

      // Get the cloud viewpoint
      getCloudViewPoint (cloud_tr_.header.frame_id, viewpoint_cloud_, &tf_);

      // Find the base plane
      vector<int> inliers;
      vector<double> coeff;
      fitSACPlane (cloud_tr_, indices_in_bounds, inliers, coeff, viewpoint_cloud_, sac_distance_threshold_, 100);

      // Remove points below the plane
      for (unsigned int i = 0; i < indices_in_bounds.size (); i++)
      {
        if (cloud_geometry::distances::pointToPlaneDistanceSigned (cloud_tr_.pts[indices_in_bounds.at (i)], coeff) < 0)
          inliers.push_back (indices_in_bounds.at (i));
      }

      // Get the remaining points
      vector<int> remaining_indices;
      sort (indices_in_bounds.begin (), indices_in_bounds.end ());
      sort (inliers.begin (), inliers.end ());

      set_difference (indices_in_bounds.begin (), indices_in_bounds.end (), inliers.begin (), inliers.end (),
                      inserter (remaining_indices, remaining_indices.begin ()));

      // Find the object clusters supported by it
      vector<vector<int> > object_clusters;
      findClusters (cloud_, remaining_indices, 0.01, object_clusters, 5);

      if (object_clusters.size () != 0)
        ROS_DEBUG ("Number of remaining clusters on base: %d. Selecting the largest cluster with %d points as the plug candidate.", (int)object_clusters.size (), (int)object_clusters[0].size ());

//#define DEBUG 1
#if DEBUG
      // Print the cluster dimensions on screen
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
        ROS_DEBUG ("   Cluster %d has %d points.", i, object_clusters[i].size ());
      }
#endif

      if (object_clusters.size () == 0)       // Nothing left ?
      {
        p_stow_.stowed = false;

      }
      else
      {
        // Assume the largest one is the one we're interested in for now
        // NOTE: This is not the final version of the code ! We're still doing tests !
        Point32 minP, maxP;
        cloud_geometry::statistics::getMinMax (cloud_, object_clusters[0], minP, maxP);
        p_stow_.stowed = true;
        p_stow_.plug_centroid.x =  ( maxP.x + minP.x ) / 2.0;
        p_stow_.plug_centroid.y =  ( maxP.y + minP.y ) / 2.0;
        p_stow_.plug_centroid.z =  ( maxP.z + minP.z ) / 2.0;
        p_stow_.header.frame_id = cloud_.header.frame_id;
      }

      if (publish_debug_)
      {
        cloud_annotated_.header = cloud_.header;
#if DEBUG
//        indices_in_bounds = inliers;
        indices_in_bounds = remaining_indices;
        cloud_annotated_.pts.resize (indices_in_bounds.size ());
        cloud_annotated_.chan[0].vals.resize (indices_in_bounds.size ());
        for (unsigned int i = 0; i < indices_in_bounds.size (); i++)
        {
          cloud_annotated_.pts[i].x = cloud_.pts[indices_in_bounds.at (i)].x;
          cloud_annotated_.pts[i].y = cloud_.pts[indices_in_bounds.at (i)].y;
          cloud_annotated_.pts[i].z = cloud_.pts[indices_in_bounds.at (i)].z;
          cloud_annotated_.chan[0].vals[i] = 0;
        }
#endif
        if (p_stow_.stowed)
        {
          cloud_annotated_.pts.resize (1); cloud_annotated_.chan[0].vals.resize (1);
          cloud_annotated_.pts[0].x = p_stow_.plug_centroid.x;
          cloud_annotated_.pts[0].y = p_stow_.plug_centroid.y;
          cloud_annotated_.pts[0].z = p_stow_.plug_centroid.z;
          cloud_annotated_.chan[0].vals[0] = 255;
          ROS_INFO ("Debug publishing enabled with %d points.", (int)cloud_annotated_.pts.size ());
        }
        else
        {
          cloud_annotated_.pts.resize (0); cloud_annotated_.chan[0].vals.resize (0);
        }

        node_.publish ("~plug_stow_cloud_debug", cloud_annotated_);
      }

      t2 = ros::Time::now ();
      double time_spent = (t2 - t1).toSec ();
      if (p_stow_.stowed)
        ROS_DEBUG ("Plug pose estimated in %g seconds.", time_spent);
      else
        ROS_DEBUG ("No plug found after %g seconds spent.", time_spent);

      node_.publish ("~plug_stow_info", p_stow_);
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

      sort (clusters.begin (), clusters.end (), compareRegions);        // Sort the regions
      reverse (clusters.begin (), clusters.end ());

      // Destroy the tree
      // delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud given via a set of point indices with SAmple Consensus methods
      * \param points the point cloud message
      * \param indices a pointer to a set of point cloud indices to test
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    int
      fitSACPlane (PointCloud &points, vector<int> indices, vector<int> &inliers, vector<double> &coeff,
                   const robot_msgs::PointStamped &viewpoint_cloud, double dist_thresh, int min_pts)
    {
      if ((int)indices.size () < min_pts)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (-1);
      }

      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
      sample_consensus::SAC *sac             = new sample_consensus::LMedS (model, dist_thresh);
      //sac->setMaxIterations (500);
      model->setDataSet (&points, indices);

      // Search for the best plane
      if (sac->computeModel ())
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < min_pts)
        {
          //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
          inliers.resize (0);
          coeff.resize (0);
          return (-1);
        }
        sac->computeCoefficients (coeff);   // Compute the model coefficients
        sac->refineCoefficients (coeff);    // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        // Flip plane normal according to the viewpoint information
        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points.pts.at (inliers[0]), viewpoint_cloud);
        //ROS_INFO ("Found a model supported by %d inliers: [%g, %g, %g, %g]\n", sac->getInliers ().size (),
        //          coeff[0], coeff[1], coeff[2], coeff[3]);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (-1);
      }
      delete sac;
      delete model;
      return (0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the view point from where the scans were taken in the incoming PointCloud message frame
      * \param cloud_frame the point cloud message TF frame
      * \param viewpoint_cloud the resultant view point in the incoming cloud frame
      * \param tf a pointer to a TransformListener object
      */
    void
      getCloudViewPoint (string cloud_frame, PointStamped &viewpoint_cloud, tf::TransformListener *tf)
    {
      // Figure out the viewpoint value in the point cloud frame
      PointStamped viewpoint_laser;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0, 0, 0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf->transformPoint (cloud_frame, viewpoint_laser, viewpoint_cloud);
        //ROS_INFO ("Cloud view point in frame %s is: %g, %g, %g.", cloud_frame.c_str (),
        //          viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
      }
      catch (tf::ConnectivityException)
      {
        ROS_WARN ("Could not transform a point from frame %s to frame %s!", viewpoint_laser.header.frame_id.c_str (), cloud_frame.c_str ());
        // Default to 0.05, 0, 0.942768
        viewpoint_cloud.point.x = 0.05; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.942768;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Transform a given point from its current frame to a given target frame
      * \param tf a pointer to a TransformListener object
      * \param target_frame the target frame to transform the point into
      * \param stamped_in the input point
      * \param stamped_out the output point
      */
    inline void
      transformPoint (tf::TransformListener *tf, const std::string &target_frame,
                      const tf::Stamped< robot_msgs::Point32 > &stamped_in, tf::Stamped< robot_msgs::Point32 > &stamped_out)
    {
      tf::Stamped<tf::Point> tmp;
      tmp.stamp_ = stamped_in.stamp_;
      tmp.frame_id_ = stamped_in.frame_id_;
      tmp[0] = stamped_in.x;
      tmp[1] = stamped_in.y;
      tmp[2] = stamped_in.z;

      tf->transformPoint (target_frame, tmp, tmp);

      stamped_out.stamp_ = tmp.stamp_;
      stamped_out.frame_id_ = tmp.frame_id_;
      stamped_out.x = tmp[0];
      stamped_out.y = tmp[1];
      stamped_out.z = tmp[2];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Transform a value (on Z) from a source frame to a target frame at a certain moment in time with TF
      * \param z_val the value to transform
      * \param src_frame the source frame to transform the value from
      * \param tgt_frame the target frame to transform the value into
      * \param stamp a given time stamp
      * \param tf a pointer to a TransformListener object
      */
    inline double
      transformZValueTF (double z_val, std::string src_frame, std::string tgt_frame, ros::Time stamp, tf::TransformListener *tf)
    {
      robot_msgs::Point32 temp;
      temp.x = temp.y = 0;
      temp.z = z_val;
      tf::Stamped<robot_msgs::Point32> temp_stamped (temp, stamp, src_frame);
      transformPoint (tf, tgt_frame, temp_stamped, temp_stamped);
      return (temp_stamped.z);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////


};

#endif

