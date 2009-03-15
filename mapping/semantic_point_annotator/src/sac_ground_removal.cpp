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
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>
#include <robot_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <tf/transform_listener.h>

#include <sys/time.h>

using namespace std;
using namespace robot_msgs;

class GroundRemoval
{
  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_noground_;

    tf::TransformListener tf_;
    PointStamped viewpoint_cloud_;

    // Parameters
    double z_threshold_;
    int sac_min_points_per_model_, sac_max_iterations_;
    double sac_distance_threshold_;
    int planar_refine_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    GroundRemoval (ros::Node& anode) : node_ (anode), tf_ (anode)
    {
      node_.param ("~z_threshold", z_threshold_, 0.1);                          // 10cm threshold for ground removal
      node_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.03);   // 3 cm threshold

      node_.param ("~planar_refine", planar_refine_, 1);                        // enable a final planar refinement step?
      node_.param ("~sac_min_points_per_model", sac_min_points_per_model_, 6);  // 6 points minimum per line
      node_.param ("~sac_max_iterations", sac_max_iterations_, 200);            // maximum 200 iterations

      string cloud_topic ("full_cloud");

      vector<pair<string, string> > t_list;
      node_.getPublishedTopics (&t_list);
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

      node_.subscribe (cloud_topic, cloud_, &GroundRemoval::cloud_cb, this, 1);
      node_.advertise<PointCloud> ("cloud_ground_filtered", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (node_.hasParam ("~z_threshold")) node_.getParam ("~z_threshold", z_threshold_);
      if (node_.hasParam ("~sac_distance_threshold")) node_.getParam ("~sac_distance_threshold", sac_distance_threshold_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a PointCloud message into LaserScan clusters
      * \param points pointer to the point cloud message
      * \param indices pointer to a list of point indices
      * \param clusters the resultant clusters
      * \param idx the index of the channel containing the laser scan index
      */
    void
      splitPointsBasedOnLaserScanIndex (PointCloud *points, vector<int> *indices, vector<vector<int> > &clusters, int idx)
    {
      vector<int> seed_queue;
      int prev_idx = -1;
      // Process all points in the indices vector
      for (unsigned int i = 0; i < indices->size (); i++)
      {
        // Get the current laser scan measurement index
        int cur_idx = points->chan[idx].vals.at (indices->at (i));

        if (cur_idx > prev_idx)   // Still the same laser scan ?
        {
          seed_queue.push_back (indices->at (i));
          prev_idx = cur_idx;
        }
        else                      // Have we found a new scan ?
        {
          prev_idx = -1;
          vector<int> r;
          r.resize (seed_queue.size ());
          for (unsigned int j = 0; j < r.size (); j++)
            r[j] = seed_queue[j];
          clusters.push_back (r);
          seed_queue.resize (0);
        }
      }
      // Copy the last laser scan as well
      if (seed_queue.size () > 0)
      {
        vector<int> r;
        r.resize (seed_queue.size ());
        for (unsigned int j = 0; j < r.size (); j++)
          r[j] = seed_queue[j];
        clusters.push_back (r);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a line model in a point cloud given via a set of point indices with SAmple Consensus methods
      * \param points the point cloud message
      * \param indices a pointer to a set of point cloud indices to test
      * \param inliers the resultant inliers
      */
    bool
      fitSACLine (PointCloud *points, vector<int> *indices, vector<int> &inliers)
    {
      if ((int)indices->size () < sac_min_points_per_model_)
        return (false);

      // Create and initialize the SAC model
      sample_consensus::SACModelLine *model = new sample_consensus::SACModelLine ();
      sample_consensus::SAC *sac            = new sample_consensus::RANSAC (model, sac_distance_threshold_);
      sac->setMaxIterations (sac_max_iterations_);
      sac->setProbability (0.99);

      model->setDataSet (points, *indices);

      vector<double> line_coeff;
      // Search for the best model
      if (sac->computeModel (0))
      {
        // Obtain the inliers and the planar model coefficients
        if ((int)sac->getInliers ().size () < sac_min_points_per_model_)
          return (false);
        //inliers    = sac->getInliers ();

        sac->computeCoefficients ();             // Compute the model coefficients
        line_coeff = sac->refineCoefficients (); // Refine them using least-squares
        inliers    = model->selectWithinDistance (line_coeff, sac_distance_threshold_);

        // Project the inliers onto the model
        //model->projectPointsInPlace (sac->getInliers (), coeff);
      }
      else
        return (false);

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      //check to see if the point cloud is empty
      if (cloud_.pts.empty ())
      {
        ROS_WARN ("Received an empty point cloud");
        return;
      }

      ROS_INFO ("Received %d data points with %d channels (%s).", cloud_.pts.size (), cloud_.chan.size (), cloud_geometry::getAvailableChannels (&cloud_).c_str ());
      int idx_idx = cloud_geometry::getChannelIndex (&cloud_, "index");
      if (idx_idx == -1)
      {
        ROS_ERROR ("Channel 'index' missing in input PointCloud message!");
        return;
      }
      if (cloud_.pts.size () == 0)
        return;

      updateParametersFromServer ();
      // Copy the header
      cloud_noground_.header = cloud_.header;

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Get the cloud viewpoint
      getCloudViewPoint (cloud_.header.frame_id, viewpoint_cloud_, &tf_);

      // Transform z_threshold_ from the parameter parameter frame (parameter_frame_) into the point cloud frame
      z_threshold_ = transformDoubleValueTF (z_threshold_, "base_footprint", cloud_.header.frame_id, cloud_.header.stamp, &tf_);

      // Select points whose Z dimension is close to the ground (0,0,0 in base_footprint)
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

      vector<vector<int> > clusters;
      // Split the points into clusters based on their laser scan information
      splitPointsBasedOnLaserScanIndex (&cloud_, &possible_ground_indices, clusters, idx_idx);

      ROS_INFO ("Number of clusters: %d", clusters.size ());

      vector<int> ground_inliers;
      nr_p = 0;

      // Prepare enough space
      vector<vector<int> > cluster_ground_inliers (clusters.size ());
      // Parallelize the search
#pragma omp parallel for schedule(dynamic)
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // Find the dominant plane in the space of possible ground indices
        fitSACLine (&cloud_, &clusters[cc], cluster_ground_inliers[cc]);
      }

      // Copy the line inliers (ground results)
      for (unsigned int cc = 0; cc < cluster_ground_inliers.size (); cc++)
      {
        if (cluster_ground_inliers[cc].size () == 0)
          ROS_WARN ("Couldn't fit a model for cluster %d (%d points).", cc, clusters[cc].size ());

        int cur_size = ground_inliers.size ();
        ground_inliers.resize (cur_size + cluster_ground_inliers[cc].size ());
        for (unsigned int i = 0; i < cluster_ground_inliers[cc].size (); i++)
          ground_inliers[cur_size + i] = cluster_ground_inliers[cc][i];
      }
      ROS_INFO ("Total number of ground inliers before refinement: %d.", ground_inliers.size ());

      // Do we attempt to do a planar refinement to remove points "below" the plane model found ?
      if (planar_refine_ > 0)
      {
        // Get the remaining point indices
        vector<int> remaining_possible_ground_indices;
        sort (possible_ground_indices.begin (), possible_ground_indices.end ());
        sort (ground_inliers.begin (), ground_inliers.end ());
        set_difference (possible_ground_indices.begin (), possible_ground_indices.end (), ground_inliers.begin (), ground_inliers.end (),
                        inserter (remaining_possible_ground_indices, remaining_possible_ground_indices.begin ()));

        // Estimate the plane from the line inliers
        Eigen::Vector4d plane_parameters;
        double curvature;
        cloud_geometry::nearest::computeSurfaceNormalCurvature (&cloud_, &ground_inliers, plane_parameters, curvature);

        //make sure that there are inliers to refine
        if(!ground_inliers.empty()){
          // Flip plane normal according to the viewpoint information
          Point32 vp_m;
          vp_m.x = viewpoint_cloud_.point.x - cloud_.pts.at (ground_inliers[0]).x;
          vp_m.y = viewpoint_cloud_.point.y - cloud_.pts.at (ground_inliers[0]).y;
          vp_m.z = viewpoint_cloud_.point.z - cloud_.pts.at (ground_inliers[0]).z;

          // Dot product between the (viewpoint - point) and the plane normal
          double cos_theta = (vp_m.x * plane_parameters (0) + vp_m.y * plane_parameters (1) + vp_m.z * plane_parameters (2));

          // Flip the plane normal
          if (cos_theta < 0)
          {
            for (int d = 0; d < 3; d++)
              plane_parameters (d) *= -1;
            // Hessian form (D = nc . p_plane (centroid here) + p)
            plane_parameters (3) = -1 * (plane_parameters (0) * cloud_.pts.at (ground_inliers[0]).x +
                plane_parameters (1) * cloud_.pts.at (ground_inliers[0]).y +
                plane_parameters (2) * cloud_.pts.at (ground_inliers[0]).z);
          }

          // Compute the distance from the remaining points to the model plane, and add to the inliers list if they are below
          for (unsigned int i = 0; i < remaining_possible_ground_indices.size (); i++)
          {
            double distance_to_ground  = cloud_geometry::distances::pointToPlaneDistanceSigned (&cloud_.pts.at (remaining_possible_ground_indices[i]), plane_parameters);
            if (distance_to_ground > 0)
              continue;
            ground_inliers.push_back (remaining_possible_ground_indices[i]);
          }
        }
      }
      ROS_INFO ("Total number of ground inliers after refinement: %d.", ground_inliers.size ());

#if DEBUG
      // Prepare new arrays
      cloud_noground_.pts.resize (possible_ground_indices.size ());
      cloud_noground_.chan.resize (1);
      cloud_noground_.chan[0].name = "rgb";
      cloud_noground_.chan[0].vals.resize (possible_ground_indices.size ());

      cloud_noground_.pts.resize (ground_inliers.size ());
      cloud_noground_.chan[0].vals.resize (ground_inliers.size ());
      float r = rand () / (RAND_MAX + 1.0);
      float g = rand () / (RAND_MAX + 1.0);
      float b = rand () / (RAND_MAX + 1.0);

      for (unsigned int i = 0; i < ground_inliers.size (); i++)
      {
        cloud_noground_.pts[i].x = cloud_.pts.at (ground_inliers[i]).x;
        cloud_noground_.pts[i].y = cloud_.pts.at (ground_inliers[i]).y;
        cloud_noground_.pts[i].z = cloud_.pts.at (ground_inliers[i]).z;
        cloud_noground_.chan[0].vals[i] = getRGB (r, g, b);
      }
      node_.publish ("cloud_ground_filtered", cloud_noground_);

      return;
#endif

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
      node_.publish ("cloud_ground_filtered", cloud_noground_);
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
        ROS_INFO ("Cloud view point in frame %s is: %g, %g, %g.", cloud_frame.c_str (),
                  viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
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
    /** \brief Transform a value from a source frame to a target frame at a certain moment in time with TF
      * \param val the value to transform
      * \param src_frame the source frame to transform the value from
      * \param tgt_frame the target frame to transform the value into
      * \param stamp a given time stamp
      * \param tf a pointer to a TransformListener object
      */
    inline double
      transformDoubleValueTF (double val, std::string src_frame, std::string tgt_frame, ros::Time stamp, tf::TransformListener *tf)
    {
      robot_msgs::Point32 temp;
      temp.x = temp.y = 0;
      temp.z = val;
      tf::Stamped<robot_msgs::Point32> temp_stamped (temp, stamp, src_frame);
      transformPoint (tf, tgt_frame, temp_stamped, temp_stamped);
      return (temp_stamped.z);
    }

};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("sac_ground_removal");

  // For efficiency considerations please make sure the input PointCloud is in a frame with Z point upwards
  GroundRemoval p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

