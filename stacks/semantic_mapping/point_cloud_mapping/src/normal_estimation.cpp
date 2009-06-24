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

@b normal_estimation_omp estimates local surface properties at each 3D point, such as surface normals, curvature estimates,
moment invariants, etc.

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

//#define DEBUG
#include <sys/time.h>

using namespace std;
using namespace robot_msgs;

class NormalEstimation
{
  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_down_, cloud_normals_;

    tf::TransformListener tf_;

    // Kd-tree stuff
    cloud_kdtree::KdTree *kdtree_;
    vector<vector<int> > points_indices_;

    // Parameters
    bool compute_moments_;
    double radius_;
    int k_;
    // additional downsampling parameters
    int downsample_;
    Point leaf_width_;
    double cut_distance_;

    vector<cloud_geometry::Leaf> leaves_;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    NormalEstimation (ros::Node& anode) : node_ (anode),  tf_ (anode)
    {
      node_.param ("~search_radius", radius_, 0.02);      // 2cm radius by default
      node_.param ("~search_k_closest", k_, 25);          // 25 k-neighbors by default
      node_.param ("~compute_moments", compute_moments_, false);  // Do not compute moment invariants by default

      node_.param ("~downsample", downsample_, 1);                        // Downsample cloud before normal estimation
      node_.param ("~downsample_leaf_width_x", leaf_width_.x, 0.05);      // 5cm radius by default
      node_.param ("~downsample_leaf_width_y", leaf_width_.y, 0.05);      // 5cm radius by default
      node_.param ("~downsample_leaf_width_z", leaf_width_.z, 0.05);      // 5cm radius by default
      node_.param ("~cut_distance", cut_distance_, 10.0);   // 10m by default

      if (downsample_ != 0)
        k_ = 10;          // Reduce the size of K significantly

      string cloud_topic ("tilt_laser_cloud");

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

      node_.subscribe (cloud_topic, cloud_, &NormalEstimation::cloud_cb, this, 1);
      node_.advertise<PointCloud> ("cloud_normals", 1);

#ifdef DEBUG
      cloud_normals_.chan.resize (1);
      cloud_normals_.chan[0].name = "intensities";
#endif
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~NormalEstimation () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (node_.hasParam ("~downsample")) node_.getParam ("~downsample", downsample_);
      if (downsample_ != 0)
        k_ = 10;
      else
        k_ = 25;

      if (node_.hasParam ("~downsample_leaf_width_x")) node_.getParam ("~downsample_leaf_width_x", leaf_width_.x);
      if (node_.hasParam ("~downsample_leaf_width_y")) node_.getParam ("~downsample_leaf_width_y", leaf_width_.y);
      if (node_.hasParam ("~downsample_leaf_width_z")) node_.getParam ("~downsample_leaf_width_z", leaf_width_.z);

      if (node_.hasParam ("~cut_distance"))
      {
        node_.getParam ("~cut_distance", cut_distance_);
        leaves_.resize (0);
        ROS_INFO ("Done clearing leaves.");
      }
    }

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

      // Figure out the viewpoint value in the cloud_frame frame
      PointStamped viewpoint_cloud;
      getCloudViewPoint (cloud_.header.frame_id, viewpoint_cloud, tf_);

      ros::Time ts = ros::Time::now ();

      // If a-priori downsampling is enabled...
      if (downsample_ != 0)
      {
        ros::Time ts1 = ros::Time::now ();
        int d_idx = cloud_geometry::getChannelIndex (cloud_, "distances");
        try
        {
          cloud_geometry::downsamplePointCloud (cloud_, cloud_down_, leaf_width_, leaves_, d_idx, cut_distance_);
        }
        catch (std::bad_alloc)
        {
//          cloud_geometry::downsamplePointCloudSet (cloud_, cloud_down_, leaf_width_, d_idx, cut_distance_);
        }

        ROS_INFO ("Downsampling enabled. Number of points left: %d / %d in %g seconds.", (int)cloud_down_.pts.size (), (int)cloud_.pts.size (), (ros::Time::now () - ts1).toSec ());
      }

      // Resize
#ifdef DEBUG
      cloud_normals_.header = cloud_.header;
      cloud_normals_.pts.resize (cloud_.pts.size ());
      cloud_normals_.chan[0].vals.resize (cloud_.pts.size ());
#else
      // We need to copy the original point cloud data, and this looks like a good way to do it
      int original_chan_size;
      if (downsample_ != 0)
      {
        cloud_normals_ = cloud_down_;
        // There's no point in saving: intensity, indices, distances, timestamps once we go to downsampled 3D
        original_chan_size = 0;
      }
      else
      {
        cloud_normals_ = cloud_;
        original_chan_size = cloud_.chan.size ();
      }

      // Allocate the extra needed channels
      if (compute_moments_)
        cloud_normals_.chan.resize (original_chan_size + 7);     // Allocate 7 more channels
      else
        cloud_normals_.chan.resize (original_chan_size + 4);     // Allocate 4 more channels
      cloud_normals_.chan[original_chan_size + 0].name = "nx";
      cloud_normals_.chan[original_chan_size + 1].name = "ny";
      cloud_normals_.chan[original_chan_size + 2].name = "nz";
      cloud_normals_.chan[original_chan_size + 3].name = "curvature";
      if (compute_moments_)
      {
        cloud_normals_.chan[original_chan_size + 4].name = "j1";
        cloud_normals_.chan[original_chan_size + 5].name = "j2";
        cloud_normals_.chan[original_chan_size + 6].name = "j3";
      }
      for (unsigned int d = original_chan_size; d < cloud_normals_.chan.size (); d++)
      {
        if (downsample_ != 0)
          cloud_normals_.chan[d].vals.resize (cloud_down_.pts.size ());
        else
          cloud_normals_.chan[d].vals.resize (cloud_.pts.size ());
      }
#endif

      // Create Kd-Tree
      kdtree_ = new cloud_kdtree::KdTreeANN (cloud_normals_);

      // Allocate enough space for point indices
      points_indices_.resize (cloud_normals_.pts.size ());
      for (int i = 0; i < (int)cloud_normals_.pts.size (); i++)
        points_indices_[i].resize (k_);

      ROS_INFO ("Kd-tree created in %g seconds.", (ros::Time::now () - ts).toSec ());

      ts = ros::Time::now ();
      // Get the nearest neighbors for all points
      for (int i = 0; i < (int)cloud_normals_.pts.size (); i++)
      {
        vector<float> distances;
        kdtree_->nearestKSearch (i, k_, points_indices_[i], distances);
      }
      ROS_INFO ("Nearest neighbors found in %g seconds.\n", (ros::Time::now () - ts).toSec ());

#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)cloud_normals_.pts.size (); i++)
      {
        // Compute the point normals (nx, ny, nz), surface curvature estimates (c), and moment invariants (j1, j2, j3)
        Eigen::Vector4d plane_parameters;
        double curvature, j1, j2, j3;
        cloud_geometry::nearest::computePointNormal (cloud_normals_, points_indices_[i], plane_parameters, curvature);

        if (compute_moments_)
          cloud_geometry::nearest::computeMomentInvariants (cloud_normals_, points_indices_[i], j1, j2, j3);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud_normals_.pts[i], viewpoint_cloud);

#ifdef  DEBUG
        cloud_normals_.pts[i].x = plane_parameters (0);
        cloud_normals_.pts[i].y = plane_parameters (1);
        cloud_normals_.pts[i].z = plane_parameters (2);
        cloud_normals_.chan[0].vals[i] = curvature;
#else
        cloud_normals_.chan[original_chan_size + 0].vals[i] = plane_parameters (0);
        cloud_normals_.chan[original_chan_size + 1].vals[i] = plane_parameters (1);
        cloud_normals_.chan[original_chan_size + 2].vals[i] = plane_parameters (2);
        cloud_normals_.chan[original_chan_size + 3].vals[i] = curvature;
        if (compute_moments_)
        {
          cloud_normals_.chan[original_chan_size + 4].vals[i] = j1;
          cloud_normals_.chan[original_chan_size + 5].vals[i] = j2;
          cloud_normals_.chan[original_chan_size + 6].vals[i] = j3;
        }
#endif
      }

      ROS_INFO ("Local features estimated in %g seconds.\n", (ros::Time::now () - ts).toSec ());

      node_.publish ("cloud_normals", cloud_normals_);

      delete kdtree_;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("normal_estimation_node");

  NormalEstimation p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

