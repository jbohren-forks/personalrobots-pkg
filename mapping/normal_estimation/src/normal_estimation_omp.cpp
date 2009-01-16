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
#include <std_msgs/PointCloud.h>
#include <std_msgs/PointStamped.h>

#include <tf/transform_listener.h>

// Cloud kd-tree
#include <cloud_kdtree/kdtree.h>

// Cloud geometry
#include <cloud_geometry/point.h>
#include <cloud_geometry/areas.h>
#include <cloud_geometry/lapack.h>
#include <cloud_geometry/nearest.h>
#include <cloud_geometry/intersections.h>

//#define DEBUG
#include <sys/time.h>

using namespace std;
using namespace std_msgs;

class NormalEstimation : public ros::Node
{
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
    bool downsample_;
    Point leaf_width_;
    double cut_distance_;

    vector<cloud_geometry::Leaf> leaves_;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    NormalEstimation () : ros::Node ("normal_estimation_omp"), tf_(*this)
    {
      param ("~search_radius", radius_, 0.02);      // 2cm radius by default
      param ("~search_k_closest", k_, 25);          // 25 k-neighbors by default
      param ("~compute_moments", compute_moments_, false);  // Do not compute moment invariants by default

      param ("~downsample", downsample_, true);    // Downsample cloud before normal estimation
      param ("~downsample_leaf_width_x", leaf_width_.x, 0.02);      // 2cm radius by default
      param ("~downsample_leaf_width_y", leaf_width_.y, 0.02);      // 2cm radius by default
      param ("~downsample_leaf_width_z", leaf_width_.z, 0.02);      // 2cm radius by default
      param ("~cut_distance", cut_distance_, 10.0);   // 10m by default

      if (downsample_)
        k_ = 10;          // Reduce the size of K significantly

      string cloud_topic ("tilt_laser_cloud");

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

      subscribe (cloud_topic.c_str (), cloud_, &NormalEstimation::cloud_cb, 1);
      advertise<PointCloud> ("cloud_normals", 1);

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
      if (hasParam ("~downsample_leaf_width_x")) getParam ("~downsample_leaf_width_x", leaf_width_.x);
      if (hasParam ("~downsample_leaf_width_y")) getParam ("~downsample_leaf_width_y", leaf_width_.y);
      if (hasParam ("~downsample_leaf_width_z")) getParam ("~downsample_leaf_width_z", leaf_width_.z);

      if (hasParam ("~cut_distance"))
      {
        getParam ("~cut_distance", cut_distance_);
        leaves_.resize (0);
        ROS_INFO ("Done clearing leaves.");
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      updateParametersFromServer ();

      ROS_INFO ("Received %d data points.", cloud_.pts.size ());
      if (cloud_.pts.size () == 0)
        return;

      // Figure out the viewpoint value in the base_link frame
      PointStamped viewpoint_laser, viewpoint_cloud;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0,0,0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf_.transformPoint ("base_link", viewpoint_laser, viewpoint_cloud);
      }
      catch (tf::ConnectivityException)
      {
        viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
      }

      timeval t1, t2;
      double time_spent;

      // If a-priori downsampling is enabled...
      if (downsample_)
      {
        gettimeofday (&t1, NULL);
        int d_idx = cloud_geometry::getChannelIndex (&cloud_, "distances");
        cloud_geometry::downsamplePointCloud (&cloud_, cloud_down_, leaf_width_, leaves_, d_idx, cut_distance_);

        gettimeofday (&t2, NULL);
        time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
        ROS_INFO ("Downsampling enabled. Number of points left: %d in %g seconds.", cloud_down_.pts.size (), time_spent);
      }

      // Resize
#ifdef DEBUG
      cloud_normals_.header = cloud_.header;
      cloud_normals_.pts.resize (cloud_.pts.size ());
      cloud_normals_.chan[0].vals.resize (cloud_.pts.size ());
#else
      // We need to copy the original point cloud data, and this looks like a good way to do it
      int original_chan_size;
      if (downsample_)
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
        if (downsample_)
          cloud_normals_.chan[d].vals.resize (cloud_down_.pts.size ());
        else
          cloud_normals_.chan[d].vals.resize (cloud_.pts.size ());
      }
#endif

      gettimeofday (&t1, NULL);

      // Create Kd-Tree
      kdtree_ = new cloud_kdtree::KdTree (&cloud_normals_);

      // Allocate enough space for point indices
      points_indices_.resize (cloud_normals_.pts.size ());
      for (int i = 0; i < (int)cloud_normals_.pts.size (); i++)
        points_indices_[i].resize (k_);

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Kd-tree created in %g seconds.", time_spent);

      gettimeofday (&t1, NULL);
      // Get the nerest neighbors for all points
      for (int i = 0; i < (int)cloud_normals_.pts.size (); i++)
      {
        vector<double> distances (k_);
        kdtree_->nearestKSearch (i, k_, points_indices_[i], distances);
      }
      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Nearest neighbors found in %g seconds.", time_spent);

      gettimeofday (&t1, NULL);
      #pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)cloud_normals_.pts.size (); i++)
      {
        // Compute the point normals (nx, ny, nz), surface curvature estimates (c), and moment invariants (j1, j2, j3)
        Eigen::Vector4d plane_parameters;
        double curvature, j1, j2, j3;
        cloud_geometry::nearest::computeSurfaceNormalCurvature (&cloud_normals_, &points_indices_[i], plane_parameters, curvature);
        if (compute_moments_)
          cloud_geometry::nearest::computeMomentInvariants (&cloud_normals_, &points_indices_[i], j1, j2, j3);

        // See if we need to flip any plane normals
        Point32 vp_m;
        vp_m.x = viewpoint_cloud.point.x - cloud_normals_.pts[i].x;
        vp_m.y = viewpoint_cloud.point.y - cloud_normals_.pts[i].y;
        vp_m.z = viewpoint_cloud.point.z - cloud_normals_.pts[i].z;
        //double norm = sqrt (vp_m.x * vp_m.x + vp_m.y * vp_m.y + vp_m.z * vp_m.z);

        // Dot product between the (viewpoint - point) and the plane normal
        double cos_theta = (vp_m.x * plane_parameters (0) + vp_m.y * plane_parameters (1) + vp_m.z * plane_parameters (2));// / norm;

        // Flip the plane normal
        if (cos_theta < 0)
        {
          for (int d = 0; d < 3; d++)
            plane_parameters (d) *= -1;
        }
#ifdef  DEBUG
        cloud_normals_.pts[i].x = plane_parameters (0);
        cloud_normals_.pts[i].y = plane_parameters (1);
        cloud_normals_.pts[i].z = plane_parameters (2);
        cloud_normals_.chan[0].vals[i] = fabs (plane_parameters (3));
#else
        cloud_normals_.chan[original_chan_size + 0].vals[i] = plane_parameters (0);
        cloud_normals_.chan[original_chan_size + 1].vals[i] = plane_parameters (1);
        cloud_normals_.chan[original_chan_size + 2].vals[i] = plane_parameters (2);
        cloud_normals_.chan[original_chan_size + 3].vals[i] = fabs (plane_parameters (3));
        if (compute_moments_)
        {
          cloud_normals_.chan[original_chan_size + 4].vals[i] = j1;
          cloud_normals_.chan[original_chan_size + 5].vals[i] = j2;
          cloud_normals_.chan[original_chan_size + 6].vals[i] = j3;
        }
#endif
      }

      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Local features estimated in %g seconds.", time_spent);

      publish ("cloud_normals", cloud_normals_);

      delete kdtree_;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  NormalEstimation p;
  p.spin ();

  ros::fini ();
  return (0);
}
/* ]--- */

