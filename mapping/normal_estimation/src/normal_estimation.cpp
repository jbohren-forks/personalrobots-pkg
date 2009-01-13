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

@b normal_estimation estimates local surface properties at each 3D point, such as surface normals, curvature estimates,
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

class NormalEstimation : public ros::node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_normals_;

    std::vector<int> indices_;

    tf::TransformListener tf_;

    // Kd-tree stuff
    cloud_kdtree::KdTree *kdtree_;

    // Parameters
    bool compute_moments_;
    double radius_;
    int k_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    NormalEstimation () : ros::node ("normal_estimation"), tf_(*this)
    {
      param ("~search_radius", radius_, 0.02);      // 2cm radius by default
      param ("~search_k_closest", k_, 30);          // 30 k-neighbors by default
      param ("~compute_moments", compute_moments_, false);  // Do not compute moment invariants by default

      string cloud_topic ("tilt_laser_cloud");

      vector<pair<string, string> > t_list;
      get_published_topics (&t_list);
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (cloud_topic) == string::npos)
          ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());
      }

      subscribe (cloud_topic.c_str (), cloud_, &NormalEstimation::cloud_cb, 1);
      advertise<PointCloud> ("cloud_normals", 1);

      cloud_normals_.chan.resize (1);
      cloud_normals_.chan[0].name = "intensities";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~NormalEstimation () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points.", cloud_.pts.size ());
      if (cloud_.pts.size () == 0)
        return;

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

      // Resize
#ifdef DEBUG
      cloud_normals_.header = cloud_.header;
      cloud_normals_.pts.resize (cloud_.pts.size ());
      cloud_normals_.chan[0].vals.resize (cloud_.pts.size ());
#else
      cloud_normals_ = cloud_;
      cloud_normals_.chan.resize (cloud_.chan.size () + 7);     // Allocate 7 more channels
      cloud_normals_.chan[cloud_.chan.size () + 0].name = "nx";
      cloud_normals_.chan[cloud_.chan.size () + 1].name = "ny";
      cloud_normals_.chan[cloud_.chan.size () + 2].name = "nz";
      cloud_normals_.chan[cloud_.chan.size () + 3].name = "curvature";
      if (compute_moments_)
      {
        cloud_normals_.chan[cloud_.chan.size () + 4].name = "j1";
        cloud_normals_.chan[cloud_.chan.size () + 5].name = "j2";
        cloud_normals_.chan[cloud_.chan.size () + 6].name = "j3";
      }
      for (unsigned int d = cloud_.chan.size (); d < cloud_normals_.chan.size (); d++)
        cloud_normals_.chan[d].vals.resize (cloud_.pts.size ());
#endif

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Create Kd-Tree
      kdtree_ = new cloud_kdtree::KdTree (&cloud_);

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Kd-tree created in %g seconds.", time_spent);

      // Process all points
      gettimeofday (&t1, NULL);
      int nr_p = 0, nr_flipped = 0;
      std_msgs::Point32 vp_m;

      for (unsigned int i = 0; i < cloud_.pts.size (); i++)
      {
        if (kdtree_->nearestKSearch (i, k_))  // (kdtree_->radiusSearch (cloud_.pts[i], radius_))
        {
          kdtree_->getNeighborsIndices (indices_);

          // Compute the point normals (nx, ny, nz), surface curvature estimates (c), and moment invariants (j1, j2, j3)
          Eigen::Vector4d plane_parameters;
          double curvature, j1, j2, j3;
          cloud_geometry::nearest::computeSurfaceNormalCurvature (&cloud_, &indices_, plane_parameters, curvature);
          if (compute_moments_)
            cloud_geometry::nearest::computeMomentInvariants (&cloud_, &indices_, j1, j2, j3);

          // See if we need to flip any plane normals
          vp_m.x = viewpoint_cloud.point.x - cloud_.pts[i].x;
          vp_m.y = viewpoint_cloud.point.y - cloud_.pts[i].y;
          vp_m.z = viewpoint_cloud.point.z - cloud_.pts[i].z;
          double norm = sqrt (vp_m.x * vp_m.x + vp_m.y * vp_m.y + vp_m.z * vp_m.z);

          // Dot product between the (viewpoint - point) and the plane normal
          double cos_theta = (vp_m.x * plane_parameters (0) + vp_m.y * plane_parameters (1) + vp_m.z * plane_parameters (2)) / norm;

          // Flip the plane normal
          if (cos_theta < 0)
          {
            for (int d = 0; d < 3; d++)
              plane_parameters (d) *= -1;
            nr_flipped++;
          }
#ifdef DEBUG
          cloud_normals_.pts[nr_p].x = plane_parameters (0);
          cloud_normals_.pts[nr_p].y = plane_parameters (1);
          cloud_normals_.pts[nr_p].z = plane_parameters (2);
          cloud_normals_.chan[0].vals[nr_p] = fabs (plane_parameters (3));
#else
          cloud_normals_.chan[cloud_.chan.size () + 0].vals[nr_p] = plane_parameters (0);
          cloud_normals_.chan[cloud_.chan.size () + 1].vals[nr_p] = plane_parameters (1);
          cloud_normals_.chan[cloud_.chan.size () + 2].vals[nr_p] = plane_parameters (2);
          cloud_normals_.chan[cloud_.chan.size () + 3].vals[nr_p] = fabs (plane_parameters (3));
          if (compute_moments_)
          {
            cloud_normals_.chan[cloud_.chan.size () + 4].vals[nr_p] = j1;
            cloud_normals_.chan[cloud_.chan.size () + 5].vals[nr_p] = j2;
            cloud_normals_.chan[cloud_.chan.size () + 6].vals[nr_p] = j3;
          }
#endif
          nr_p++;
        }
        else
        {
          ROS_ERROR ("Error estimating features for point index %d!", i);
          nr_p++;
        }
      }
      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Local features estimated in %g seconds. Number of point normals flipped: %d.", time_spent, nr_flipped);

      publish ("cloud_normals", cloud_normals_);

      delete kdtree_;
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  fprintf (stderr, "This version has been deprecated. Please use normal_estimation_omp instead.\n");
  return (-1);

  ros::init (argc, argv);

  NormalEstimation p;
  p.spin ();

  ros::fini ();
  return (0);
}
/* ]--- */

