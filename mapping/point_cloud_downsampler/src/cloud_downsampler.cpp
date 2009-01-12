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

@b cloud_downsampler uniformly downsamples a 3D point cloud.

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <std_msgs/Point.h>
#include <std_msgs/PointCloud.h>

// Cloud geometry
#include <cloud_geometry/point.h>

#include <sys/time.h>

using namespace std;
using namespace std_msgs;

struct Leaf
{
  float centroid_x, centroid_y, centroid_z;
  int nr_points;
};

class CloudDownsampler : public ros::node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_down_;

    vector<Leaf> leaves_;

    // Parameters
    Point leaf_width_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CloudDownsampler () : ros::node ("cloud_downsampler")
    {
      param ("~leaf_width_x", leaf_width_.x, 0.025);      // 2.5cm radius by default
      param ("~leaf_width_y", leaf_width_.y, 0.025);      // 2.5cm radius by default
      param ("~leaf_width_z", leaf_width_.z, 0.025);      // 2.5cm radius by default

      ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);

      subscribe ("tilt_laser_cloud", cloud_, &CloudDownsampler::cloud_cb, 1);
      advertise<PointCloud> ("cloud_downsampled", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~CloudDownsampler () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      downsampleCloud (PointCloud *cloud_in, PointCloud &cloud_out)
    {
      // Copy the header (and thus the frame_id) + allocate enough space for points
      cloud_out.header = cloud_in->header;
      cloud_out.pts.resize (cloud_in->pts.size ());

      Point32 minP, maxP, minB, maxB, divB;
      cloud_geometry::getMinMax (cloud_, minP, maxP);

      // Compute the minimum and maximum bounding box values
      minB.x = (int)(floor (minP.x / leaf_width_.x));
      maxB.x = (int)(floor (maxP.x / leaf_width_.x));

      minB.y = (int)(floor (minP.y / leaf_width_.y));
      maxB.y = (int)(floor (maxP.y / leaf_width_.y));

      minB.z = (int)(floor (minP.z / leaf_width_.z));
      maxB.z = (int)(floor (maxP.z / leaf_width_.z));

      // Compute the number of divisions needed along all axis
      divB.x = maxB.x - minB.x + 1;
      divB.y = maxB.y - minB.y + 1;
      divB.z = maxB.z - minB.z + 1;

      // Allocate the space needed
      if (leaves_.capacity () < divB.x * divB.y * divB.z)
        leaves_.reserve (1.5 * divB.x * divB.y * divB.z);

      leaves_.resize (divB.x * divB.y * divB.z);

      for (unsigned int cl = 0; cl < leaves_.size (); cl++)
      {
        if (leaves_[cl].nr_points > 0)
        {
          leaves_[cl].centroid_x = leaves_[cl].centroid_y = leaves_[cl].centroid_z = 0.0;
          leaves_[cl].nr_points = 0;
        }
      }

      // First pass: go over all points and insert them into the right leaf
      for (unsigned int cp = 0; cp < cloud_.pts.size (); cp++)
      {
        int i = (int)(floor (cloud_.pts[cp].x / leaf_width_.x));
        int j = (int)(floor (cloud_.pts[cp].y / leaf_width_.y));
        int k = (int)(floor (cloud_.pts[cp].z / leaf_width_.z));

        int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
        leaves_[idx].centroid_x += cloud_.pts[cp].x;
        leaves_[idx].centroid_y += cloud_.pts[cp].y;
        leaves_[idx].centroid_z += cloud_.pts[cp].z;
        leaves_[idx].nr_points++;
      }

      // Second pass: go over all leaves and compute centroids
      int nr_p = 0;
      for (unsigned int cl = 0; cl < leaves_.size (); cl++)
      {
        if (leaves_[cl].nr_points > 0)
        {
          cloud_out.pts[nr_p].x = leaves_[cl].centroid_x / leaves_[cl].nr_points;
          cloud_out.pts[nr_p].y = leaves_[cl].centroid_y / leaves_[cl].nr_points;
          cloud_out.pts[nr_p].z = leaves_[cl].centroid_z / leaves_[cl].nr_points;
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

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      downsampleCloud (&cloud_, cloud_down_);

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Cloud downsampled in %g seconds. Number of points left: %d.", time_spent, cloud_down_.pts.size ());

      publish ("cloud_downsampled", cloud_down_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  CloudDownsampler p;
  p.spin ();

  ros::fini ();
  return (0);
}
/* ]--- */

