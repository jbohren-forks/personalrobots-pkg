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

class CloudDownsampler : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_, cloud_down_;

    vector<cloud_geometry::Leaf> leaves_;

    // Parameters
    Point leaf_width_;
    double cut_distance_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CloudDownsampler () : ros::Node ("cloud_downsampler")
    {
      param ("~leaf_width_x", leaf_width_.x, 0.025);      // 2.5cm radius by default
      param ("~leaf_width_y", leaf_width_.y, 0.025);      // 2.5cm radius by default
      param ("~leaf_width_z", leaf_width_.z, 0.025);      // 2.5cm radius by default

      ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);

      param ("~cut_distance", cut_distance_, 10.0);         // 10m by default

      string cloud_topic ("tilt_laser_cloud");

      vector<pair<string, string> > t_list;
      getPublishedTopics (&t_list);
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (cloud_topic) == string::npos)
          ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", cloud_topic.c_str ());
      }

      subscribe (cloud_topic.c_str (), cloud_, &CloudDownsampler::cloud_cb, 1);
      advertise<PointCloud> ("cloud_downsampled", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~CloudDownsampler () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points.", cloud_.pts.size ());
      if (cloud_.pts.size () == 0)
        return;

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      int d_idx = cloud_geometry::getChannelIndex (&cloud_, "distances");
      cloud_geometry::downsamplePointCloud (&cloud_, cloud_down_, leaf_width_, leaves_, d_idx, cut_distance_);

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

