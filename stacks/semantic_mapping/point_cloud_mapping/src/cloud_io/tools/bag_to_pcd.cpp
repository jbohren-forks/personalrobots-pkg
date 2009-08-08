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

@b bag_pcd is a simple node that gets a complete point cloud and saves it into a PCD (Point Cloud Data) format.

 **/

// ROS core
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <fstream>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/point.h>

class BagToPcd
{
  protected:
    ros::NodeHandle nh_;

  public:

    // Save data to disk ?
    char fn_[80];
    bool dump_to_disk_;

    tf::TransformListener tf_;
    std::string cloud_topic_;

    ros::Subscriber cloud_sub_;

    ////////////////////////////////////////////////////////////////////////////////
    BagToPcd () : dump_to_disk_ (false)
    {
      cloud_topic_ = "tilt_laser_cloud";
      cloud_sub_ = nh_.subscribe (cloud_topic_, 1, &BagToPcd::cloud_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
    cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      if (cloud->points.size () == 0)
        return;
      geometry_msgs::PointStamped pin, pout;
      pin.header.frame_id = "laser_tilt_mount_link";
      pin.data.x = pin.data.y = pin.data.z = 0.0;

      try
      {
        tf_.transformPoint (cloud->header.frame_id, pin, pout);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF::ConectivityException caught while trying to transform a point from frame %s into %s!", cloud->header.frame_id.c_str (), pin.header.frame_id.c_str ());
      }
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s). Viewpoint is <%.3f, %.3f, %.3f>", (int)cloud->points.size (), cloud->header.frame_id.c_str (),
                (int)cloud->channels.size (), cloud_geometry::getAvailableChannels (cloud).c_str (), pout.data.x, pout.data.y, pout.data.z);

      double c_time = cloud->header.stamp.sec * 1e3 + cloud->header.stamp.nsec;
      sprintf (fn_, "%.0f.pcd", c_time);
      if (dump_to_disk_)
      {
        ROS_INFO ("Data saved to %s (%f).", fn_, c_time);
        {
          sensor_msgs::PointCloud cloud_out;
          cloud_out.header = cloud->header;
          cloud_out.points    = cloud->points;
          cloud_out.channels   = cloud->channels;
          // Add information about the viewpoint - rudimentary stuff
          if (cloud_geometry::getChannelIndex (cloud_out, "vx") == -1)
          {
            cloud_out.channels.resize (cloud->channels.size () + 3);
            cloud_out.channels[cloud->channels.size () - 3].name = "vx";
            cloud_out.channels[cloud->channels.size () - 2].name = "vy";
            cloud_out.channels[cloud->channels.size () - 1].name = "vz";
            cloud_out.channels[cloud->channels.size () - 3].values.resize (cloud->points.size ());
            cloud_out.channels[cloud->channels.size () - 2].values.resize (cloud->points.size ());
            cloud_out.channels[cloud->channels.size () - 1].values.resize (cloud->points.size ());
            for (unsigned int i = 0; i < cloud->points.size (); i++)
            {
              cloud_out.channels[cloud->channels.size () - 3].values[i] = pout.data.x;
              cloud_out.channels[cloud->channels.size () - 2].values[i] = pout.data.y;
              cloud_out.channels[cloud->channels.size () - 1].values[i] = pout.data.z;
            }
          }
          cloud_io::savePCDFileASCII (fn_, cloud_out, 5);
        }
      }
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "bag_pcd");

  BagToPcd b;
  b.dump_to_disk_ = true;
  ros::spin ();

  return (0);
}
/* ]--- */
