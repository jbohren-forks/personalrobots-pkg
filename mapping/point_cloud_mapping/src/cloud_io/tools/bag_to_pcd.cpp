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
#include <ros/node.h>

#include <tf/transform_listener.h>

#include <fstream>

#include <robot_msgs/PointStamped.h>
#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/point.h>

using namespace robot_msgs;

class BagToPcd
{
  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_;

    // Save data to disk ?
    char fn_[80];
    bool dump_to_disk_;

    tf::TransformListener tf_;
    std::string cloud_topic_;

    ////////////////////////////////////////////////////////////////////////////////
    BagToPcd (ros::Node& anode) : node_ (anode), dump_to_disk_ (false), tf_ (anode)
    {
      cloud_topic_ = "tilt_laser_cloud";
      node_.subscribe (cloud_topic_, cloud_, &BagToPcd::cloud_cb, this, 1);
      ROS_INFO ("Listening for incoming data on topic %s", node_.mapName (cloud_topic_).c_str ());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      PointStamped pin, pout;
      pin.header.frame_id = "laser_tilt_mount_link";
      pin.point.x = pin.point.y = pin.point.z = 0.0;

      try
      {
        tf_.transformPoint (cloud_.header.frame_id, pin, pout);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF::ConectivityException caught while trying to transform a point from frame %s into %s!", cloud_.header.frame_id.c_str (), pin.header.frame_id.c_str ());
      }
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s). Viewpoint is <%.3f, %.3f, %.3f>", (int)cloud_.pts.size (), cloud_.header.frame_id.c_str (),
                (int)cloud_.chan.size (), cloud_geometry::getAvailableChannels (cloud_).c_str (), pout.point.x, pout.point.y, pout.point.z);

      double c_time = cloud_.header.stamp.sec * 1e3 + cloud_.header.stamp.nsec;
      sprintf (fn_, "%.0f.pcd", c_time);
      if (dump_to_disk_)
      {
        {
          // Add information about the viewpoint - rudimentary stuff
          cloud_.chan.resize (cloud_.chan.size () + 3);
          cloud_.chan[cloud_.chan.size () - 3].name = "vx";
          cloud_.chan[cloud_.chan.size () - 2].name = "vy";
          cloud_.chan[cloud_.chan.size () - 1].name = "vz";
          cloud_.chan[cloud_.chan.size () - 3].vals.resize (cloud_.pts.size ());
          cloud_.chan[cloud_.chan.size () - 2].vals.resize (cloud_.pts.size ());
          cloud_.chan[cloud_.chan.size () - 1].vals.resize (cloud_.pts.size ());
          for (unsigned int i = 0; i < cloud_.pts.size (); i++)
          {
            cloud_.chan[cloud_.chan.size () - 3].vals[i] = pout.point.x;
            cloud_.chan[cloud_.chan.size () - 2].vals[i] = pout.point.y;
            cloud_.chan[cloud_.chan.size () - 1].vals[i] = pout.point.z;
          }
          cloud_io::savePCDFile (fn_, cloud_, 5);
        }
        ROS_INFO ("Data saved to %s (%f).", fn_, c_time);
      }
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("bag_pcd");
  BagToPcd b (ros_node);
  b.dump_to_disk_ = true;

  ros_node.spin ();

  return (0);
}
/* ]--- */
