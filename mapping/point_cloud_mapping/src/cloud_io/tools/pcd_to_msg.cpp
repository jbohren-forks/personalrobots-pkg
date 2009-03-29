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

@b pcd_generator is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages.

 **/

// ROS core
#include <ros/node.h>
//#include <ros/time.h>

#include <robot_msgs/PointCloud.h>

#include <point_cloud_mapping/cloud_io.h>

#include <tf/transform_broadcaster.h>

using namespace std;

class PCDGenerator
{
  protected:
    string tf_frame_;
    ros::Node& node_;
    tf::TransformBroadcaster broadcaster_;
    tf::Stamped<tf::Transform> transform_;
    
  public:

    // ROS messages
    robot_msgs::PointCloud msg_cloud_;

    string file_name_, cloud_topic_;
    double rate_;

    PCDGenerator (ros::Node& anode) : tf_frame_ ("base_link"), node_ (anode), broadcaster_ (anode),
                                      transform_ (btTransform (btQuaternion (0, 0, 0), btVector3 (0, 0, 0)), ros::Time::now (), tf_frame_, tf_frame_)
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      cloud_topic_ = "cloud_pcd";
      node_.advertise<robot_msgs::PointCloud> (cloud_topic_.c_str (), 1);
      ROS_INFO ("Publishing data on topic %s.", node_.mapName (cloud_topic_).c_str ());
      
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      if (file_name_ == "" || cloud_io::loadPCDFile (file_name_.c_str (), msg_cloud_) == -1)
        return (-1);
      msg_cloud_.header.frame_id = tf_frame_;
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      double interval = rate_ * 1e+6;
      while (node_.ok ())
      {
        usleep (interval);

        transform_.stamp_ = ros::Time::now ();
        broadcaster_.sendTransform (transform_);

        ROS_INFO ("Publishing data (%d points) on topic %s in frame %s.", (int)msg_cloud_.pts.size (), node_.mapName (cloud_topic_).c_str (), msg_cloud_.header.frame_id.c_str ());
        msg_cloud_.header.stamp = ros::Time::now ();
        node_.publish ("cloud_pcd", msg_cloud_);
      }

      return (true);
    }


};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_ERROR ("Syntax is: %s <file.pcd> rate", argv[0]);
    return (-1);
  }

  ros::init (argc, argv);

  ros::Node ros_node ("pcd_generator");

  PCDGenerator c (ros_node);
  c.file_name_ = string (argv[1]);
  c.rate_      = atof (argv[2]);
  ROS_INFO ("Loading file %s...", c.file_name_.c_str ());

  if (c.start () == -1)
  {
    ROS_ERROR ("Could not load file. Exiting.");
    return (-1);
  }
  c.spin ();

  return (0);
}
/* ]--- */
