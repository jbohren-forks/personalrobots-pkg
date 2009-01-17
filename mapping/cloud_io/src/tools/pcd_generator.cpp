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
#include <ros/time.h>
#include <ros/common.h>

#include <std_msgs/PointCloud.h>

#include <cloud_io/cloud_io.h>

#include <fstream>

using namespace std;

class PCDGenerator: public ros::Node
{
  public:

    // ROS messages
    std_msgs::PointCloud msg_cloud_;

    std::string file_name_;

    PCDGenerator () : ros::Node ("pcd_generator")
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      string cloud_topic ("cloud_pcd");
      advertise<std_msgs::PointCloud>(cloud_topic.c_str (), 1);
      ROS_INFO ("Publishing data on topic %s.", cloud_topic.c_str ());
    }

    ~PCDGenerator ()
    {
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      cloud_io::loadPCDFile (file_name_.c_str (), msg_cloud_);
      msg_cloud_.header.frame_id = "base_link";
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool spin ()
    {
      while (ok ())
      {
        usleep (1000000);

        publish ("cloud_pcd", msg_cloud_);
      }

      return true;
    }

  
};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 1)
  {
    ROS_ERROR ("Need one PCD file as parameter!");
    return (-1);
  }
  
  ros::init (argc, argv);

  PCDGenerator c;
  c.file_name_ = std::string (argv[1]);

  c.start ();
  c.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */
