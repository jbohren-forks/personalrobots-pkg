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

#include <std_msgs/PointStamped.h>
#include <std_msgs/PointCloud.h>

#include <cloud_io/cloud_io.h>

using namespace std_msgs;

class BagToPcd: public ros::node
{
  public:

    // ROS messages
    PointCloud cloud_;

    // Save data to disk ?
    char fn_[80];
    bool dump_to_disk_;

    tf::TransformListener tf_;


    ////////////////////////////////////////////////////////////////////////////////
    BagToPcd () : ros::node ("bag_pcd"), dump_to_disk_(false), tf_(*this)
    {
      subscribe ("tilt_laser_cloud", cloud_, &BagToPcd::cloud_cb, 1);
    }

    ////////////////////////////////////////////////////////////////////////////////
    virtual ~BagToPcd () { }

    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Dump a point cloud to disk
     * \param fn the name of the output file
     * \param cloud the point cloud data message
     * \param vx the X coordinate of the viewpoint
     * \param vy the Y coordinate of the viewpoint
     * \param vz the Z coordinate of the viewpoint
     */
    void
      saveCloud (char *fn, PointCloud cloud, double vx, double vy, double vz)
    {
      std::ofstream fs;
      fs.precision (5);
      fs.open (fn);

      int nr_pts = cloud.get_pts_size ();
      int dim    = cloud.get_chan_size ();
      fs << "# [MetaInfo] Viewpoint " << vx << "," << vy << "," << vz << std::endl;
      fs << "COLUMNS x y z";
      for (int d = 0; d < dim; d++)
        fs << " " << cloud.chan[d].name;
      fs << std::endl;
      fs << "POINTS " << nr_pts << std::endl;
      fs << "DATA ascii" << std::endl;

      for (int i = 0; i < nr_pts; i++)
      {
        fs << cloud.pts[i].x << " " << cloud.pts[i].y << " " << cloud.pts[i].z;
        for (int d = 0; d < dim; d++)
          fs << " " << cloud.chan[d].vals[i];
        fs << std::endl;
      }
      fs.close ();
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      PointStamped pin, pout;
      pin.header.frame_id = "laser_tilt_mount_link";
      pin.point.x = pin.point.y = pin.point.z = 0.0;

      tf_.transformPoint ("base_link", pin, pout);

      fprintf (stderr, "Received %d data points. Viewpoint is <%.3f, %.3f, %.3f>\n", cloud_.pts.size (), pout.point.x, pout.point.y, pout.point.z);

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
        //saveCloud (fn_, cloud_, pout.point.x, pout.point.y, pout.point.z);
        fprintf (stderr, "Data saved to %s.\n", fn_);
      }
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  BagToPcd b;
  b.dump_to_disk_ = true;
  b.spin ();

  ros::fini ();

  return (0);
}
/* ]--- */

