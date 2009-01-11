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
 * $Id: collision_map.cpp 9108 2009-01-09 05:13:36Z veedee $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b collision_map is a node providing a map of the occupied space around the robot as discretized boxes (center,
dimension), useful for collision detection.

  \note This version assumes that all boxes have the same dimensions, but the message type allows for different box
sizes.

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <std_msgs/Point.h>
#include <std_msgs/PointCloud.h>

// Cloud geometry
#include <cloud_geometry/point.h>

#include "collision_map/CollisionMap.h"
#include <tf/transform_listener.h>
#include <sys/time.h>

using namespace std;
using namespace std_msgs;

struct Leaf
{
  int i_, j_, k_;
  int nr_points_;
};

class CollisionMapper : public ros::node
{
  public:

    // ROS messages
    PointCloud cloud_;
    collision_map::CollisionMap c_map_;

    tf::TransformListener tf_;

    vector<Leaf> leaves_;

    // Parameters
    Point leaf_width_, robot_max_;

    int min_nr_points_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CollisionMapper () : ros::node ("collision_map"), tf_(*this)
    {
      param ("~leaf_width_x", leaf_width_.x, 0.025);      // 2.5cm radius by default
      param ("~leaf_width_y", leaf_width_.y, 0.025);      // 2.5cm radius by default
      param ("~leaf_width_z", leaf_width_.z, 0.025);      // 2.5cm radius by default

      param ("~robot_max_x", robot_max_.x, 1.5);          // 1.5m radius by default
      param ("~robot_max_y", robot_max_.y, 1.5);          // 1.5m radius by default
      param ("~robot_max_z", robot_max_.z, 1.5);          // 1.5m radius by default

      param ("~min_nr_points", min_nr_points_, 2);        // Need at least 2 points per box to consider it "occupied"

      ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);
      ROS_INFO ("Using a maximum bounding box around the robot of size: %g,%g,%g.", robot_max_.x, robot_max_.y, robot_max_.z);

      subscribe ("tilt_laser_cloud", cloud_, &CollisionMapper::cloud_cb, 1);
      advertise<collision_map::CollisionMap> ("collision_map", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~CollisionMapper () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      // Copy the header (and implicitly the frame_id)
      c_map_.header = cloud_.header;
      c_map_.centers.resize (cloud_.pts.size ());
      c_map_.dimensions.resize (cloud_.pts.size ());

      //updateParametersFromServer ();
      // Square the limits
      robot_max_.x *= robot_max_.x;
      robot_max_.y *= robot_max_.y;
      robot_max_.z *= robot_max_.z;

      PointStamped base_link_origin, map_origin;
      base_link_origin.point.x = base_link_origin.point.y = base_link_origin.point.z = 0.0;
      base_link_origin.header.frame_id = "base_link";
      base_link_origin.header.stamp = 0;

//       tf_.transformPoint ("base_link", base_link_origin, map_origin);
      ROS_INFO ("Received %d data points.", cloud_.pts.size ());

      timeval t1, t2;
      gettimeofday (&t1, NULL);

      // Get a set of point indices that respect our bounding limits around the robot
      Point32 minP, maxP;
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = FLT_MIN;
      double distance_sqr_x, distance_sqr_y, distance_sqr_z;
      for (unsigned int i = 0; i < cloud_.pts.size (); i++)
      {
        // We split the "distance" on all 3 dimensions to allow greater flexibility
        distance_sqr_x = fabs ((cloud_.pts[i].x - base_link_origin.point.x) * (cloud_.pts[i].x - base_link_origin.point.x));
        distance_sqr_y = fabs ((cloud_.pts[i].y - base_link_origin.point.y) * (cloud_.pts[i].y - base_link_origin.point.y));
        distance_sqr_z = fabs ((cloud_.pts[i].z - base_link_origin.point.z) * (cloud_.pts[i].z - base_link_origin.point.z));

        // If the point is within the bounds, use it for minP/maxP calculations
        if (distance_sqr_x < robot_max_.x && distance_sqr_y < robot_max_.y && distance_sqr_z < robot_max_.z)
        {
          minP.x = (cloud_.pts[i].x < minP.x) ? cloud_.pts[i].x : minP.x;
          minP.y = (cloud_.pts[i].y < minP.y) ? cloud_.pts[i].y : minP.y;
          minP.z = (cloud_.pts[i].z < minP.z) ? cloud_.pts[i].z : minP.z;

          maxP.x = (cloud_.pts[i].x > maxP.x) ? cloud_.pts[i].x : maxP.x;
          maxP.y = (cloud_.pts[i].y > maxP.y) ? cloud_.pts[i].y : maxP.y;
          maxP.z = (cloud_.pts[i].z > maxP.z) ? cloud_.pts[i].z : maxP.z;
        }
      }

      // Compute the minimum and maximum bounding box values
      Point32 minB, maxB, divB;
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

      // Allocate the space needed (+ extra)
      if (leaves_.capacity () < divB.x * divB.y * divB.z)
        leaves_.reserve (divB.x * divB.y * divB.z);

      leaves_.resize (divB.x * divB.y * divB.z);

      for (unsigned int cl = 0; cl < leaves_.size (); cl++)
      {
        if (leaves_[cl].nr_points_ > 0)
          leaves_[cl].i_ = leaves_[cl].j_ = leaves_[cl].k_ = leaves_[cl].nr_points_ = 0;
      }

      // First pass: go over all points and count them into the right leaf
      for (unsigned int cp = 0; cp < cloud_.pts.size (); cp++)
      {
        int i = (int)(floor (cloud_.pts[cp].x / leaf_width_.x));
        int j = (int)(floor (cloud_.pts[cp].y / leaf_width_.y));
        int k = (int)(floor (cloud_.pts[cp].z / leaf_width_.z));

        int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
        leaves_[idx].i_ = i;
        leaves_[idx].j_ = j;
        leaves_[idx].k_ = k;
        leaves_[idx].nr_points_++;
      }

      // Second pass: go over all leaves and add them to the map
      int nr_c = 0;
      for (unsigned int cl = 0; cl < leaves_.size (); cl++)
      {
        if (leaves_[cl].nr_points_ > min_nr_points_)
        {
          c_map_.centers[nr_c].x = leaves_[cl].i_ * divB.x + minB.x;
          c_map_.centers[nr_c].y = leaves_[cl].j_ * divB.y + minB.y;
          c_map_.centers[nr_c].z = leaves_[cl].k_ * divB.z + minB.z;
          c_map_.dimensions[nr_c].x = leaf_width_.x;
          c_map_.dimensions[nr_c].y = leaf_width_.y;
          c_map_.dimensions[nr_c].z = leaf_width_.z;

          nr_c++;
        }
      }
      c_map_.centers.resize (nr_c);
      c_map_.dimensions.resize (nr_c);

      gettimeofday (&t2, NULL);
      double time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("Collision map computed in %g seconds. Number of boxes: %d.", time_spent, c_map_.centers.size ());

      publish ("collision_map", c_map_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  CollisionMapper p;
  p.spin ();

  ros::fini ();
  return (0);
}
/* ]--- */

