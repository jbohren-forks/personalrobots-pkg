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

@b collision_map is a node providing a map of the occupied space around the robot as discretized boxes (center,
dimension) or spheres (center, radius), useful for collision detection.

The node can either send all requested map data or incremental updates/changes (additions) to the client/subscriber.

Additionally, the node can subtract all boxes/spheres which intersect with a given Oriented Bounding Box.

  \note This version assumes that all boxes have the same dimensions, but the message type allows for different box
sizes.

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <std_msgs/Point.h>
#include <std_msgs/PointCloud.h>

#include <Eigen/Core>
#include <cloud_geometry/transforms.h>

#include <boost/thread/mutex.hpp>

#include "collision_map/OrientedBoundingBox.h"
#include "collision_map/CollisionMap.h"
#include <tf/transform_listener.h>
#include <sys/time.h>

using namespace std;
using namespace std_msgs;
using namespace collision_map;

struct Leaf
{
  int i_, j_, k_;
  int nr_points_;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
  compareLeaf (const Leaf &l1, const Leaf &l2)
{
  if (l1.i_ < l2.i_)
    return (true);
  else if (l1.i_ > l2.i_)
    return (false);
  else if (l1.j_ < l2.j_)
    return (true);
  else if (l1.j_ > l2.j_)
    return (false);
  else if (l1.k_ < l2.k_)
    return (true);
  else
    return (false);
}

class CollisionMapper : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_;
    CollisionMap c_map_;
    OrientedBoundingBox box_sub_obj_;

    tf::TransformListener tf_;

    vector<Leaf> leaves_;

    // Parameters
    Point leaf_width_, robot_max_;
    bool only_updates_, subtract_object_;

    int min_nr_points_;
    
    int object_data_type_;
    double sphere_radius_;
    
    enum ObjectTypes { O_SPHERE, O_ORIENTEDBOX };
    
    boost::mutex m_lock_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CollisionMapper () : ros::Node ("collision_map"), tf_(*this)
    {
      param ("~object_type", object_data_type_, (int)O_SPHERE);
      param ("~leaf_width_x", leaf_width_.x, 0.015);       // 2.5cm diameter by default
      param ("~leaf_width_y", leaf_width_.y, 0.015);       // 2.5cm diameter by default
      param ("~leaf_width_z", leaf_width_.z, 0.015);       // 2.5cm diameter by default
      param ("~sphere_radius", sphere_radius_, 0.015);     // 1.5cm radius by default

      param ("~robot_max_x", robot_max_.x, 1.5);           // 1.5m radius by default
      param ("~robot_max_y", robot_max_.y, 1.5);           // 1.5m radius by default
      param ("~robot_max_z", robot_max_.z, 1.5);           // 1.5m radius by default

      param ("~min_nr_points", min_nr_points_, 1);         // Need at least 1 point per box to consider it "occupied"
      
      param ("~only_updates", only_updates_, false);       // Send the entire map or just incremental updates from the past state
      param ("~subtract_object", subtract_object_, false); // Subtract an OBB received via the collision_subtract_object topic
      
      if (subtract_object_)
        subscribe ("collision_subtract_object", box_sub_obj_, &CollisionMapper::subtract_cb, 1);
        
      ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);
      ROS_INFO ("Using a maximum bounding box around the robot of size: %g,%g,%g.", robot_max_.x, robot_max_.y, robot_max_.z);

      // Square the limits (simplified point distances below)
      robot_max_.x = robot_max_.x * robot_max_.x;
      robot_max_.y = robot_max_.y * robot_max_.y;
      robot_max_.z = robot_max_.z * robot_max_.z;

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

      subscribe (cloud_topic.c_str (), cloud_, &CollisionMapper::cloud_cb, 1);
      advertise<collision_map::CollisionMap> ("collision_map", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~CollisionMapper () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (hasParam ("~leaf_width_x")) getParam ("~leaf_width_x", leaf_width_.x);
      if (hasParam ("~leaf_width_y")) getParam ("~leaf_width_y", leaf_width_.y);
      if (hasParam ("~leaf_width_z")) getParam ("~leaf_width_z", leaf_width_.z);
      if (hasParam ("~sphere_radius")) getParam ("~sphere_radius", sphere_radius_);

      if (hasParam ("~robot_max_x"))
      {
        double rx;
        getParam ("~robot_max_x", rx);
        robot_max_.x = rx * rx;
      }
      if (hasParam ("~robot_max_y"))
      {
        double ry;
        getParam ("~robot_max_y", ry);
        robot_max_.y = ry * ry;
      }
      if (hasParam ("~robot_max_z"))
      {
        double rz;
        getParam ("~robot_max_z", rz);
        robot_max_.z = rz * rz;
      }

      if (hasParam ("~min_nr_points")) getParam ("~min_nr_points", min_nr_points_);
      if (hasParam ("~object_type")) getParam ("~object_type", object_data_type_);
      if (hasParam ("~only_updates")) getParam ("~only_updates", only_updates_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      computeCollisionMap (PointCloud *points, vector<Leaf> &leaves, CollisionMap &cmap)
    {
      // Copy the header (and implicitly the frame_id)
      cmap.header = cloud_.header;
      if (object_data_type_ == O_SPHERE)
        cmap.spheres.resize (cloud_.pts.size ());
      else if (object_data_type_ == O_ORIENTEDBOX)
        cmap.boxes.resize (cloud_.pts.size ());

      double sphere_diameter = 2 * sphere_radius_;

      PointStamped base_origin, torso_lift_origin;
      base_origin.point.x = base_origin.point.y = base_origin.point.z = 0.0;
      base_origin.header.frame_id = "torso_lift_link";
      base_origin.header.stamp = ros::Time();

      try
      {
        tf_.transformPoint ("base_link", base_origin, torso_lift_origin);
        //ROS_INFO ("Robot 'origin' is : %g,%g,%g", torso_lift_origin.point.x, torso_lift_origin.point.y, torso_lift_origin.point.z);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF not running or wrong TF frame specified! Defaulting to 0,0,0.");
        torso_lift_origin = base_origin;
      }
      // Get a set of point indices that respect our bounding limits around the robot
      vector<int> indices (cloud_.pts.size ());
      int nr_p = 0;

      Point32 minP, maxP;
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = FLT_MIN;
      double distance_sqr_x, distance_sqr_y, distance_sqr_z;
      for (unsigned int i = 0; i < cloud_.pts.size (); i++)
      {
        // We split the "distance" on all 3 dimensions to allow greater flexibility
        distance_sqr_x = fabs ((cloud_.pts[i].x - torso_lift_origin.point.x) * (cloud_.pts[i].x - torso_lift_origin.point.x));
        distance_sqr_y = fabs ((cloud_.pts[i].y - torso_lift_origin.point.y) * (cloud_.pts[i].y - torso_lift_origin.point.y));
        distance_sqr_z = fabs ((cloud_.pts[i].z - torso_lift_origin.point.z) * (cloud_.pts[i].z - torso_lift_origin.point.z));

        // If the point is within the bounds, use it for minP/maxP calculations
        if (distance_sqr_x < robot_max_.x && distance_sqr_y < robot_max_.y && distance_sqr_z < robot_max_.z)
        {
          minP.x = (cloud_.pts[i].x < minP.x) ? cloud_.pts[i].x : minP.x;
          minP.y = (cloud_.pts[i].y < minP.y) ? cloud_.pts[i].y : minP.y;
          minP.z = (cloud_.pts[i].z < minP.z) ? cloud_.pts[i].z : minP.z;

          maxP.x = (cloud_.pts[i].x > maxP.x) ? cloud_.pts[i].x : maxP.x;
          maxP.y = (cloud_.pts[i].y > maxP.y) ? cloud_.pts[i].y : maxP.y;
          maxP.z = (cloud_.pts[i].z > maxP.z) ? cloud_.pts[i].z : maxP.z;
          indices[nr_p] = i;
          nr_p++;
        }
      }
      indices.resize (nr_p);
      
      // Compute the minimum and maximum bounding box values
      Point32 minB, maxB, divB;
      
      if (object_data_type_ == O_SPHERE)
      {
        minB.x = (int)(floor (minP.x / sphere_diameter));
        maxB.x = (int)(floor (maxP.x / sphere_diameter));

        minB.y = (int)(floor (minP.y / sphere_diameter));
        maxB.y = (int)(floor (maxP.y / sphere_diameter));

        minB.z = (int)(floor (minP.z / sphere_diameter));
        maxB.z = (int)(floor (maxP.z / sphere_diameter));

      }
      else if (object_data_type_ == O_ORIENTEDBOX)
      {
        minB.x = (int)(floor (minP.x / leaf_width_.x));
        maxB.x = (int)(floor (maxP.x / leaf_width_.x));

        minB.y = (int)(floor (minP.y / leaf_width_.y));
        maxB.y = (int)(floor (maxP.y / leaf_width_.y));

        minB.z = (int)(floor (minP.z / leaf_width_.z));
        maxB.z = (int)(floor (maxP.z / leaf_width_.z));
      }

      // Compute the number of divisions needed along all axis
      divB.x = maxB.x - minB.x + 1;
      divB.y = maxB.y - minB.y + 1;
      divB.z = maxB.z - minB.z + 1;

      // Allocate the space needed (+ extra)
      if (leaves.capacity () < divB.x * divB.y * divB.z)
        leaves.reserve (divB.x * divB.y * divB.z);

      leaves.resize (divB.x * divB.y * divB.z);

      for (unsigned int cl = 0; cl < leaves.size (); cl++)
      {
        if (leaves[cl].nr_points_ > 0)
          leaves[cl].i_ = leaves[cl].j_ = leaves[cl].k_ = leaves[cl].nr_points_ = 0;
      }

      // First pass: go over all points and count them into the right leaf
      int i = 0, j = 0, k = 0;
      for (unsigned int cp = 0; cp < indices.size (); cp++)
      {
        if (object_data_type_ == O_SPHERE)
        {
          i = (int)(floor (cloud_.pts[indices.at (cp)].x / sphere_diameter));
          j = (int)(floor (cloud_.pts[indices.at (cp)].y / sphere_diameter));
          k = (int)(floor (cloud_.pts[indices.at (cp)].z / sphere_diameter));
        }
        else if (object_data_type_ == O_ORIENTEDBOX)
        {
          i = (int)(floor (cloud_.pts[indices.at (cp)].x / leaf_width_.x));
          j = (int)(floor (cloud_.pts[indices.at (cp)].y / leaf_width_.y));
          k = (int)(floor (cloud_.pts[indices.at (cp)].z / leaf_width_.z));
        }

        int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
        leaves[idx].i_ = i;
        leaves[idx].j_ = j;
        leaves[idx].k_ = k;
        leaves[idx].nr_points_++;
      }

      // Second pass: go over all leaves and add them to the map
      int nr_c = 0;
      for (unsigned int cl = 0; cl < leaves.size (); cl++)
      {
        if (leaves[cl].nr_points_ >= min_nr_points_)
        {
          if (object_data_type_ == O_SPHERE)
          {
            cmap.spheres[nr_c].radius = sphere_radius_;
            cmap.spheres[nr_c].center.x = (leaves[cl].i_ + 1) * sphere_diameter - sphere_radius_;
            cmap.spheres[nr_c].center.y = (leaves[cl].j_ + 1) * sphere_diameter - sphere_radius_;
            cmap.spheres[nr_c].center.z = (leaves[cl].k_ + 1) * sphere_diameter - sphere_radius_;
          }
          else if (object_data_type_ == O_ORIENTEDBOX)
          {
            cmap.boxes[nr_c].extents.x = leaf_width_.x / 2.0;
            cmap.boxes[nr_c].extents.y = leaf_width_.y / 2.0;
            cmap.boxes[nr_c].extents.z = leaf_width_.z / 2.0;
            cmap.boxes[nr_c].center.x = (leaves[cl].i_ + 1) * leaf_width_.x - cmap.boxes[nr_c].extents.x; // + minB.x;
            cmap.boxes[nr_c].center.y = (leaves[cl].j_ + 1) * leaf_width_.y - cmap.boxes[nr_c].extents.y; // + minB.y;
            cmap.boxes[nr_c].center.z = (leaves[cl].k_ + 1) * leaf_width_.z - cmap.boxes[nr_c].extents.z; // + minB.z;
            cmap.boxes[nr_c].axis.x = cmap.boxes[nr_c].axis.y = cmap.boxes[nr_c].axis.z = 0.0;
            cmap.boxes[nr_c].angle = 0.0;
          }
          nr_c++;
        }
      }
      if (object_data_type_ == O_SPHERE)
        cmap.spheres.resize (nr_c);
      if (object_data_type_ == O_ORIENTEDBOX)
        cmap.boxes.resize (nr_c);
        
      sort (leaves.begin (), leaves.end (), compareLeaf);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      subtractCollisionMap (vector<Leaf> *prev_model, vector<Leaf> *cur_model, CollisionMap &map)
    {
      vector<Leaf> model_difference;

      double sphere_diameter = 2 * sphere_radius_;

      // Assume the models are sorted
      set_difference (prev_model->begin (), prev_model->end (), cur_model->begin (), cur_model->end (),
                      inserter (model_difference, model_difference.begin ()), compareLeaf);

      // Create the map
      int nr_c = 0;
      for (unsigned int cl = 0; cl < model_difference.size (); cl++)
      {
        if (model_difference[cl].nr_points_ >= min_nr_points_)
        {
          if (object_data_type_ == O_SPHERE)
          {
            map.spheres[nr_c].radius = sphere_radius_;
            map.spheres[nr_c].center.x = (model_difference[cl].i_ + 1) * sphere_diameter - sphere_radius_;
            map.spheres[nr_c].center.y = (model_difference[cl].j_ + 1) * sphere_diameter - sphere_radius_;
            map.spheres[nr_c].center.z = (model_difference[cl].k_ + 1) * sphere_diameter - sphere_radius_;
          }
          else if (object_data_type_ == O_ORIENTEDBOX)
          {
            map.boxes[nr_c].extents.x = leaf_width_.x / 2.0;
            map.boxes[nr_c].extents.y = leaf_width_.y / 2.0;
            map.boxes[nr_c].extents.z = leaf_width_.z / 2.0;
            map.boxes[nr_c].center.x = (model_difference[cl].i_ + 1) * leaf_width_.x - map.boxes[nr_c].extents.x; // + minB.x;
            map.boxes[nr_c].center.y = (model_difference[cl].j_ + 1) * leaf_width_.y - map.boxes[nr_c].extents.y; // + minB.y;
            map.boxes[nr_c].center.z = (model_difference[cl].k_ + 1) * leaf_width_.z - map.boxes[nr_c].extents.z; // + minB.z;
            map.boxes[nr_c].axis.x = map.boxes[nr_c].axis.y = map.boxes[nr_c].axis.z = 0.0;
            map.boxes[nr_c].angle = 0.0;
          }
          nr_c++;
        }
      }
      if (object_data_type_ == O_SPHERE)
        map.spheres.resize (nr_c);
      if (object_data_type_ == O_ORIENTEDBOX)
        map.boxes.resize (nr_c);
    }
      
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb ()
    {
      ROS_INFO ("Received %u data points.", (unsigned int)cloud_.pts.size ());

      m_lock_.lock ();
      updateParametersFromServer ();
      m_lock_.unlock ();

      timeval t1, t2;
      double time_spent;
      
      // @bogus message for Tully - Radu discussion next week
      ROS_WARN ("Did you transform your points into the map frame today?");
            
      gettimeofday (&t1, NULL);
      // If we're only interested in doing map updates
      if (only_updates_ && leaves_.size () > 0)
      {
        CollisionMap new_c_map;
        vector<Leaf> new_leaves;

        computeCollisionMap (&cloud_, new_leaves, new_c_map);
        
        c_map_.header = cloud_.header;
        if (object_data_type_ == O_SPHERE)
          c_map_.spheres.resize (max (new_leaves.size (), leaves_.size ()));
        else if (object_data_type_ == O_ORIENTEDBOX)
          c_map_.boxes.resize (max (new_leaves.size (), leaves_.size ()));

        m_lock_.lock ();
        subtractCollisionMap (&leaves_, &new_leaves, c_map_);
        m_lock_.unlock ();
      }
      else
      {
        m_lock_.lock ();
        computeCollisionMap (&cloud_, leaves_, c_map_);
        m_lock_.unlock ();
      }
      
      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      if (object_data_type_ == O_SPHERE)
        ROS_INFO ("Collision map computed in %g seconds. Number of spheres: %u.", time_spent, (unsigned int)c_map_.spheres.size ());
      else if (object_data_type_ == O_ORIENTEDBOX)
        ROS_INFO ("Collision map computed in %g seconds. Number of boxes: %u.", time_spent, (unsigned int)c_map_.boxes.size ());

      publish ("collision_map", c_map_);
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
      getRotatedBoxBounds (OrientedBoundingBox *box, Eigen::Matrix3d rotation, Eigen::Vector3d &minB, Eigen::Vector3d &maxB)
    {
      // Rotate the 2 bounding points using the given transform
      minB (0) = box->center.x - box->extents.x;
      minB (1) = box->center.y - box->extents.y;
      minB (2) = box->center.z - box->extents.z;

      maxB (0) = box->center.x + box->extents.x;
      maxB (1) = box->center.y + box->extents.y;
      maxB (2) = box->center.z + box->extents.z;
      
      minB = rotation * minB;
      maxB = rotation * maxB;
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      isBoxInsideBounds (Point32 *center, Point32 *extents, Eigen::Vector3d minB, Eigen::Vector3d maxB)
    {
      // Check all 8 points
      float ce_x = center->x - extents->x, ce_y = center->y - extents->y, ce_z = center->z - extents->z;
      float cex  = center->x + extents->x, cey  = center->y + extents->y, cez  = center->z + extents->z;

      // (ce_x, ce_y, ce_z);
      if ( (ce_x >= minB (0) && ce_x <= maxB (0)) && (ce_y >= minB (1) && ce_y <= maxB (1)) && (ce_z >= minB (2) && ce_z <= maxB (2)) )
        return (true);
      // (ce_x, cey, ce_z);
      if ( (ce_x >= minB (0) && ce_x <= maxB (0)) && (cey >= minB (1) && cey <= maxB (1)) && (ce_z >= minB (2) && ce_z <= maxB (2)) )
        return (true);
      // (cex, cey, ce_z);
      if ( (cex >= minB (0) && cex <= maxB (0)) && (cey >= minB (1) && cey <= maxB (1)) && (ce_z >= minB (2) && ce_z <= maxB (2)) )
        return (true);
      // (cex, ce_y, ce_z);
      if ( (cex >= minB (0) && cex <= maxB (0)) && (ce_y >= minB (1) && ce_y <= maxB (1)) && (ce_z >= minB (2) && ce_z <= maxB (2)) )
        return (true);
      // (cex, ce_y, cez);
      if ( (cex >= minB (0) && cex <= maxB (0)) && (ce_y >= minB (1) && ce_y <= maxB (1)) && (cez >= minB (2) && cez <= maxB (2)) )
        return (true);
      // (cex, cey, cez);
      if ( (cex >= minB (0) && cex <= maxB (0)) && (cey >= minB (1) && cey <= maxB (1)) && (cez >= minB (2) && cez <= maxB (2)) )
        return (true);
      // (ce_x, cey, cez);
      if ( (ce_x >= minB (0) && ce_x <= maxB (0)) && (cey >= minB (1) && cey <= maxB (1)) && (cez >= minB (2) && cez <= maxB (2)) )
        return (true);
      // (ce_x, ce_y, cez);
      if ( (ce_x >= minB (0) && ce_x <= maxB (0)) && (ce_y >= minB (1) && ce_y <= maxB (1)) && (cez >= minB (2) && cez <= maxB (2)) )
        return (true);
        
      return (false);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      subtract_cb ()
    {
      ROS_INFO ("Received OBB with the following coordinates: center [%f, %f, %f], extents [%f, %f, %f], axis [%f, %f, %f], angle [%f]",
                box_sub_obj_.center.x, box_sub_obj_.center.y, box_sub_obj_.center.z,
                box_sub_obj_.extents.x, box_sub_obj_.extents.y, box_sub_obj_.extents.z,
                box_sub_obj_.axis.x, box_sub_obj_.axis.y, box_sub_obj_.axis.z,
                box_sub_obj_.angle);

      // Get the inverse rotation matrix from the axis-angle
      Eigen::Matrix3d rot_mat;
      cloud_geometry::transforms::convertAxisAngleToRotationMatrix (box_sub_obj_.axis, -box_sub_obj_.angle, rot_mat);
      
      Eigen::Vector3d minB, maxB;
      getRotatedBoxBounds (&box_sub_obj_, rot_mat, minB, maxB);
      
      m_lock_.lock ();
      updateParametersFromServer ();
      m_lock_.unlock ();

      timeval t1, t2;
      double time_spent;
      
      gettimeofday (&t1, NULL);
      
      // Subtract the received oriented bounding box
      m_lock_.lock ();
      Point32 center, extents;
      if (leaves_.size () > 0)
      {
        // Check all leaves against the given OBB
        for (unsigned int cl = 0; cl < leaves_.size (); cl++)
        {
          if (leaves_[cl].nr_points_ >= min_nr_points_) // Only check valid leaves
          {
            // Test if any of the points of the current leaf/box are inside the OBB
            extents.x = leaf_width_.x / 2.0;
            extents.y = leaf_width_.y / 2.0;
            extents.z = leaf_width_.z / 2.0;
            center.x = (leaves_[cl].i_ + 1) * leaf_width_.x - extents.x;
            center.y = (leaves_[cl].j_ + 1) * leaf_width_.y - extents.y;
            center.z = (leaves_[cl].k_ + 1) * leaf_width_.z - extents.z;
            
            bool inside = isBoxInsideBounds (&center, &extents, minB, maxB);
            
            if (inside)
            {
              // Invalidate this leaf
              leaves_[cl].nr_points_ = 0;
            }
          }
          
        }
      }
      m_lock_.unlock ();
      
      gettimeofday (&t2, NULL);
      time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
      ROS_INFO ("OBB subtracted from the map in %g seconds.", time_spent);
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

