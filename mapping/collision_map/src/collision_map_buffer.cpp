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

@b collision_map_buffer is a node providing a map of the occupied space around the robot as discretized boxes (center,
dimension, orientation) useful for collision detection.

 **/

// ROS core
#include <ros/node.h>
// ROS messages
#include <std_msgs/Point.h>
#include <std_msgs/PointCloud.h>

#include <Eigen/Core>
#include <cloud_geometry/transforms.h>

#include <boost/thread/mutex.hpp>

#include <robot_msgs/OrientedBoundingBox.h>
#include <robot_msgs/CollisionMap.h>
#include <robot_srvs/RecordStaticMapTrigger.h>
#include <robot_srvs/SubtractObjectFromCollisionMap.h>

#include <tf/transform_listener.h>
#include <sys/time.h>

using namespace std;
using namespace std_msgs;
using namespace robot_msgs;
using namespace robot_srvs;

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

class CollisionMapperBuffer : public ros::Node
{
  public:

    // ROS messages
    PointCloud cloud_;
    CollisionMap final_collision_map_;

    vector<Leaf> static_leaves_, cur_leaves_, final_leaves_;
    list<vector<Leaf> > decaying_maps_;

    OrientedBoundingBox box_sub_obj_;

    tf::TransformListener tf_;

    // Parameters
    Point leaf_width_, robot_max_;

    int min_nr_points_;

    boost::mutex static_map_lock_, m_lock_;

    // The size of the buffer window
    int window_size_;
    bool acquire_static_map_;
    ros::Time acquire_static_map_time_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CollisionMapperBuffer () : ros::Node ("collision_map_buffer"), tf_(*this)
    {
      param ("~leaf_width_x", leaf_width_.x, 0.02);       // 2cm diameter by default
      param ("~leaf_width_y", leaf_width_.y, 0.02);       // 2cm diameter by default
      param ("~leaf_width_z", leaf_width_.z, 0.02);       // 2cm diameter by default

      param ("~robot_max_x", robot_max_.x, 1.5);           // 1.5m radius by default
      param ("~robot_max_y", robot_max_.y, 1.5);           // 1.5m radius by default
      param ("~robot_max_z", robot_max_.z, 1.5);           // 1.5m radius by default

      param ("~min_nr_points", min_nr_points_, 1);         // Need at least 1 point per box to consider it "occupied"

      ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);
      ROS_INFO ("Using a maximum bounding box around the robot of size: %g,%g,%g.", robot_max_.x, robot_max_.y, robot_max_.z);

      param ("~window_size", window_size_, 5);             // Use the latest 5 collision maps + static by default

      // Square the limits (simplified point distances below)
      robot_max_.x = robot_max_.x * robot_max_.x;
      robot_max_.y = robot_max_.y * robot_max_.y;
      robot_max_.z = robot_max_.z * robot_max_.z;

      string cloud_topic ("full_cloud");

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

      subscribe (cloud_topic.c_str (), cloud_, &CollisionMapperBuffer::cloud_cb, 1);
      advertise<CollisionMap> ("collision_map_buffer", 1);

      advertiseService ("~record_static_map", &CollisionMapperBuffer::getStaticMap, this);
      advertiseService ("~subtract_object", &CollisionMapperBuffer::subtractObject, this);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~CollisionMapperBuffer () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (hasParam ("~leaf_width_x")) getParam ("~leaf_width_x", leaf_width_.x);
      if (hasParam ("~leaf_width_y")) getParam ("~leaf_width_y", leaf_width_.y);
      if (hasParam ("~leaf_width_z")) getParam ("~leaf_width_z", leaf_width_.z);

      if (hasParam ("~window_size")) getParam ("~window_size", window_size_);

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
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute a CollisionMap from a Leaf vector.
      * \param leaves the Leaf vector 
      * \param cmap the resultant collision map
      */
    void 
      computeCollisionMapFromLeaves (vector<Leaf> *leaves, CollisionMap &cmap)
    {
      cmap.boxes.resize (leaves->size ());
      // Second pass: go over all leaves and add them to the map
      int nr_c = 0;
      for (unsigned int cl = 0; cl < leaves->size (); cl++)
      {
        if (leaves->at (cl).nr_points_ >= min_nr_points_)
        {
          cmap.boxes[nr_c].extents.x = leaf_width_.x / 2.0;
          cmap.boxes[nr_c].extents.y = leaf_width_.y / 2.0;
          cmap.boxes[nr_c].extents.z = leaf_width_.z / 2.0;
          cmap.boxes[nr_c].center.x = (leaves->at (cl).i_ + 1) * leaf_width_.x - cmap.boxes[nr_c].extents.x; // + minB.x;
          cmap.boxes[nr_c].center.y = (leaves->at (cl).j_ + 1) * leaf_width_.y - cmap.boxes[nr_c].extents.y; // + minB.y;
          cmap.boxes[nr_c].center.z = (leaves->at (cl).k_ + 1) * leaf_width_.z - cmap.boxes[nr_c].extents.z; // + minB.z;
          cmap.boxes[nr_c].axis.x = cmap.boxes[nr_c].axis.y = cmap.boxes[nr_c].axis.z = 0.0;
          cmap.boxes[nr_c].angle = 0.0;
          nr_c++;
        }
      }
      cmap.boxes.resize (nr_c);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute a Leaf vector from an unorganized PointCloud
      * \param points the PointCloud message
      * \param leaves the resultant Leaf vector
      */
    void 
      computeLeaves (PointCloud *points, vector<Leaf> &leaves)
    {
      PointStamped base_origin, torso_lift_origin;
      base_origin.point.x = base_origin.point.y = base_origin.point.z = 0.0;
      base_origin.header.frame_id = "torso_lift_link";
      base_origin.header.stamp = ros::Time();

      try
      {
        tf_.transformPoint (points->header.frame_id, base_origin, torso_lift_origin);
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
        i = (int)(floor (cloud_.pts[indices.at (cp)].x / leaf_width_.x));
        j = (int)(floor (cloud_.pts[indices.at (cp)].y / leaf_width_.y));
        k = (int)(floor (cloud_.pts[indices.at (cp)].z / leaf_width_.z));

        int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
        leaves[idx].i_ = i;
        leaves[idx].j_ = j;
        leaves[idx].k_ = k;
        leaves[idx].nr_points_++;
      }

      sort (leaves.begin (), leaves.end (), compareLeaf);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Pointcloud callback function */
    void
      cloud_cb ()
    {
      m_lock_.lock ();
      updateParametersFromServer ();
      m_lock_.unlock ();

      timeval t1, t2;
      double time_spent;

      // Copy the header (and implicitly the frame_id)
      final_collision_map_.header = cloud_.header;

      ROS_INFO ("Received %u data points.", (unsigned int)cloud_.pts.size ());

      // Static map acquisition has been triggered via the service call
      if (acquire_static_map_)
      {
        // Do not compute any collision maps until we receive a cloud with a higher timestamp 
        if (cloud_.header.stamp < acquire_static_map_time_)
          return;

        // Compute the static collision map
        gettimeofday (&t1, NULL);

        computeLeaves (&cloud_, static_leaves_);

        // Clear the static map flag
        static_map_lock_.lock ();
        acquire_static_map_ = false;
        static_map_lock_.unlock ();

        gettimeofday (&t2, NULL);
        time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
        ROS_INFO ("Static collision map computed in %g seconds. Number of boxes: %u.", time_spent, (unsigned int)static_leaves_.size ());

      }
      else
      {
        // Rotate N maps in the queue
        gettimeofday (&t1, NULL);

        m_lock_.lock ();
        computeLeaves (&cloud_, cur_leaves_);
        m_lock_.unlock ();

        decaying_maps_.push_back (cur_leaves_);

        // If we have window_size maps, combine them together
        vector<Leaf> model_reunion;
        if ((int)decaying_maps_.size () > window_size_)
        {
          final_leaves_.clear ();
          list<vector<Leaf> >::const_iterator it = decaying_maps_.begin ();
          for ( ; it != decaying_maps_.end (); ++it)
          {
            // Assume the models are sorted
            set_union (final_leaves_.begin (), final_leaves_.end (), (*it).begin (), (*it).end (),
                       inserter (model_reunion, model_reunion.begin ()), compareLeaf);
            final_leaves_ = model_reunion;
            model_reunion.clear ();
          }
          decaying_maps_.pop_front ();
        }
        else
          final_leaves_ = cur_leaves_;

        // Include the static map in the reunion
        set_union (final_leaves_.begin (), final_leaves_.end (), static_leaves_.begin (), static_leaves_.end (),
                    inserter (model_reunion, model_reunion.begin ()), compareLeaf);

        computeCollisionMapFromLeaves (&model_reunion, final_collision_map_);

        gettimeofday (&t2, NULL);
        time_spent = t2.tv_sec + (double)t2.tv_usec / 1000000.0 - (t1.tv_sec + (double)t1.tv_usec / 1000000.0);
        ROS_INFO ("Collision map with %u boxes computed in %g seconds. Total maps in the queue %d.",
                  (unsigned int)final_collision_map_.boxes.size (), time_spent, decaying_maps_.size ());

        publish ("collision_map_buffer", final_collision_map_);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief CollisionMapBuffer service callback */
    bool
      getStaticMap (RecordStaticMapTrigger::request &req, RecordStaticMapTrigger::response &resp)
    {
      static_map_lock_.lock ();
      acquire_static_map_      = true;
      acquire_static_map_time_ = req.map_time;
      static_map_lock_.unlock ();

      ROS_INFO ("Got a request to compute a new static map at %f.", acquire_static_map_time_.toSec ());

      // Wait until the scan is ready, sleep for 10ms
      ros::Duration tictoc (0, 10000000);
      while (acquire_static_map_)
      {
        tictoc.sleep ();
      }

      resp.status = 0;      // success (!)

      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief CollisionMapBuffer service callback */
    bool
      subtractObject (SubtractObjectFromCollisionMap::request &req, SubtractObjectFromCollisionMap::response &resp)
    {
      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  CollisionMapperBuffer p;

//   collision_map::RememberMap::request req;
//   collision_map::RememberMap::response resp;
//   req.map_time = ros::Time::now ();
//   ros::service::call ("collision_map_buffer", req, resp);

  p.spin ();

  ros::fini ();
  return (0);
}
/* ]--- */

