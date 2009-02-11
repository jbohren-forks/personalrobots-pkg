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
#include <robot_msgs/Point.h>
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_msgs/VisualizationMarker.h>

#include <Eigen/Core>
#include <cloud_geometry/point.h>
#include <cloud_geometry/transforms.h>

#include <boost/thread/mutex.hpp>

#include <robot_msgs/OrientedBoundingBox.h>
#include <robot_msgs/CollisionMap.h>
#include <robot_srvs/RecordStaticMapTrigger.h>
#include <robot_srvs/SubtractObjectFromCollisionMap.h>

#include <tf/transform_listener.h>
#include <sys/time.h>

using namespace std;
using namespace robot_msgs;
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

    // Internal map representations
    vector<Leaf> static_leaves_, cur_leaves_, final_leaves_;
    list<vector<Leaf> > decaying_maps_;

    // TF
    tf::TransformListener tf_;

    // Parameters
    robot_msgs::Point leaf_width_, robot_max_;
    int min_nr_points_;
    string end_effector_frame_l_, end_effector_frame_r_;

    // Mutices
    boost::mutex static_map_lock_, object_subtract_lock_, cloud_frame_lock_, m_lock_;

    // The size of the buffer window
    int window_size_;
    bool acquire_static_map_;
    ros::Time acquire_static_map_time_;

    // Internal parameters
    string cloud_frame_;
    PoseStamped gripper_orientation_link_;
    robot_msgs::Point32 min_object_b_, max_object_b_;
    bool subtract_object_;
    int m_id_;

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

      param ("~end_effector_frame_l", end_effector_frame_l_, string ("r_gripper_l_fingertip_link"));     // The frame of the end effector (used for object subtraction)
      param ("~end_effector_frame_r", end_effector_frame_r_, string ("r_gripper_r_fingertip_link"));     // The frame of the end effector (used for object subtraction)

      // Square the limits (simplified point distances below)
      robot_max_.x = robot_max_.x * robot_max_.x;
      robot_max_.y = robot_max_.y * robot_max_.y;
      robot_max_.z = robot_max_.z * robot_max_.z;

      acquire_static_map_ = false;                        // Do not acquire a static map

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

      // Gripper orientation/position
      gripper_orientation_link_.pose.orientation.x = 0.0;
      gripper_orientation_link_.pose.orientation.y = 0.0;
      gripper_orientation_link_.pose.orientation.z = 0.0;
      gripper_orientation_link_.pose.orientation.w = 1.0;
      subtract_object_ = false;

      m_id_ = 0;
      advertise<robot_msgs::VisualizationMarker>("visualizationMarker", 100);
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
      * \param centers a resultant PointCloud message containing the centers of the leaves
      */
    void 
      computeLeaves (PointCloud *points, vector<Leaf> &leaves, PointCloud &centers)
    {
      PointStamped base_origin, torso_lift_origin;
      base_origin.point.x = base_origin.point.y = base_origin.point.z = 0.0;
      base_origin.header.frame_id = "torso_lift_link";
      base_origin.header.stamp = ros::Time ();

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
      for (unsigned int i = 0; i < points->pts.size (); i++)
      {
        // We split the "distance" on all 3 dimensions to allow greater flexibility
        distance_sqr_x = fabs ((points->pts[i].x - torso_lift_origin.point.x) * (points->pts[i].x - torso_lift_origin.point.x));
        distance_sqr_y = fabs ((points->pts[i].y - torso_lift_origin.point.y) * (points->pts[i].y - torso_lift_origin.point.y));
        distance_sqr_z = fabs ((points->pts[i].z - torso_lift_origin.point.z) * (points->pts[i].z - torso_lift_origin.point.z));

        // If the point is within the bounds, use it for minP/maxP calculations
        if (distance_sqr_x < robot_max_.x && distance_sqr_y < robot_max_.y && distance_sqr_z < robot_max_.z)
        {
          minP.x = (points->pts[i].x < minP.x) ? points->pts[i].x : minP.x;
          minP.y = (points->pts[i].y < minP.y) ? points->pts[i].y : minP.y;
          minP.z = (points->pts[i].z < minP.z) ? points->pts[i].z : minP.z;

          maxP.x = (points->pts[i].x > maxP.x) ? points->pts[i].x : maxP.x;
          maxP.y = (points->pts[i].y > maxP.y) ? points->pts[i].y : maxP.y;
          maxP.z = (points->pts[i].z > maxP.z) ? points->pts[i].z : maxP.z;
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

      // Return a point cloud message containing the centers of the leaves as well
      centers.header = points->header;
      centers.pts.resize (indices.size ());
      float extents[3];
      extents[0] = leaf_width_.x / 2.0;
      extents[1] = leaf_width_.y / 2.0;
      extents[2] = leaf_width_.z / 2.0;

      // First pass: go over all points and count them into the right leaf
      int i = 0, j = 0, k = 0;
      for (unsigned int cp = 0; cp < indices.size (); cp++)
      {
        i = (int)(floor (points->pts[indices.at (cp)].x / leaf_width_.x));
        j = (int)(floor (points->pts[indices.at (cp)].y / leaf_width_.y));
        k = (int)(floor (points->pts[indices.at (cp)].z / leaf_width_.z));

        int idx = ( (k - minB.z) * divB.y * divB.x ) + ( (j - minB.y) * divB.x ) + (i - minB.x);
        leaves[idx].i_ = i;
        leaves[idx].j_ = j;
        leaves[idx].k_ = k;
        leaves[idx].nr_points_++;

        // Get the point
        centers.pts[cp].x = (i + 1) * leaf_width_.x - extents[0];
        centers.pts[cp].y = (j + 1) * leaf_width_.y - extents[1];
        centers.pts[cp].z = (k + 1) * leaf_width_.z - extents[2];
      }

      sort (leaves.begin (), leaves.end (), compareLeaf);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      sendMarker (Point32 pt, const std::string &frame_id, double radius = 0.02)
    {
      VisualizationMarker mk;
      mk.header.stamp = ros::Time::now();

      mk.header.frame_id = frame_id;

      mk.id = ++m_id_;
      mk.type = VisualizationMarker::SPHERE;
      mk.action = VisualizationMarker::ADD;
      mk.x = pt.x;
      mk.y = pt.y;
      mk.z = pt.z;

      mk.roll = mk.pitch = mk.yaw = 0;
      mk.xScale = mk.yScale = mk.zScale = radius * 2.0;

      mk.alpha = 255;
      mk.r = 255;
      mk.g = 10;
      mk.b = 10;

      publish ("visualizationMarker", mk);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain the position of the end effector (center of the two fingers) in the required target frame.
      * \param tgt_frame the target TF frame
      * \param stamp the time stamp
      * \param center the resultant center position
      */
    inline bool
      getEndEffectorPosition (string tgt_frame, ros::Time stamp, Point32 &center)
    {
      PointStamped src, tgt;
      src.header.frame_id = end_effector_frame_l_;
      src.header.stamp    = stamp;

      src.point.x = src.point.y = src.point.z = 0.0;
      try
      {
        tf_.transformPoint (tgt_frame, src, tgt);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
        return (false);
      }
      catch (tf::ExtrapolationException)
      {
	ROS_ERROR("Extrapolation exception from %s to %s.", tgt_frame.c_str(), src.header.frame_id.c_str());
      }

      center.x = tgt.point.x; center.y = tgt.point.y; center.z = tgt.point.z;

      src.header.frame_id = end_effector_frame_r_;
      try
      {
        tf_.transformPoint (tgt_frame, src, tgt);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
        return (false);
      }
      catch (tf::ExtrapolationException)
      {
	ROS_ERROR("Extrapolation exception from %s to %s.", tgt_frame.c_str(), src.header.frame_id.c_str());
      }

      center.x += tgt.point.x; center.y += tgt.point.y; center.z += tgt.point.z;
      center.x /= 2.0;         center.y /= 2.0;         center.z /= 2.0;
      return (true);

    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Prune the leaves inside a given bounding box
      * \param leaves the set of leaves to prune data from
      * \param points the point cloud representing the centers of the leaves
      * \param center a point in the center of the gripper
      * \param source_frame the TF frame in which the points are represented
      * \param target_frame the TF frame in which the bounds are represented
      * \param min_b the minimum bounds of the box
      * \param max_b the maximum bounds of the box
      */
    void
      pruneLeaves (vector<Leaf> &leaves, PointCloud *points, Point32 *center, string source_frame, string target_frame,
                   Point32 *min_b, Point32 *max_b)
    {
      PointCloud points_tgt;

      // Transform the entire cloud at once
      try
      {
        tf_.transformPointCloud (target_frame, *points, points_tgt);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
        return;
      }
      catch (tf::ExtrapolationException)
      {
	ROS_ERROR("Extrapolation exception from %s to %s.", target_frame.c_str(), points->header.frame_id.c_str());
      }

      vector<int> object_indices (points_tgt.pts.size ());
      int nr_p = 0;
      // Check and mark point indices in the bounds of the objects
      for (unsigned int i = 0; i < points_tgt.pts.size (); i++)
      {
        if (points_tgt.pts[i].x > min_b->x &&
            points_tgt.pts[i].x < max_b->x &&
            points_tgt.pts[i].y > min_b->y &&
            points_tgt.pts[i].y < max_b->y &&
            points_tgt.pts[i].z > min_b->z &&
            points_tgt.pts[i].z < max_b->z)
        {
          object_indices[nr_p] = i;
          nr_p++;
        }
      }
      object_indices.resize (nr_p);

      // Copy the indices from object_indices into a temporary cloud
      PointCloud object_points;
      object_points.header = points_tgt.header;
      object_points.pts.resize (object_indices.size ());
      for (unsigned int i = 0; i < object_indices.size (); i++)
      {
        object_points.pts[i].x = points_tgt.pts[object_indices.at (i)].x;
        object_points.pts[i].y = points_tgt.pts[object_indices.at (i)].y;
        object_points.pts[i].z = points_tgt.pts[object_indices.at (i)].z;
      }

      PointStamped ee_local, ee_global;      // Transform the end effector position in global (source frame)
      ee_local.point.x = ee_local.point.y = ee_local.point.z = 0.0;
      ee_local.header.frame_id = target_frame;
      ee_local.header.stamp = points->header.stamp;

      // Transform the points back into the source frrame
      PointCloud points_src;
      try
      {
        tf_.transformPointCloud (source_frame, object_points, points_src);
        tf_.transformPoint (source_frame, ee_local, ee_global);
      }
      catch (tf::ConnectivityException)
      {
        ROS_ERROR ("TF not running or wrong TF end_effector_frame specified!");
        return;
      }

      ROS_INFO ("End effector position is: [%f, %f, %f].", ee_global.point.x, ee_global.point.y, ee_global.point.z);

      // Compute the leaves
      vector<Leaf> object_leaves;
      computeLeaves (&points_src, object_leaves, object_points);

      // Go over the leaves and subtract the ones on the object
      vector<Leaf> model_difference;
      set_difference (leaves.begin (), leaves.end (), object_leaves.begin (), object_leaves.end (),
                      inserter (model_difference, model_difference.begin ()), compareLeaf);
      leaves = model_difference;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Pointcloud callback function */
    void
      cloud_cb ()
    {
      ROS_INFO ("Received %u data points.", (unsigned int)cloud_.pts.size ());

      // Get the new parameters from the server
      m_lock_.lock ();
      updateParametersFromServer ();
      m_lock_.unlock ();

      timeval t1, t2;
      double time_spent;

      // Get the position of the end effector
      Point32 ee_center;
      if (!getEndEffectorPosition (cloud_.header.frame_id, cloud_.header.stamp, ee_center))
        return;

//       sendMarker (ee_center, cloud_.header.frame_id);

      // Copy the header (and implicitly the frame_id)
      final_collision_map_.header = cloud_.header;

      // Static map acquisition has been triggered via the service call
      if (acquire_static_map_)
      {
        // Do not compute any collision maps until we receive a cloud with a higher timestamp 
        if (cloud_.header.stamp < acquire_static_map_time_)
          return;

        // Compute the static collision map
        gettimeofday (&t1, NULL);

        // We do not subtract anything when we compute the static map
        PointCloud centers;
        computeLeaves (&cloud_, static_leaves_, centers);

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
        vector<Leaf> model_reunion;
        // Rotate N maps in the queue
        gettimeofday (&t1, NULL);

        // Compute the leaves for the current dataset
        PointCloud centers;
        m_lock_.lock ();
        computeLeaves (&cloud_, cur_leaves_, centers);
        m_lock_.unlock ();

        // Check the points against the object bounds
        object_subtract_lock_.lock ();
        pruneLeaves (cur_leaves_, &centers, &ee_center, cloud_.header.frame_id, end_effector_frame_l_, &min_object_b_, &max_object_b_);
        object_subtract_lock_.unlock ();

        // Push the current leaves onto the queue
        decaying_maps_.push_back (cur_leaves_);

        // If we have window_size maps, combine them together
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
    /** \brief CollisionMapBuffer "record_static_map" service callback */
    bool
      getStaticMap (RecordStaticMapTrigger::Request &req, RecordStaticMapTrigger::Response &resp)
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
    /** \brief CollisionMapBuffer "subtract_object" service callback */
    bool
      subtractObject (SubtractObjectFromCollisionMap::Request &req, SubtractObjectFromCollisionMap::Response &resp)
    {
      ROS_INFO ("Got request to subtract object.");
      Point32 center;
      center.x = (req.object.min_bound.x + req.object.max_bound.x) / 2.0;
      center.y = (req.object.min_bound.y + req.object.max_bound.y) / 2.0;
      center.z = (req.object.min_bound.z + req.object.max_bound.z) / 2.0;

      object_subtract_lock_.lock ();

      min_object_b_.x = req.object.min_bound.x - center.x;
      min_object_b_.y = req.object.min_bound.y - center.y;
      min_object_b_.z = req.object.min_bound.z - center.z;

      max_object_b_.x = req.object.max_bound.x - center.x;
      max_object_b_.y = req.object.max_bound.y - center.y;
      max_object_b_.z = req.object.max_bound.z - center.z;

      subtract_object_ = true;

      object_subtract_lock_.unlock ();

      resp.status = 0;      // success (!)

      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  CollisionMapperBuffer p;

//   RecordStaticMapTrigger::Request req;
//   RecordStaticMapTrigger::Response resp;
//   req.map_time = ros::Time::now ();
//   ros::service::call ("record_static_map", req, resp);

  // Wait until the scan is ready, sleep for 1s
  ros::Duration tictoc (10.0, 0);
  tictoc.sleep ();

  // Box example: 22.2 cm x 10.5 cm x 5.8 cm
/*  SubtractObjectFromCollisionMap::Request req;
  req.object.min_bound.x = req.object.min_bound.y = req.object.min_bound.z = 0.0;
  req.object.max_bound.z = 0.35; //0.222;
  req.object.max_bound.x = 0.105 * 2;
  req.object.max_bound.y = 0.058 * 4;
  SubtractObjectFromCollisionMap::Response resp;
  ros::service::call ("~subtract_object", req, resp);
*/
  p.spin ();

  
  return (0);
}
/* ]--- */

