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
 * $Id: doors_detector.cpp 9895 2009-01-21 21:05:27Z tfoote $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b doors_detector detects and returns a list of doors from 3D point cloud data (laser).

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

// ROS core
#include <ros/node.h>
#include <roslib/Header.h>
#include <robot_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include "geometric_helper.h"

// Include the service call type
#include "door_handle_detector/DoorsDetector.h"

#include <tf/message_notifier.h>


using namespace std;
using namespace robot_msgs;

class DoorDetector
{
  protected:
    ros::Node& node_;

  public:
    // ROS messages
    PointCloud cloud_in_, cloud_regions_;
    Point32 z_axis_;
    PolygonalMap pmap_;
    tf::MessageNotifier<robot_msgs::PointCloud>*  message_notifier_;

    PointStamped viewpoint_cloud_;

    tf::TransformListener tf_;

    string input_cloud_topic_, parameter_frame_, door_frame_, cloud_frame_;
    roslib::Header cloud_header_;
    ros::Time cloud_time_;
    unsigned int num_clouds_received_;

    // Parameters regarding geometric constraints for the door/handle
    double door_min_height_, door_min_width_, door_max_height_, door_max_width_, door_min_z_;

    // Parameters regarding the _fast_ normals/plane computation using a lower quality (downsampled) dataset
    double leaf_width_;
    double sac_distance_threshold_;
    double normal_angle_tolerance_;
    int k_search_;

    // Parameters for the euclidean clustering/cluster rejection
    double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
    int euclidean_cluster_min_pts_;

    // Parameters for "rectangularity" constraints
    double rectangle_constrain_edge_height_;
    double rectangle_constrain_edge_angle_;

    double maximum_search_radius_;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DoorDetector (ros::Node& anode) : node_ (anode), message_notifier_ (NULL), tf_ (anode)
    {
      // ---[ Parameters regarding geometric constraints for the door/handle
      {
          node_.param ("~parameter_frame", parameter_frame_, string ("base_footprint"));

        // Frame _independent_ parameters (absolute values)
          node_.param ("~door_min_height", door_min_height_, 1.2);                  // minimum height of a door: 1.2m
          node_.param ("~door_max_height", door_max_height_, 3.0);                  // maximum height of a door: 3m
          node_.param ("~door_min_width", door_min_width_, 0.7);                    // minimum width of a door: 0.7m
          node_.param ("~door_max_width", door_max_width_, 1.4);                    // maximum width of a door: 1.4m
          ROS_DEBUG ("Using the following thresholds for door detection [min-max height / min-max width]: %f-%f / %f-%f.",
                     door_min_height_, door_max_height_, door_min_width_, door_max_width_);

          // NOTE: it makes _absolutely_ no sense to search for information far away from the robot as the data is incredibly sparse !
          node_.param ("~maximum_search_radius", maximum_search_radius_, 10.0);     // only consider points closer than this value to the robot
          maximum_search_radius_ *= maximum_search_radius_;                         // squared for faster processing

          // This parameter constrains the door polygon to resamble a rectangle,
          // that is: the two lines parallel (with some angular threshold) to the Z-axis must have some minimal length
          node_.param ("~rectangle_constrain_edge_height", rectangle_constrain_edge_height_, 0.4);        // each side of the door has to be at least 40 cm
          node_.param ("~rectangle_constrain_edge_angle", rectangle_constrain_edge_angle_, 15.0);         // maximum angle threshold between a side and the Z-axis: 15 degrees
          rectangle_constrain_edge_angle_ = cloud_geometry::deg2rad (rectangle_constrain_edge_angle_);

          // Frame _dependent_ parameters (need to be converted!)
          node_.param ("~door_min_z", door_min_z_, 0.1);                            // the minimum Z point on the door must be lower than this value
      }

      // ---[ Parameters regarding optimizations / real-time computations
      leaf_width_ = 0.03;              // 2.5cm box size by default
      k_search_   = 10;                // 10 k-neighbors by default
      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;

      normal_angle_tolerance_ = cloud_geometry::deg2rad (5.0); // Maximum angular difference in normal space for inliers wrt the Z-axis
      // Parameters regarding the thresholds for Euclidean region growing/clustering
      euclidean_cluster_angle_tolerance_    = cloud_geometry::deg2rad (10.0);
      euclidean_cluster_min_pts_            = 500;                // 500 points
      euclidean_cluster_distance_tolerance_ = 0.04;               // 4 cm

      // This should be set to whatever the leaf_width * 1.5 factor is in the downsampler
      sac_distance_threshold_ = leaf_width_ * 1.5;            // 4.5 cm by default

      // Temporary parameters
      node_.param ("~input_cloud_topic", input_cloud_topic_, string ("snapshot_cloud"));
      node_.advertiseService ("doors_detector", &DoorDetector::detectDoor, this);
      node_.advertise<robot_msgs::VisualizationMarker> ("visualizationMarker", 100);

      node_.advertise<PolygonalMap> ("~door_frames", 1);
      node_.advertise<PointCloud> ("~door_regions", 1);
      cloud_regions_.chan.resize (1); cloud_regions_.chan[0].name = "rgb";
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief This is the main service callback: it gets called whenever a request to find a new door is given     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detectDoor (door_handle_detector::DoorsDetector::Request &req, door_handle_detector::DoorsDetector::Response &resp)
    {
      ros::Time start_time = ros::Time::now ();
      ros::Duration delay = ros::Duration().fromSec (25);
      cout << "start time " << start_time.toSec () << endl;
      cout << "Waiting for laser scan to come in newer than " << (start_time + delay).toSec() << endl;

      // door frame
      door_frame_ = req.door.header.frame_id;

      // receive a new laser scan
      num_clouds_received_ = 0;
      message_notifier_ = new tf::MessageNotifier<robot_msgs::PointCloud> (&tf_, &node_,  boost::bind (&DoorDetector::cloud_cb, this, _1),
                                                                           input_cloud_topic_.c_str (), door_frame_, 1);
      ros::Duration tictoc = ros::Duration().fromSec (1.0);
      while ((int)num_clouds_received_ < 1)//|| (cloud_in_.header.stamp < (start_time + delay)))
        tictoc.sleep ();
      delete message_notifier_;

      // Cloud frame
      cloud_frame_  = cloud_in_.header.frame_id;
      cloud_time_   = cloud_in_.header.stamp;
      cloud_header_ = cloud_in_.header;

      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", cloud_in_.pts.size (), cloud_frame_.c_str (),
                cloud_in_.chan.size (), cloud_geometry::getAvailableChannels (&cloud_in_).c_str ());

      ros::Time ts;
      ros::Duration duration;
      ts = ros::Time::now ();

      // Get the cloud viewpoint
      getCloudViewPoint (cloud_frame_, viewpoint_cloud_, &tf_);

      // ---[ Optimization: downsample the cloud in the bounding box for faster processing
//      maximum_search_radius_
//      vector<int> indices_in_bounds;
//      for (unsigned int i = 0; i < cloud_in.pts.size (); i++)
//      {
//cloud_geometry::distances::pointToPointDistanceSqr ()
//      }
//      obtainCloudIndicesSet (&cloud_in_, indices_in_bounds, req, &tf_, parameter_frame_, door_min_z_bounds_, door_max_z_bounds_, door_frame_multiplier_);

      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      PointCloud cloud_down_;
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        Point leaf_width_xyz;
        leaf_width_xyz.x = leaf_width_xyz.y = leaf_width_xyz.z = leaf_width_;
        cloud_geometry::downsamplePointCloud (&cloud_in_, cloud_down_, leaf_width_xyz, leaves, -1);
        //cloud_geometry::downsamplePointCloud (&cloud_in_, &indices_in_bounds, cloud_down_, leaf_width_xyz, leaves, -1);
        ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_xyz.x, leaf_width_xyz.y, leaf_width_xyz.z, cloud_down_.pts.size ());
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }
      leaves.resize (0);    // dealloc memory used for the downsampling process

      sendMarker (viewpoint_cloud_.point.x, viewpoint_cloud_.point.y, viewpoint_cloud_.point.z, cloud_frame_, &node_);

      // Create Kd-Tree and estimate the point normals in the original point cloud
      //estimatePointNormals (cloud_in_, cloud_down_, k_search_, viewpoint_cloud_);

      // Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_xy;
//      cloud_geometry::getPointIndicesAxisPerpendicularNormals (&cloud_down_, 0, 1, 2, normal_angle_tolerance_, &z_axis_, indices_xy);
      // Check all points
      for (unsigned int i = 0; i < cloud_down_.pts.size (); i++)
      {
        robot_msgs::Point32 p;
        p.x = cloud_down_.chan[0].vals[i];
        p.y = cloud_down_.chan[1].vals[i];
        p.z = cloud_down_.chan[2].vals[i];
        // Compute the angle between their normal and the given axis
//        double angle = acos (cloud_geometry::dot (&p, &z_axis_));
        double angle = acos (p.x * z_axis_.x + p.y * z_axis_.y + p.z * z_axis_.z);
        if (fabs (M_PI / 2.0 - angle) < normal_angle_tolerance_)
          indices_xy.push_back (i);
      }

      // Output the point regions
      cloud_regions_.header = cloud_down_.header;
      cloud_regions_.pts.resize (0);
      cloud_regions_.chan[0].vals.resize (0);
//      for (unsigned int cc = 0; cc < clusters.size (); cc++)
//      {
        float r = rand () / (RAND_MAX + 1.0);
        float g = rand () / (RAND_MAX + 1.0);
        float b = rand () / (RAND_MAX + 1.0);
//        for (unsigned int j = 0; j< clusters[cc].size (); j++)
        for (unsigned int j = 0; j < indices_xy.size (); j++)
        {
          //cloud_regions_.pts.push_back (cloud_down_.pts[clusters[cc][j]]);
          cloud_regions_.pts.push_back (cloud_down_.pts[indices_xy[j]]);
          cloud_regions_.chan[0].vals.push_back (getRGB (r, g, b));
        }
//      }
      node_.publish ("~door_regions", cloud_regions_);

      // Split the Z-perpendicular points into clusters
      vector<vector<int> > clusters;
      findClusters (&cloud_down_, &indices_xy, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2,
                    euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
      sort (clusters.begin (), clusters.end (), compareRegions);
      reverse (clusters.begin (), clusters.end ());

      vector<vector<double> > coeff (clusters.size ()); // Need to save all coefficients for all models
      pmap_.header = cloud_header_;
      pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map

      ROS_INFO (" - Process all clusters (%i)", clusters.size ());
      vector<int> inliers;
      vector<int> handle_indices;
      vector<double> goodness_factor (clusters.size());
      robot_msgs::Point32 min_p, max_p, handle_center;

      // Transform door_min_z_ from the parameter parameter frame (parameter_frame_) into the point cloud frame
      try
      {
        door_min_z_ = transformDoubleValueTF (door_min_z_, parameter_frame_, cloud_frame_, cloud_time_, &tf_);
      }
      catch (tf::ExtrapolationException e)
      {
        ROS_ERROR ("Error transforming door_min_z from %s to %s. Message: %s.", parameter_frame_.c_str (), cloud_frame_.c_str (), e.what ());
        return (false);
      }

#pragma omp parallel for schedule(dynamic)
      // Process all clusters
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        // initial goodness factor
        goodness_factor[cc] = 1;

        // Find the best plane in this cluster
        if (!fitSACPlane (cloud_down_, clusters[cc], inliers, coeff[cc], &viewpoint_cloud_, sac_distance_threshold_, euclidean_cluster_min_pts_))
        {
          goodness_factor[cc] = 0;
          continue;
        }

        // Compute the convex hull
        cloud_geometry::areas::convexHull2D (&cloud_down_, &inliers, &coeff[cc], pmap_.polygons[cc]);

        // Filter the region based on its height and width
        cloud_geometry::statistics::getMinMax (&pmap_.polygons[cc], min_p, max_p);

        // ---[ Quick test for min_p.z (!)
        if (min_p.z > door_min_z_)
        {
          goodness_factor[cc] = 0;
          ROS_DEBUG ("[WARN] Door candidate rejected because min.Z (%g) is higher than the user specified threshold (%g)!", min_p.z, door_min_z_);
          continue;
        }

        // ---[ Get the limits of the two "parallel to the Z-axis" lines defining the door
        double height_df1 = 0.0, height_df2 = 0.0;
        if (!checkDoorEdges (&pmap_.polygons[cc], &z_axis_, rectangle_constrain_edge_height_, rectangle_constrain_edge_angle_,
                             height_df1, height_df2))
        {
          goodness_factor[cc] = 0;
          ROS_DEBUG ("[WARN] Door candidate rejected because the length of the door edges (%g / %g) is smaller than the user specified threshold (%g)!",
                     height_df1, height_df2, rectangle_constrain_edge_height_);
          continue;
        }

        // ---[ Compute the door width and height
        double door_frame = sqrt ( (max_p.x - min_p.x) * (max_p.x - min_p.x) + (max_p.y - min_p.y) * (max_p.y - min_p.y) );
        double door_height = fabs (max_p.z - min_p.z);
        // Adapt the goodness factor for each cluster
        if (door_frame < door_min_width_ || door_height < door_min_height_ || door_frame > door_max_width_ || door_height > door_max_height_)
        {
          goodness_factor[cc] = 0;
          ROS_DEBUG ("[WARN] Door candidate rejected because its width/height (%g / %g) is smaller than the user specified threshold (%g / %g -> %g / %g)!",
                     door_frame, door_height, door_min_width_, door_min_height_, door_max_width_, door_max_height_);
          continue;
        }
        double area = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);
        goodness_factor[cc] *= (area / (door_frame * door_height));

        // ---[ Compute the distance from the door to the prior of the door
        double door_distance = fmax (0.001,
                                     fmin (cloud_geometry::distances::pointToPointXYDistance (&req.door.door_p1, &min_p),
                                           fmin (cloud_geometry::distances::pointToPointXYDistance (&req.door.door_p1, &max_p),
                                                 fmin (cloud_geometry::distances::pointToPointXYDistance (&req.door.door_p2, &min_p),
                                                       cloud_geometry::distances::pointToPointXYDistance (&req.door.door_p2, &max_p)))));
        goodness_factor[cc] /= door_distance;
      } // loop over clusters


      // Count the number of remaining clusters with non-null goodness factor
      int doors_found_cnt = 0;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        if (goodness_factor[cc] != 0)
          doors_found_cnt++;
        else
          pmap_.polygons[cc].points.resize (0);
      }

      ROS_INFO (" - Found %d / %d potential door candidates.", doors_found_cnt, clusters.size ());
      resp.doors.resize (doors_found_cnt);

      // Copy all clusters
      int nr_d = 0;
      for (int cc = 0; cc < (int)clusters.size (); cc++)
      {
        if (goodness_factor[cc] == 0)
          continue;

        // Save the weight (we need to reorder at the end)
        resp.doors[nr_d].weight = goodness_factor[cc];

        // Get the min_p and max_p of selected cluster
        cloud_geometry::statistics::getLargestXYPoints (&pmap_.polygons[cc], min_p, max_p);

        // Transform the requested door frame parameters into the cloud frame id
        tf::Stamped<Point32> door_p1 (min_p, cloud_time_, cloud_frame_);
        tf::Stamped<Point32> door_p2 (max_p, cloud_time_, cloud_frame_);

        door_p2.z = door_p1.z;
        try
        {
          transformPoint (&tf_, door_frame_, door_p1, door_p1);
          transformPoint (&tf_, door_frame_, door_p2, door_p2);
        }
        catch (tf::ExtrapolationException e)
        {
          ROS_ERROR ("Error transforming the door_frame from %s to %s. Message: %s.", cloud_frame_.c_str (), door_frame_.c_str (), e.what ());
          return (false);
        }

        // Reply doors message in same frame as request doors message
        resp.doors[nr_d].header.frame_id = req.door.header.frame_id;

        resp.doors[nr_d].height = fabs (max_p.z - min_p.z);
        resp.doors[nr_d].header.stamp = cloud_time_;
        resp.doors[nr_d].header.frame_id = door_frame_;
        resp.doors[nr_d].door_p1 = door_p1;
        resp.doors[nr_d].door_p2 = door_p2;

        resp.doors[nr_d].door_boundary = pmap_.polygons[cc];
        resp.doors[nr_d].normal.x      = coeff[cc][0];
        resp.doors[nr_d].normal.y      = coeff[cc][1];
        resp.doors[nr_d].normal.z      = coeff[cc][2];

        // Need min/max Z
        cloud_geometry::statistics::getMinMax (&pmap_.polygons[cc], min_p, max_p);
        resp.doors[nr_d].height = fabs (max_p.z - min_p.z);

        nr_d++;
      }

      // Check if any cluster respected all our constraints (i.e., has a goodness_factor > 0)
      if (nr_d == 0)
      {
        ROS_ERROR ("did not find a door");
        return (false);
      }

      // Order the results based on the weight (e.g. goodness factor)
      sort (resp.doors.begin (), resp.doors.end (), compareDoorsWeight);
      reverse (resp.doors.begin (), resp.doors.end ());

      duration = ros::Time::now () - ts;
      ROS_INFO ("Door(s) found and ordered by weight. Result in frame %s", resp.doors[0].header.frame_id.c_str ());
      for (int cd = 0; cd < nr_d; cd++)
      {
        ROS_INFO ("  %d -> P1 = [%g, %g, %g]. P2 = [%g, %g, %g]. Width = %g. Height = %g. Weight = %g.", cd,
                  resp.doors[cd].door_p1.x, resp.doors[cd].door_p1.y, resp.doors[cd].door_p1.z, resp.doors[cd].door_p2.x, resp.doors[cd].door_p2.y, resp.doors[cd].door_p2.z,
                  sqrt ( (resp.doors[cd].door_p1.x - resp.doors[cd].door_p2.x) * (resp.doors[cd].door_p1.x - resp.doors[cd].door_p2.x) +
                         (resp.doors[cd].door_p1.y - resp.doors[cd].door_p2.y) * (resp.doors[cd].door_p1.y - resp.doors[cd].door_p2.y) ),
                  resp.doors[cd].height, resp.doors[cd].weight);
      }
      ROS_INFO ("  Total time: %g.", duration.toSec ());

      node_.publish ("~door_frames", pmap_);

      ROS_INFO ("Finished detecting door");
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Main point cloud callback.                                                                           */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void cloud_cb (const tf::MessageNotifier<robot_msgs::PointCloud>::MessagePtr& cloud)
    {
      cout << "scan received" << endl;
      cloud_in_ = *cloud;
      num_clouds_received_++;
    }
};



/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("doors_detector_node");

  DoorDetector p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

