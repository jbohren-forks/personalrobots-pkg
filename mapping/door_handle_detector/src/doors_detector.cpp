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

@b doors_detector detects and returns a list of door _candidates_ (no magic here!) from 3D point cloud data acquired using a tilting laser sensor.

\note Assumes the door frame points are given in the same coordinate system as the incoming point cloud message!

 **/

// ROS core
#include <ros/node.h>
#include <roslib/Header.h>
#include <robot_msgs/PolygonalMap.h>

// Most of the geometric routines that contribute to the door finding job are located here
#include <door_handle_detector/geometric_helper.h>

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
    PointCloud cloud_orig_, cloud_tr_, cloud_regions_;
    Point32 x_axis_, z_axis_;
    PolygonalMap pmap_;
    tf::MessageNotifier<robot_msgs::PointCloud>*  message_notifier_;

    PointStamped viewpoint_cloud_;

    tf::TransformListener tf_;

    string input_cloud_topic_, parameter_frame_, door_frame_;
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

    // Prune door candidates
    double maximum_search_radius_, maximum_search_radius_limit_, maximum_scan_angle_limit_;


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
          node_.param ("~maximum_search_radius", maximum_search_radius_, 8.0);     // only consider points closer than this value to the robot
          maximum_search_radius_ *= maximum_search_radius_;                        // squared for faster processing

          // Heuristically discard all door candidates which have their polygonal bounds at angles larger than this value with the viewpoint
          node_.param ("~maximum_scan_angle", maximum_scan_angle_limit_, 140.0);
          maximum_scan_angle_limit_ = cloud_geometry::deg2rad (maximum_scan_angle_limit_);

          // This parameter constrains the door polygon to resamble a rectangle,
          // that is: the two lines parallel (with some angular threshold) to the Z-axis must have some minimal length
          node_.param ("~rectangle_constrain_edge_height", rectangle_constrain_edge_height_, 0.4);        // each side of the door has to be at least 40 cm
          node_.param ("~rectangle_constrain_edge_angle", rectangle_constrain_edge_angle_, 15.0);         // maximum angle threshold between a side and the Z-axis: 15 degrees
          rectangle_constrain_edge_angle_ = cloud_geometry::deg2rad (rectangle_constrain_edge_angle_);

          // Frame _dependent_ parameters (need to be converted!)
          node_.param ("~door_min_z", door_min_z_, 0.1);                            // the minimum Z point on the door must be lower than this value
      }

      // ---[ Parameters regarding optimizations / real-time computations
      leaf_width_ = 0.02;              // 2cm box size by default
      k_search_   = 10;                // 10 k-neighbors by default
      x_axis_.x = 1; x_axis_.y = 0; x_axis_.z = 0;
      z_axis_.x = 0; z_axis_.y = 0; z_axis_.z = 1;

      normal_angle_tolerance_ = cloud_geometry::deg2rad (10.0); // Maximum angular difference in normal space for inliers wrt the Z-axis
      // Parameters regarding the thresholds for Euclidean region growing/clustering
      euclidean_cluster_angle_tolerance_    = cloud_geometry::deg2rad (10.0);
      euclidean_cluster_min_pts_            = 500;                // 500 points
      euclidean_cluster_distance_tolerance_ = 0.05;               // 5 cm

      // This should be set to whatever the leaf_width * 2 factor is in the downsampler
      sac_distance_threshold_ = leaf_width_ * 2;                  // 4 cm by default

      // Heuristically discard all door candidates which have their polygonal bounds at distances more than this value
      maximum_search_radius_limit_ = maximum_search_radius_ - 0.2;

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

      ros::Time ts;
      ros::Duration duration;
      ts = ros::Time::now ();

      // New strategy: transform the PCD (Point Cloud Data) into the parameter_frame, and work there
      if (parameter_frame_ != cloud_orig_.header.frame_id)
      {
        try
        {
          tf_.transformPointCloud (parameter_frame_, cloud_orig_, cloud_tr_);
        }
        catch (tf::ConnectivityException e)
        {
          ROS_ERROR ("Could not transform point cloud from frame %s to frame %s.", cloud_orig_.header.frame_id.c_str (), parameter_frame_.c_str ());
        }
      }
      else
        cloud_tr_ = cloud_orig_;            // This can be replaced by modifying TF to copy (point to) the data

      // Get the cloud viewpoint in the parameter frame
      getCloudViewPoint (parameter_frame_, viewpoint_cloud_, &tf_);

      // ---[ Optimization: select a subset of the points for faster processing
      // Select points whose distances are up to Xm from the robot
      int nr_p = 0;
      vector<int> indices_in_bounds (cloud_tr_.pts.size ());
      for (unsigned int i = 0; i < cloud_tr_.pts.size (); i++)
      {
        double dist = cloud_geometry::distances::pointToPointDistanceSqr (&viewpoint_cloud_.point, &cloud_tr_.pts[i]);
        if (dist < maximum_search_radius_)
          indices_in_bounds[nr_p++] = i;
      }
      indices_in_bounds.resize (nr_p);

      // NOTE: <leaves_> gets allocated internally in downsamplePointCloud() and is not deallocated on exit
      PointCloud cloud_down_;
      vector<cloud_geometry::Leaf> leaves;
      try
      {
        Point leaf_width_xyz;
        leaf_width_xyz.x = leaf_width_xyz.y = leaf_width_xyz.z = leaf_width_;
        cloud_geometry::downsamplePointCloud (&cloud_tr_, &indices_in_bounds, cloud_down_, leaf_width_xyz, leaves, -1);
        ROS_INFO ("Number of points after downsampling with a leaf of size [%f,%f,%f]: %d.", leaf_width_xyz.x, leaf_width_xyz.y, leaf_width_xyz.z, cloud_down_.pts.size ());
      }
      catch (std::bad_alloc)
      {
        // downsamplePointCloud should issue a ROS_ERROR on screen, so we simply exit here
        return (false);
      }
      leaves.resize (0);    // dealloc memory used for the downsampling process

#if DEBUG
      sendMarker (viewpoint_cloud_.point.x, viewpoint_cloud_.point.y, viewpoint_cloud_.point.z, parameter_frame_, &node_);
#endif
      // Create Kd-Tree and estimate the point normals in the original point cloud
      estimatePointNormals (cloud_tr_, cloud_down_, k_search_, viewpoint_cloud_);

      // Select points whose normals are perpendicular to the Z-axis
      vector<int> indices_xy;
      cloud_geometry::getPointIndicesAxisPerpendicularNormals (&cloud_down_, 0, 1, 2, normal_angle_tolerance_, &z_axis_, indices_xy);

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
      pmap_.header = cloud_down_.header;
      pmap_.polygons.resize (clusters.size ());         // Allocate space for the polygonal map

      ROS_INFO (" - Process all clusters (%i)", clusters.size ());
      vector<int> inliers;
      vector<int> handle_indices;
      vector<double> goodness_factor (clusters.size());
      robot_msgs::Point32 min_p, max_p, handle_center;

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

        if (inliers.size () == 0)
        {
          goodness_factor[cc] = 0;
          continue;
        }

        // Compute the convex hull
        cloud_geometry::areas::convexHull2D (&cloud_down_, &inliers, &coeff[cc], pmap_.polygons[cc]);

        // ---[ If any of the points on the hull are near/outside the imposed "maximum search radius to viewpoint" limit, discard the candidate
        for (int i = 0; i < (int)pmap_.polygons[cc].points.size (); i++)
        {
          double dist = cloud_geometry::distances::pointToPointDistanceSqr (&viewpoint_cloud_.point, &pmap_.polygons[cc].points[i]);
          if (dist > maximum_search_radius_limit_)
          {
            goodness_factor[cc] = 0;
            ROS_DEBUG ("[WARN] Door candidate rejected because the polygonal bounds are near the edges of the cloud!");
            break;
          }
        }

        // ---[ If any of the points on the hull are near/outside the imposed "maximum scan angle to viewpoint" limit, discard the candidate
        // Select points which are acquired at in a horizontal (Z perpendicular) angle of 120 degrees (-60; +60)
        for (int i = 0; i < (int)pmap_.polygons[cc].points.size (); i++)
        {
          // Get the angle with the X axis (0->2pi)
          double angle = cloud_geometry::angles::getAngle2D (pmap_.polygons[cc].points[i].x, pmap_.polygons[cc].points[i].y) + M_PI / 2.0;
          if ( (angle > maximum_scan_angle_limit_) || (angle < (M_PI - maximum_scan_angle_limit_)) )
          {
            goodness_factor[cc] = 0;
            ROS_DEBUG ("[WARN] Door candidate rejected because the polygonal bounds are near the edges of the cloud!");
            break;
          }
        }

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


      // Merge clusters with the same or similar door frame
      double similar_door_eps = 0.1;
      Point32 min_pcc, max_pcc, min_pdd, max_pdd;
      for (unsigned int cc = 0; cc < clusters.size (); cc++)
      {
        if (goodness_factor[cc] == 0)
          continue;
        cloud_geometry::statistics::getMinMax (&pmap_.polygons[cc], min_pcc, max_pcc);
        double area_cc = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[cc], coeff[cc]);

        for (unsigned int dd = cc; dd < clusters.size (); dd++)
        {
          if (cc == dd || goodness_factor[dd] == 0)
            continue;
          cloud_geometry::statistics::getMinMax (&pmap_.polygons[dd], min_pdd, max_pdd);

          // Check if any of the doors have at least a common frame side
          if ((fabs (min_pcc.x - min_pdd.x) < similar_door_eps && fabs (min_pcc.y - min_pdd.y) < similar_door_eps) ||
              (fabs (max_pcc.x - max_pdd.x) < similar_door_eps && fabs (max_pcc.y - max_pdd.y) < similar_door_eps)
              )
          {
            // Check if the normals of the planes are parallel
            double angle = acos ( cloud_geometry::dot (&coeff[cc], &coeff[dd]) );
            if ( (angle < normal_angle_tolerance_) || ( (M_PI - angle) < normal_angle_tolerance_ ) )
            {
              // Check which area is bigger
              double area_dd = cloud_geometry::areas::compute2DPolygonalArea (pmap_.polygons[dd], coeff[dd]);
              if (area_dd < area_cc)
                goodness_factor[dd] = 0;
              else
                goodness_factor[cc] = 0;
            }
          }
        }
      }

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

        // Reply doors message in same frame as request doors message
        resp.doors[nr_d].header.frame_id = req.door.header.frame_id;

        resp.doors[nr_d].header.frame_id = door_frame_;
        resp.doors[nr_d].door_p1 = min_p;
        resp.doors[nr_d].door_p2 = max_p;

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
      cloud_orig_ = *cloud;
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", cloud_orig_.pts.size (), cloud_orig_.header.frame_id.c_str (),
                cloud_orig_.chan.size (), cloud_geometry::getAvailableChannels (&cloud_orig_).c_str ());
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

