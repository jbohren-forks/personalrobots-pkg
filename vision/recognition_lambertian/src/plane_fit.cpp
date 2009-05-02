/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 */

// ROS core
#include <ros/node.h>
// ROS messages
#include <robot_msgs/PointCloud.h>
#include <robot_msgs/Polygon3D.h>

// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>

using namespace std;
using namespace robot_msgs;

class PlanarFit
{

  void getPointIndicesInZBounds (const PointCloud &points, double z_min, double z_max, vector<int> &indices);
  bool fitSACPlanes (PointCloud *points, vector<int> &indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff,
                     const robot_msgs::Point32 &viewpoint_cloud, double dist_thresh, int n_max, int min_points_per_model = 100);

  /**
      * Input
            o list of X,Y,Z,x,y points
            o Z min, Z max (only process points within min to max range
            o Support  where "Support" is a vertical distance above the plane where an object not on that plane will be considered to be supported by that plane  (OK, if this takes time, don't do it right now)
            o A where A is the minimal area a plane should have (again, don't bother with for now unless its already there)
            o N max number of planes to find in order of size
      * Output
            o list of indices in order of input list. Each indices is the tuple x,y,A where x,y is the pixel and A is "computed attribute"
                  + 0 is illegal range or othersize "no data"
                  + 1-N where this numbers the planes in terms of area. 1 is largest plane, 2 is next largest ...
                  + -1, -2, -3 means supported object by cluster (or for now for speed, all points that are not a 0 or not a plane are -1).


\NOTE: not sure if I understand the support parameter :(

   **/
  void
    segmentPlanes (PointCloud &points, double z_min, double z_max, double support, double min_area, int n_max,
                   vector<vector<int> > &indices, vector<vector<double> > &models)
  {
    // This should be given as a parameter as well, or set global, etc
    double sac_distance_threshold_ = 0.007;        // 2cm distance threshold for inliers (point-to-plane distance)

    vector<int> indices_in_bounds;
    // Get the point indices within z_min <-> z_max
    getPointIndicesInZBounds (points, z_min, z_max, indices_in_bounds);

    // We need to know the viewpoint where the data was acquired
    // For simplicity, assuming 0,0,0 for stereo data in the stereo frame - however if this is not true, use TF to get
    //the point in a different frame !
    Point32 viewpoint;
    viewpoint.x = viewpoint.y = viewpoint.z = 0;

    // Use the entire data to estimate the plane equation.
    // NOTE: if this is slow, we can downsample first, then fit (check mapping/point_cloud_mapping/src/planar_fit.cpp)
 //   vector<vector<int> > inliers;
    indices.clear(); //Points that are in plane
    models.clear();  //Plane equations
//    vector<vector<double> > models;
    fitSACPlanes (&points, indices_in_bounds, indices, models, viewpoint, sac_distance_threshold_, n_max);

    // Check the list of planar areas found against the minimally imposed area
    for (unsigned int i = 0; i < models.size (); i++)
    {
      // Compute the convex hull of the area
      // NOTE: this is faster than computing the concave (alpha) hull, so let's see how this works out
      Polygon3D polygon;
      cloud_geometry::areas::convexHull2D (points, indices[i], models[i], polygon);

      // Compute the area of the polygon
      double area = cloud_geometry::areas::compute2DPolygonalArea (polygon, models[i]);

      // If the area is smaller, reset this planar model
      if (area < min_area)
      {
        models[i].resize (0);
        indices[i].resize (0);
        continue;
      }
    }

//    // Copy all the planar models inliers to indices
//    for (unsigned int i = 0; i < inliers.size (); i++)
//    {
//      if (inliers[i].size () == 0) continue;
//
//      int old_indices_size = indices.size ();
//      indices.resize (old_indices_size + inliers[i].size ());
//      for (unsigned int j = 0; j < inliers[i].size (); j++)
//        indices[old_indices_size + j] = inliers[i][j];
//    }
  }

  protected:
    ros::Node& node_;

  public:

    // ROS messages
    PointCloud cloud_, cloud_plane_, cloud_outliers_;

    double z_min_, z_max_, support_, min_area_;
    int n_max_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlanarFit (ros::Node& anode) : node_ (anode)
    {
      node_.param ("~z_min", z_min_, 0.5);
      node_.param ("~z_max", z_max_, 1.5);
      node_.param ("~support", support_, 0.1);
      node_.param ("~min_area", min_area_, 0.2);
      node_.param ("~n_max", n_max_, 1);

      string cloud_topic ("cloud_pcd");

      vector<pair<string, string> > t_list;
      node_.getPublishedTopics (&t_list);
      bool topic_found = false;
      for (vector<pair<string, string> >::iterator it = t_list.begin (); it != t_list.end (); it++)
      {
        if (it->first.find (node_.mapName (cloud_topic)) != string::npos)
        {
          topic_found = true;
          break;
        }
      }
      if (!topic_found)
        ROS_WARN ("Trying to subscribe to %s, but the topic doesn't exist!", node_.mapName (cloud_topic).c_str ());

      node_.subscribe (cloud_topic, cloud_, &PlanarFit::cloud_cb, this, 1);
      node_.advertise<PointCloud> ("~plane", 1);
      node_.advertise<PointCloud> ("~outliers", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb ()
    {
      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_.pts.size (), cloud_.header.frame_id.c_str (),
                (int)cloud_.chan.size (), cloud_geometry::getAvailableChannels (cloud_).c_str ());
      if (cloud_.pts.size () == 0)
      {
        ROS_ERROR ("No data points found. Exiting...");
        return;
      }

      ros::Time ts = ros::Time::now ();

      vector<vector<int> > indices;
      vector<vector<double> > models;
      segmentPlanes (cloud_, z_min_, z_max_, support_, min_area_, n_max_, indices, models);

      if((int)indices.size() > 0){
      cloud_geometry::getPointCloud (cloud_, indices[0], cloud_plane_);
      cloud_geometry::getPointCloudOutside (cloud_, indices[0], cloud_outliers_);
      ROS_INFO ("Planar model found with %d / %d inliers in %g seconds.\n", (int)indices[0].size (), (int)cloud_.pts.size (), (ros::Time::now () - ts).toSec ());
      }

      node_.publish ("~plane", cloud_plane_);
      node_.publish ("~outliers", cloud_outliers_);
    }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Obtain a subset of the point cloud indices between two given Z (min, max) values
  * \param points the point cloud message
  * \param z_min the minimum Z value
  * \param z_max the maximum Z value
  */
void
  PlanarFit::getPointIndicesInZBounds (const PointCloud &points, double z_min, double z_max, vector<int> &indices)
{
  indices.resize (points.pts.size ());
  int nr_p = 0;
  for (unsigned int i = 0; i < points.pts.size (); i++)
  {
    if ((points.pts[i].z >= z_min && points.pts[i].z <= z_max))
    {
      indices[nr_p] = i;
      nr_p++;
    }
  }
  indices.resize (nr_p);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Find a list of plane models in a point cloud with SAmple Consensus methods
  * \param points the point cloud message
  * \param indices a subset of point indices to use
  * \param inliers the resultant planar model inliers
  * \param coeff the resultant plane model coefficients
  * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
  * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
  * \param n_max maximum number of planar models to search for
  * \param min_points_per_model the minimum number of points allowed for a planar model (default: 100)
  */
bool
  PlanarFit::fitSACPlanes (PointCloud *points, vector<int> &indices, vector<vector<int> > &inliers, vector<vector<double> > &coeff,
                           const robot_msgs::Point32 &viewpoint_cloud, double dist_thresh, int n_max, int min_points_per_model)
{
  // Create and initialize the SAC model
  sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
  sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
  sac->setMaxIterations (100);
  model->setDataSet (points, indices);

  int nr_models = 0, nr_points_left = indices.size ();
  while (nr_models < n_max && nr_points_left > min_points_per_model)
  {
    // Search for the best plane
    if (sac->computeModel ())
    {
      vector<double> model_coeff;
      sac->computeCoefficients (model_coeff);                              // Compute the model coefficients
      sac->refineCoefficients (model_coeff);                               // Refine them using least-squares
      coeff.push_back (model_coeff);

      // Get the list of inliers
      vector<int> model_inliers;
      model->selectWithinDistance (model_coeff, dist_thresh, model_inliers);
      inliers.push_back (model_inliers);

      // Flip the plane normal towards the viewpoint
      cloud_geometry::angles::flipNormalTowardsViewpoint (model_coeff, points->pts.at (model_inliers[0]), viewpoint_cloud);

      ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)model_inliers.size (),
                model_coeff[0], model_coeff[1], model_coeff[2], model_coeff[3]);

      // Remove the current inliers from the global list of points
      nr_points_left = sac->removeInliers ();
      nr_models++;
    }
    else
    {
      ROS_ERROR ("Could not compute a planar model for %d points.", nr_points_left);
      break;
    }
  }

  delete sac;
  delete model;
  return (true);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv);

  ros::Node ros_node ("planar_fit");

  PlanarFit p (ros_node);
  ros_node.spin ();

  return (0);
}
/* ]--- */

