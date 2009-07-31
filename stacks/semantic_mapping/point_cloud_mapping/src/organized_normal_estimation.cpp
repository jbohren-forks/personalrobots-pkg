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
 * $Id$
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b organized_normal_estimation estimates basic local surface properties at each 3D point, such as surface normals, curvatures,
                               and moment invariants, for an _organized_ point cloud dataset.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <robot_msgs/PointCloud.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;
using namespace robot_msgs;

class OrganizedNormalEstimation
{
  protected:
    ros::NodeHandle nh_;

  public:

    // ROS messages
    PointCloud cloud_down_, cloud_normals_;

    // Parameters
    double max_z_;
    int downsample_factor_, k_, nr_cols_, nr_rows_;

    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_norm_pub_;
    
    vector<int> nn_indices_;
    int nr_points_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    OrganizedNormalEstimation ()
    {
      nh_.param ("~search_k_closest", k_, 2);                   // Use a window of 2 k-neighbors by default
      nh_.param ("~downsample_factor", downsample_factor_, 4);  // Use every nth point
      // Use a trick to define the maximum angle between two points-viewpoint to avoid bogus normals
      nh_.param ("~k_max_z", max_z_, -1.0);//0.03);
      
      // Size of the organized point cloud is: nr_cols_ * nr_rows_. Defaulting to something like the SwissRanger
      nh_.param ("~nr_cols", nr_cols_, 176);
      nh_.param ("~nr_rows", nr_rows_, 144);
      
      string cloud_topic ("organized_pcd");

      ros::VP_string t_list;
      nh_.getPublishedTopics (t_list);
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

      cloud_sub_ = nh_.subscribe (cloud_topic, 1, &OrganizedNormalEstimation::cloud_cb, this);
      cloud_norm_pub_ = nh_.advertise<PointCloud> ("cloud_normals", 1);
      
      cloud_normals_.chan.resize (4);   // Reserve space for 4 channels: nx, ny, nz, curvature
      cloud_normals_.chan[0].name = "nx";
      cloud_normals_.chan[1].name = "ny";
      cloud_normals_.chan[2].name = "nz";
      cloud_normals_.chan[3].name = "curvatures";
      
      // Reserve enough space for the neighborhood
      nn_indices_.resize ((k_ + k_ + 1) * (k_ + k_ + 1));

      // Reduce by a factor of N
      nr_points_ = lrint (ceil (nr_cols_ / (double)downsample_factor_)) *
                   lrint (ceil (nr_rows_ / (double)downsample_factor_));
      cloud_normals_.pts.resize (nr_points_);
      // Reserve space for 4 channels: nx, ny, nz, curvature
      for (unsigned int d = 0; d < cloud_normals_.chan.size (); d++)
        cloud_normals_.chan[d].vals.resize (nr_points_);

      
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~OrganizedNormalEstimation () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      // Update the downsampling factor
      int downsample_new = 0;
      nh_.getParam ("~downsample_factor", downsample_new);
      if (downsample_new != downsample_factor_)
      {
        downsample_factor_ = downsample_new;
        nr_points_ = lrint (ceil (nr_cols_ / (double)downsample_factor_)) *
                     lrint (ceil (nr_rows_ / (double)downsample_factor_));
        cloud_normals_.pts.resize (nr_points_);
        // Reserve space for 4 channels: nx, ny, nz, curvature
        for (unsigned int d = 0; d < cloud_normals_.chan.size (); d++)
          cloud_normals_.chan[d].vals.resize (nr_points_);
      }
      
      // Update the number of neighbors
      int k_new = 0;
      nh_.getParam ("~search_k_closest", k_new);
      if (k_new != k_)
      {
        k_ = k_new;
        nn_indices_.resize ((k_ + k_ + 1) * (k_ + k_ + 1));
      }
      nh_.getParam ("~max_z", max_z_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (const PointCloudConstPtr& cloud)
    {
      updateParametersFromServer ();

      ROS_DEBUG ("Received %d data points in frame %s with %d channels (%s).", (int)cloud->pts.size (), cloud->header.frame_id.c_str (),
                 (int)cloud->chan.size (), cloud_geometry::getAvailableChannels (cloud).c_str ());
      if (cloud->pts.size () == 0)
      {
        ROS_ERROR ("No data points found. Exiting...");
        return;
      }

      // Viewpoint value in the point cloud frame should be 0,0,0
      Point32 viewpoint_cloud;
      viewpoint_cloud.x = viewpoint_cloud.y = viewpoint_cloud.z = 0.0;

      cloud_normals_.header = cloud->header;
            
      // Estimate point normals and copy the relevant data
      //cloud_geometry::nearest::computeOrganizedPointCloudNormals (cloud_normals_, cloud, k_, downsample_factor_, nr_cols_, nr_rows_, max_z_, viewpoint_cloud);
      
      // Note: the above is an optimized copy of cloud_geometry::nearest::computeOrganizedPointCloudNormals
      // For general-purpose applications, please use computeOrganizedPointCloudNormals () and not this code.
      ros::Time ts = ros::Time::now ();
      Eigen::Vector4d plane_parameters;
      double curvature;

      int j = 0;
#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)cloud->pts.size (); i++)
      {
        // Obtain the <u,v> pixel values
        int u = i / nr_cols_;
        int v = i % nr_cols_;

        // Get every Nth pixel in both rows, cols
        if ((u % downsample_factor_ != 0) || (v % downsample_factor_ != 0))
          continue;

        // Copy the data
        cloud_normals_.pts[j] = cloud->pts[i];

        // Get all point neighbors in a k x k window
        int l = 0;
        for (int x = -k_; x < k_+1; x++)
        {
          for (int y = -k_; y < k_+1; y++)
          {
            int idx = (u+x) * nr_cols_ + (v+y);
            if (idx == i)
              continue;
            // If the index is not in the point cloud, continue
            if (idx < 0 || idx >= (int)cloud->pts.size ())
              continue;
            // If the difference in Z (depth) between the query point and the current neighbor is smaller than max_z
            if (max_z_ != -1)
            {
              if ( fabs (cloud_normals_.pts[j].z - cloud->pts[idx].z) <  max_z_ )
                nn_indices_[l++] = idx;
            }
            else
              nn_indices_[l++] = idx;
          }
        }
        nn_indices_.resize (l);
        if (nn_indices_.size () < 4)
        {
          //ROS_ERROR ("Not enough neighboring indices found for point %d (%f, %f, %f).", i, cloud->pts[i].x, cloud->pts[i].y, cloud->pts[i].z);
          continue;
        }

        // Compute the point normals (nx, ny, nz), cloud curvature estimates (c)
        cloud_geometry::nearest::computePointNormal (cloud, nn_indices_, plane_parameters, curvature);
        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud->pts[i], viewpoint_cloud);

        cloud_normals_.chan[0].vals[j] = plane_parameters (0);
        cloud_normals_.chan[1].vals[j] = plane_parameters (1);
        cloud_normals_.chan[2].vals[j] = plane_parameters (2);
        cloud_normals_.chan[3].vals[j] = curvature;
        j++;
      }
      
      ROS_INFO ("Normals estimated in %g seconds.", (ros::Time::now () - ts).toSec ());

      cloud_norm_pub_.publish (cloud_normals_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "organized_normal_estimation_node");

  OrganizedNormalEstimation p;
  ros::spin ();

  return (0);
}
/* ]--- */

