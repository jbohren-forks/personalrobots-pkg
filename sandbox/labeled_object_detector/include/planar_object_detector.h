/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//! \author Alex Sorokin 

// ROS core
#include <ros/ros.h>
// ROS messages
#include <robot_msgs/PointCloud.h>


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
#include <point_cloud_mapping/geometry/projections.h>

#include <angles/angles.h>

// transform library
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <point_cloud_assembler/BuildCloud.h>

namespace labeled_object_detector
{

class PlanarObjectDetector
{
protected:
  ros::NodeHandle n_;
  
  robot_msgs::PointCloudConstPtr cloud_;
  
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;

  ros::Publisher marker_pub_;
  
  tf::TransformListener tf_;
  tf::TransformBroadcaster broadcaster_;
  
  double min_height_;

  double max_link_distance_;

  robot_msgs::PointCloud full_cloud;


public:

  robot_msgs::PointCloud filtered_cloud_;
  
  std::vector<int> cluster_ids_;
  unsigned int num_clusters_;
  
  std::vector<std::vector<int> > clouds_by_indices_;
  
  std::vector<boost::shared_ptr<sample_consensus::SACModelPlane> > plane_models_; 
  std::vector<std::vector<double> > plane_coeffs_;
  
  boost::shared_ptr<cloud_kdtree::KdTreeANN> object_points_kd_tree_;


  boost::mutex proc_mutex_;

public:
  PlanarObjectDetector();

  void setup();

  void cloudCallback(const robot_msgs::PointCloudConstPtr& the_cloud);
  void detectObject(const robot_msgs::PointCloud& point_cloud);

  void addObjectFrame(robot_msgs::PointStamped origin, const std::vector<double>& plane);

  bool fitSACPlane (const robot_msgs::PointCloud& points, const std::vector<int> &indices, 
                    std::vector<int> &inliers, std::vector<double> &coeff, // output
                    boost::shared_ptr<sample_consensus::SACModelPlane> &model_output, // output
                    double dist_thresh, int min_points_per_model);

  void fitObjectModel2Cloud(const unsigned int model_id,
                            const robot_msgs::PointCloud& global_cloud,
                            const robot_msgs::PointCloud& local_cloud,
                            const std::vector<int> observation_ids);

  void publishObjectMarker(float w,float h);
  void publishObjectMarker2(float x,float y,float z);
};



}
