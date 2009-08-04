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

#ifndef OBJDET_PLANAR_OBJECT_DETECTOR_H
#define OBJDET_PLANAR_OBJECT_DETECTOR_H

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>


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

#include "labeled_object_detector/BoundingBox.h"
#include "object_model.h"

namespace labeled_object_detector
{

class PlanarObjectDetector
{
protected:
  ros::NodeHandle n_;
  
  boost::shared_ptr<tf::TransformListener> tf_;
  
  double min_height_;

  double max_link_distance_;
  double max_search_radius_;
  int min_points_per_model_;


  std::string annotation_channel_;
  std::string target_object_;

  float target_label_;


  std::string local_frame_;
  std::string fixed_frame_;

  sensor_msgs::PointCloud filtered_cloud_;
  
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

  void setTFListener(  boost::shared_ptr<tf::TransformListener> tf_listener);


  void detectObjects(const sensor_msgs::PointCloud& point_cloud,ObjectModelDeque& objects);

  bool fitSACPlane (const sensor_msgs::PointCloud& points, const std::vector<int> &indices, 
                    std::vector<int> &inliers, std::vector<double> &coeff, // output
                    boost::shared_ptr<sample_consensus::SACModelPlane> &model_output, // output
                    double dist_thresh, int min_points_per_model);

  bool fitObjectModel2Cloud(const unsigned int model_id,
                            const sensor_msgs::PointCloud& global_cloud,
                            const sensor_msgs::PointCloud& local_cloud,
                            const std::vector<int> observation_ids,
                            PlanarObjectModelPtr& out_object);




  void makeObjectFrame(const geometry_msgs::PointStamped origin, const std::vector<double>& plane, const std::string object_frame,PlanarObjectModelPtr object);

  void makeObjectMarker(const geometry_msgs::Point pt1,const geometry_msgs::Point pt2,const unsigned int model_id,PlanarObjectModelPtr object);

};



}


#endif
