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
#ifndef OBJDET_DETECT_NEAREST_OBJECT_ACTION_H
#define OBJDET_DETECT_NEAREST_OBJECT_ACTION_H

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
#include <point_cloud_mapping/geometry/distances.h>

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
#include <planar_object_detector.h>

#include <robot_actions/action.h>

#include <robot_msgs/PoseStamped.h>

namespace labeled_object_detector
{

class DetectNearestObjectAction: public robot_actions::Action<robot_msgs::PoseStamped, robot_msgs::PoseStamped> {

protected:
  PlanarObjectDetector detector_;

  ros::NodeHandle n_;
  
  robot_msgs::PointCloudConstPtr cloud_;
  
  boost::shared_ptr<ros::Subscriber>  cloud_sub_;
  ros::Publisher  cloud_pub_;
  ros::Publisher  box_pub_;

  ros::Publisher marker_pub_;
  
  boost::shared_ptr<tf::TransformListener> tf_;
  tf::TransformBroadcaster broadcaster_;

  std::string fixed_frame_;
  robot_msgs::PointCloud full_cloud;

  robot_msgs::PoseStamped goal_;
  robot_msgs::PoseStamped goal_fixed_;

public:

  boost::mutex proc_mutex_;

public:
  DetectNearestObjectAction(const std::string& name);


  void setup();

  void cloudCallback(const robot_msgs::PointCloudConstPtr& the_cloud);

  virtual robot_actions::ResultStatus execute(const robot_msgs::PoseStamped& goal, robot_msgs::PoseStamped& feedback);
};



}


#endif
