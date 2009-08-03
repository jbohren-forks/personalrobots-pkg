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
#ifndef PCD_NOVELTY_NOVELTY_ESTIMATOR_NODE_H
#define PCD_NOVELTY_NOVELTY_ESTIMATOR_NODE_H

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>


#include "novelty_estimator.h"

namespace pcd_novelty
{

class NoveltyEstimatorNode
{
protected:
  NoveltyEstimator estimator_;

  ros::NodeHandle n_;
  
  sensor_msgs::PointCloudConstPtr cloud_hist_;
  sensor_msgs::PointCloudConstPtr cloud_observed_;
  
  ros::Subscriber cloud_hist_sub_;
  ros::Subscriber cloud_sub_;

  ros::Publisher  cloud_pub_;

  ros::Publisher  marker_pub_;
  
public:

  boost::mutex proc_mutex_;

public:
  NoveltyEstimatorNode();

  void setup();

  void hist_cloudCallback(const sensor_msgs::PointCloudConstPtr& the_cloud);
  void observed_cloudCallback(const sensor_msgs::PointCloudConstPtr& the_cloud);


};



}


#endif
