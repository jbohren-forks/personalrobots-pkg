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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <ransac_ground_plane_extraction/ransac_ground_plane_extraction_node.h>

using namespace ransac_ground_plane_extraction;

RansacGroundPlaneExtractionNode::RansacGroundPlaneExtractionNode(std::string node_name):ros::node(node_name),obstacle_cloud_(NULL)
{
  this->param<std::string>("ransac_ground_plane_extraction/listen_topic",listen_topic_,"full_cloud");
  this->param<std::string>("ransac_ground_plane_extraction/publish_ground_plane_topic",publish_ground_plane_topic_,"ground_plane");
  this->param<std::string>("ransac_ground_plane_extraction/publish_obstacle_topic",publish_obstacle_topic_,"obstacle_cloud");

  this->param<double>("ransac_ground_plane_extraction/min_ignore_distance",min_ignore_distance_,-0.01);
  this->param<double>("ransac_ground_plane_extraction/max_ignore_distance",max_ignore_distance_,0.01);
  this->param<double>("ransac_ground_plane_extraction/distance_threshold",distance_threshold_,0.01);

  this->param<double>("ransac_ground_plane_extraction/filter_delta",filter_delta_,0.5);
  this->param<int>("ransac_ground_plane_extraction/max_ransac_iterations",max_ransac_iterations_,500);

  subscribe(listen_topic_,  cloud_msg_,  &RansacGroundPlaneExtractionNode::cloudCallback,1);
  advertise<std_msgs::PointCloud>(publish_obstacle_topic_,1);
//  advertise<std_msgs::>(publish_topic_, plane_msg_);

  ground_plane_extractor_.max_iterations_ = max_ransac_iterations_;
  ground_plane_extractor_.filter_delta_ = filter_delta_;
}

RansacGroundPlaneExtractionNode::~RansacGroundPlaneExtractionNode()
{
  unadvertise(publish_obstacle_topic_);
  unsubscribe(listen_topic_);
}


void RansacGroundPlaneExtractionNode::cloudCallback()
{

  std_msgs::Point32 plane_point;
  std_msgs::Point32 plane_normal;

  std_msgs::Point32 estimated_plane_point;
  std_msgs::Point32 estimated_plane_normal;

  ground_plane_extractor_.findGround(cloud_msg_,min_ignore_distance_,max_ignore_distance_,distance_threshold_,plane_point,plane_normal);
  ground_plane_extractor_.updateGround(plane_point,plane_normal,estimated_plane_point,estimated_plane_normal);
  obstacle_cloud_ =  ground_plane_extractor_.removeGround(cloud_msg_, distance_threshold_, estimated_plane_point,estimated_plane_normal);

//  publish(publish_ground_plane_topic_,ground_plane_msg_);
  publish(publish_obstacle_topic_,*obstacle_cloud_);
}


