/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 * Authors: Sachin Chitta
 */

#ifndef RANSAC_GROUND_PLANE_EXTRACTION_NODE_H
#define RANSAC_GROUND_PLANE_EXTRACTION_NODE_H

#include <ransac_ground_plane_extraction/ransac_ground_plane_extraction.h>
#include <std_msgs/PointCloud.h>
#include <std_msgs/Point32.h>
#include <std_msgs/PointStamped.h>
#include <pr2_msgs/PlaneStamped.h>
#include <ros/node.h>
#include <tf/transform_listener.h>


namespace ransac_ground_plane_extraction {

  class RansacGroundPlaneExtractionNode : public ros::Node
 {
  /**
   * @brief Ransac ground plane extraction
   * access
   */
  public:  

   RansacGroundPlaneExtractionNode(std::string node_name);

   ~RansacGroundPlaneExtractionNode();
  
   private:

  double max_ignore_distance_;

  double min_ignore_distance_;

  double distance_threshold_;

  double far_remove_distance_threshold_;

  double far_remove_distance_;

  double filter_delta_;

  int max_ransac_iterations_;

  RansacGroundPlaneExtraction ground_plane_extractor_;

  std_msgs::PointCloud cloud_msg_; /**< Filled by subscriber with new clouds*/

  std_msgs::PointCloud *obstacle_cloud_;

  std::string listen_topic_;

  std::string publish_ground_plane_topic_;

  std::string publish_obstacle_topic_;

  bool publish_obstacle_cloud_;

  void cloudCallback();

  tf::TransformListener tf_; /**< Used to do transforms */

  };
}
#endif



