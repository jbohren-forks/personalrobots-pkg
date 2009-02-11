
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

#ifndef RANSAC_GROUND_PLANE_EXTRACTION_H
#define RANSAC_GROUND_PLANE_EXTRACTION_H

#include <smartScan.h>
#include <robot_msgs/Point32.h>
#include <robot_msgs/Point.h>
#include <robot_msgs/PointStamped.h>
#include <robot_msgs/Vector3.h>

namespace ransac_ground_plane_extraction {

  class RansacGroundPlaneExtraction
 {
  /**
   * @brief Ransac ground plane extraction
   * access
   */
  public:
  
   RansacGroundPlaneExtraction();
  
   int findGround(const robot_msgs::PointCloud& baseFrameCloud, const double &min_ignore_distance, const double &max_ignore_distance, const double &distance_threshold, robot_msgs::Point32 &planePoint, robot_msgs::Point32 &planeNormal);

  robot_msgs::PointCloud *removeGround(const robot_msgs::PointCloud& baseFrameCloud, double remove_distance, const robot_msgs::Point32 &planePoint, robot_msgs::Point32 &planeNormal, const robot_msgs::PointStamped &origin, const double &threshold_distance, const double &far_remove_distance);
  robot_msgs::PointCloud *removeGround(const robot_msgs::PointCloud& baseFrameCloud, double remove_distance, const robot_msgs::Point &point_plane, robot_msgs::Vector3 &normal_plane, const robot_msgs::PointStamped &origin, const double &threshold_distance, const double &far_remove_distance);

  void updateGround(const robot_msgs::Point32 &new_plane_point, const robot_msgs::Point32 &new_plane_normal, robot_msgs::Point32 &return_plane_point, robot_msgs::Point32 &return_plane_normal);

  double filter_delta_;

  int max_iterations_;

   private:

  robot_msgs::Point32 point_plane_;

  robot_msgs::Point32 normal_plane_;
  
  };
}
#endif


