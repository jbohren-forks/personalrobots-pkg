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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */

#ifndef POINT_CLOUD_UTILS_SCAN_ASSEMBLER_H
#define POINT_CLOUD_UTILS_SCAN_ASSEMBLER_H

#include <string>

#include "ros/node.h"

#include "tf/transform_listener.h"
#include "laser_scan/laser_scan.h"

// Messages
#include "std_msgs/LaserScan.h"
#include "std_msgs/PointCloud.h"

namespace point_cloud_utils {

/**
 * \brief Wrapper around LaserProjection object to help aggregate laser scans
 * Manages some of the annoying bookkeeping associated with assembling multiple laser scans together
 * into a single point cloud.
 **/
class ScanAssembler
{
public:

  ScanAssembler(ros::Node& cur_node) ;
  ~ScanAssembler() ;
  
  /**
   * \brief Clears any previously aggregated point cloud data in order to begin collecting new data
   * \param target_frame The target frame in the transform tree that we want the point cloud data to reside data in
   * \param reserve Specifies how many points should be preallocated in the cloud (defaults to 0)
   */
  void startNewCloud(const std::string& target_frame, unsigned int reserve = 0) ;
  
  /**
   * \brief Adds a single scan to the current point cloud
   * Note that this call can fail if the source frame and target frame don't exist in the transform tree
   * \param scan The scan that we want to add to the point cloud
   * \return Negative value on error
   */
  int addScan(const std_msgs::LaserScan& scan ) ;
  
  /**
   * \brief Retrieves the current assembled point cloud
   * The timestamp on the cloud is the timestamp of the last scan that was added
   * \param cloud Point Cloud to populate
   */
  void getPointCloud(std_msgs::PointCloud& cloud) const ;

private:
  tf::TransformListener tf_ ;
  std_msgs::PointCloud cloud_ ;
  unsigned int point_count_ ;
  laser_scan::LaserProjection projector_ ;
} ;

}



#endif // POINT_CLOUD_UTILS_SCAN_ASSEMBLER_H
