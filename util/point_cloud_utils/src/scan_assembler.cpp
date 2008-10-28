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

#include "point_cloud_utils/scan_assembler.h"

using namespace point_cloud_utils ;
using namespace std_msgs ;

ScanAssembler::ScanAssembler(ros::node& cur_node) : tf_(cur_node)
{
  startNewCloud("invalid_frame", 0) ;
}

ScanAssembler::~ScanAssembler()
{
  
}

void ScanAssembler::startNewCloud(const std::string& target_frame, unsigned int reserve)
{
  cloud_.set_pts_size(reserve) ;
  cloud_.header.frame_id = target_frame ;

  cloud_.set_chan_size(1) ;
  cloud_.chan[0].name = "intensities" ;
  cloud_.chan[0].set_vals_size( reserve ) ;
  
  point_count_ = 0 ;
}

int ScanAssembler::addScan(const std_msgs::LaserScan& scan)
{
  PointCloud target_frame_cloud ;                                                     // Stores the current scan in the target frame

  tf_.transformLaserScanToPointCloud(cloud_.header.frame_id, target_frame_cloud, scan) ;              //! \todo Add a try/catch block around this TF calls
  
  cloud_.header.stamp = scan.header.stamp ;                                           // Give the cloud the stamp of the latest scan received

  unsigned int new_point_count = point_count_ + scan.get_ranges_size() ;              // Defines the point count after adding the data
  if (cloud_.get_pts_size() <  new_point_count)                                       // Will the new data fit in our current cloud?
  {
    cloud_.set_pts_size(new_point_count) ;                                            //    If not, then resize accordingly
    cloud_.chan[0].set_vals_size(new_point_count) ;
  }
  
  for(unsigned int j = 0; j < target_frame_cloud.get_pts_size(); j++)                 // Copy points from the current scan's cloud to the aggregated cloud
  {
    cloud_.pts[point_count_].x        = target_frame_cloud.pts[j].x ;  
    cloud_.pts[point_count_].y        = target_frame_cloud.pts[j].y ;  
    cloud_.pts[point_count_].z        = target_frame_cloud.pts[j].z ;
    cloud_.chan[0].vals[point_count_] = target_frame_cloud.chan[0].vals[j] ;
    point_count_++ ;
  }
  
  return 0 ;
}

void ScanAssembler::getPointCloud(std_msgs::PointCloud& cloud) const
{
  cloud = cloud_ ;
  cloud.set_pts_size(point_count_) ;
}

void ScanAssembler::getPointCloudFloat32(std_msgs::PointCloudFloat32& cloud) const
{
  cloud = *( (PointCloudFloat32*) &cloud_) ;                                          // Old and new pointclouds have the same memory footprint, so this casting is ok.
  cloud.set_pts_size(point_count_) ;
}
