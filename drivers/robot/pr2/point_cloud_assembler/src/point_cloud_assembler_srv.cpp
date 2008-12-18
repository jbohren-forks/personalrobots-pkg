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

#include "ros/node.h"
#include "tf/transform_listener.h"
#include "std_msgs/PointCloud.h"

#include <deque>

// Service
#include "point_cloud_assembler/BuildCloud.h"

// Laser projection
#include "laser_scan/laser_scan.h"

#include "math.h"

using namespace std_msgs;

using namespace std ;

namespace point_cloud_assembler
{

/**
 * \brief Maintains a history of point clouds and generates an aggregate point cloud upon request
 * \todo Clean up the doxygen part of this header
 * params
 *  * "~tf_cache_time_secs" (double) - The cache time (in seconds) to holds past transforms
 *  * "~tf_extrap_limit" (double) - The extrapolation limit for TF (in seconds)
 *  * "~max_scans" (unsigned int) - The number of scans to store in the assembler's history, until they're thrown away
 */
class PointCloudAssemblerSrv : public ros::node
{
public:

  tf::TransformListener* tf_ ;

  PointCloud scan_ ;

  unsigned int max_scans_ ;
  bool ignore_laser_skew_ ;

  deque<PointCloud> scan_hist_ ;           //!< Stores history of scans. We want them in time-ordered, which is (in most situations) the same as time-of-receipt-ordered
  unsigned int total_pts_ ;                //!< Stores the total number of range points in the entire stored history of scans. Useful for estimating points/scan

  PointCloudAssemblerSrv() : ros::node("point_cloud_assembler_srv")
  {
    // **** Initialize TransformListener ****
    double tf_cache_time_secs ;
    param("~tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
    if (tf_cache_time_secs < 0)
      ROS_ERROR("Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;
    unsigned long long tf_cache_time = tf_cache_time_secs*1000000000ULL ;
    tf_ = new tf::TransformListener(*this, true, tf_cache_time) ;
    ROS_INFO("TF Cache Time: %f Seconds", tf_cache_time_secs) ;

    // **** Set TF Extrapolation Limit ****
    double tf_extrap_limit_secs ;
    param("~tf_extrap_limit", tf_extrap_limit_secs, 0.00) ;
    if (tf_extrap_limit_secs < 0.0)
      ROS_ERROR("Parameter tf_extrap_limit<0 (%f)", tf_extrap_limit_secs) ;

    ros::Duration extrap_limit ;
    extrap_limit.fromSec(tf_extrap_limit_secs) ;
    tf_->setExtrapolationLimit(extrap_limit) ;
    ROS_INFO("TF Extrapolation Limit: %f Seconds", tf_extrap_limit_secs) ;

    // ***** Set max_scans *****
    const int default_max_scans = 400 ;
    int tmp_max_scans ;
    param("~max_scans", tmp_max_scans, default_max_scans) ;
    if (tmp_max_scans < 0)
    {
      ROS_ERROR("Parameter max_scans<0 (%i)", tmp_max_scans) ;
      tmp_max_scans = default_max_scans ;
    }
    max_scans_ = tmp_max_scans ;
    ROS_INFO("Max Scans in History: %u", max_scans_) ;
    total_pts_ = 0 ;    // We're always going to start with no points in our history

    // ***** Start Services *****
    advertise_service(get_name()+"/build_cloud", &PointCloudAssemblerSrv::buildCloud, this, 0) ; // Don't spawn threads so that we can avoid dealing with mutexing [for now]
    subscribe("cloud_in", scan_, &PointCloudAssemblerSrv::scans_callback, 40) ;      // Choose something reasonably large, so data doesn't get dropped
  }

  ~PointCloudAssemblerSrv()
  {
    unadvertise_service(get_name()+"/build_cloud") ;
    unsubscribe("cloud_in") ;
    delete tf_ ;
  }

  void scans_callback()
  {
    if (scan_hist_.size() == max_scans_)                                                // Is our deque full?
    {
      total_pts_ -= scan_hist_.front().get_pts_size() ;                              // We're removing an elem, so this reduces our total point count
      scan_hist_.pop_front() ;                                                          // The front of the deque has the oldest elem, so we can get rid of it
    }

    scan_hist_.push_back(scan_) ;                                                       // Add the newest scan to the back of the deque
    total_pts_ += scan_.get_pts_size() ;                                             // Add the new scan to the running total of points

    //printf("LaserScanAssemblerSrv:: Got Scan: TotalPoints=%u", total_pts_) ;
  }

  bool buildCloud(BuildCloud::request& req, BuildCloud::response& resp)
  {
    // Allocate space for the cloud
    resp.cloud.set_pts_size( total_pts_ ) ;                                             // There's no way to have a point cloud bigger than our entire history of scans.
    resp.cloud.set_chan_size(1) ;
    resp.cloud.chan[0].name = "intensities" ;                                           //! \todo More smartly deal with multiple channels
    resp.cloud.chan[0].set_vals_size( total_pts_ ) ;

    unsigned int cloud_count = 0 ;                                                      // Store the number of points in the current cloud

    PointCloud target_frame_cloud ;                                              // Stores the current scan in the target frame

    unsigned int i = 0 ;

    // Find the beginning of the request. Probably should be a search
    while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
            scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
    {
      i++ ;
    }

    unsigned int start_index = i ;

    unsigned int tf_ex_count = 0 ;              // Keep a count of how many TF exceptions we got (for diagnostics)
    unsigned int scan_lines_count = 0 ;         // Keep a count of how many scan lines we aggregated (for diagnostics)

    // Populate the cloud
    while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
            scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
    {
      const PointCloud& cur_scan = scan_hist_[i] ;

      try
      {
        tf_->transformPointCloud(req.target_frame_id, cur_scan, target_frame_cloud) ;

        for(unsigned int j = 0; j < target_frame_cloud.get_pts_size(); j++)               // Populate full_cloud from the cloud
        {
          resp.cloud.pts[cloud_count].x        = target_frame_cloud.pts[j].x ;
          resp.cloud.pts[cloud_count].y        = target_frame_cloud.pts[j].y ;
          resp.cloud.pts[cloud_count].z        = target_frame_cloud.pts[j].z ;
          resp.cloud.chan[0].vals[cloud_count] = target_frame_cloud.chan[0].vals[j] ;
          cloud_count++ ;
        }
        resp.cloud.header = target_frame_cloud.header ;                                   //! \todo Find a better place to do this/way to do this
      }
      catch(tf::TransformException& ex)
      {
        //ROS_WARN("Transform Exception %s", ex.what()) ;
        tf_ex_count++ ;
      }

      scan_lines_count++ ;
      i++ ;                                                                             // Check the next scan in the scan history
    }
    int end_index = i ;

    resp.cloud.set_pts_size( cloud_count ) ;                                            // Resize the output accordingly
    resp.cloud.chan[0].set_vals_size( cloud_count ) ;

    ROS_INFO("Point Cloud Results: Aggregated from index %u->%u. Total ScanLines: %u.  BufferSize: %u", start_index, end_index, scan_lines_count, scan_hist_.size()) ;
    if (tf_ex_count > 0)
      ROS_WARN("%u TF Exceptions while generating Point Cloud", tf_ex_count) ;

    return true ;
  }
};

}

using namespace point_cloud_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  PointCloudAssemblerSrv pc_assembler;
  pc_assembler.spin();
  ros::fini();
  return 0;
}
