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


/**

@mainpage

@htmlinclude manifest.html

@b point_cloud_assembler accumulates scan lines from a laser range finder mounted on a tilting stage and generates 3D point clouds.

<hr>

@section usage Usage

To use this node, the tilting stage and laser have to be started. A convenient method for starting them is on the way, perhaps as a script.

@par Example

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/<a href="../../std_msgs/html/classstd__msgs_1_1LaserScan.html">LaserScan</a> : laser scan data

<hr>

@section parameters ROS parameters

 **/


#include "ros/node.h"
#include "tf/transform_listener.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/PointCloud.h"
 
#include <deque>


// Service
#include "point_cloud_assembler/BuildCloud.h"

// Laser projection
#include "laser_scan_utils/laser_scan.h"

#include "math.h"

using namespace std_msgs;

using namespace std ;

namespace point_cloud_assembler
{

class PointCloudAssembler : public ros::node
{
public:

  tf::TransformListener tf_;
  laser_scan::LaserProjection projector_;

  LaserScan scan_;

  unsigned int max_scans_ ;
  bool ignore_laser_skew_ ;

  deque<LaserScan> scan_hist_ ;            //!< Stores history of scans. We want them in time-ordered, which is (in most situations) the same as time-of-receipt-ordered
  unsigned int total_pts_ ;                //!< Stores the total number of range points in the entire stored history of scans. Useful for estimating points/scan
  
  
  PointCloudAssembler() : ros::node("point_cloud_assembler"), tf_(*this)
  {
    tf_.setExtrapolationLimit(ros::Duration(1.0)) ;

    advertise_service("build_cloud", &PointCloudAssembler::buildCloud, this, 0) ;      // Don't spawn threads so that we can avoid dealing with mutexing [for now]
    subscribe("scan", scan_, &PointCloudAssembler::scans_callback, 40) ;

    int tmp_max_scans;
    param("point_cloud_assembler/max_scans", tmp_max_scans, 400) ;
    if (tmp_max_scans < 0)
      tmp_max_scans = 400;

    max_scans_ = tmp_max_scans;

    param("~ignore_laser_skew", ignore_laser_skew_, true) ;

    total_pts_ = 0 ;                                                                   // We're always going to start with no points in our history
  }

  virtual ~PointCloudAssembler()
  { 
    unadvertise_service("build_cloud") ;
    unsubscribe("scan") ;
  }

  void scans_callback()
  {
    if (scan_hist_.size() == max_scans_)                                                // Is our deque full?
    {
      total_pts_ -= scan_hist_.front().get_ranges_size() ;                              // We're removing an elem, so this reduces our total point count
      scan_hist_.pop_front() ;                                                          // The front of the deque has the oldest elem, so we can get rid of it
    }
    
    scan_hist_.push_back(scan_) ;                                                       // Add the newest scan to the back of the deque
    total_pts_ += scan_.get_ranges_size() ;                                             // Add the new scan to the running total of points
    
    //printf("PointCloudAssembler:: Got Scan: TotalPoints=%u", total_pts_) ;
  }

  bool buildCloud(BuildCloud::request& req, BuildCloud::response& resp)
  {
    // Allocate space for the cloud
    resp.cloud.set_pts_size( total_pts_ ) ;                                             // There's no way to have a point cloud bigger than our entire history of scans.    
    resp.cloud.set_chan_size(1) ;
    resp.cloud.chan[0].name = "intensities" ;
    resp.cloud.chan[0].set_vals_size( total_pts_ ) ;
    
    unsigned int cloud_count = 0 ;                                                      // Store the number of points in the current cloud

    PointCloud projector_cloud ;                                                 // Stores the current scan after being projected into the laser frame
    PointCloud target_frame_cloud ;                                              // Stores the current scan in the target frame
    
    unsigned int i = 0 ;
    
    // Find the beginning of the request. Probably should be a search
    while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
            scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
    {
      i++ ;
    }

    printf(" Start i=%u\n", i) ;
    
    // Populate the cloud
    while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
            scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
    {
      const LaserScan& cur_scan = scan_hist_[i] ;

      try
      {
        if (ignore_laser_skew_)  // Do it the fast (approximate) way
	{
	  projector_.projectLaser(cur_scan, projector_cloud) ;
          tf_.transformPointCloud(req.target_frame_id, projector_cloud, target_frame_cloud) ;
	}
	else                     // Do it the slower (more accurate) way
	{
	  tf_.transformLaserScanToPointCloud(req.target_frame_id, target_frame_cloud, cur_scan) ;
        }

        for(unsigned int j = 0; j < target_frame_cloud.get_pts_size(); j++)               // Populate full_cloud from the cloud
        {
          resp.cloud.pts[cloud_count].x        = target_frame_cloud.pts[j].x ;
          resp.cloud.pts[cloud_count].y        = target_frame_cloud.pts[j].y ;
          resp.cloud.pts[cloud_count].z        = target_frame_cloud.pts[j].z ;
          resp.cloud.chan[0].vals[cloud_count] = target_frame_cloud.chan[0].vals[j] ;
          cloud_count++ ;
        }
      }
      catch(tf::TransformException& ex)
      {
        ROS_WARN("Transform Exception %s", ex.what()) ;
        
        return true ;
      }
              
      resp.cloud.header = target_frame_cloud.header ;                                   // Find a better place to do this/way to do this

      i++ ;                                                                             // Check the next scan in the scan history
    }
    
    printf(" End i=%u\n", i) ;
    
    resp.cloud.set_pts_size( cloud_count ) ;                                            // Resize the output accordingly
    resp.cloud.chan[0].set_vals_size( cloud_count ) ;

    return true ;
  }
};

}

using namespace point_cloud_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  PointCloudAssembler pc_assembler;
  pc_assembler.spin();
  ros::fini();
  return 0;
}
