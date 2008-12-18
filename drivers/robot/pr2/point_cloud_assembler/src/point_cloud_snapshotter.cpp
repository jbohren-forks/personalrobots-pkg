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

// Services
#include "point_cloud_assembler/BuildCloud.h"

// Messages
#include "std_msgs/PointCloud.h"
#include "pr2_mechanism_controllers/LaserScannerSignal.h"

using namespace std_msgs ;

/***
 * This uses the point_cloud_assembler's build_cloud service call to grab all the scans/clouds between two tilt-laser shutters
 * params
 *  * "~fixed_frame" (string) - This is the frame that the scanned data is assumed to be stationary in.  The
 *                                  output clouds are also published in this frame.
 */

namespace point_cloud_assembler
{

class PointCloudSnapshotter : public ros::node
{

public:

  pr2_mechanism_controllers::LaserScannerSignal prev_signal_;
  pr2_mechanism_controllers::LaserScannerSignal cur_signal_;

  bool first_time_ ;

  std::string fixed_frame_ ;

  PointCloudSnapshotter() : ros::node("point_cloud_snapshotter")
  {
    prev_signal_.header.stamp.fromNSec(0) ;

    advertise<PointCloud> ("full_cloud", 1) ;
    subscribe("laser_scanner_signal", cur_signal_, &PointCloudSnapshotter::scannerSignalCallback, 40) ;

    param("~fixed_frame", fixed_frame_, std::string("NO_FRAME_DEFINED")) ;

    first_time_ = true ;
  }

  ~PointCloudSnapshotter()
  {
    unsubscribe("laser_scanner_signal") ;
    unadvertise("full_cloud") ;
  }

  void scannerSignalCallback()
  {
    if (first_time_)
    {
      prev_signal_ = cur_signal_ ;
      first_time_ = false ;
    }
    else
    {
      BuildCloud::request req ;
      BuildCloud::response resp ;

      req.begin = prev_signal_.header.stamp ;
      req.end   = cur_signal_.header.stamp ;
      req.target_frame_id = fixed_frame_ ;

      //printf("PointCloudSnapshotter::Making Service Call...\n") ;
      ros::service::call("build_cloud", req, resp) ;
      //printf("PointCloudSnapshotter::Done with service call\n") ;

      publish("full_cloud", resp.cloud) ;
      ROS_INFO("Snapshotter::Published Cloud size=%u", resp.cloud.get_pts_size()) ;

      prev_signal_ = cur_signal_ ;
    }
  }
} ;

}

using namespace point_cloud_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  PointCloudSnapshotter snapshotter ;
  snapshotter.spin();
  ros::fini();
  return 0;
}
