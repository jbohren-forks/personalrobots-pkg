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
#include "std_msgs/PointCloudFloat32.h"
#include "pr2_mechanism_controllers/LaserScannerSignal.h"

using namespace std_msgs ;

/***
 * This uses the point_cloud_assembler's build_cloud service call to grab all the laser scans between two tilt-laser shutters
 */

namespace point_cloud_assembler
{

class PointCloudSnapshotter : public ros::node
{
  
public:

  pr2_mechanism_controllers::LaserScannerSignal prev_signal_;
  pr2_mechanism_controllers::LaserScannerSignal cur_signal_;
    
  PointCloudSnapshotter() : ros::node("point_cloud_snapshotter")
  {
    prev_signal_.header.stamp.fromNSec(0) ;
    
    advertise<PointCloudFloat32> ("full_cloud", 1) ;
    subscribe("laser_scanner_signal", cur_signal_, &PointCloudSnapshotter::scannerSignalCallback, 40) ;
  }
  
  ~PointCloudSnapshotter()
  {
    unadvertise("full_cloud") ;
  }
  
  void scannerSignalCallback()
  {
    BuildCloud::request req ;
    BuildCloud::response resp ;
    
    req.begin = prev_signal_.header.stamp ;
    req.end   = cur_signal_.header.stamp ;
    req.target_frame_id = "base" ;
    
    printf("Making Service Call...\n") ;
    ros::service::call("build_cloud", req, resp) ;
    printf("Done with service call\n") ;
    
    publish("full_cloud", resp.cloud) ;
    printf("Published Cloud size=%u\n", resp.cloud.get_pts_size()) ;
    
    prev_signal_ = cur_signal_ ;
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
