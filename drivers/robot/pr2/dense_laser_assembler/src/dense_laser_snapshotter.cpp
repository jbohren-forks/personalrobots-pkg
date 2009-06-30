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

#include "ros/ros.h"

// Services
#include "dense_laser_assembler/BuildLaserSnapshot.h"

// Messages
#include "calibration_msgs/DenseLaserSnapshot.h"
#include "pr2_mechanism_controllers/LaserScannerSignal.h"


using namespace dense_laser_assembler ;

class DenseLaserSnapshotter
{

public:

  DenseLaserSnapshotter()
  {
    prev_signal_.header.stamp.fromNSec(0) ;

    snapshot_pub_ = n_.advertise<calibration_msgs::DenseLaserSnapshot> ("dense_laser_snapshot", 1) ;
    signal_sub_ = n_.subscribe("laser_scanner_signal", 40, &DenseLaserSnapshotter::scannerSignalCallback, this) ;

    first_time_ = true ;
  }

  ~DenseLaserSnapshotter()
  {

  }

  void scannerSignalCallback(const pr2_mechanism_controllers::LaserScannerSignalConstPtr& cur_signal)
  {
    if (cur_signal->signal == 128 || cur_signal->signal == 129)       // These codes imply that this is the first signal during a given profile type
      first_time_ = true ;


    if (first_time_)
    {
      prev_signal_ = *cur_signal ;
      first_time_ = false ;
    }
    else
    {
      printf("About to make request\n") ;

      BuildLaserSnapshot::Request req ;
      BuildLaserSnapshot::Response resp ;

      req.start = prev_signal_.header.stamp ;
      req.end   = cur_signal->header.stamp ;



      if (!ros::service::call("dense_laser_assembler_srv/build_laser_snapshot", req, resp))
        ROS_ERROR("Failed to call service on dense laser assembler.");

      printf("Displaying Data") ;
      printf("header.stamp: %f\n", resp.snapshot.header.stamp.toSec()) ;
      printf("header.frame_id: %s", resp.snapshot.header.frame_id.c_str()) ;
      printf("ranges.size()=%u\n", resp.snapshot.ranges.size()) ;
      printf("intensities.size()=%u\n", resp.snapshot.intensities.size()) ;
      printf("joint_positions.size()=%u\n", resp.snapshot.joint_positions.size()) ;
      //printf("scan_start.size()=%u\n", resp.snapshot.scan_start.size()) ;

      /*printf("Displaying times:\n") ;
      for (unsigned int i=0; i<resp.snapshot.scan_start.size(); i++)
      {
        printf("   %u: %f\n", i, resp.snapshot.scan_start[i].toSec()) ;
      }*/

      printf("About to publish\n") ;
      snapshot_pub_.publish(resp.snapshot) ;
      printf("Done publishing\n") ;

      prev_signal_ = *cur_signal ;
    }
  }

private:
  ros::NodeHandle n_ ;
  ros::Subscriber signal_sub_ ;
  ros::Publisher snapshot_pub_ ;

  pr2_mechanism_controllers::LaserScannerSignal prev_signal_;

  bool first_time_ ;
} ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dense_laser_snapshotter") ;
  DenseLaserSnapshotter snapshotter ;
  ros::spin() ;

  return 0;
}
