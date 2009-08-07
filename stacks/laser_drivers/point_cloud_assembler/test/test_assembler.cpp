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

/* Author: Vijay Pradeep */

#include <string>
#include <gtest/gtest.h>
#include "ros/node.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "boost/thread.hpp"

using namespace ros;
using namespace sensor_msgs;


static const unsigned int last_seq = 76500 ;

int g_argc;
char** g_argv;

class TestAssembler : public testing::Test
{
public:

  Node* node_;

  sensor_msgs::PointCloud cloud_msg_ ;
  boost::mutex cloud_mutex_ ;
  sensor_msgs::PointCloud safe_cloud_ ;
  int cloud_counter_ ;

  LaserScan scan_msg_ ;
  boost::mutex scan_mutex_ ;
  LaserScan safe_scan_ ;
  int scan_counter_ ;


  void CloudCallback()
  {
    cloud_mutex_.lock() ;
    cloud_counter_++ ;
    safe_cloud_ = cloud_msg_ ;
    cloud_mutex_.unlock() ;
    ROS_INFO("Got Cloud with %u points", cloud_msg_.get_points_size()) ;
  }

  void ScanCallback()
  {
    scan_mutex_.lock() ;
    scan_counter_++ ;
    safe_scan_ = scan_msg_ ;
    scan_mutex_.unlock() ;
  }

protected:
  /// constructor
  TestAssembler()
  {
    cloud_counter_ = 0 ;
    scan_counter_ = 0 ;

    init(g_argc, g_argv);
    node_ = new Node("TestAssembler");
  }

  /// Destructor
  ~TestAssembler()
  {
    delete node_;
  }
};


TEST_F(TestAssembler, test)
{
  ASSERT_TRUE(node_->subscribe("snapshot_cloud", cloud_msg_, &TestAssembler::CloudCallback,
                               (TestAssembler*)this, 10)) ;
  ASSERT_TRUE(node_->subscribe("tilt_scan", scan_msg_, &TestAssembler::ScanCallback,
                               (TestAssembler*)this, 10)) ;

  // wait while bag is played back

  int local_scan_counter_ ;

  scan_mutex_.lock() ;
  local_scan_counter_ = scan_counter_ ;
  scan_mutex_.unlock() ;

  while (scan_counter_ == 0)
    usleep(1e6) ;

  bool waiting = true ;
  while( waiting )
  {
    usleep(1e4) ;

    scan_mutex_.lock() ;
    waiting = (scan_msg_.header.seq < last_seq) ;
    scan_mutex_.unlock() ;

  }

  usleep(1e6) ;

  ASSERT_EQ(cloud_counter_, 5) ;

  unsigned int cloud_size ;
  cloud_mutex_.lock() ;
  cloud_size = safe_cloud_.get_points_size() ;
  cloud_mutex_.unlock() ;

  ASSERT_TRUE(cloud_size > 0) ;

  SUCCEED();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
