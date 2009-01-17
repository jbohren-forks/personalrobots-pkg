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
#include "std_msgs/PointCloud.h"
#include "std_msgs/LaserScan.h"
#include "boost/thread.hpp"

using namespace ros;
using namespace std_msgs;


static const unsigned int last_seq = 4100 ;

int g_argc;
char** g_argv;

class TestAssembler : public testing::Test
{
public:

  Node* node_;

  PointCloud cloud_msg_ ;
  boost::mutex cloud_mutex_ ;
  PointCloud safe_cloud_ ;
  double cloud_counter_ ;

  LaserScan scan_msg_ ;
  boost::mutex scan_mutex_ ;
  LaserScan safe_scan_ ;
  double scan_counter_ ;


  void CloudCallback()
  {
    cloud_counter_++;
    cloud_mutex_.lock() ;
    safe_cloud_ = cloud_msg_ ;
    cloud_mutex_.unlock() ;
    ROS_INFO("Got Cloud with %u points", cloud_msg_.get_pts_size()) ;
  }

  void ScanCallback()
  {
    scan_counter_++ ;
    scan_mutex_.lock() ;
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
    fini();
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
  while (scan_counter_ == 0 && cloud_counter_ == 0)
    usleep(1e6) ;
  while( scan_msg_.header.seq < last_seq)
    usleep(1e4) ;

  usleep(1e6) ;

  ASSERT_EQ(cloud_counter_, (unsigned int)4) ;

  cloud_mutex_.lock() ;
  ASSERT_TRUE(safe_cloud_.get_pts_size() > 0) ;
  cloud_mutex_.unlock() ;

  SUCCEED();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
