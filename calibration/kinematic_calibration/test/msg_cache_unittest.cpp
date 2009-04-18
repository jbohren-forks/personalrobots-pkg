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

//! \author Vijay Pradeep

#include <gtest/gtest.h>
#include "../deprecated/msg_cache.h"
#include <robot_msgs/PointStamped.h>
#include <ros/time.h>

using namespace kinematic_calibration ;

static const double epsilon = 1e-6 ;

TEST(KINEMATIC_CALIBRATION_MSG_CACHE, test1)
{
  MsgCache<robot_msgs::PointStamped> cache(10) ;
  robot_msgs::PointStamped sample ;

  sample.header.stamp =ros::Time(1.0) ;
  sample.point.x = 10 ;
  cache.insertData(sample) ;

  sample.header.stamp =ros::Time(2.0) ;
  sample.point.x = 11 ;
  cache.insertData(sample) ;

  sample.header.stamp =ros::Time(3.0) ;
  sample.point.x = 21 ;
  cache.insertData(sample) ;

  robot_msgs::PointStamped data_out ;
  cache.findClosestAfter(ros::Time(1.5), data_out) ;
  EXPECT_EQ(data_out.point.x, 11) ;

  cache.findClosestBefore(ros::Time(1.5), data_out) ;
  EXPECT_EQ(data_out.point.x, 10) ;

  //int int_out ;
  //cache.interpolate<int>(ros::Time(1.5), int_out) ;
  //EXPECT_EQ(int_out, 4) ;
}

TEST(KINEMATIC_CALIBRATION_MSG_CACHE, test2)
{
  MsgCacheListener<robot_msgs::PointStamped> cache_listener(10) ;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
