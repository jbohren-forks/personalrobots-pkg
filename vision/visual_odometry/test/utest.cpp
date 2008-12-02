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

/*
 * utest.cpp
 *
 *  Created on: Dec 2, 2008
 *      Author: jdchen
 */


#include <gtest/gtest.h>

#include "CvTest3DPoseEstimate.h"

// this shall go away, replace with rosconsole stuff (?)
#include <iostream>

/// Test sparse bundle adjustment
bool testSparseBundleAdjustment() {
  CvTest3DPoseEstimate testVisOdom;
  char* ros_pkg_path = getenv("ROS_PACKAGE_PATH");
  if (ros_pkg_path==NULL) {
    printf("Error: environment variable ROS_PACKAGE_PATH is not set");
    return false;
  }

  testVisOdom.input_data_path_ = string(ros_pkg_path).append("/vision/visual_odometry/test/Data/");
  testVisOdom.output_data_path_ = string(ros_pkg_path).append("/vision/visual_odometry/test/Output/");
  testVisOdom.verbose_ = false;

  testVisOdom.mTestType = CvTest3DPoseEstimate::BundleAdj;

  return testVisOdom.test();
}


TEST(VisualOdometry, SparseBundleAdjustment) {
  EXPECT_TRUE(testSparseBundleAdjustment());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

