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

/* Author: Wim Meeussen */

#include <string>
#include <gtest/gtest.h>
#include "ros/node.h"
#include "robot_msgs/PoseWithCovariance.h"
#include "std_msgs/PoseWithRatesStamped.h"


using namespace ros;
using namespace robot_msgs;
using namespace std_msgs;


static const double time_end     = 1230660828.0;
static const double ekf_duration = 43.138;
static const double EPS_trans    = 0.01;
static const double EPS_rot      = 0.001;


int g_argc;
char** g_argv;

class TestEKF : public testing::Test
{
public:
  PoseWithCovariance ekf_msg_, ekf_begin_;
  PoseWithRatesStamped odom_msg_;
  double ekf_counter_, odom_counter_;
  Time ekf_time_begin_, odom_time_begin_;
  node* node_;

  void OdomCallback()
  {
    // get initial time
    if (odom_counter_ == 0)
      odom_time_begin_ = odom_msg_.header.stamp;

    // count number of callbacks
    odom_counter_++;
  }


  void EKFCallback()
  {
    // get initial time
    if (ekf_counter_ == 0){
      ekf_time_begin_ = ekf_msg_.header.stamp;
      ekf_begin_ = ekf_msg_;
    }

    // count number of callbacks
    ekf_counter_++;
  }


protected:
  /// constructor
  TestEKF() 
  {
    ekf_counter_ = 0;
    odom_counter_ = 0;

    init(g_argc, g_argv); 
    node_ = new node("TestEKF");
  }


  /// Destructor
  ~TestEKF()
  {
    fini();
    delete node_;
  }
};




TEST_F(TestEKF, test)
{
  ROS_INFO("Subscribing to odom_estimation/odom_combined");
  ASSERT_TRUE(node_->subscribe("odom_estimation/odom_combined", ekf_msg_, &TestEKF::EKFCallback, 
			       (TestEKF*)this, 10));

  ROS_INFO("Subscribing to odom");
  ASSERT_TRUE(node_->subscribe("odom", odom_msg_, &TestEKF::OdomCallback, 
			       (TestEKF*)this, 10));

  // wait while bag is played back
  while (ekf_counter_ == 0 && odom_counter_ == 0)
    usleep(1e6);
  while( odom_msg_.header.stamp.toSec() < time_end)
    usleep(1e4);

  // unsubscribe
  ASSERT_TRUE(node_->unsubscribe("odom_estimation/odom_combined"));
  ASSERT_TRUE(node_->unsubscribe("odom"));

  // check if callback was called enough times
  ASSERT_TRUE(ekf_counter_ > 200);

  // check if time interval is correct
  ASSERT_TRUE(Duration(ekf_msg_.header.stamp - ekf_time_begin_).toSec() < ekf_duration * 1.25);
  ASSERT_TRUE(Duration(ekf_msg_.header.stamp - ekf_time_begin_).toSec() > ekf_duration * 0.85);

  // check if ekf time is same as odom time
  ASSERT_TRUE(Duration(ekf_time_begin_ - odom_time_begin_).toSec() < 1.0);
  ASSERT_TRUE(Duration(ekf_time_begin_ - odom_time_begin_).toSec() > -1.0);
  ASSERT_TRUE(Duration(ekf_msg_.header.stamp - odom_msg_.header.stamp).toSec() < 1.0);
  ASSERT_TRUE(Duration(ekf_msg_.header.stamp - odom_msg_.header.stamp).toSec() > -1.0);

  // check filter result
  ASSERT_TRUE(ekf_begin_.pose.position.x > 0.038043 - EPS_trans);
  ASSERT_TRUE(ekf_begin_.pose.position.x < 0.038043 + EPS_trans);
  ASSERT_TRUE(ekf_begin_.pose.position.y > -0.001618 - EPS_trans);
  ASSERT_TRUE(ekf_begin_.pose.position.y < -0.001618 + EPS_trans);
  ASSERT_TRUE(ekf_begin_.pose.position.z > 0.000000 - EPS_trans);
  ASSERT_TRUE(ekf_begin_.pose.position.z < 0.000000 + EPS_trans);
  ASSERT_TRUE(ekf_begin_.pose.orientation.x > 0.000000 - EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.x < 0.000000 + EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.y > 0.000000 - EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.y < 0.000000 + EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.z > 0.088400 - EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.z < 0.088400 + EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.w > 0.996085 - EPS_rot);
  ASSERT_TRUE(ekf_begin_.pose.orientation.w < 0.996085 + EPS_rot);

  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
