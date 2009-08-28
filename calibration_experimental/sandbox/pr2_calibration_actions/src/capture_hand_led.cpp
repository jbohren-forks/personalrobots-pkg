/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include "pr2_calibration_actions/capture_hand_led.h"


using namespace pr2_calibration_actions;
using namespace message_filters;
using namespace std;
using namespace calibration_message_filters;

CaptureHandLED::CaptureHandLED(ros::NodeHandle nh) :
  nh_(nh),
  sync_(led_sub_, led_image_sub_, 1)
{
  // Configure the LED inputs
  led_sub_.subscribe(nh_, "led", 1);
  led_image_sub_.subscribe(nh_, "led_image", 1);
  sync_.registerCallback( boost::bind(&CaptureHandLED::ledCallback, this, _1, _2) );
  led_cache_.setCacheSize(100);     //! \todo Don't hardcode cache size

  // Configure the JointStates input
  joint_states_sub_ = nh_.subscribe("joint_states", 1, &CaptureHandLED::jointStatesCallback, this);
  joint_states_cache_.setCacheSize(100);

  //! \todo This joint names should be params
  vector<string> joint_names;
  joint_names.push_back("r_shoulder_pan_joint");
  joint_names.push_back("r_shoulder_lift_joint");
  joint_names.push_back("r_upper_arm_roll_joint");
  joint_states_deflater_.setDeflationJointNames(joint_names);

  // Configure the stationary checker
  ChannelTolerance led_tol;
  led_tol.tol_.push_back(1.01);
  led_tol.tol_.push_back(1.01);

  ChannelTolerance joint_states_tol;
  joint_states_tol.tol_.push_back(0.01);
  joint_states_tol.tol_.push_back(0.01);
  joint_states_tol.tol_.push_back(0.01);

  vector<Cache<Deflated>*> cache_vec(2);
  cache_vec[0] = static_cast<Cache<Deflated>*>((void*) &led_cache_);
  cache_vec[1] = static_cast<Cache<Deflated>*>((void*) &joint_states_cache_);

  vector<ChannelTolerance> tolerance_vec(2);
  tolerance_vec[0] = led_tol;
  tolerance_vec[1] = joint_states_tol;

  stationary_checker_.init(cache_vec, tolerance_vec);
}


void CaptureHandLED::ledCallback(const calibration_msgs::ImagePointStampedConstPtr& led,
                                 const sensor_msgs::ImageConstPtr& led_image)
{
  DeflatedImagePtr deflated_ptr(new DeflatedImage);
  deflated_ptr->header = led->header;
  deflated_ptr->deflated_.resize(2);
  deflated_ptr->deflated_[0] = led->image_point.x;
  deflated_ptr->deflated_[1] = led->image_point.y;
  deflated_ptr->msg = led_image;
  ROS_DEBUG("Adding to LED Cache");
  led_cache_.add(deflated_ptr);

  checkStationary();
}

void CaptureHandLED::jointStatesCallback(const pr2_mechanism_msgs::JointStatesConstPtr& joint_states)
{
  DeflatedJointStatesPtr deflated_ptr(new DeflatedJointStates);
  joint_states_deflater_.deflate(joint_states, *deflated_ptr);
  ROS_DEBUG("Adding to JointStates Cache");
  joint_states_cache_.add(deflated_ptr);

  //checkStationary();
}

void CaptureHandLED::checkStationary()
{
  ros::Time stationary_time;
  ros::Duration interval_duration(1,0);

  stationary_time = stationary_checker_.isLatestStationary(interval_duration);

  if (stationary_time != ros::Time(0,0))
    ROS_INFO("Found a stationary elem");
  else
    ROS_INFO("Not stationary");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capture_hand_led_node");
  ros::NodeHandle nh;
  CaptureHandLED hand_led(nh);

  ros::spin();
}


