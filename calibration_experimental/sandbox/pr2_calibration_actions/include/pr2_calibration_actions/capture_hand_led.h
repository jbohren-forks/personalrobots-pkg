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

#ifndef PR2_CALIBRATION_ACTIONS_CAPTURE_HAND_LED_H_
#define PR2_CALIBRATION_ACTIONS_CAPTURE_HAND_LED_H_

#include "ros/ros.h"
#include "message_filters/cache.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "calibration_msgs/ImagePointStamped.h"
#include "sensor_msgs/Image.h"
#include "calibration_message_filters/deflated.h"
#include "calibration_message_filters/joint_states_deflater.h"
#include "calibration_message_filters/stationary_checker.h"

// msgs
#include "pr2_mechanism_msgs/JointStates.h"

namespace pr2_calibration_actions
{

class CaptureHandLED
{
public:
  CaptureHandLED(ros::NodeHandle nh);



private:

  ros::NodeHandle nh_;

  // ***** LED Image/Pixel Stuff *****
  typedef calibration_message_filters::DeflatedMsg<sensor_msgs::Image> DeflatedImage;
  typedef boost::shared_ptr<DeflatedImage> DeflatedImagePtr;
  typedef boost::shared_ptr<const DeflatedImage> DeflatedImageConstPtr;

  message_filters::Subscriber<calibration_msgs::ImagePointStamped> led_sub_;
  message_filters::Subscriber<sensor_msgs::Image> led_image_sub_;
  message_filters::TimeSynchronizer<calibration_msgs::ImagePointStamped, sensor_msgs::Image> sync_;
  message_filters::Cache<DeflatedImage> led_cache_;

  // ***** JointStates Stuff *****
  typedef calibration_message_filters::DeflatedMsg<pr2_mechanism_msgs::JointStates> DeflatedJointStates;
  typedef boost::shared_ptr<DeflatedJointStates> DeflatedJointStatesPtr;
  typedef boost::shared_ptr<const DeflatedJointStates> DeflatedJointStatesConstPtr;

  ros::Subscriber joint_states_sub_;
  calibration_message_filters::JointStatesDeflater joint_states_deflater_;
  message_filters::Cache<DeflatedJointStates> joint_states_cache_;

  // ***** Stationary Check Stuff *****
  calibration_message_filters::StationaryChecker stationary_checker_;

  // ***** Implementation *****
  void ledCallback(const calibration_msgs::ImagePointStampedConstPtr& led,
                   const sensor_msgs::ImageConstPtr& led_image);
  void jointStatesCallback(const pr2_mechanism_msgs::JointStatesConstPtr& joint_states);
  void checkStationary();
};


}


#endif
