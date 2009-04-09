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

#ifndef LED_DETECTION_LED_DETECTION_NODE_H_
#define LED_DETECTION_LED_DETECTION_NODE_H_


#include <string>

#include "ros/node.h"

#include "tf/transform_listener.h"


#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "robot_msgs/PointStamped.h"
#include "topic_synchronizer/topic_synchronizer.h"


#include "boost/thread.hpp"

#include "led_detection/led_detector.h"

namespace led_detection
{

/**
 *
 */
class LedDetectionNode
{
public:
  LedDetectionNode(ros::Node* node) ;

private:
  void msgTimeout(ros::Time time) ;
  void msgCallback(ros::Time time) ;

  ros::Node* node_ ;
  TopicSynchronizer<led_detection::LedDetectionNode> sync_ ;
  tf::TransformListener tf_ ;
  LedDetector led_detector ;

  // Config variables
  std::string led_frame_ ;      //!< Frame defining LED origin (assumes LED points z-out at frame origin)
  bool publish_debug_ ;         //!< Flag defining if we should publish a debugging image
  bool use_led_pose_ ;          //!< True: Use LED pose via TF to help find LED.  False: Ignore prior information

  image_msgs::Image image_msg_ ;
  image_msgs::CamInfo cam_info_msg_ ;
  image_msgs::Image debug_image_msg_ ;
};



}



#endif // LED_DETECTION_LED_DETECTION_NODE_H_
