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

#include "led_detection/led_detector_node.h"

#include "robot_msgs/PoseStamped.h"
#include "robot_msgs/PointStamped.h"
#include "image_msgs/ImagePointStamped.h"

using namespace led_detection ;

LedDetectionNode::LedDetectionNode(ros::Node* node) : node_(node),
  sync_(node_, this, &LedDetectionNode::msgCallback,
        ros::Duration().fromSec(0.05),
        &LedDetectionNode::msgTimeout ),
  tf_(*node_)
{
  bool exists ;

  node_->param("~use_led_pose", use_led_pose_, false) ;
  if (use_led_pose_)
    ROS_INFO("Using TF & LED Pose in detection") ;
  else
    ROS_INFO("Ignoring TF & LED Pose in detection") ;

  if (use_led_pose_)
  {
    exists = node_->getParam("~led_frame", led_frame_) ;
    if (exists)
    {
      ROS_INFO("LED is at origin of frame '%s'", led_frame_.c_str()) ;
    }
    else
    {
      ROS_ERROR("Couldn't find param ~led_frame. Either define '~led_frame', or set '~use_led_pose' to FALSE") ;
      return ;
    }
  }

  node_->param("~publish_debug", publish_debug_, true) ;

  sync_.subscribe("image",    image_msg_,    1) ;
  sync_.subscribe("cam_info", cam_info_msg_, 1) ;

  node_->advertise<image_msgs::ImagePointStamped>("~led", 10) ; //!todo Magic #
  node_->advertise<image_msgs::Image>("~debug_image", 1) ;

  sync_.ready() ;

}

void LedDetectionNode::msgCallback(ros::Time t)
{
  printf("%f - In callback\n", t.toSec()) ;
  // Define the LED somewhere in the world
  robot_msgs::PoseStamped led_world ;
  led_world.header.frame_id    = led_frame_ ;
  led_world.header.stamp       = t ;
  led_world.pose.position.x    = 0.0 ;
  led_world.pose.position.y    = 0.0 ;
  led_world.pose.position.z    = 0.0 ;
  led_world.pose.orientation.x = 0.0 ;
  led_world.pose.orientation.y = 0.0 ;
  led_world.pose.orientation.z = 0.0 ;
  led_world.pose.orientation.w = 1.0 ;

  image_msgs::ImagePointStamped led_pix ;
  bool found ;
  if (use_led_pose_)
  {
    robot_msgs::PoseStamped led_cam ;
    try
    {
      std::string target_frame = cam_info_msg_.header.frame_id ;
      bool transformable ;
      // Hack, since we don't have a good way to use TF notifiers and topic_synchronizers in series
      transformable = tf_.canTransform(target_frame, led_world.header.frame_id, led_world.header.stamp, ros::Duration().fromSec(0.05)) ;
      if (!transformable)
	ROS_WARN("canTransform FAILED!") ;
      // Determine the LED's pose in the camera's frame
      tf_.transformPose(cam_info_msg_.header.frame_id, led_world, led_cam) ;
    }
    catch(tf::TransformException& ex)
    {
      ROS_WARN("Transform Exception %s", ex.what()) ;
      return ;
    }
    // Extract the LED from the image (ie. Do actual math)
    found = led_detector.findLed(image_msg_, cam_info_msg_, &led_cam.pose, led_pix.image_point, debug_image_msg_) ;
  }
  else
    found = led_detector.findLed(image_msg_, cam_info_msg_, NULL,          led_pix.image_point, debug_image_msg_) ;

  // Publish the LED's pixel location, if it was found
  if (found)
  {
    led_pix.header = cam_info_msg_.header ;
    node_->publish("~led", led_pix) ;
  }

  if (publish_debug_)
  {
    debug_image_msg_.header = image_msg_.header ;
    node_->publish("~debug_image", debug_image_msg_) ;
  }
}

void LedDetectionNode::msgTimeout(ros::Time t)
{
  printf("%f - Timeout\n", t.toSec()) ;
}

using namespace led_detection ;

int main(int argc, char** argv)
{
  ros::init(argc, argv) ;

  ros::Node node("led_detector") ;

  LedDetectionNode led_detector(&node) ;

  node.spin() ;

  return 0 ;
}

