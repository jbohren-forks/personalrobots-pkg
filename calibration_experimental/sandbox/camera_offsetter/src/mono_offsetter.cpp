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

#include "camera_offsetter/generic_offsetter.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

namespace camera_offsetter
{

class MonoOffsetter : public GenericOffsetter
{
public:
  MonoOffsetter()
  {
    if (!nh_.getParam("~cam_name", cam_name_))
      ROS_FATAL("Couldn't find param [~cam_name]");

    image_sub_ = nh_.subscribe(cam_name_ + "/image_rect", 1, &MonoOffsetter::imageCb, this);
    info_sub_  = nh_.subscribe(cam_name_ + "/cam_info",   1, &MonoOffsetter::infoCb,  this);

    image_pub_ = nh_.advertise<sensor_msgs::Image>(cam_name_ + "_offset/image_rect", 1);
    info_pub_  = nh_.advertise<sensor_msgs::CameraInfo>(cam_name_ + "_offset/cam_info", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    sensor_msgs::Image next_image = *msg;
    next_image.header.frame_id = msg->header.frame_id + frame_suffix_;
    image_pub_.publish(next_image);

    publishTransform(msg->header.stamp, next_image.header.frame_id, msg->header.frame_id);
  }

  void infoCb(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    sensor_msgs::CameraInfo next_info = *msg;
    next_info.header.frame_id = msg->header.frame_id + frame_suffix_;
    info_pub_.publish(next_info);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber image_sub_;
  ros::Subscriber info_sub_;

  ros::Publisher image_pub_;
  ros::Publisher info_pub_;


  std::string cam_name_;
} ;

}

using namespace camera_offsetter;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_offsetter");

  MonoOffsetter offsetter;

  ros::spin();
}
