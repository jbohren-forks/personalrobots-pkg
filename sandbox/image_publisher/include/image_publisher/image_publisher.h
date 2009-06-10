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

#include <ros/ros.h>
#include <image_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv_latest/CvBridge.h>

class ImagePublisher
{
public:
  ImagePublisher(const std::string& topic, const ros::NodeHandle& node_handle,
                 bool republishing = false);

  ~ImagePublisher();

  /** @todo: fix const-correctness once ROS updated */
  uint32_t getNumSubscribers() /*const*/;

  std::string getTopic() /*const*/;
  std::string getTopicThumbnail() /*const*/;
  std::string getTopicCompressed() /*const*/;

  void publish(const image_msgs::Image& message) /*const*/;
  void publish(const image_msgs::ImageConstPtr& message) /*const*/;

  void shutdown();

private:
  void publishThumbnailImage(image_msgs::Image& thumbnail) /*const*/;
  void publishCompressedImage(sensor_msgs::CompressedImage& compressed) /*const*/;
  
  // ROS stuff
  ros::NodeHandle node_handle_;
  ros::Publisher image_pub_;
  ros::Publisher thumbnail_pub_;
  ros::Publisher compressed_pub_;
  mutable image_msgs::CvBridge cv_bridge_;

  // Thumbnail parameters
  int thumbnail_size_;

  // Compression parameters
  int compression_params_[3];
  std::string extension_;
  std::string format_;

  bool republishing_;
};
