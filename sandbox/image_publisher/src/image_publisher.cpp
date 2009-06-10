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

#include <image_publisher/image_publisher.h>

#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

ImagePublisher::ImagePublisher(const std::string& topic, const ros::NodeHandle& node_handle,
                               bool republishing)
  : node_handle_(node_handle),
    republishing_(republishing)
{
  if (!republishing_)
    image_pub_ = node_handle_.advertise<image_msgs::Image>(topic, 1);
  thumbnail_pub_ = node_handle_.advertise<image_msgs::Image>(topic + "_thumbnail", 1);
  compressed_pub_ = node_handle_.advertise<sensor_msgs::CompressedImage>(topic + "_compressed", 1);
  
  node_handle_.param("thumbnail_size", thumbnail_size_, 128);
  node_handle_.param("compression_type", format_, std::string("jpeg"));
  if (format_ == "jpeg") {
    compression_params_[0] = CV_IMWRITE_JPEG_QUALITY;
    node_handle_.param("compression_level", compression_params_[1], 80); // 80% quality
  }
  else if (format_ == "png") {
    compression_params_[0] = CV_IMWRITE_PNG_COMPRESSION;
    node_handle_.param("compression_level", compression_params_[1], 9); // max compression
  }
  else {
    ROS_FATAL("Unknown compression type '%s', valid options are 'jpeg' and 'png'",
              format_.c_str());
    node_handle_.shutdown();
    return;
  }
  compression_params_[2] = 0;
  extension_ = '.' + format_;
}

ImagePublisher::~ImagePublisher()
{
}

uint32_t ImagePublisher::getNumSubscribers() //const
{
  return image_pub_.getNumSubscribers() + thumbnail_pub_.getNumSubscribers()
    + compressed_pub_.getNumSubscribers();
}

std::string ImagePublisher::getTopic() //const
{
  return image_pub_.getTopic();
}

std::string ImagePublisher::getTopicThumbnail() //const
{
  return thumbnail_pub_.getTopic();
}

std::string ImagePublisher::getTopicCompressed() //const
{
  return compressed_pub_.getTopic();
}

void ImagePublisher::publish(const image_msgs::Image& message) //const
{
  if (!republishing_)
    image_pub_.publish(message);
  
  uint32_t thumb_subscribers = thumbnail_pub_.getNumSubscribers();
  uint32_t compressed_subscribers = compressed_pub_.getNumSubscribers();
  if (thumb_subscribers == 0 && compressed_subscribers == 0)
    return;

  // Convert to IPL image
  /** @todo: support depths other than 8-bit */
  if (message.depth != "uint8" && message.depth != "int8") {
    ROS_ERROR("Unsupported image depth: %s", message.depth.c_str());
    return;
  }

  int channels = message.uint8_data.layout.dim[2].size;
  std::string encoding;
  if (channels == 1)
    encoding = "mono";
  else if (channels == 3)
    encoding = "rgb"; /** @todo: avoid BGR->RGB conversion? */
  else {
    /** @todo: RGBA, BGRA. Can we do anything with other encodings? */
    ROS_ERROR("Unsupported number of image channels: %d", channels);
    return;
  }
  
  if (!cv_bridge_.fromImage(message, encoding)) {
    ROS_ERROR("Could not convert from %s to %s", message.encoding.c_str(), encoding.c_str());
    return;
  }
  
  if (thumb_subscribers > 0) {
    image_msgs::Image thumbnail;
    thumbnail.header = message.header;
    thumbnail.label = message.label;
    publishThumbnailImage(thumbnail);
  }

  if (compressed_subscribers > 0) {
    sensor_msgs::CompressedImage compressed;
    compressed.header = message.header;
    compressed.label = message.label;
    compressed.encoding = encoding;
    publishCompressedImage(compressed);
  }
}

void ImagePublisher::publish(const image_msgs::ImageConstPtr& message) //const
{
  publish(*message);
}

void ImagePublisher::shutdown()
{
  image_pub_.shutdown();
  thumbnail_pub_.shutdown();
  compressed_pub_.shutdown();
}

void ImagePublisher::publishThumbnailImage(image_msgs::Image& thumbnail) //const
{
  const IplImage* image = cv_bridge_.toIpl();
  int width = image->width;
  int height = image->height;
  float aspect = std::sqrt((float)width / height);
  int scaled_width  = thumbnail_size_ * aspect + 0.5;
  int scaled_height = thumbnail_size_ / aspect + 0.5;

  cv::WImageBuffer_b buffer(scaled_width, scaled_height, image->nChannels);
  cvResize(image, buffer.Ipl());

  if (image_msgs::CvBridge::fromIpltoRosImage(buffer.Ipl(), thumbnail)) {
    thumbnail_pub_.publish(thumbnail);
  } else {
    ROS_ERROR("Unable to create thumbnail image message");
  }
}

void ImagePublisher::publishCompressedImage(sensor_msgs::CompressedImage& compressed) //const
{
  const IplImage* image = cv_bridge_.toIpl();
  CvMat* buf = cvEncodeImage(extension_.c_str(), image, compression_params_);

  compressed.format = format_;
  compressed.uint8_data.layout.dim.resize(2);
  compressed.uint8_data.layout.dim[0].label = "height";
  compressed.uint8_data.layout.dim[0].size = image->height;
  compressed.uint8_data.layout.dim[0].stride = 0;

  compressed.uint8_data.layout.dim[1].label = "width";
  compressed.uint8_data.layout.dim[1].size = image->width;
  compressed.uint8_data.layout.dim[1].stride = 0;

  compressed.uint8_data.data.resize(buf->width);
  memcpy(&compressed.uint8_data.data[0], buf->data.ptr, buf->width);

  compressed_pub_.publish(compressed);
}
