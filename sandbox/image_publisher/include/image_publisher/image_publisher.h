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

/**
 * \brief Publishes images efficiently across a bandwidth-limited network
 * connection.
 *
 * ImagePublisher will, on demand (i.e. if there are subscribers), publish
 * low-memory versions of the image message on separate topics. An ImagePublisher
 * constructed with base topic "camera/image" will advertise:
 *
 * - camera/image : The original image
 * - camera/image_thumbnail : The image scaled down to thumbnail size
 * - camera/image_compressed : A compressed (JPEG or PNG) version of the image
 */
class ImagePublisher
{
public:
  /*!
   * \brief Constructor
   *
   * Construct an ImagePublisher publishing the raw image on the base topic
   * name (if !republishing) and low-memory images on derived topic names.
   * \param topic           Base topic name for publishing raw image
   * \param republishing    If true, do not publish the raw image
   */
  ImagePublisher(const std::string& topic, const ros::NodeHandle& node_handle,
                 bool republishing = false);

  ~ImagePublisher();

  /*!
   * \brief Returns the number of subscribers that are currently connected to
   * this ImagePublisher.
   *
   * Returns the total number of subscribers to the raw, thumbnail and compressed topics.
   */
  uint32_t getNumSubscribers() /*const*/;

  /*!
   * \brief Returns the topic that this ImagePublisher will publish the raw
   * image on.
   */
  std::string getTopic() /*const*/;

  /*!
   * \brief Returns the topic that this ImagePublisher will publish the thumbnail
   * image on.
   */
  std::string getTopicThumbnail() /*const*/;

  /*!
   * \brief Returns the topic that this ImagePublisher will publish the compressed
   * image on.
   */
  std::string getTopicCompressed() /*const*/;

  /*!
   * \brief Publish an image on the topics associated with this ImagePublisher.
   */
  void publish(const image_msgs::Image& message) /*const*/;

  /*!
   * \brief Publish an image on the topics associated with this ImagePublisher.
   */
  void publish(const image_msgs::ImageConstPtr& message) /*const*/;

  /*!
   * \brief Shutdown the advertisements associated with this ImagePublisher.
   */
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

/** @todo: Handle dynamic updates to parameters */
/** @todo: fix const-correctness once ROS updated */
