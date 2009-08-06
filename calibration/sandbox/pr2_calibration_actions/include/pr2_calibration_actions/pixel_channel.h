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

#ifndef PR2_CALIBRATION_ACTIONS_PIXEL_CHANNEL_H_
#define PR2_CALIBRATION_ACTIONS_PIXEL_CHANNEL_H_

#include <boost/shared_ptr.hpp>
#include <calibration_message_filters/deflated.h>
#include <calibration_message_filters/channel_tolerance.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "sorted_deque.h"


// Messages
#include "pr2_calibration_actions/PixelChannelConfig.h"
#include "sensor_msgs/Image.h"
#include "calibration_msgs/ImagePointStamped.h"

namespace pr2_calibration_actions
{

class PixelChannel
{
public:
  typedef calibration_message_filters::DeflatedMsg<sensor_msgs::Image> DeflatedImage;
  typedef boost::function<void (const DeflatedImage&)> StationaryCallback;

  PixelChannel(const PixelChannelConfig& config, StationaryCallback cb = NULL);

  void registerStationaryCallback(StationaryCallback cb);

  SortedDeque<DeflatedImage> stationary_list_;
private:
  typedef boost::shared_ptr<DeflatedImage> DeflatedImagePtr;
  typedef boost::shared_ptr<const DeflatedImage> DeflatedImageConstPtr;

  ros::NodeHandle nh_;
  message_filters::Subscriber<calibration_msgs::ImagePointStamped> pixel_sub_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::TimeSynchronizer<calibration_msgs::ImagePointStamped, sensor_msgs::Image> sync_;

  SortedDeque<DeflatedImage> cache_;
  SortedDeque<DeflatedImage> need_to_process_;
  StationaryCallback stationary_callback_;

  ChannelTolerance tol_;
  ros::Duration padding_;
  unsigned int min_samples_;
  std::string channel_name_;

  void syncCallback(const calibration_msgs::ImagePointStampedConstPtr& led,
                    const sensor_msgs::ImageConstPtr& led_image);
  void processElems();

  void processStationary(const DeflatedImage& deflated);
};


}

#endif

