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

#include "pr2_calibration_actions/pixel_channel.h"
#include "calibration_message_filters/stationary_utils.h"

using namespace calibration_message_filters;
using namespace pr2_calibration_actions;
using namespace std;

static const unsigned int SYNC_QUEUE_SIZE = 5;

PixelChannel::PixelChannel(const PixelChannelConfig& config, StationaryCallback cb) :
  sync_(SYNC_QUEUE_SIZE)
{
  registerStationaryCallback(cb);

  cache_.setMaxSize(config.cache_size);
  tol_.tol_.resize(2);
  tol_.tol_[0] = config.tolerance;
  tol_.tol_[1] = config.tolerance;

  if (config.padding < ros::Duration(0,0))
  {
    ROS_WARN("Received a negative padding. Negating it");
    padding_ = -config.padding;
  }
  else
    padding_ = config.padding;

  min_samples_ = config.min_samples;
  if (min_samples_ == 0)
  {
    ROS_WARN("min_samples cannot be 0. Setting to 1");
    min_samples_ = 1;
  }

  channel_name_ = config.channel_name;

  pixel_sub_.subscribe(nh_, config.pixel_topic, 1);
  image_sub_.subscribe(nh_, config.image_topic, 1);
  sync_.registerCallback( boost::bind(&PixelChannel::syncCallback, this, _1, _2) );
  sync_.connectInput(pixel_sub_, image_sub_);
}

void PixelChannel::registerStationaryCallback(StationaryCallback cb)
{
  stationary_callback_ = cb;
}

void PixelChannel::syncCallback(const calibration_msgs::ImagePointStampedConstPtr& pixel,
                                const sensor_msgs::ImageConstPtr& image)
{
  DeflatedImage deflated;
  deflated.header = pixel->header;
  deflated.deflated_.resize(2);
  deflated.deflated_[0] = pixel->image_point.x;
  deflated.deflated_[1] = pixel->image_point.y;
  deflated.msg = image;
  cache_.add(deflated);
  need_to_process_.add(deflated);
  processElems();
}

void PixelChannel::processElems()
{
  if (cache_.size() == 0)
  {
    ROS_DEBUG("Cache is empty. Going to skip processElems()");
    return;
  }

  // Remove the old stuff in the cache
  if (need_to_process_.front().header.stamp < ros::Time().fromSec(padding_.toSec()))
  {
    ROS_WARN("oldest time is %.3f. Padding is %.3f. Funny stuff might happen",
             need_to_process_.front().header.stamp.toSec(), padding_.toSec());
  }
  cache_.removeAllBeforeTime( need_to_process_.front().header.stamp - padding_*2);

  vector<DeflatedImage> interval_elems;
  while (need_to_process_.size() > 0)
  {
    const DeflatedImage& first = need_to_process_.front();

    ros::Time start = first.header.stamp - padding_;
    ros::Time end   = first.header.stamp + padding_;
    if (start < cache_.front().header.stamp)
    {   
      ROS_DEBUG("neg_padded elem is older than oldest cache elem by [%.3fs]. We can delete it",
                (cache_.front().header.stamp - start).toSec() );
      ROS_DEBUG("First Time:  %u, %u", first.header.stamp.sec, first.header.stamp.nsec);
      ROS_DEBUG("cache.front: %u, %u", cache_.front().header.stamp.sec, cache_.front().header.stamp.nsec);
      need_to_process_.pop_front();
    }
    else
    {
      ROS_DEBUG("Not immeadiately deleting");
      interval_elems = cache_.getSurroundingInterval(start, end);
      if (cache_.back().header.stamp < end)
      {
        ROS_DEBUG("end:         %u, %u", end.sec, end.nsec);
        ROS_DEBUG("cache.back:  %u, %u", cache_.back().header.stamp.sec, cache_.back().header.stamp.nsec);
        break;
      }
      else
      {
        if(interval_elems.size() >= min_samples_)
        {
          vector<double> range;
          StationaryUtils::computeChannelRange<DeflatedImage>(interval_elems, range);
          if (StationaryUtils::isChannelStationary(range, tol_))
	  {
	    ROS_DEBUG("Is stationary. About to processStationary");
            processStationary(first);
	  }
        }
	ROS_DEBUG("Done with this elem. About to delete it");
        need_to_process_.pop_front();
      }
    }
  }
}

void PixelChannel::processStationary(const DeflatedImage& deflated)
{
  //stationary_list_.add(deflated);

  ROS_INFO("%s: Stationary Image: time: %u, %u", channel_name_.c_str(), deflated.header.stamp.sec, deflated.header.stamp.nsec);

  if (stationary_callback_)
    stationary_callback_(deflated);
}






