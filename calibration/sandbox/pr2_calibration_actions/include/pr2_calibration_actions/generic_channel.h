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

#ifndef PR2_CALIBRATION_ACTIONS_GENERIC_CHANNEL_H_
#define PR2_CALIBRATION_ACTIONS_GENERIC_CHANNEL_H_


#include <boost/function.hpp>
#include <pr2_calibration_actions/ChannelFeedback.h>
#include <calibration_message_filters/stationary_utils.h>

namespace pr2_calibration_actions
{

class GenericChannel
{
public:
  typedef boost::function<void (const ChannelFeedback&)> FeedbackCallback;

  GenericChannel() { }
  ~GenericChannel() { }

  void registerFeedbackCb(FeedbackCallback cb)
  {
    feedback_cb_ = cb;
  }

protected:
  template<class DeflatedT>
  void processElems(SortedDeque<DeflatedT>& cache, SortedDeque<DeflatedT>& need_to_process,
                    boost::function<void(const DeflatedT&)> stationary_cb)
  {
    if (cache.size() == 0)
    {
      ROS_DEBUG("Cache is empty. Going to skip processElems()");
      return;
    }

    // Remove the old stuff in the cache
    if (need_to_process.front().header.stamp < ros::Time().fromSec(padding_.toSec()))
    {
      ROS_WARN("oldest time is %.3f. Padding is %.3f. Funny stuff might happen",
               need_to_process.front().header.stamp.toSec(), padding_.toSec());
    }
    cache.removeAllBeforeTime( need_to_process.front().header.stamp - padding_*2);


    bool keep_processing = true;
    while (keep_processing && need_to_process.size() > 0)
    {
      ROS_DEBUG("About to start processing elem on the front of need_to_process...");

      const DeflatedT& first = need_to_process.front();

      // Determine in what interval we should be searching the cache
      ros::Time start = first.header.stamp - padding_;
      ros::Time end   = first.header.stamp + padding_;

      ROS_ERROR_COND(cache.size() == 0, "Empty cache. This is probably a bug in GenericChannel");

      // Grab all the elems in our target interval
      std::vector<DeflatedT> interval_elems;
      interval_elems = cache.getSurroundingInterval(start, end);

      ROS_ERROR_COND(interval_elems.size() == 0, "interval_elems.size==0. This shouldn't happen");

      std::vector<double> ranges;
      calibration_message_filters::StationaryUtils::computeChannelRange<DeflatedT>(interval_elems, ranges);

      ChannelFeedback feedback;
      feedback.channel_name   = channel_name_;
      feedback.interval_start = start;
      feedback.interval_end   = end;
      feedback.cache_start    = cache.front().header.stamp;
      feedback.cache_end      = cache.back().header.stamp;
      feedback.num_samples    = interval_elems.size();
      feedback.ranges         = ranges;
      feedback.stationary     = false;


      if (start < cache.front().header.stamp)
      {
        // If the cache didn't have an element that occurred before the start time, then we don't even process the elem
        ROS_DEBUG("neg_padded elem is older than oldest cache elem by [%.3fs]. We can delete it",
                  (cache.front().header.stamp - start).toSec() );
        ROS_DEBUG("First Time:  %u, %u", first.header.stamp.sec, first.header.stamp.nsec);
        ROS_DEBUG("cache.front: %u, %u", cache.front().header.stamp.sec, cache.front().header.stamp.nsec);
        need_to_process.pop_front();
      }
      else if (end > cache.back().header.stamp)
      {
        // If the cache didn't have an element that occurred after the end time,
        //   then pause processing until we get more data
        ROS_DEBUG("end:         %u, %u", end.sec, end.nsec);
        ROS_DEBUG("cache.back:  %u, %u", cache.back().header.stamp.sec, cache.back().header.stamp.nsec);
        keep_processing = false;
      }
      else if (interval_elems.size() < min_samples_)
      {
        // We didn't find enough samples in the interval that we said we care about
        ROS_DEBUG("Not enough samples. interval_elems.size() is [%u]", interval_elems.size());
        need_to_process.pop_front();
      }
      else
      {
        // All of the basic checks passed, so now we need to actually do the stationary check
        if (calibration_message_filters::StationaryUtils::isChannelStationary(ranges, tol_))
        {
          ROS_DEBUG("Elem is stationary. About to processStationary");
          if (stationary_cb)      // NULL check
            stationary_cb(first);
        }
        else
          ROS_DEBUG("Not stationary");
        need_to_process.pop_front();
      }

      processFeedback(feedback);
    }
  }

  void processFeedback(const ChannelFeedback& fb)
  {
    if (feedback_cb_)
      feedback_cb_(fb);
  }

  ChannelTolerance tol_;
  ros::Duration padding_;
  unsigned int min_samples_;
  std::string channel_name_;

private:
  FeedbackCallback feedback_cb_;

};


}


#endif
