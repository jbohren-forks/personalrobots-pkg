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


#include "calibration_message_filters/stationary_checker.h"

using namespace std;
using namespace calibration_message_filters;

struct LatestTimeComp
{
  LatestTimeComp() { }

  bool operator() ( StationaryChecker::CachePtr& first,
                    StationaryChecker::CachePtr& second)
  {
    return ( first->getLatestTime() < second->getLatestTime() );
  }
};


StationaryChecker::StationaryChecker()
{

}

StationaryChecker::StationaryChecker(CachePtrVec caches, std::vector<ChannelTolerance> all_channel_tol)
{
  init(caches, all_channel_tol);
}

void StationaryChecker::init(CachePtrVec caches, std::vector<ChannelTolerance> all_channel_tol)
{
  caches_ = caches;
  all_channel_tol_ = all_channel_tol;
}

ros::Time StationaryChecker::isLatestStationary(const ros::Duration& interval_duration)
{
  // Find the new time at which all the filters have recevied at least one message. This
  // can be though of as `min( cache::getLatestTime() )` , over all the caches.
  ros::Time min_latest_time;

  if (caches_.size() == 0)
  {
    ROS_WARN("0 caches are registered in the stationary filter. You are using this code incorrectly");
    return ros::Time(0,0);
  }

  min_latest_time = (*std::min_element(caches_.begin(), caches_.end(), LatestTimeComp()))->getLatestTime();

  ros::Time interval_end = min_latest_time;
  ros::Time interval_start = min_latest_time - interval_duration;

  // Go through each of the caches
  vector<double> ranges;
  for (unsigned int i=0; i < caches_.size(); i++)
  {
    vector<DeflatedConstPtr> deflated_channel = caches_[i]->getInterval(interval_start, interval_end);

    if (deflated_channel.size() == 0)
      return ros::Time(0,0);

    computeChannelRange(deflated_channel, ranges);
    bool channel_stationary = isChannelStationary(ranges, all_channel_tol_[i]);
    if (!channel_stationary)
      return ros::Time(0,0);
  }

  // if we got here, then all the channels were stationary

  // Return the halfway point of this interval
  return ( interval_end - interval_duration*.5);
}

void StationaryChecker::computeChannelRange(const std::vector<DeflatedConstPtr>& channel,
                                            std::vector<double>& ranges)
{
  ROS_ERROR_COND(channel.size() == 0, "Tried to preprocess a deflated channel of size 0");

  std::vector<double> min_val;
  std::vector<double> max_val;

  min_val = channel[0]->deflated_;
  max_val = channel[0]->deflated_;

  const unsigned int N = channel[0]->deflated_.size();

  //ROS_INFO("channel[0]->deflated_.size() = %u", N);

  // Find the min and max for each sub-channel
  for (unsigned int i=0; i<channel.size(); i++)
  {
    assert(channel[i]->deflated_.size() == N);
    for (unsigned int j=1; j<N; j++)
    {
      min_val[j] = min (min_val[j], channel[i]->deflated_[j]);
      max_val[j] = max (max_val[j], channel[i]->deflated_[j]);
    }
  }

  // Compute range for each subchannel
  ranges.resize(N);
  for (unsigned int i=0; i<N; i++)
    ranges[i] = max_val[i] - min_val[i];
}

bool StationaryChecker::isChannelStationary(const std::vector<double>& ranges,
                                            const ChannelTolerance& tol)
{
  ROS_ERROR_COND(ranges.size() != tol.tol_.size(), "ranges.size() [%u] should match tol.tol_.size() [%u]", ranges.size(), tol.tol_.size());
  const unsigned int N = ranges.size();
  // Success Check
  for (unsigned int i=0; i<N; i++)
    if (ranges[i] > tol.tol_[i])
      return false;
  return true;
}
