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


#ifndef CALIBRATION_MESSAGE_FILTERS_STATIONARY_UTILS_H_
#define CALIBRATION_MESSAGE_FILTERS_STATIONARY_UTILS_H_

#include "message_filters/cache.h"
#include "calibration_message_filters/deflated.h"
#include "calibration_message_filters/channel_tolerance.h"
#include "calibration_message_filters/joint_states_deflater.h"
#include <algorithm>

namespace calibration_message_filters
{

class StationaryUtils
{
public:
  typedef boost::shared_ptr<const Deflated> DeflatedConstPtr;

  static void computeChannelPtrRange(const std::vector<DeflatedConstPtr>& channel,
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
        min_val[j] = std::min (min_val[j], channel[i]->deflated_[j]);
        max_val[j] = std::max (max_val[j], channel[i]->deflated_[j]);
      }
    }

    // Compute range for each subchannel
    ranges.resize(N);
    for (unsigned int i=0; i<N; i++)
      ranges[i] = max_val[i] - min_val[i];
  }

  template<class DeflatedT>
  static void computeChannelRange(const std::vector<DeflatedT>& channel,
                           std::vector<double>& ranges)
  {
    ROS_ERROR_COND(channel.size() == 0, "Tried to preprocess a deflated channel of size 0");

    std::vector<double> min_val;
    std::vector<double> max_val;

    min_val = channel[0].deflated_;
    max_val = channel[0].deflated_;

    const unsigned int N = channel[0].deflated_.size();

    // Find the min and max for each sub-channel
    for (unsigned int i=0; i<channel.size(); i++)
    {
      assert(channel[i].deflated_.size() == N);
      for (unsigned int j=1; j<N; j++)
      {
        min_val[j] = std::min (min_val[j], channel[i].deflated_[j]);
        max_val[j] = std::max (max_val[j], channel[i].deflated_[j]);
      }
    }

    // Compute range for each subchannel
    ranges.resize(N);
    for (unsigned int i=0; i<N; i++)
      ranges[i] = max_val[i] - min_val[i];
  }

  static bool isChannelStationary(const std::vector<double>& ranges,
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

};



}

#endif
