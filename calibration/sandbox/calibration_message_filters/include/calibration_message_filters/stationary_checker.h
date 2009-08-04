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


#ifndef CALIBRATION_MESSAGE_FILTERS_STATIONARY_CHECKER_H_
#define CALIBRATION_MESSAGE_FILTERS_STATIONARY_CHECKER_H_

#include "message_filters/cache.h"
#include "calibration_message_filters/deflated.h"
#include "calibration_message_filters/channel_tolerance.h"
#include "calibration_message_filters/joint_states_deflater.h"

namespace calibration_message_filters
{

class StationaryChecker
{
public:
  typedef message_filters::Cache<Deflated>* CachePtr;
  typedef std::vector<CachePtr> CachePtrVec;

  StationaryChecker();
  StationaryChecker(CachePtrVec caches, std::vector<ChannelTolerance> all_channel_tol);
  void init(CachePtrVec caches, std::vector<ChannelTolerance> all_channel_tol);

  ros::Time isLatestStationary(const ros::Duration& interval_duration);
private:
  typedef boost::shared_ptr<const Deflated> DeflatedConstPtr;

  void computeChannelRange(const std::vector<DeflatedConstPtr>& channel,
                           std::vector<double>& ranges);

  CachePtrVec caches_;

  std::vector<ChannelTolerance> all_channel_tol_;

  bool isChannelStationary(const std::vector<double>& ranges,
                           const ChannelTolerance& tol);

};

}

#endif
