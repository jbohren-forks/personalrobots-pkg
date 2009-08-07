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

#ifndef PR2_CALIBRATION_ACTIONS_JOINT_STATES_CHANNEL_H_
#define PR2_CALIBRATION_ACTIONS_JOINT_STATES_CHANNEL_H_

#include <boost/shared_ptr.hpp>
#include <calibration_message_filters/deflated.h>
#include <calibration_message_filters/channel_tolerance.h>
#include <calibration_message_filters/joint_states_deflater.h>
#include <message_filters/subscriber.h>
#include "sorted_deque.h"

// Messages
#include "pr2_calibration_actions/JointStatesChannelConfig.h"
#include "mechanism_msgs/JointStates.h"

namespace pr2_calibration_actions
{

class JointStatesChannel
{
public:
  typedef calibration_message_filters::DeflatedMsg<mechanism_msgs::JointStates> DeflatedJointStates;
  typedef boost::function <void(const DeflatedJointStates&)> StationaryCallback;
  JointStatesChannel(const JointStatesChannelConfig& config, StationaryCallback cb = NULL);

  void registerStationaryCallback(StationaryCallback cb);

  SortedDeque<DeflatedJointStates> stationary_list_;
private:

  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_;

  SortedDeque<DeflatedJointStates> cache_;
  SortedDeque<DeflatedJointStates> need_to_process_;
  StationaryCallback stationary_callback_;
  calibration_message_filters::JointStatesDeflater deflater_;

  ChannelTolerance tol_;
  ros::Duration padding_;
  unsigned int min_samples_;

  void jointStatesCallback(const mechanism_msgs::JointStatesConstPtr& msg);
  void processElems();

  void processStationary(const DeflatedJointStates& deflated);
};


}

#endif

