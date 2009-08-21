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

#ifndef PR2_CALIBRATION_ACTIONS_ROBOT_PIXELS_CAPTURE_H_
#define PR2_CALIBRATION_ACTIONS_ROBOT_PIXELS_CAPTURE_H_

#include "pixel_channel.h"
#include "joint_states_channel.h"

// messages
#include "pr2_calibration_actions/RobotPixelsConfig.h"
#include "pr2_calibration_actions/RobotPixelsResult.h"
#include "pr2_calibration_actions/RobotPixelsFeedback.h"

namespace pr2_calibration_actions
{

class RobotPixelsCapture
{
public:
  typedef calibration_message_filters::DeflatedMsg<mechanism_msgs::JointStates> DeflatedJointStates;
  typedef calibration_message_filters::DeflatedMsg<sensor_msgs::Image> DeflatedImage;
  typedef boost::function<void(const RobotPixelsResult&)> CompletionCallback;
  typedef boost::function<void(const RobotPixelsFeedback&)> FeedbackCallback;

  RobotPixelsCapture(const RobotPixelsConfig& config,
                     CompletionCallback completion_cb = NULL,
                     FeedbackCallback feedback_cb_ = NULL);

  void registerFeedbackCb(FeedbackCallback cb);

  void shutdown();

private:
  void processMatch(const DeflatedJointStates& deflated_joint_states,
               const std::vector<DeflatedImage>& deflated_image);
  void jointStatesCb(const DeflatedJointStates& deflated);
  void pixelCb(unsigned int channel, const DeflatedImage& deflated);
  void searchForMatch(const ros::Time& time);

  void updateJointStatesFeedback(const ChannelFeedback& channel_feedback);
  void updatePixelFeedback(unsigned int channel, const ChannelFeedback& channel_feedback);

  boost::scoped_ptr<JointStatesChannel> joint_states_channel_;
  boost::scoped_ptr<SortedDeque<DeflatedJointStates> > stationary_joint_states_;

  std::vector<boost::shared_ptr<PixelChannel> > pixel_channels_;
  std::vector<boost::shared_ptr<SortedDeque<DeflatedImage> > > stationary_pixels_;

  ros::Duration joint_states_timeshift_;
  std::vector<CrossPixelTimeshift> pixel_timeshifts_;

  CompletionCallback completion_cb_;
  FeedbackCallback feedback_cb_;

  RobotPixelsFeedback cur_feedback_;

  // Mutexes
  boost::mutex joint_states_mutex_;
  boost::mutex pixels_mutex_;
  boost::mutex match_found_mutex_;

  bool match_found_;
};

}

#endif
