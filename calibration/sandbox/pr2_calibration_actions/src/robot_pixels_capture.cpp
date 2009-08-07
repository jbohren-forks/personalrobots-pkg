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

#include "pr2_calibration_actions/robot_pixels_capture.h"

using namespace calibration_message_filters;
using namespace pr2_calibration_actions;
using namespace std;

RobotPixelsCapture::RobotPixelsCapture(const RobotPixelsConfig& config, CompletionCallback completion_cb)
{
  completion_cb_ = completion_cb;

  match_found_ = false;

  joint_states_channel_.reset( new JointStatesChannel(config.joint_states_config));
  joint_states_channel_->registerStationaryCallback(boost::bind(&RobotPixelsCapture::jointStatesCb, this, _1));
  stationary_joint_states_.reset(new SortedDeque<DeflatedJointStates>());
  stationary_joint_states_->setMaxSize(100);

  const unsigned int N = config.pixel_configs.size();
  pixel_channels_.resize(N);
  stationary_pixels_.resize(N);

  for (unsigned int i=0; i<N; i++)
  {
    pixel_channels_[i].reset(new PixelChannel(config.pixel_configs[i]));
    pixel_channels_[i]->registerStationaryCallback(boost::bind(&RobotPixelsCapture::pixelCb, this, i, _1));
    stationary_pixels_[i].reset(new SortedDeque<DeflatedImage>());
    stationary_pixels_[i]->setMaxSize(100);
  }

  joint_states_timeshift_ = config.joint_states_timeshift;
  pixel_timeshifts_ = config.pixel_timeshifts;

  if (joint_states_timeshift_ < ros::Duration(0,0))
  {
    ROS_WARN("JointStatesTimeshift duration is < 0. Negating");
    joint_states_timeshift_ = -joint_states_timeshift_;
  }

  for (unsigned int i=0; i<pixel_timeshifts_.size(); i++)
  {
    ROS_FATAL_COND(pixel_timeshifts_[i].chan1 >= N,
                   "Tried to define an pixel timeshift out of bounds timeshift. chan1=%u, num_channels=%u",
                   pixel_timeshifts_[i].chan1, N);
    ROS_FATAL_COND(pixel_timeshifts_[i].chan2 >= N,
                   "Tried to define an pixel timeshift out of bounds timeshift. chan2=%u, num_channels=%u",
                   pixel_timeshifts_[i].chan2, N);
    if (pixel_timeshifts_[i].max_timeshift < ros::Duration(0,0))
    {
      ROS_WARN("Cross pixel timeshift duration is < 0. Negating");
      pixel_timeshifts_[i].max_timeshift = -pixel_timeshifts_[i].max_timeshift;
    }
  }

}

void RobotPixelsCapture::jointStatesCb(const DeflatedJointStates& deflated)
{
  {
    boost::mutex::scoped_lock lock(joint_states_mutex_);
    ROS_DEBUG("About to add to the statioanry_joint_states list...");
    stationary_joint_states_->add(deflated);
  }

  ROS_INFO("Got a stationary joint state");
  searchForMatch(deflated.header.stamp);
}

void RobotPixelsCapture::pixelCb(unsigned int channel, const DeflatedImage& deflated)
{
  {
    boost::mutex::scoped_lock lock(pixels_mutex_);
    stationary_pixels_[channel]->add(deflated);
  }

  searchForMatch(deflated.header.stamp);
}

void RobotPixelsCapture::searchForMatch(const ros::Time& time)
{
  const unsigned int N = stationary_pixels_.size();


  DeflatedJointStates joint_states;
  bool success;

  {
    boost::mutex::scoped_lock lock(joint_states_mutex_);
    if (stationary_joint_states_->size() == 0)
    {
      ROS_INFO("Haven't received data from JointStates yet");
      return;
    }
    success = stationary_joint_states_->getClosestElem(time, joint_states);
  }
  ROS_ERROR_COND(!success, "Error trying to get closest JointStates. elem. This is a bug in RobotPixelsCapture");

  vector<DeflatedImage> pixel_vec;
  pixel_vec.resize(stationary_pixels_.size());

  {
    boost::mutex::scoped_lock lock(pixels_mutex_);
    for (unsigned int i=0; i < N; i++)
    {
      if (stationary_pixels_[i]->size() == 0)
      {
        ROS_INFO("Haven't received data for PixelChannel[%u] yet", i);
        return;
      }
      success = stationary_pixels_[i]->getClosestElem(time, pixel_vec[i]);
      ROS_ERROR_COND(!success, "Error trying to get closest pixel elem. This is a bug in RobotPixelsCapture");
    }
  }


  // If timeshift is 0, then it's a special case, and we skip the timeshift checking
  if (joint_states_timeshift_ != ros::Duration(0,0))
  {
    // Verify joint states timeshift is within bounds for each pixel channel
    for (unsigned int i=0; i < N; i++)
    {
      double time_diff = fabs((joint_states.header.stamp - pixel_vec[i].header.stamp).toSec());
      if (time_diff > joint_states_timeshift_.toSec())
      {
        ROS_INFO("Difference between joint_states and PixelChannel[%u] is [%.2fs]. Bigger than [%.2fs]",
                  i, time_diff, joint_states_timeshift_.toSec());
        return;
      }
      else
      {
        ROS_INFO("Difference between joint_states and PixelChannel[%u] is [%.2fs]. Smaller than [%.2fs]",
                  i, time_diff, joint_states_timeshift_.toSec());
      }
    }
  }
  else
    ROS_INFO("Ignote joint states timeshift");

  // Verify all cross pixel timeshifts are within bounds
  for (unsigned int i=0; i<pixel_timeshifts_.size(); i++)
  {
    double time_diff = fabs( (pixel_vec[pixel_timeshifts_[i].chan1].header.stamp -
                              pixel_vec[pixel_timeshifts_[i].chan2].header.stamp).toSec() );
    if (time_diff > pixel_timeshifts_[i].max_timeshift.toSec())
    {
      ROS_INFO("Difference PixelChannel[%u] & PixelChannel[%u] is [%.2fs]. Bigger than [%.2fs]",
                pixel_timeshifts_[i].chan1, pixel_timeshifts_[i].chan2, time_diff,
                pixel_timeshifts_[i].max_timeshift.toSec() );
      return;
    }
  }

  {
    boost::mutex::scoped_lock lock(match_found_mutex_);
    if (!match_found_)
    {
      match_found_ = true;
      ROS_INFO("Found a match!");
      processMatch(joint_states, pixel_vec);
    }
    else
      ROS_INFO("Found 'another' match!");
  }
}

void RobotPixelsCapture::processMatch(const DeflatedJointStates& deflated_joint_states,
                                      const std::vector<DeflatedImage>& deflated_images)
{
  RobotPixelsResult result;
  joint_states_channel_->buildResult(deflated_joint_states, result.joint_states_result);

  const unsigned int N = deflated_images.size();

  assert(pixel_channels_.size() == N);

  result.pixel_results.resize(N);

  for (unsigned int i=0; i<N; i++)
    pixel_channels_[i]->buildResult(deflated_images[i], result.pixel_results[i]);

  if (completion_cb_)
    completion_cb_(result);
}

void RobotPixelsCapture::shutdown()
{
  joint_states_channel_->shutdown();

  for(unsigned int i=0; i<pixel_channels_.size(); i++)
  {
    pixel_channels_[i]->shutdown();
  }
}
