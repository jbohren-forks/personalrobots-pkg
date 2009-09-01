/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser

#include "pr2_mechanism_control/recorder.h"

namespace pr2_mechanism {

Recorder::Recorder() : robot_(NULL), is_running_(false) {}

Recorder::~Recorder()
{
  stop();
  while (is_running_)
    usleep(10000);
  pub_.shutdown();
}

void Recorder::channel(unsigned int index, const std::string &name)
{
  if (robot_) {  // Makes sure init hasn't been called
    ROS_FATAL("Cannot call channel after init");
    return;
  }
  if (msg_[0].channels.size() <= index)
    msg_[0].channels.resize(index + 1);
  msg_[0].channels[index].name = name;
}

bool Recorder::init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &node, const std::string &topic)
{
  robot_ = robot;

  msg_[1].channels.resize(msg_[0].channels.size());
  for (size_t i = 0; i < msg_[0].channels.size(); ++i)
  {
    msg_[1].channels[i].name = msg_[0].channels[i].name;
    msg_[0].channels[i].values.resize(100);
    msg_[1].channels[i].values.resize(100);
  }

  node_ = node;
  pub_ = node_.advertise<pr2_mechanism_msgs::BufferedData>(topic, 2);

  filling_ = 0;
  publishing_ = 0;

  keep_running_ = true;
  thread_ = boost::thread(&Recorder::publishingLoop, this);

  return true;
}

void Recorder::record(unsigned int index, float value)
{
  if (!robot_)
    return; // init wasn't called
  assert(index >= 0 && index < msg_[filling_].channels.size());

  int ms = (int)fmod(robot_->getTime().toSec() * 1000.0, 1000.0);
  if (ms % 100 == 0)
  {
    if (publishing_ != other(filling_))
    {
      filling_ = other(filling_);
      msg_[filling_].header.stamp = robot_->getTime();
    }
  }

  pr2_mechanism_msgs::BufferedData &msg = msg_[filling_];
  msg.channels[index].values[ms % 100] = value;
}

void Recorder::publishingLoop()
{
  ROS_DEBUG("Entering publishing loop (namespace: %s)", node_.getNamespace().c_str());
  is_running_ = true;
  while (keep_running_)
  {
    ros::spinOnce();
    if (publishing_ == filling_)
    {
      usleep(10000);
    }
    else
    {
      pub_.publish(msg_[publishing_]);
      publishing_ = other(publishing_);
    }
  }
  ROS_DEBUG("Exiting publishing loop (namespace: %s)", node_.getNamespace().c_str());
  is_running_ = false;

}

}
