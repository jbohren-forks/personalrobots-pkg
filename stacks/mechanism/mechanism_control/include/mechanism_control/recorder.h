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

#ifndef REALTIME_TOOLS_RECORDER_H
#define REALTIME_TOOLS_RECORDER_H

#include <string>
#include <boost/thread/thread.hpp>

#include "ros/node_handle.h"
#include "mechanism_model/robot.h"
#include "realtime_tools/realtime_tools.h"

#include "mechanism_msgs/BufferedData.h"

namespace mechanism {

class Recorder
{
public:
  Recorder();
  ~Recorder();

  // Call channel
  void channel(unsigned int index, const std::string &name);
  bool init(mechanism::RobotState *robot, const ros::NodeHandle &node, const std::string &topic = "trace");

  void record(unsigned int index, float value);

private:
  ros::NodeHandle node_;
  ros::Publisher pub_;
  mechanism::RobotState *robot_;

  mechanism_msgs::BufferedData msg_[2];
  int other(int i) { return i == 0 ? 1 : 0; }
  int filling_;  // Index of msg_ which the RT process is currently filling with data
  int publishing_;  // Index of msg_ which is being publishing (or waiting to be published)

  boost::thread thread_;
  void publishingLoop();
  bool is_running_;
  bool keep_running_;
  void stop() { keep_running_ = false; }
};

}

#endif
