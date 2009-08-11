/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef NODE_SEQUENCER_SEQUENCED_TASK_H
#define NODE_SEQUENCER_SEQUENCED_TASK_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;
using std_msgs::String;

namespace node_sequencer 
{

class SequencedTask
{
public:
  SequencedTask(const string& name, const string& ns) : done_(false), connected_(false), name_(name)
  {
    subscriber_ = node_.subscribe(ns + "/" + "can_start", 1000, &SequencedTask::startCallback, this);
    publisher_ = node_.advertise<String>(ns + "/" + "completed", 1000, bind(&SequencedTask::connectionCallback, this, _1));
    
    ros::Duration d(.1);
    while (node_.ok() && !done_) {
      ros::spinOnce();
      d.sleep();
    }
  }

  ~SequencedTask()
  {
    String msg;
    msg.data = name_;
    ros::Duration d(.1);
    while (node_.ok() && !connected_) {
      d.sleep();
      ROS_DEBUG_STREAM_NAMED ("sequenced_task", "Waiting for sequencer node to connect to " << name_);
    }
    
    ROS_DEBUG_STREAM_NAMED ("sequenced_task", "Setting completion message for task " << name_);
    if (node_.ok())
      publisher_.publish(msg);

  }

  void startCallback (const std_msgs::String::ConstPtr& msg)
  {
    if (msg->data == name_) {
      ROS_DEBUG_STREAM_NAMED ("sequenced_task", "Received start callback for " << name_);
      done_ = true;
    }
  }

  void connectionCallback (const ros::SingleSubscriberPublisher& pub)
  {
    ROS_DEBUG_STREAM_NAMED ("sequenced_task", "Received connection callback");
    connected_=true;
  }

private:

  bool done_;
  bool connected_;
  string name_;
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};
  



} // namespace


#endif // TOPOLOGICAL_MAP_TOPOLOGICAL_MAP_H
