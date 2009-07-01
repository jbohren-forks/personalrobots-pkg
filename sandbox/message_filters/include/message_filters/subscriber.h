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

#ifndef MESSAGE_FILTERS_SUBSCRIBER_H
#define MESSAGE_FILTERS_SUBSCRIBER_H

#include <ros/ros.h>

#include <boost/signals.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/noncopyable.hpp>

namespace message_filters
{

template<class M>
class Subscriber : public boost::noncopyable
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef boost::function<void(const MConstPtr&)> Callback;
  typedef boost::signal<void(const MConstPtr&)> Signal;

  Subscriber(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0)
  {
    subscribeTo(nh, topic, queue_size, transport_hints, callback_queue);
  }

  Subscriber()
  {
  }

  ~Subscriber()
  {
    sub_.shutdown();
  }

  void subscribeTo(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0)
  {
    ros::SubscribeOptions ops;
    ops.init<M>(topic, queue_size, boost::bind(&Subscriber<M>::cb, this, _1));
    ops.callback_queue = callback_queue;
    ops.transport_hints = transport_hints;
    sub_ = nh.subscribe(ops);
  }

  boost::signals::connection connect(const Callback& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return signal_.connect(callback);
  }

private:
  void cb(const MConstPtr& m)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    signal_(m);
  }

  ros::Subscriber sub_;
  Signal signal_;
  boost::mutex signal_mutex_;
};

}

#endif
