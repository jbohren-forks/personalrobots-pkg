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

#ifndef MESSAGE_FILTERS_TIME_SEQUENCER_H
#define MESSAGE_FILTERS_TIME_SEQUENCER_H

#include <boost/noncopyable.hpp>

#include <ros/ros.h>

#include "connection.h"

namespace message_filters
{

template<class M>
class TimeSequencer : public boost::noncopyable
{
public:
  typedef boost::shared_ptr<M const> MConstPtr ;
  typedef boost::function<void(const MConstPtr&)> Callback;
  typedef boost::signal<void(const MConstPtr&)> Signal;

  template<class F>
  TimeSequencer(F& f, ros::Duration delay, ros::Duration update_rate, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle())
  : delay_(delay)
  , update_rate_(update_rate)
  , queue_size_(queue_size)
  , nh_(nh)
  {
    init();
    connectTo(f);
  }

  TimeSequencer(ros::Duration delay, ros::Duration update_rate, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle())
  : delay_(delay)
  , update_rate_(update_rate)
  , queue_size_(queue_size)
  , nh_(nh)
  {
    init();
  }

  template<class A>
  void connectTo(A& a)
  {
    incoming_connection_ = a.connect(boost::bind(&TimeSequencer::cb, this, _1));
  }

  ~TimeSequencer()
  {
    update_timer_.stop();

    incoming_connection_.disconnect();
  }

  Connection connect(const Callback& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return Connection(boost::bind(&TimeSequencer::disconnect, this, _1), signal_.connect(callback));
  }

  void add(const MConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(messages_mutex_);
    if (msg->header.stamp < last_time_)
    {
      return;
    }

    messages_.insert(msg);

    if (queue_size_ != 0 && messages_.size() > queue_size_)
    {
      messages_.erase(*messages_.begin());
    }
  }

private:
  class MessageSort
  {
  public:
    bool operator()(const MConstPtr& lhs, const MConstPtr& rhs) const
    {
      return lhs->header.stamp < rhs->header.stamp;
    }
  };
  typedef std::multiset<MConstPtr, MessageSort> S_Message;
  typedef std::vector<MConstPtr> V_Message;

  void cb(const MConstPtr& msg)
  {
    add(msg);
  }

  void disconnect(const Connection& c)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    signal_.disconnect(c.getBoostConnection());
  }

  void dispatch()
  {
    V_Message to_call;

    {
      boost::mutex::scoped_lock lock(messages_mutex_);

      while (!messages_.empty())
      {
        const MConstPtr& m = *messages_.begin();
        if (m->header.stamp + delay_ <= ros::Time::now())
        {
          last_time_ = m->header.stamp;
          to_call.push_back(m);
          messages_.erase(messages_.begin());
        }
        else
        {
          break;
        }
      }
    }

    {
      boost::mutex::scoped_lock lock(signal_mutex_);

      typename V_Message::iterator it = to_call.begin();
      typename V_Message::iterator end = to_call.end();
      for (; it != end; ++it)
      {
        signal_(*it);
      }
    }
  }

  void update(const ros::TimerEvent&)
  {
    dispatch();
  }

  void init()
  {
    update_timer_ = nh_.createTimer(update_rate_, &TimeSequencer::update, this);
  }



  ros::Duration delay_;
  ros::Duration update_rate_;
  uint32_t queue_size_;
  ros::NodeHandle nh_;

  ros::Timer update_timer_;

  Connection incoming_connection_;
  Signal signal_;
  boost::mutex signal_mutex_;


  S_Message messages_;
  boost::mutex messages_mutex_;
  ros::Time last_time_;
};

}

#endif
