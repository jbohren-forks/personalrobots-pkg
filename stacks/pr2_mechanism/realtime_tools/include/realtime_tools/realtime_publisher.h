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

/*
 * Publishing ROS messages is difficult, as the publish function is
 * not realtime safe.  This class provides the proper locking so that
 * you can call publish in realtime and a separate (non-realtime)
 * thread will ensure that the message gets published over ROS.
 *
 * Author: Stuart Glaser
 */
#ifndef REALTIME_PUBLISHER_H
#define REALTIME_PUBLISHER_H

#include <string>
#include <ros/node_handle.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace realtime_tools {

template <class Msg>
class RealtimePublisher
{
private:
  void construct(int queue_size)
  {
    publisher_ = node_.advertise<Msg>(topic_, queue_size);
    keep_running_ = true;
    thread_ = boost::thread(&RealtimePublisher::publishingLoop, this);
  }
public:

  // Deprecated
  __attribute__((deprecated)) RealtimePublisher(const std::string &topic, int queue_size)
    : topic_(topic), is_running_(false), keep_running_(false), turn_(REALTIME)
  {
    construct(queue_size);
  }

  RealtimePublisher(const ros::NodeHandle &node, const std::string &topic, int queue_size)
    : topic_(topic), node_(node), is_running_(false), keep_running_(false), turn_(REALTIME)
  {
    construct(queue_size);
  }

  ~RealtimePublisher()
  {
    stop();
    while (is_running())
      usleep(100);

    publisher_.shutdown();
  }

  void stop()
  {
    keep_running_ = false;
    updated_cond_.notify_one();  // So the publishing loop can exit
  }

  Msg msg_;

  void lock()
  {
    msg_mutex_.lock();
  }

  bool trylock()
  {
    if (msg_mutex_.try_lock())
    {
      if (turn_ == REALTIME)
      {
        return true;
      }
      else
      {
        msg_mutex_.unlock();
        return false;
      }
    }
    else
      return false;
  }

  void unlock()
  {
    msg_mutex_.unlock();
  }

  void unlockAndPublish()
  {
    turn_ = NON_REALTIME;
    msg_mutex_.unlock();
    updated_cond_.notify_one();
  }

  bool is_running() const { return is_running_; }

  void publishingLoop()
  {
    is_running_ = true;
    turn_ = REALTIME;
    while (keep_running_)
    {
      Msg outgoing;
      // Locks msg_ and copies it
      {
        boost::unique_lock<boost::mutex> lock(msg_mutex_);
        while (turn_ != NON_REALTIME)
        {
          if (!keep_running_)
            break;
          updated_cond_.wait(lock);
        }

        outgoing = msg_;
        turn_ = REALTIME;
      }

      // Sends the outgoing message
      if (keep_running_)
        publisher_.publish(outgoing);
    }
    is_running_ = false;
  }

  std::string topic_;

private:

  ros::NodeHandle node_;
  ros::Publisher publisher_;
  bool is_running_;
  bool keep_running_;

  boost::thread thread_;

  boost::mutex msg_mutex_;  // Protects msg_
  boost::condition_variable updated_cond_;

  enum {REALTIME, NON_REALTIME};
  int turn_;  // Who's turn is it to use msg_?
};

}

#endif
