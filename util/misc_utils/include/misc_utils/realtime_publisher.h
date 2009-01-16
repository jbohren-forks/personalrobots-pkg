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
#include <ros/node.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <realtime_thread/realtime_thread.h>

namespace misc_utils {

template <class Msg>
class RealtimePublisher
{
public:
  RealtimePublisher(const std::string &topic, int queue_size)
    : topic_(topic), node_(NULL), is_running_(false), keep_running_(false), turn_(REALTIME)
  {
    if ((node_ = ros::Node::instance()) == NULL)
    {
      int argc = 0;  char **argv = NULL;
      ros::init(argc, argv);
      node_ = new ros::Node("realtime_publisher", ros::Node::DONT_HANDLE_SIGINT);
    }

    node_->advertise<Msg>(topic_, queue_size);

    if (0 != realtime_cond_create(&updated_cond_))
    {
      perror("realtime_cond_create");
      abort();
    }
    if (0 != realtime_mutex_create(&msg_lock_))
    {
      perror("realtime_mutex_create");
      abort();
    }
    keep_running_ = true;
    thread_ = boost::thread(&RealtimePublisher::publishingLoop, this);
  }

  ~RealtimePublisher()
  {
    stop();
    while (is_running())
      usleep(100);

    // TODO: fix when multiple nodes per process are supported

    // Don't unadvertise topic because other threads within the
    // process may still be publishing on the topic
    //node_->unadvertise(topic_);

    // Destroy thread resources
    realtime_cond_delete(&updated_cond_);
    realtime_mutex_delete(&msg_lock_);
  }

  void stop()
  {
    keep_running_ = false;
    realtime_cond_signal(&updated_cond_);  // So the publishing loop can exit
  }

  Msg msg_;

  int lock()
  {
    return realtime_mutex_lock(&msg_lock_);
  }

  bool trylock()
  {
    if (0 == realtime_mutex_trylock(&msg_lock_))
    {
      if (turn_ == REALTIME)
      {
        return true;
      }
      else
      {
        realtime_mutex_unlock(&msg_lock_);
        return false;
      }
    }
    else
      return false;
  }

  void unlock()
  {
    realtime_mutex_unlock(&msg_lock_);
  }

  void unlockAndPublish()
  {
    turn_ = NON_REALTIME;
    realtime_mutex_unlock(&msg_lock_);
    realtime_cond_signal(&updated_cond_);
  }

  bool is_running() const { return is_running_; }

  void publishingLoop()
  {
    RealtimeTask task;

    int err = realtime_shadow_task(&task);
    if (err)
      ROS_WARN("Unable to shadow task: %d\n", err);

    is_running_ = true;
    turn_ = REALTIME;
    while (keep_running_)
    {
      // Locks msg_ and copies it
      realtime_mutex_lock(&msg_lock_);
      while (turn_ != NON_REALTIME)
      {
        if (!keep_running_)
          break;
        realtime_cond_wait(&updated_cond_, &msg_lock_);
      }

      Msg outgoing(msg_);
      turn_ = REALTIME;
      realtime_mutex_unlock(&msg_lock_);

      // Sends the outgoing message
      if (keep_running_)
        node_->publish(topic_, outgoing);
    }
    is_running_ = false;
  }

  std::string topic_;

private:

  ros::Node *node_;
  bool is_running_;
  bool keep_running_;

  boost::thread thread_;

  RealtimeMutex msg_lock_;  // Protects msg_
  RealtimeCond updated_cond_;

  enum {REALTIME, NON_REALTIME};
  int turn_;  // Who's turn is it to use msg_?
};

}

#endif
