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


#ifndef TOPIC_SYNCHRONIZER_HH
#define TOPIC_SYNCHRONIZER_HH

#include "rosthread/condition.h"

template <class N>
class TopicSynchronizer
{
  private:

  N* node_;
  void (N::*callback_)();
  
  ros::thread::condition cond_all_;

  int expected_count_;

  int count_;

  bool done_;

  ros::Time waiting_time_;

  void msg_cb(void* p)
  {
    ros::Time* time = (ros::Time*)(p);

    cond_all_.lock();

    // If first to time, wait
    if (count_ == 0)
    {
      wait_for_others(time);
      return;
    }

    // If behind, skip
    if (*time < waiting_time_)
    {
      cond_all_.unlock();
      return;
    }

    // If at time, increment and signal or wait
    if (*time == waiting_time_)
    {
      count_++;
      if (count_ == expected_count_)
      {
        cond_all_.broadcast();
      }

      while (!done_ && *time == waiting_time_)
        cond_all_.wait();
      
      cond_all_.unlock();
      return;
    }

    // If ahead, wake up others and then wait
    if (*time > waiting_time_)
    {
      cond_all_.broadcast();
      wait_for_others(time);
    }
  }

  void wait_for_others(ros::Time* time)
  {
    count_ = 1;
    done_ = false;

    waiting_time_ = *time;
    bool timed_out = false;

    while (count_ < expected_count_ && *time == waiting_time_ && !timed_out)
      if (!cond_all_.timed_wait(1))
      {
        printf(" Timed out waiting for other images...\n");
        timed_out = true;
      }

    if (*time == waiting_time_ && !timed_out)
      (*node_.*callback_)();

    if (*time == waiting_time_)
    {
      done_ = true;
      count_ = 0;
      cond_all_.broadcast();
    }
    cond_all_.unlock();
  }

  public:

  TopicSynchronizer(N* node, void (N::*callback)()) : node_(node), callback_(callback), expected_count_(0), count_(0), done_(false)
  { }

  template <class M>
  void subscribe(std::string topic_name, M& msg, int queue_size)
  {
    node_->subscribe(topic_name, msg, &TopicSynchronizer<N>::msg_cb, this, &(msg.header.stamp), queue_size);
    expected_count_++;
  }
};

#endif
