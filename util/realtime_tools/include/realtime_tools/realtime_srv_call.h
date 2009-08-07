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
 * Calling a ROS service is difficult, as the service call function is
 * not realtime safe.  This class provides the proper locking so that
 * you can preform a service call in realtime and a separate (non-realtime)
 * thread will ensure that the message gets published over ROS.
 * This is based on the realtime publisher originally written by Stu Glaser.
 *
 * Author: Melonee Wise
 */
#ifndef REALTIME_SRV_CALL_H
#define REALTIME_SRV_CALL_H

#include <string>
#include <ros/ros.h> 
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

namespace realtime_tools {

template <class SrvReq, class SrvRes>
class RealtimeSrvCall
{
public:
  RealtimeSrvCall(const ros::NodeHandle &node, const std::string &topic)
    : node_(node), is_running_(false), keep_running_(false), turn_(REALTIME)
  {
    client_ = node_.serviceClient<SrvReq, SrvRes>(topic);

    // Makes the trylock() fail until the service is ready
    lock();
    ROS_INFO("RealtimeSrvCall is waiting for %s", topic.c_str() );
    ros::service::waitForService(topic);
    ROS_INFO("RealtimeSrvCall is finished waiting for %s", topic.c_str());
    unlock();

    keep_running_ = true;
    thread_ = boost::thread(&RealtimeSrvCall::callLoop, this);
  }

  ~RealtimeSrvCall()
  {
    stop();
    while (is_running())
      usleep(100);
    
    client_.shutdown();
  }

  void stop()
  {
    keep_running_ = false;
    updated_cond_.notify_one();  // So the call loop can exit
  }

  SrvReq srv_req_;
  SrvRes srv_res_;

  void lock()
  {
    srv_mutex_.lock();
  }

  bool trylock()
  {
    if (srv_mutex_.try_lock())
    {
      if (turn_ == REALTIME)
      {
        return true;
      }
      else
      {
        srv_mutex_.unlock();
        return false;
      }
    }
    else
      return false;
  }

  void unlock()
  {
    srv_mutex_.unlock();
  }

  void unlockAndCall()
  {
    turn_ = NON_REALTIME;
    srv_mutex_.unlock();
    updated_cond_.notify_one();
  }

  bool is_running() const { return is_running_; }

  void callLoop()
  {
    is_running_ = true;
    turn_ = REALTIME;
    while (keep_running_)
    {
      SrvReq outgoing;
      SrvRes incoming;
      // Locks Srv_ and copies it
      {
        boost::unique_lock<boost::mutex> lock(srv_mutex_);
        while (turn_ != NON_REALTIME)
        {
          if (!keep_running_)
            break;
          updated_cond_.wait(lock);
        }

        outgoing = srv_req_;
        incoming = srv_res_;
        turn_ = REALTIME;
      }
      
      // Sends the outgoing message
      if (keep_running_)
        client_.call(srv_req_, srv_res_);
               
    }
    is_running_ = false;
  }


private:

  ros::NodeHandle node_;
  ros::ServiceClient client_;
  bool is_running_;
  bool keep_running_;

  boost::thread thread_;

  boost::mutex srv_mutex_;  // Protects srv_
  boost::condition_variable updated_cond_;

  enum {REALTIME, NON_REALTIME};
  int turn_;  // Who's turn is it to use srv_?
};

}

#endif
