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

#ifndef CALIBRATION_MESSAGE_FILTERS_SANDWICH_H_
#define CALIBRATION_MESSAGE_FILTERS_SANDWICH_H_

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals.hpp>

#include "message_filters/cache.h"

#include "ros/time.h"
#include "calibration_message_filters/sandwich_elem.h"

namespace calibration_message_filters
{

template<class M, class S>
class Sandwich
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef boost::shared_ptr<S const> SConstPtr;
  typedef boost::function<void(const SandwichElem<M,S>&)> Callback;
  typedef boost::signal<void(const SandwichElem<M,S>&)> Signal;

  template<class F1>
  Sandwich(F1& f1, message_filters::Cache<S>& cache, unsigned int queue_size=1,
           const ros::Duration& padding_before=ros::Duration(0,0),
           const ros::Duration& padding_after=ros::Duration(0,0),
           const ros::Duration& expiration_limit=ros::Duration(10,0) )
  {
    setQueueSize(queue_size);
    setPadding(padding_before, padding_after);
    setExpirationLimit(expiration_limit);
    connectTo(f1, cache);
  }

  Sandwich(unsigned int queue_size=1,
           const ros::Duration& padding_before=ros::Duration(0,0),
           const ros::Duration& padding_after=ros::Duration(0,0),
           const ros::Duration& expiration_limit=ros::Duration(10,0) )
  {
    cache_ = NULL;
    setQueueSize(queue_size);
    setPadding(padding_before, padding_after);
    setExpirationLimit(expiration_limit);
  }

  template<class F1>
  void connectInput(F1& f1, message_filters::Cache<S>& cache)
  {
    {
      boost::mutex::scoped_lock lock(signal_mutex_);
      incoming_msg_connection_  = f1.connect(boost::bind(&Sandwich::addToQueue, this, _1));
      incoming_gate_connection_ = cache.connect(boost::bind(&Sandwich::addToGate,  this, _1));
    }

    {
      boost::mutex::scoped_lock lock(data_mutex_);
      cache_ = &cache;
    }
  }

  void connectCacheInput(message_filters::Cache<S>& cache)
  {
    {
      boost::mutex::scoped_lock lock(signal_mutex_);
      incoming_gate_connection_ = cache.registerCallback(boost::bind(&Sandwich::addToGate,  this, _1));
    }

    {
      boost::mutex::scoped_lock lock(data_mutex_);
      cache_ = &cache;
    }
  }

  message_filters::Connection registerCallback(const Callback& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return message_filters::Connection(boost::bind(&Sandwich::disconnect, this, _1), signal_.connect(callback));
  }

  ~Sandwich()
  {
    incoming_msg_connection_.disconnect();
    incoming_gate_connection_.disconnect();
  }

  void setQueueSize(unsigned int queue_size)
  {
    boost::mutex::scoped_lock lock(data_mutex_);

    if (queue_size == 0)
      return;

    queue_size_ = queue_size;
  }

  void setPadding(const ros::Duration& padding_before,
                  const ros::Duration& padding_after)
  {
    padding_before_ = padding_before;
    padding_after_  = padding_after;
  }

  void setExpirationLimit(const ros::Duration& expiration_limit)
  {
    boost::mutex::scoped_lock lock(data_mutex_);
    if (expiration_limit < ros::Duration(0,0))
      return;
    expiration_limit_ = expiration_limit;
  }

  void addToQueue(const MConstPtr& msg)
  {

    {
      boost::mutex::scoped_lock lock(data_mutex_);

      while (queue_.size() >= queue_size_)                       // Keep popping off old data until we have space for a new msg
        queue_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it

      // Add the msg to the queue, sorted by header.stamp
      typename std::deque<MConstPtr >::reverse_iterator rev_it = queue_.rbegin();
      // Keep walking backwards along deque until we hit the beginning,
      //   or until we find a timestamp that's smaller than (or equal to) msg's timestamp
      while(rev_it != queue_.rend() && (*rev_it)->header.stamp > msg->header.stamp)
        rev_it++;
      // Add msg to the cache
      queue_.insert(rev_it.base(), msg);

    }

    processQueue();
  }

  void addToGate(const SConstPtr& msg)
  {
    processQueue();
  }

private:
  void disconnect(const message_filters::Connection& c)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    signal_.disconnect(c.getBoostConnection());
  }

  void processQueue()
  {
    std::vector<SandwichElem<M,S> > valid_elems;
    {

      boost::mutex::scoped_lock lock(data_mutex_);

      if (!cache_)
        return;

      // Process all elems in the queue
      typename std::deque<MConstPtr >::iterator it = queue_.begin();

      while (it != queue_.end())
      {
        // Check if we need to throw away the elem
        if ( (*it)->header.stamp - padding_before_ < cache_->getLatestTime() - expiration_limit_ )
        {
          it = queue_.erase(it);
        }
        else
        {
          // See what the interval looks like
          std::vector<SConstPtr > interval_msgs = cache_->getSurroundingInterval((*it)->header.stamp - padding_before_,
                                                                                 (*it)->header.stamp + padding_after_);
          if (interval_msgs.size() < 2)
          {
            ++it;
          }
          else if((*it)->header.stamp + padding_after_  < interval_msgs.back()->header.stamp &&
                  (*it)->header.stamp - padding_before_ > interval_msgs.front()->header.stamp)
          {
            SandwichElem<M,S> sandwich_elem;
            sandwich_elem.data = *it;
            sandwich_elem.interval = interval_msgs;
            it = queue_.erase(it);
            valid_elems.push_back(sandwich_elem);
          }
          else
          {
            ++it;
          }
        }
      }
    }

    // Call the callback for each elem that was within range
    {
      boost::mutex::scoped_lock lock(signal_mutex_);
      for (unsigned int i=0; i<valid_elems.size(); i++)
        signal_(valid_elems[i]) ;
    }

  }

  boost::mutex signal_mutex_;
  message_filters::Connection incoming_msg_connection_;
  message_filters::Connection incoming_gate_connection_;
  Signal signal_;

  boost::mutex data_mutex_;
  std::deque<MConstPtr > queue_;
  message_filters::Cache<S>* cache_;
  ros::Duration expiration_limit_;
  unsigned int queue_size_;
  ros::Duration padding_before_;
  ros::Duration padding_after_;

} ;








}


#endif
