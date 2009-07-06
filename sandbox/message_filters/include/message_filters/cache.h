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

/*! \mainpage
 *  \htmlinclude manifest.html
 */

#ifndef MESSAGE_FILTERS_CACHE_H_
#define MESSAGE_FILTERS_CACHE_H_

#include <deque>
#include "boost/thread.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/signals.hpp>
#include <boost/noncopyable.hpp>
#include "ros/time.h"

namespace message_filters
{

/**
 * \brief Stores a time history of messages
 * Given a stream of messages, the most recent N messages are cached in a ring buffer,
 * from which time intervals of the cache can then be retrieved by the client. This assumes
 * that the messages are being received with monotonically increasing timestamps in header.stamp.
 */
template<class M>
class Cache : public boost::noncopyable
{
public:
  typedef boost::shared_ptr<M const> MConstPtr ;
  typedef boost::function<void(const MConstPtr&)> Callback;
  typedef boost::signal<void(const MConstPtr&)> Signal;

  template<class A>
  Cache(A& a, unsigned int cache_size = 1)
  {
    setCacheSize(cache_size) ;
    connectTo(a) ;
  }

  /**
   * Initializes a Messsage Cache without specifying a parent filter. This implies that in
   * order to populate the cache, the user then has to call add themselves
   */
  Cache(unsigned int cache_size = 1)
  {
    setCacheSize(cache_size);
  }

  template<class A>
  void connectTo(A& a)
  {
    incoming_connection_ = a.connect(boost::bind(&Cache::add, this, _1));
  }

  ~Cache()
  {
    incoming_connection_.disconnect();
  }

  /**
   * Set the size of the cache.
   * \param cache_size The new size the cache should be. Must be > 0
   */
  void setCacheSize(unsigned int cache_size)
  {
    if (cache_size == 0)
    {
      //ROS_ERROR("Cannot set max_size to 0") ;
      return ;
    }

    cache_size_ = cache_size ;
  }

  boost::signals::connection connect(const Callback& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return signal_.connect(callback);
  }

  /**
   * Add the most recent message to the cache, and pop off any elements that are too old.
   * This method is registered with a data provider when connectTo is called.
   */
  void add(const MConstPtr& msg)
  {
    //printf("  Cache Size: %u\n", cache_.size()) ;
    {
      boost::mutex::scoped_lock lock(cache_lock_);

      while (cache_.size() >= cache_size_)                       // Keep popping off old data until we have space for a new msg
        cache_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it

      cache_.push_back(msg) ;                                    // Add the newest message to the back of the deque
    }

    {
      boost::mutex::scoped_lock lock(signal_mutex_);
      // Sequentially call each registered call
      signal_(msg);
    }
  }

  /**
   * Receive a vector of messages that occur between a start and end time (inclusive).
   * This call is non-blocking, and only aggregates messages it has already received.
   * It will not wait for messages have not yet been received, but occur in the interval.
   * \param start The start of the requested interval
   * \param end The end of the requested interval
   */
  std::vector<MConstPtr> getInterval(const ros::Time& start, const ros::Time& end)
  {
    boost::mutex::scoped_lock lock(cache_lock_);

    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = 0 ;
    while(start_index < cache_.size() &&
          cache_[start_index]->header.stamp < start)
    {
      start_index++ ;
    }

    // Find the ending index. (Find the first index after the end of interval)
    unsigned int end_index = start_index ;
    while(end_index < cache_.size() &&
          cache_[end_index]->header.stamp <= end)
    {
      end_index++ ;
    }

    std::vector<MConstPtr> interval_elems ;
    interval_elems.reserve(end_index - start_index) ;
    for (unsigned int i=start_index; i<end_index; i++)
    {
      interval_elems.push_back(cache_[i]) ;
    }

    return interval_elems ;
  }

  /**
   * Grab the newest element that occurs right before the specified time.
   * \param time Time that must occur right after the returned elem
   * \returns shared_ptr to the newest elem that occurs before 'time'. NULL if doesn't exist
   */
  MConstPtr getElemBeforeTime(const ros::Time& time)
  {
    boost::mutex::scoped_lock lock(cache_lock_);

    MConstPtr out ;

    unsigned int i=0 ;
    int elem_index = -1 ;
    while (i<cache_.size() &&
           cache_[i]->header.stamp < time)
    {
      elem_index = i ;
      i++ ;
    }

    if (elem_index >= 0)
      out = cache_[elem_index] ;

    return out ;
  }

  /**
   * Grab the oldest element that occurs right after the specified time.
   * \param time Time that must occur right before the returned elem
   * \returns shared_ptr to the oldest elem that occurs after 'time'. NULL if doesn't exist
   */
  MConstPtr getElemAfterTime(const ros::Time& time)
  {
    boost::mutex::scoped_lock lock(cache_lock_);

    MConstPtr out ;

    int i=cache_.size()-1 ;
    int elem_index = -1 ;
    while (i>=0 &&
           cache_[i]->header.stamp > time)
    {
      elem_index = i ;
      i-- ;
    }

    if (elem_index >= 0)
      out = cache_[elem_index] ;
    else
      out.reset() ;

    return out ;
  }

  /**
   * Returns the timestamp associated with the newest packet cache
   */
  ros::Time getLatestTime()
  {
    boost::mutex::scoped_lock lock(cache_lock_);

    ros::Time latest_time(0, 0) ;

    if (cache_.size() > 0)
      latest_time = cache_.back()->header.stamp ;

    return latest_time ;
  }

private:
  boost::mutex cache_lock_ ;            //!< Lock for cache_
  std::deque<MConstPtr > cache_ ;       //!< Cache for the messages
  unsigned int cache_size_ ;            //!< Maximum number of elements allowed in the cache.

  boost::signals::connection incoming_connection_;
  Signal signal_;
  boost::mutex signal_mutex_;
};

}


#endif /* MESSAGE_FILTERS_CACHE_H_ */
