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

#ifndef MESSAGE_FILTERS_MSG_CACHE_H_
#define MESSAGE_FILTERS_MSG_CACHE_H_

#include <deque>
#include "boost/thread.hpp"
#include "boost/shared_ptr.hpp"
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
class MsgCache
{
public:
  typedef boost::shared_ptr<M const> MsgPtr ;

  MsgCache(unsigned int cache_size = 1)
  {
    setCacheSize(cache_size) ;
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


  template<class T>
  /**
   * Links Consumer's input to some provider's output.
   * \param provider The filter from which we want to receive data
   */
  void subscribe(T& provider)
  {
    //printf("Called MsgCache Subscribe\n") ;
    provider.addOutputCallback(boost::bind(&MsgCache::addToCache, this, _1)) ;
  }

  /**
   * Add the most recent message to the cache, and pop off any elements that are too old.
   * This method is registered with a data provider when subscribe is called.
   */
  void addToCache(const MsgPtr& msg)
  {
    //printf("  Cache Size: %u\n", cache_.size()) ;
    cache_lock_.lock() ;

    while (cache_.size() >= cache_size_)                       // Keep popping off old data until we have space for a new msg
      cache_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it

    cache_.push_back(msg) ;                                    // Add the newest message to the back of the deque
    cache_lock_.unlock() ;

    // Sequentially call each registered call
    for (unsigned int i=0; i<output_callbacks_.size(); i++)
      output_callbacks_[i]() ;
  }

  /**
   * Called by another filter that wants the output of this filter
   * \param callback The function that is called when data is available
   */
  void addOutputCallback(const boost::function<void()>& callback)
  {
    output_callbacks_.push_back(callback) ;
  }


  /**
   * Receive a vector of messages that occur between a start and end time (inclusive).
   * This call is non-blocking, and only aggregates messages it has already received.
   * It will not wait for messages have not yet been received, but occur in the interval.
   * \param start The start of the requested interval
   * \param end The end of the requested interval
   */
  std::vector<MsgPtr> getInterval(const ros::Time& start, const ros::Time& end)
  {
    cache_lock_.lock() ;

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

    std::vector<MsgPtr> interval_elems ;
    interval_elems.reserve(end_index - start_index) ;
    for (unsigned int i=start_index; i<end_index; i++)
    {
      interval_elems.push_back(cache_[i]) ;
    }

    cache_lock_.unlock() ;

    return interval_elems ;
  }

  /**
   * Grab the newest element that occurs right before the specified time.
   * \param time Time that must occur right after the returned elem
   * \returns shared_ptr to the newest elem that occurs before 'time'. NULL if doesn't exist
   */
  MsgPtr getElemBeforeTime(const ros::Time& time)
  {
    MsgPtr out ;

    cache_lock_.lock() ;
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

    cache_lock_.unlock() ;

    return out ;
  }

  /**
   * Grab the oldest element that occurs right after the specified time.
   * \param time Time that must occur right before the returned elem
   * \returns shared_ptr to the oldest elem that occurs after 'time'. NULL if doesn't exist
   */
  MsgPtr getElemAfterTime(const ros::Time& time)
  {
    MsgPtr out ;

    cache_lock_.lock() ;

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

    cache_lock_.unlock() ;

    return out ;
  }

  /**
   * Returns the timestamp associated with the newest packet cache
   */
  ros::Time getLatestTime()
  {
    ros::Time latest_time(0, 0) ;

    cache_lock_.lock() ;
    if (cache_.size() > 0)
      latest_time = cache_.back()->header.stamp ;
    cache_lock_.unlock() ;

    return latest_time ;
  }

private:
  boost::mutex cache_lock_ ;            //!< Lock for cache_
  std::deque<MsgPtr > cache_ ;          //!< Cache for the messages
  unsigned int cache_size_ ;            //!< Maximum number of elements allowed in the cache.

  //! Array of callbacks to be called whenever new data is received
  std::vector<boost::function<void()> > output_callbacks_ ;
} ;

}


#endif /* MSG_CACHE_H_ */
