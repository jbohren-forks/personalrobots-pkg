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

//! \author Vijay Pradeep

#ifndef KINEMATIC_CALIBRATION_MSG_CACHE_H_
#define KINEMATIC_CALIBRATION_MSG_CACHE_H_

#include <deque>
#include <stdexcept>
#include <assert.h>

#include "ros/node.h"
#include "ros/time.h"

#include "boost/thread.hpp"

using namespace std ;

namespace kinematic_calibration
{

/**
 * \brief Stores a history of message series (that contain ros Headers) and performs time-lookup operations on it
 **/
template<class T>
class MsgCache
{
public:
  /**
   * \param N max number of elements to store in history
   */
  MsgCache(unsigned int N) ;
  void insertData(const T &data) ;

  void findClosestBefore(const ros::Time& time, T& data_out) ;
  void findClosestAfter (const ros::Time& time, T& data_out) ;

protected:
  std::deque<T> storage_ ;
  unsigned int max_elems_ ;

};

/**
 * Threadsafe, ros-ified version of MsgCache.
 */
template <class T>
class MsgCacheListener
{
public:
  MsgCacheListener(unsigned int N) ;
  void subscribe(ros::Node* node, const std::string& topic) ;
  void findClosestBefore(const ros::Time& time, T& data_out) ;
  void findClosestAfter(const ros::Time& time, T& data_out) ;

private:
  void msgCallback() ;
  MsgCache<T> cache_ ;
  T msg_in_ ;
  boost::mutex cache_lock_ ;
};


template<class T>
MsgCache<T>::MsgCache(unsigned int N) : max_elems_(N)
{

}

template<class T>
void MsgCache<T>::insertData(const T& data)
{
  if (storage_.size() > 0)
  {
    if (storage_.back().header.stamp > data.header.stamp)
      throw std::runtime_error("WARNING: Time history is not monotonic.  Weird stuff will definitely happen") ;
  }

  assert(storage_.size() <= max_elems_) ;

  if (storage_.size() == max_elems_)
    storage_.pop_front() ;

  storage_.push_back(data) ;
}

template<class T>
void MsgCache<T>::findClosestBefore(const ros::Time& time, T& data_out)
{
  typename deque<T>::reverse_iterator it ;
  it = storage_.rbegin() ;
  while( it < storage_.rend() )
  {
    if (it->header.stamp < time)
    {
      data_out = *it ;
      return ;
    }
    ++it ;
  }
  throw std::runtime_error("ERROR: All elements in cache occur after the requested time") ;
}

template<class T>
void MsgCache<T>::findClosestAfter(const ros::Time& time, T& data_out)
{
  typename deque<T>::iterator it ;
  it = storage_.begin() ;
  while( it < storage_.end() )
  {
    if (it->header.stamp > time)
    {
      data_out = *it ;
      return ;
    }
    ++it ;
  }
  throw std::runtime_error("ERROR: All elements in cache occur before the requested time") ;
}


//******** Message Cache Listener **********

template<class T>
MsgCacheListener<T>::MsgCacheListener(unsigned int N) : cache_(N)
{

}

template<class T>
void MsgCacheListener<T>::subscribe(ros::Node* node, const std::string& topic)
{
  node->subscribe(topic, msg_in_, &MsgCacheListener<T>::msgCallback, this, 100) ; //! \todo Magic number
}

template<class T>
void MsgCacheListener<T>::msgCallback()
{
  cache_lock_.lock() ;
  cache_.insertData(msg_in_) ;
  cache_lock_.unlock() ;
}

template<class T>
void MsgCacheListener<T>::findClosestBefore(const ros::Time& time, T& data_out)
{
  cache_lock_.lock() ;
  cache_.findClosestBefore(time, data_out) ;
  cache_lock_.unlock() ;
}

template<class T>
void MsgCacheListener<T>::findClosestAfter(const ros::Time& time, T& data_out)
{
  cache_lock_.lock() ;
  cache_.findClosestAfter(time, data_out) ;
  cache_lock_.unlock() ;
}

}




#endif // KINEMATIC_CALIBRATION_MSG_CACHE_H_
