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

#include <deque>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include "ros/time.h"
#include "ros/console.h"

#ifndef PR2_CALIBRATION_ACTIONS_SORTED_DEQUE_H_
#define PR2_CALIBRATION_ACTIONS_SORTED_DEQUE_H_

namespace pr2_calibration_actions
{

template <class M>
class SortedDeque : public std::deque<M>
{
public:

  using std::deque<M>::size;
  using std::deque<M>::front;
  using std::deque<M>::pop_front;
  using std::deque<M>::begin;
  using std::deque<M>::end;
  using std::deque<M>::rbegin;
  using std::deque<M>::rend;
  using std::deque<M>::insert;
  using std::deque<M>::at;
  using std::deque<M>::erase;

  SortedDeque(std::string logger = "deque") : std::deque<M>(), logger_(logger)
  {
    getStamp = &SortedDeque<M>::getStructStamp;
  }

  SortedDeque(boost::function<const ros::Time&(const M&)> getStampFunc,
              std::string logger = "deque") : std::deque<M>(), logger_(logger)
  {
    getStamp = getStampFunc;
  }

  //! Max size of 0 implies infinite
  void setMaxSize(unsigned int max_size)
  {
    max_size_ = max_size ;
  }

  void add(const M& msg)
  {
    ROS_DEBUG("Called add()");
    ROS_DEBUG_STATS("   ");
    if (max_size_ != 0)
    {
      while (size() >= max_size_)                // Keep popping off old data until we have space for a new msg
      {
        pop_front() ;                            // The front of the deque has the oldest elem, so we can get rid of it
        ROS_DEBUG("   Popping element");
        ROS_DEBUG_STATS("   ");
      }
    }

    typename std::deque<M>::reverse_iterator rev_it = rbegin();

    // Keep walking backwards along deque until we hit the beginning,
    //   or until we find a timestamp that's smaller than (or equal to) msg's timestamp
    while(rev_it != rend() && getStamp(*rev_it) > getStamp(msg))
      rev_it++;

    // Add msg to the cache
    insert(rev_it.base(), msg);
    ROS_DEBUG("   Done inserting");
    ROS_DEBUG_STATS("   ");
  }

  std::vector<M> getInterval(const ros::Time& start, const ros::Time& end)
  {
    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = 0 ;
    while(start_index < size() &&
          getStamp(at(start_index)) < start)
    {
      start_index++ ;
    }

    // Find the ending index. (Find the first index after the end of interval)
    unsigned int end_index = start_index ;
    while(end_index < size() &&
          getStamp(at(end_index)) <= end)
    {
      end_index++ ;
    }

    std::vector<M> interval_elems ;
    interval_elems.reserve(end_index - start_index) ;
    for (unsigned int i=start_index; i<end_index; i++)
    {
      interval_elems.push_back(at(i)) ;
    }

    return interval_elems ;
  }

  /**
   * Retrieve the smallest interval of messages that surrounds an interval from start to end.
   * If the messages in the cache do not surround (start,end), then this will return the interval
   * that gets closest to surrounding (start,end)
   */
  std::vector<M> getSurroundingInterval(const ros::Time& start, const ros::Time& end)
  {
    // Find the starting index. (Find the first index after [or at] the start of the interval)
    unsigned int start_index = size()-1;
    while(start_index > 0 &&
          getStamp(at(start_index)) > start)
    {
      start_index--;
    }
    unsigned int end_index = start_index;
    while(end_index < size()-1 &&
          getStamp(at(end_index)) < end)
    {
      end_index++;
    }

    std::vector<M> interval_elems;
    interval_elems.reserve(end_index - start_index + 1) ;
    for (unsigned int i=start_index; i<=end_index; i++)
    {
      interval_elems.push_back(at(i)) ;
    }

    return interval_elems;
  }

  /**
   * Grab the newest element that occurs right before the specified time.
   */
  bool getElemBeforeTime(const ros::Time& time, M& out)
  {
    unsigned int i=0 ;
    int elem_index = -1 ;
    while (i<size() &&
           getStamp(at(i)) < time)
    {
      elem_index = i ;
      i++ ;
    }

    if (elem_index >= 0)
    {
      out = at(elem_index);
      return true;
    }
    //out = M();
    return false;
  }

  /**
   * Grab the oldest element that occurs right after the specified time.
   */
  bool getElemAfterTime(const ros::Time& time, M& out)
  {
    int i=size()-1 ;
    int elem_index = -1 ;
    while (i>=0 &&
           getStamp(at(i)) > time)
    {
      elem_index = i ;
      i-- ;
    }

    if (elem_index >= 0)
    {
      out = at(elem_index);
      return true;
    }
    //out = M();
    return false;
  }

  bool getClosestElem(const ros::Time& time, M& out)
  {
    if (size() == 0)
      return false;

    typename std::deque<M>::iterator it = begin();
    typename std::deque<M>::iterator best = it;

    double best_diff = fabs( (time - getStamp(*best)).toSec());

    while (it != end())
    {
      double cur_diff = fabs( (time - getStamp(*it)).toSec());
      if (cur_diff < best_diff)
      {
        best_diff = cur_diff;
        best = it;
      }
      ++it;
    }
    out = *best;
    return true;
  }


  void removeAllBeforeTime(const ros::Time& time)
  {
    ROS_DEBUG("Called removeAllBeforeTime()");
    ROS_DEBUG("   Erasing all elems before time: %u %u", time.sec, time.nsec);
    typename std::deque<M>::iterator it = begin();

    while (size() > 0 && getStamp(front()) < time)
    {
      ROS_DEBUG("   Erasing elem at time: %u, %u", getStamp(front()).sec, getStamp(front()).nsec);
      pop_front();
      ROS_DEBUG("   Erased an elem");
      ROS_DEBUG_STATS("   ");
    }
    ROS_DEBUG("   Done erasing elems");
  }

  static const ros::Time& getPtrStamp(const M& m)
  {
    return m->header.stamp;
  }

  static const ros::Time& getStructStamp(const M& m)
  {
    return m.header.stamp;
  }

  static const ros::Time& getHeaderStamp(const M& m)
  {
    return m.stamp;
  }
private:
  unsigned int max_size_;
  std::string logger_;


  /*const ros::Time& getStamp(const M& m)
  {
    return getStructStamp(m);
  }*/

  boost::function<const ros::Time&(const M&)> getStamp;

  inline void ROS_DEBUG_STATS(const std::string& prefix)
  {
    ROS_DEBUG("%sdeque.size(): %u   max_size: %u", prefix.c_str(), size(), max_size_);
  }
};

}

#endif
