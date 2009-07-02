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

#ifndef MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
#define MESSAGE_FILTERS_TIME_SYNCHRONIZER_H

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/signals.hpp>
#include <boost/bind.hpp>

namespace message_filters
{

template<class M0, class M1>
class TimeSynchronizer2
{
public:
  typedef boost::shared_ptr<M0 const> M0ConstPtr;
  typedef boost::shared_ptr<M1 const> M1ConstPtr;
  typedef boost::tuple<M0ConstPtr, M1ConstPtr > Tuple;
  typedef boost::function<void(const M0ConstPtr&, const M1ConstPtr&)> Callback;
  typedef boost::signal<void(const M0ConstPtr&, const M1ConstPtr&)> Signal;

  template<class F0, class F1>
  TimeSynchronizer2(F0& f0, F1& f1, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    subscribeTo(f0, f1);
  }

  TimeSynchronizer2(uint32_t queue_size)
  : queue_size_(queue_size)
  {
  }

  ~TimeSynchronizer2()
  {
    input_connection0_.disconnect();
    input_connection1_.disconnect();
  }

  template<class F0, class F1>
  void subscribeTo(F0& f0, F1& f1)
  {
    input_connection0_.disconnect();
    input_connection1_.disconnect();

    input_connection0_ = f0.connect(boost::bind(&TimeSynchronizer2::cb0, this, _1));
    input_connection1_ = f1.connect(boost::bind(&TimeSynchronizer2::cb1, this, _1));
  }

  boost::signals::connection connect(const Callback& callback)
  {
    return signal_.connect(callback);
  }

  void add0(const M0ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<0>(t) = msg;

    checkTuple(t);
  }

  void add1(const M1ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<1>(t) = msg;

    checkTuple(t);
  }

private:
  void cb0(const M0ConstPtr& msg)
  {
    add0(msg);
  }

  void cb1(const M1ConstPtr& msg)
  {
    add1(msg);
  }

  // assumes tuples_mutex_ is already locked
  void checkTuple(Tuple& t)
  {
    if (boost::get<0>(t) && boost::get<1>(t))
    {
      {
        boost::mutex::scoped_lock lock(signal_mutex_);
        signal_(boost::get<0>(t), boost::get<1>(t));
      }

      tuples_.erase(boost::get<0>(t)->header.stamp);

      clearOldTuples();
    }

    if (tuples_.size() > queue_size_)
    {
      while (tuples_.size() > queue_size_)
      {
        tuples_.erase(tuples_.begin());
      }
    }
  }

  // assumes tuples_mutex_ is already locked
  void clearOldTuples()
  {
    typename M_TimeToTuple::iterator it = tuples_.begin();
    typename M_TimeToTuple::iterator end = tuples_.end();
    for (; it != end;)
    {
      if (it->first <= last_signal_time_)
      {
        typename M_TimeToTuple::iterator old = it;
        ++it;
        tuples_.erase(old);
      }
      else
      {
        // the map is sorted by time, so we can ignore anything after this if this one's time is ok
        break;
      }
    }
  }

  uint32_t queue_size_;

  typedef std::map<ros::Time, Tuple> M_TimeToTuple;
  M_TimeToTuple tuples_;
  boost::mutex tuples_mutex_;

  Signal signal_;
  boost::mutex signal_mutex_;
  ros::Time last_signal_time_;

  boost::signals::connection input_connection0_;
  boost::signals::connection input_connection1_;
};

}

#endif // MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
