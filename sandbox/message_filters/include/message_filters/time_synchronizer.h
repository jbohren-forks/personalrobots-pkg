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
#include <boost/type_traits/is_same.hpp>
#include <boost/noncopyable.hpp>

#include <roslib/Header.h>

#include "connection.h"

#define TIME_SYNCHRONIZER_MAX_MESSAGES 9

namespace message_filters
{

class NullType
{
public:
  roslib::Header header;
};
typedef boost::shared_ptr<NullType const> NullTypeConstPtr;

template<class M>
class NullFilter
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef boost::function<void(const MConstPtr&)> Callback;
  Connection connect(const Callback& cb)
  {
    return Connection();
  }
};

template<class M0, class M1, class M2 = NullType, class M3 = NullType, class M4 = NullType,
         class M5 = NullType, class M6 = NullType, class M7 = NullType, class M8 = NullType>
class TimeSynchronizer : public boost::noncopyable
{
public:
  typedef boost::shared_ptr<M0 const> M0ConstPtr;
  typedef boost::shared_ptr<M1 const> M1ConstPtr;
  typedef boost::shared_ptr<M2 const> M2ConstPtr;
  typedef boost::shared_ptr<M3 const> M3ConstPtr;
  typedef boost::shared_ptr<M4 const> M4ConstPtr;
  typedef boost::shared_ptr<M5 const> M5ConstPtr;
  typedef boost::shared_ptr<M6 const> M6ConstPtr;
  typedef boost::shared_ptr<M7 const> M7ConstPtr;
  typedef boost::shared_ptr<M8 const> M8ConstPtr;
  typedef boost::tuple<M0ConstPtr, M1ConstPtr, M2ConstPtr, M3ConstPtr, M4ConstPtr, M5ConstPtr, M6ConstPtr, M7ConstPtr, M8ConstPtr> Tuple;
  typedef boost::signal<void(const M0ConstPtr&, const M1ConstPtr&, const M2ConstPtr&, const M3ConstPtr&, const M4ConstPtr&, const M5ConstPtr&, const M6ConstPtr&, const M7ConstPtr&, const M8ConstPtr&)> Signal;

  template<class F0, class F1>
  TimeSynchronizer(F0& f0, F1& f1, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1);
  }

  template<class F0, class F1, class F2>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2);
  }

  template<class F0, class F1, class F2, class F3>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2, f3);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2, f3, f4);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2, f3, f4, f5);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2, f3, f4, f5, f6);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2, f3, f4, f5, f6, f7);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  TimeSynchronizer(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8, uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
    connectTo(f0, f1, f2, f3, f4, f5, f6, f7, f8);
  }

  TimeSynchronizer(uint32_t queue_size)
  : queue_size_(queue_size)
  {
    determineRealTypeCount();
  }

  ~TimeSynchronizer()
  {
    disconnectAll();
  }

  template<class F0, class F1>
  void connectTo(F0& f0, F1& f1)
  {
    NullFilter<M2> f2;
    connectTo(f0, f1, f2);
  }

  template<class F0, class F1, class F2>
  void connectTo(F0& f0, F1& f1, F2& f2)
  {
    NullFilter<M3> f3;
    connectTo(f0, f1, f2, f3);
  }

  template<class F0, class F1, class F2, class F3>
  void connectTo(F0& f0, F1& f1, F2& f2, F3& f3)
  {
    NullFilter<M4> f4;
    connectTo(f0, f1, f2, f3, f4);
  }

  template<class F0, class F1, class F2, class F3, class F4>
  void connectTo(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4)
  {
    NullFilter<M5> f5;
    connectTo(f0, f1, f2, f3, f4, f5);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5>
  void connectTo(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5)
  {
    NullFilter<M6> f6;
    connectTo(f0, f1, f2, f3, f4, f5, f6);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6>
  void connectTo(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6)
  {
    NullFilter<M7> f7;
    connectTo(f0, f1, f2, f3, f4, f5, f6, f7);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7>
  void connectTo(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7)
  {
    NullFilter<M8> f8;
    connectTo(f0, f1, f2, f3, f4, f5, f6, f7, f8);
  }

  template<class F0, class F1, class F2, class F3, class F4, class F5, class F6, class F7, class F8>
  void connectTo(F0& f0, F1& f1, F2& f2, F3& f3, F4& f4, F5& f5, F6& f6, F7& f7, F8& f8)
  {
    disconnectAll();

    input_connections_[0] = f0.connect(boost::bind(&TimeSynchronizer::cb0, this, _1));
    input_connections_[1] = f1.connect(boost::bind(&TimeSynchronizer::cb1, this, _1));
    input_connections_[2] = f2.connect(boost::bind(&TimeSynchronizer::cb2, this, _1));
    input_connections_[3] = f3.connect(boost::bind(&TimeSynchronizer::cb3, this, _1));
    input_connections_[4] = f4.connect(boost::bind(&TimeSynchronizer::cb4, this, _1));
    input_connections_[5] = f5.connect(boost::bind(&TimeSynchronizer::cb5, this, _1));
    input_connections_[6] = f6.connect(boost::bind(&TimeSynchronizer::cb6, this, _1));
    input_connections_[7] = f7.connect(boost::bind(&TimeSynchronizer::cb7, this, _1));
    input_connections_[8] = f8.connect(boost::bind(&TimeSynchronizer::cb8, this, _1));
  }

  template<class C>
  Connection connect(const C& callback)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    return Connection(boost::bind(&TimeSynchronizer::disconnect, this, _1), signal_.connect(boost::bind(callback, _1, _2, _3, _4, _5, _6, _7, _8, _9)));
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

  void add2(const M2ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<2>(t) = msg;

    checkTuple(t);
  }

  void add3(const M3ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<3>(t) = msg;

    checkTuple(t);
  }

  void add4(const M4ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<4>(t) = msg;

    checkTuple(t);
  }

  void add5(const M5ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<5>(t) = msg;

    checkTuple(t);
  }

  void add6(const M6ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<6>(t) = msg;

    checkTuple(t);
  }

  void add7(const M7ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<7>(t) = msg;

    checkTuple(t);
  }

  void add8(const M8ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(tuples_mutex_);

    Tuple& t = tuples_[msg->header.stamp];
    boost::get<8>(t) = msg;

    checkTuple(t);
  }

private:

  void disconnectAll()
  {
    for (int i = 0; i < TIME_SYNCHRONIZER_MAX_MESSAGES; ++i)
    {
      input_connections_[i].disconnect();
    }
  }

  void determineRealTypeCount()
  {
    real_type_count_ = 2;

    if (!boost::is_same<M2, NullType>::value)
    {
      ++real_type_count_;
    }
    else
    {
      if (!boost::is_same<M3, NullType>::value)
      {
        ++real_type_count_;
      }
      else
      {
        if (!boost::is_same<M4, NullType>::value)
        {
          ++real_type_count_;
        }
        else
        {
          if (!boost::is_same<M5, NullType>::value)
          {
            ++real_type_count_;
          }
          else
          {
            if (!boost::is_same<M6, NullType>::value)
            {
              ++real_type_count_;
            }
            else
            {
              if (!boost::is_same<M7, NullType>::value)
              {
                ++real_type_count_;
              }
              else
              {
                if (!boost::is_same<M8, NullType>::value)
                {
                  ++real_type_count_;
                }
                else
                {
                }
              }
            }
          }
        }
      }
    }
  }

  void cb0(const M0ConstPtr& msg)
  {
    add0(msg);
  }

  void cb1(const M1ConstPtr& msg)
  {
    add1(msg);
  }

  void cb2(const M2ConstPtr& msg)
  {
    add2(msg);
  }

  void cb3(const M3ConstPtr& msg)
  {
    add3(msg);
  }

  void cb4(const M4ConstPtr& msg)
  {
    add4(msg);
  }

  void cb5(const M5ConstPtr& msg)
  {
    add5(msg);
  }

  void cb6(const M6ConstPtr& msg)
  {
    add6(msg);
  }

  void cb7(const M7ConstPtr& msg)
  {
    add7(msg);
  }

  void cb8(const M8ConstPtr& msg)
  {
    add8(msg);
  }

  // assumes tuples_mutex_ is already locked
  void checkTuple(Tuple& t)
  {
    bool full = true;
    full &= (bool)boost::get<0>(t);
    full &= (bool)boost::get<1>(t);
    full &= real_type_count_ > 2 ? (bool)boost::get<2>(t) : true;
    full &= real_type_count_ > 3 ? (bool)boost::get<3>(t) : true;
    full &= real_type_count_ > 4 ? (bool)boost::get<4>(t) : true;
    full &= real_type_count_ > 5 ? (bool)boost::get<5>(t) : true;
    full &= real_type_count_ > 6 ? (bool)boost::get<6>(t) : true;
    full &= real_type_count_ > 7 ? (bool)boost::get<7>(t) : true;
    full &= real_type_count_ > 8 ? (bool)boost::get<8>(t) : true;

    if (full)
    {
      {
        boost::mutex::scoped_lock lock(signal_mutex_);
        signal_(boost::get<0>(t), boost::get<1>(t), boost::get<2>(t), boost::get<3>(t), boost::get<4>(t), boost::get<5>(t), boost::get<6>(t), boost::get<7>(t), boost::get<8>(t));

        last_signal_time_ = boost::get<0>(t)->header.stamp;
      }

      tuples_.erase(last_signal_time_);

      clearOldTuples();
    }

    if (queue_size_ > 0)
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

  void disconnect(const Connection& c)
  {
    boost::mutex::scoped_lock lock(signal_mutex_);
    signal_.disconnect(c.getBoostConnection());
  }

  uint32_t queue_size_;

  typedef std::map<ros::Time, Tuple> M_TimeToTuple;
  M_TimeToTuple tuples_;
  boost::mutex tuples_mutex_;

  Signal signal_;
  boost::mutex signal_mutex_;
  ros::Time last_signal_time_;

  Connection input_connections_[TIME_SYNCHRONIZER_MAX_MESSAGES];

  uint32_t real_type_count_;
};

}

#endif // MESSAGE_FILTERS_TIME_SYNCHRONIZER_H
