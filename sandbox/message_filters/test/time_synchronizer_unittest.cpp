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

#include <gtest/gtest.h>

#include "ros/time.h"
#include "message_filters/time_synchronizer.h"

using namespace message_filters;

struct Header
{
  ros::Time stamp;
};


struct Msg
{
  Header header;
  int data;
};
typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const MsgConstPtr&, const MsgConstPtr&)
  {
    ++count_;
  }

  int32_t count_;
};

TEST(TimeSynchronizer, immediate)
{
  TimeSynchronizer2<Msg, Msg> sync(1);
  Helper h;
  sync.connect(boost::bind(&Helper::cb, &h, _1, _2));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time::now();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);
  sync.add1(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(TimeSynchronizer, multipleTimes)
{
  TimeSynchronizer2<Msg, Msg> sync(1);
  Helper h;
  sync.connect(boost::bind(&Helper::cb, &h, _1, _2));
  MsgPtr m(new Msg);
  m->header.stamp = ros::Time();

  sync.add0(m);
  ASSERT_EQ(h.count_, 0);

  m.reset(new Msg);
  m->header.stamp = ros::Time(0.1);
  sync.add1(m);
  ASSERT_EQ(h.count_, 0);
  sync.add0(m);
  ASSERT_EQ(h.count_, 1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  ros::Time::setNow(ros::Time());

  return RUN_ALL_TESTS();
}


