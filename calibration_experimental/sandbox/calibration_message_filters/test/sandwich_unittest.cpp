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
#include "message_filters/cache.h"
#include "calibration_message_filters/sandwich.h"

using namespace std ;
using namespace message_filters ;
using namespace calibration_message_filters ;

struct Header
{
  ros::Time stamp ;
} ;



struct Msg
{
  Header header;
  int data;
} ;



struct GateMsg
{
  Header header;
  int junk1;
  double junk2;
} ;

typedef boost::shared_ptr<const Msg> MsgConstPtr;
typedef boost::shared_ptr<const GateMsg> GateMsgConstPtr;

MsgConstPtr buildMsg(ros::Time time, int data)
{
  boost::shared_ptr<Msg> msg_ptr(new Msg);
  msg_ptr->header.stamp = time;
  msg_ptr->data = data;
  return msg_ptr;
}

GateMsgConstPtr buildGateMsg(ros::Time time)
{
  boost::shared_ptr<GateMsg> msg_ptr(new GateMsg);
  msg_ptr->header.stamp = time;
  return msg_ptr;
}

class Helper
{
public:
  Helper()
  : count_(0)
  {}

  void cb(const SandwichElem<Msg,GateMsg>& sandwich_elem)
  {
    ++count_;
    latest_elem_ = sandwich_elem;
  }

  int32_t count_;
  SandwichElem<Msg,GateMsg> latest_elem_;
};

TEST(Sandwich, easyNoPadding)
{
  Cache<GateMsg> cache(10);
  Sandwich<Msg, GateMsg> sandwich;
  Helper h;
  //cache.connect(boost::bind(&Sandwich<Msg,GateMsg>::addToGate,  &sandwich, _1));
  sandwich.connectCacheInput(cache);
  sandwich.registerCallback(boost::bind(&Helper::cb, &h, _1));

  cache.add(buildGateMsg(ros::Time(10,0)));
  EXPECT_EQ(0, h.count_);
  cache.add(buildGateMsg(ros::Time(20,0)));
  EXPECT_EQ(0, h.count_);
  sandwich.addToQueue(buildMsg(ros::Time(15,0),1));
  EXPECT_EQ(1, h.count_);
  EXPECT_EQ((unsigned int) 2, h.latest_elem_.interval.size());
  EXPECT_EQ(h.latest_elem_.interval[0]->header.stamp, ros::Time(10,0));
  EXPECT_EQ(h.latest_elem_.interval[1]->header.stamp, ros::Time(20,0));

  sandwich.addToQueue(buildMsg(ros::Time(25,0),2));
  EXPECT_EQ(1, h.count_);
  cache.add(buildGateMsg(ros::Time(30,0)));
  EXPECT_EQ(2, h.count_);
  EXPECT_EQ((unsigned int) 2, h.latest_elem_.interval.size());
  EXPECT_EQ(h.latest_elem_.interval[0]->header.stamp, ros::Time(20,0));
  EXPECT_EQ(h.latest_elem_.interval[1]->header.stamp, ros::Time(30,0));

  sandwich.addToQueue(buildMsg(ros::Time(5,0),2));
  EXPECT_EQ(2, h.count_);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

