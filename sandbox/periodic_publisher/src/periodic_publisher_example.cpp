/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <periodic_publisher/periodic_publisher.h>
#include <std_msgs/Int32.h>

class TestPeriodicPublisher {
  public:
    TestPeriodicPublisher(periodic_publisher::PeriodicPublisher<std_msgs::Int32>& pp): pp_(pp), count_(0){
      ros::NodeHandle n;
      timer_ = n.createTimer(ros::Duration(5.0), boost::bind(&TestPeriodicPublisher::updateMessage, this));
    }

    void updateMessage(){
      std_msgs::Int32 msg;
      msg.data = count_;
      ROS_INFO("Setting message to %d", count_);
      pp_.setMessage(msg);
      count_++;
    }

  private:
    periodic_publisher::PeriodicPublisher<std_msgs::Int32>& pp_;
    int count_;
    ros::Timer timer_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "test_periodic_publisher");

  //we'll send out String messages at 1Hz for our test
  periodic_publisher::PeriodicPublisher<std_msgs::Int32> pp("test_topic", 1);

  TestPeriodicPublisher tester(pp);

  ros::spin();

}

