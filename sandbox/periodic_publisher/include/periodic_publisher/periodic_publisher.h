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
#ifndef PERIODIC_PUBLISHER
#define PERIODIC_PUBLISHER

#include <ros/ros.h>
#include <string>
#include <boost/thread.hpp> 

namespace periodic_publisher {

  template <class MessageType> class PeriodicPublisher {
    public:
      PeriodicPublisher(std::string topic, double frequency) 
        : topic_(topic), frequency_(frequency), publish_thread_(NULL){
          publish_thread_ = new boost::thread(boost::bind(&PeriodicPublisher::publishLoop, this));
        }

      ~PeriodicPublisher(){
        if(publish_thread_ != NULL){
          publish_thread_->join();
          delete publish_thread_;
        }
      }

      void setMessage(MessageType msg){
        msg_.lock();
        ROS_DEBUG("Updating message");
        msg_ = msg;
        msg_.unlock();
      }

    private:
      void publishLoop(){
        ros::NodeHandle n;
        ros::Rate r(frequency_);
        ros::Publisher pub = n.advertise<MessageType>(topic_, 1);
        while(n.ok()){
          msg_.lock();
          pub.publish(msg_);
          msg_.unlock();
          r.sleep();
        }
      }

      std::string topic_;
      double frequency_;
      MessageType msg_;
      boost::thread* publish_thread_;
  };
};
#endif
