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

#include <vector>
#include <map>

#include "ros/node.h"
#include "kinematic_calibration/msg_cache.h"
#include "kinematic_calibration/state_history.h"
#include "kinematic_calibration/mech_state_cache.h"
#include "kinematic_calibration/image_point_cache.h"

#include "kinematic_calibration/Capture.h"
#include "robot_msgs/MechanismState.h"
#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "topic_synchronizer/topic_synchronizer.h"
#include "tinyxml/tinyxml.h"

#include "boost/thread.hpp"


namespace kinematic_calibration
{


class ImagePointHandler
{

public:
  ImagePointHandler(unsigned int N) : cache_(N)
  {

  }

  ~ImagePointHandler()
  {

  }

  void imagePointCallback()
  {
    lock_.lock() ;
    cache_.insertData(msg_) ;
    lock_.unlock() ;
  }

  image_msgs::ImagePointStamped& getMsg()
  {
    return msg_ ;
  }

  bool isStable(Interval interval, unsigned int min_samples, double pos_tolerance)
  {
    bool stable ;
    lock_.lock() ;
    stable = cache_.isStable(interval, min_samples, pos_tolerance) ;
    lock_.unlock() ;
    return stable ;
  }

  ros::Time latestTime()
  {
    lock_.lock() ;
    ros::Time latest_time = cache_.latestTime() ;
    lock_.unlock() ;
    return latest_time ;
  }

  void findClosestBefore(const ros::Time& time, image_msgs::ImagePointStamped& data_out)
  {
    lock_.lock() ;
    cache_.findClosestBefore(time, data_out) ;
    lock_.unlock() ;
  }

  std::string topic_ ;

private:

  ImagePointCache cache_ ;
  boost::mutex lock_ ;
  image_msgs::ImagePointStamped msg_ ;
} ;

class CameraCalSampler
{
public:
  CameraCalSampler(ros::Node* node) ;
  ~CameraCalSampler() ;

  void intervalCallback() ;
private:
  ros::Node* node_ ;

  MsgCacheListener<robot_msgs::MechanismState> mech_state_cache_ ;
  Interval interval_msg_ ;
  std::vector<ImagePointHandler*> stream_handlers_ ;
} ;


CameraCalSampler::CameraCalSampler(ros::Node* node) : node_(node), mech_state_cache_(100)
{
  std::string config_str ;
  bool param_exists ;
  param_exists = node_->getParam("~config", config_str) ;
  if (!param_exists)
  {
    ROS_ERROR("Cannot find parameter ~config") ;
    return ;
  }

  TiXmlDocument doc ;
  doc.Parse(config_str.c_str()) ;

  TiXmlElement *root = doc.FirstChildElement("tracker");
  if (!root)
  {
    ROS_ERROR("Cannot find <tracker> tag in xml") ;
    return ;
  }

  // ********** Extract LED Tracker Streams **********
  TiXmlElement* stream_config ;
  stream_config = root->FirstChildElement("stream") ;
  const char* led_topic ;


  stream_handlers_.clear() ;
  int i=0 ;
  while(stream_config)
  {
    led_topic = stream_config->Attribute("topic") ;

    if (!led_topic)
      ROS_ERROR("Found <stream> tag without 'topic' attribute\n") ;
    else  // Found no errors
    {
      int result ;
      int cache_size ;
      result = stream_config->QueryIntAttribute("cache_size", &cache_size) ;
      switch(result)
      {
        case TIXML_SUCCESS:
        {
          ROS_INFO("Stream - name: %s  cache_size: %u", led_topic, cache_size) ;

          ImagePointHandler* cur_handler = new ImagePointHandler(cache_size) ;
          cur_handler->topic_ = led_topic ;
          node_->subscribe(led_topic, cur_handler->getMsg(), &ImagePointHandler::imagePointCallback, cur_handler, cache_size) ;
          stream_handlers_.push_back(cur_handler) ;
          break ;
        }
        case TIXML_WRONG_TYPE:
          ROS_ERROR("Stream - topic: %s  cache_size: [WRONG TYPE]", led_topic) ;
          return ;
        case TIXML_NO_ATTRIBUTE:
          ROS_ERROR("Stream - topic: %s  cache_size: [NOT FOUND]", led_topic) ;
          return ;
        default :
          ROS_ERROR("Unknown TinyXML Error Code") ;
          return ;
      }
    }
    stream_config = stream_config->NextSiblingElement("stream") ;
    i++ ;
  }

  mech_state_cache_.subscribe(node_, "mechanism_state") ;
  node_->advertise<CameraCalSample>("~cal_sample", 1) ;


  int interval_cache_size ;
  node_->param("~interval_cache_size", interval_cache_size, 1) ;
  node_->subscribe("~capture_interval", interval_msg_, &CameraCalSampler::intervalCallback, this, interval_cache_size) ;

  return ;
}

CameraCalSampler::~CameraCalSampler()
{
  for(unsigned int i=0; i<stream_handlers_.size(); i++)
  {
    node_->unsubscribe(stream_handlers_[i]->topic_) ;
    delete stream_handlers_[i] ;
  }
}

void CameraCalSampler::intervalCallback()
{
  // Wait until camera data is available
  for(unsigned int i=0; i < stream_handlers_.size(); i++)
  {
    while (stream_handlers_[i]->latestTime() < interval_msg_.end)
      usleep(100) ;
  }

  // Make sure that the camera data is stable
  for(unsigned int i=0; i < stream_handlers_.size(); i++)
  {
    if (!stream_handlers_[i]->isStable(interval_msg_, 5, 1.01))
    {
      printf("Stream %u not stable\n", i) ;
      return ;
    }
  }

  // Populate sample
  CameraCalSample sample ;
  ros::Time grab_time ;
  grab_time.fromNSec((interval_msg_.start.toNSec()+interval_msg_.end.toNSec())/2) ;
  mech_state_cache_.findClosestBefore(grab_time, sample.mech_state) ;

  sample.set_points_size(stream_handlers_.size()) ;
  for(unsigned int i=0; i < stream_handlers_.size(); i++)
    stream_handlers_[i]->findClosestBefore(grab_time, sample.points[i]) ;

  node_->publish("~cal_sample", sample) ;
}

}
