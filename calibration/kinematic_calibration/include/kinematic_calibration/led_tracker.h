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

#include "kinematic_calibration/Capture.h"
#include "robot_msgs/MechanismState.h"
#include "image_msgs/Image.h"
#include "image_msgs/CamInfo.h"
#include "topic_synchronizer/topic_synchronizer.h"
#include "tinyxml/tinyxml.h"

#include "boost/thread.hpp"


namespace kinematic_calibration
{

class LedTracker
{
public:
  LedTracker(ros::Node* node) ;


private:

  void imageCallbackAll(ros::Time t) ;
  void imageCallbackTimeout(ros::Time t) ;
  void captureCallback() ;


  ros::Node* node_ ;

  enum CaptureState {STANDBY, CAPTURING} ;
  CaptureState capture_state_ ;
  ros::Time capture_start_time_ ;
  boost::mutex capture_lock_ ;

  MsgCacheListener<robot_msgs::MechanismState> mech_state_cache_ ;
  TopicSynchronizer<kinematic_calibration::LedTracker> sync_ ;
  StateHistory all_state_hist_ ;

  Capture capture_msg_ ;

  std::vector<image_msgs::ImagePointStamped> synced_leds_ ;
  std::map<std::string, ToleranceElem> joint_map_ ;
} ;


LedTracker::LedTracker(ros::Node* node) : node_(node),
  capture_state_(STANDBY),
  mech_state_cache_(100),  //! \todo magic #
  sync_(node_, this, &LedTracker::imageCallbackAll,
        ros::Duration().fromSec(0.05),
        &LedTracker::imageCallbackTimeout),
  all_state_hist_(100)

{
  std::string config_str ;
  bool param_exists ;
  param_exists = node_->getParam("~config", config_str) ;
  if (!param_exists)
  {
    ROS_ERROR("Cannot find parameter ~config") ;
    return ;
  }

  mech_state_cache_.subscribe(node_, "mechanism_state") ;
  node_->subscribe("~capture", capture_msg_, &LedTracker::captureCallback, this, 1) ;

  TiXmlDocument doc ;
  doc.Parse(config_str.c_str()) ;

  TiXmlElement *root = doc.FirstChildElement("tracker");
  if (!root)
  {
    ROS_ERROR("Cannot find <tracker> tag in xml") ;
    return ;
  }

  // ********** Extract Joint Tolerances **********
  TiXmlElement* joint_config ;
  joint_config = root->FirstChildElement("joint") ;
  unsigned int joint_count = 0 ;
  while (joint_config)
  {
    const char* joint_name ;
    double joint_tolerance ;
    int result ;

    joint_name = joint_config->Attribute("name") ;
    result = joint_config->QueryDoubleAttribute("tol", &joint_tolerance) ;
    if (joint_name)
    {
      switch(result)
      {
        case TIXML_SUCCESS:
        {
          ROS_INFO("Joint: %s  Tolerance: %f", joint_name, joint_tolerance) ;

          // Make sure the joint doesn't already exist
          if (joint_map_.find(joint_name) == joint_map_.end())
          {
            ToleranceElem cur_tol ;
            cur_tol.index = joint_count ;
            cur_tol.tol = joint_tolerance ;
            joint_map_.insert( std::pair<string, ToleranceElem>(joint_name, cur_tol) ) ;
            joint_count++ ;
          }
          else
          {
            ROS_ERROR("Joint named '%s' already exists!", joint_name) ;
          }
          break ;
        }
        case TIXML_WRONG_TYPE:
          ROS_ERROR("Joint: %s  Tolerance: [WRONG TYPE]", joint_name) ;
          break ;
        case TIXML_NO_ATTRIBUTE:
          ROS_ERROR("Joint: %s  Tolerance: [NOT FOUND]", joint_name) ;
          break ;
        default :
          ROS_ERROR("Unknown TinyXML Error Code") ;
          break ;
      }
    }
    else
    {
      ROS_ERROR("Found joint without a 'name' attribute") ;
    }

    joint_config = joint_config->NextSiblingElement("joint") ;
  }


  // ********** Extract LED Tracker Streams **********
  TiXmlElement* stream_config ;
  stream_config = root->FirstChildElement("stream") ;
  const char* led_topic ;

  synced_leds_.clear() ;
  synced_leds_.reserve(2) ;     // Wow this is a HUGE hack. just waiting for this to cause a horrible segfault later...
  unsigned int stream_count = 0 ;
  while(stream_config)
  {
    led_topic = stream_config->Attribute("topic") ;

    if (!led_topic)
      ROS_ERROR("Found <stream> tag without 'topic' attribute\n") ;
    else  // Found no errors
    {
      ROS_INFO("%u) LedTopic: %s", stream_count, led_topic) ;
      synced_leds_.resize(stream_count+1) ; // CAN'T Resize Here!  BIG memory bug.
      sync_.subscribe(led_topic,  synced_leds_[stream_count],  1) ;
      stream_count++ ;
    }

    stream_config = stream_config->NextSiblingElement("stream") ;
  }

  node_->advertise<CameraCalSample>("~cal_sample", 1) ;


  sync_.ready() ;

  return ;
}

void LedTracker::imageCallbackAll(ros::Time t)
{
  ROS_INFO("Callback") ;

  capture_lock_.lock() ;

  robot_msgs::MechanismState mech_state ;
  mech_state_cache_.findClosestBefore(t, mech_state) ;
  CameraCalSample sample ;
  sample.mech_state = mech_state ;
  sample.header.stamp = t ;
  sample.points = synced_leds_ ;

  all_state_hist_.insertData(sample) ;

  if (capture_state_ == CAPTURING)
  {
    if(all_state_hist_.isStable(capture_start_time_, joint_map_, 3, 15))
    {
      printf("********* STABLE!!! ****************\n") ;
      CameraCalSample* sample ;
      sample = all_state_hist_.getPastSample(7) ;
      if (!sample)
        ROS_ERROR("Grabbed NULL Sample!") ;
      else
        node_->publish("~cal_sample", *sample) ;
      capture_state_ = STANDBY ;
    }
    else
    {
      printf("waiting...\n") ;
    }
  }
  capture_lock_.unlock() ;
}

void LedTracker::captureCallback()
{
  ROS_INFO("About to start capturing") ;
  capture_lock_.lock() ;
  capture_state_ = CAPTURING ;
  capture_start_time_ = capture_msg_.header.stamp ;
  capture_lock_.unlock() ;
}

void LedTracker::imageCallbackTimeout(ros::Time t)
{
  ROS_WARN("Timeout") ;
}


}
