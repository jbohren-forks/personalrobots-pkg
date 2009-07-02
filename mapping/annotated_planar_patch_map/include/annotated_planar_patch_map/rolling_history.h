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

//! \author Alex Sorokin (heavily borrowed from Vijay's code)
//! \author Vijay Pradeep 

#include "ros/node.h"
#include "tf/transform_listener.h"
#include "tf/message_notifier.h"


#include <deque>

// Service
#include "boost/thread.hpp"
#include "math.h"

#include "annotated_planar_patch_map/annotated_map_lib.h"

namespace annotated_planar_patch_map
{

/**
 * \brief Maintains a history of annotated maps and generates an aggregate map upon request
 * \todo Clean up the doxygen part of this header
 *
 * @section parameters ROS Parameters
 *
 * Reads the following parameters from the parameter server
 *  - \b "~tf_cache_time_secs" (double) - The cache time (in seconds) to holds past transforms
 *  - \b "~tf_tolerance_secs (double) - The time (in seconds) to wait after the transform for scan_in is available.
 *  - \b "~max_msgs" (unsigned int) - The number of msgs to store in the assembler's history, until they're thrown away
 *
 */
 template<class T>
class RollingHistory
{
public:
  RollingHistory(const std::string topic_name,const std::string config_prefix) ;
  ~RollingHistory() ;


public:
  //! \brief Store every new message in the history
  void msgCallback(const boost::shared_ptr<T const>& msg_ptr) ;


  //! \brief Get the message at specified time in history
  const boost::shared_ptr<T const> getMsgAtExactTime(ros::Time stamp) ;

  //! \brief Get the message at specified time in history within specified tolerance
  const boost::shared_ptr<T const> getMsgNearTime(ros::Time stamp,ros::Duration interval) ;

  const std::deque< boost::shared_ptr<T const> > getHistory(){return msg_hist_;}

private:
  ros::NodeHandle n_ ;

  std::string topic_name_;
  std::string config_prefix_;
  ros::Subscriber msg_sub_ ;

  //! \brief Msg history of msgs
  std::deque< boost::shared_ptr<T const> > msg_hist_ ;
  boost::mutex msg_hist_mutex_ ;

  //! \brief The max number of msgs to store in the history
  unsigned int max_msgs_ ;

};



template <class T>
RollingHistory<T>::RollingHistory(  const std::string topic_name, const std::string config_prefix)
{
  boost::mutex::scoped_lock lock( msg_hist_mutex_); 

  topic_name_=topic_name;
  config_prefix_=topic_name;


  // **** Initialize history ****
  double cache_time_secs ;
  ros::Node::instance()->param(config_prefix_+"cache_time_secs", cache_time_secs, 10.0) ;
  if (cache_time_secs < 0)
    ROS_ERROR("Parameter cache_time_secs<0 (%f)", cache_time_secs) ;


  // ***** Set max_msg *****
  const int default_max_msgs = 400 ;
  int tmp_max_msgs ;
  ros::Node::instance()->param(config_prefix_+"max_msg", tmp_max_msgs, default_max_msgs) ;
  if (tmp_max_msgs < 0)
  {
    ROS_ERROR("Parameter max_msgs<0 (%i)", tmp_max_msgs) ;
    tmp_max_msgs = default_max_msgs ;
  }

  max_msgs_ = tmp_max_msgs ;
  ROS_INFO("Max messages in History: %u", max_msgs_) ;

  msg_sub_ = n_.subscribe<T>( topic_name_, max_msgs_,&RollingHistory<T>::msgCallback, this);


}

 template <class T>
   RollingHistory<T>::~RollingHistory()
{
}


template <class T>
void RollingHistory<T>::msgCallback(const boost::shared_ptr<T const>& msg_ptr)
{
  ROS_DEBUG("got msg") ;

  // Add the current msg into our history
  boost::mutex::scoped_lock lock( msg_hist_mutex_);

  if (msg_hist_.size() == max_msgs_)                           // Is our deque full?
  {
    msg_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
  }
  msg_hist_.push_back(msg_ptr) ;                              // Add the newest msg to the back of the deque

}

template <class T>
const boost::shared_ptr<T const> RollingHistory<T>::getMsgAtExactTime(ros::Time stamp)
{
  boost::mutex::scoped_lock lock( msg_hist_mutex_); 

  // Determine where in our history we actually are
  unsigned int i = 0 ;

  // Find the beginning of the request. Probably should be a search
  while ( i < msg_hist_.size() &&                                                    // Don't go past end of deque
          msg_hist_[i]->header.stamp < stamp )                                    // Keep stepping until we've exceeded the start time
  {
    ROS_DEBUG_STREAM("\tPrev message " << msg_hist_[i]->header.stamp);
    i++ ;
  }
  ROS_DEBUG_STREAM("i="<<i);
  if(i>0)
    ROS_DEBUG_STREAM( "Last msg before the interval\t" << msg_hist_[i-1]->header.stamp<<"\n" );
  else
    ROS_DEBUG_STREAM( "No msg before the interval\n");

  if(i >= msg_hist_.size() )
  {
    return boost::shared_ptr<T const>();
  }

  if( msg_hist_[i]->header.stamp != stamp)
  {
    return boost::shared_ptr<T const>();
  }

  return msg_hist_[i];

}


template <class T>
const boost::shared_ptr<T const> RollingHistory<T>::getMsgNearTime(ros::Time stamp,ros::Duration interval)
{
  boost::mutex::scoped_lock lock( msg_hist_mutex_); 

  // Determine where in our history we actually are
  unsigned int i = 0 ;

  // Find the beginning of the request. Probably should be a search
  while ( i < msg_hist_.size() &&                                                    // Don't go past end of deque
          msg_hist_[i]->header.stamp < stamp )                                    // Keep stepping until we've exceeded the start time
  {
    ROS_INFO_STREAM("\tPrev message " << msg_hist_[i]->header.stamp);
    i++ ;
  }

  ros::Duration d0(10000.0),d1(10000.0);

  if(i>0)
    d0=stamp-msg_hist_[i-1]->header.stamp;

  if(i<msg_hist_.size())
    d1=msg_hist_[i]->header.stamp-stamp;

  if(d0<d1 && d0<interval)
    return msg_hist_[i-1];

  if(d1<=d0 && d1<interval)
    return msg_hist_[i];

  return boost::shared_ptr<T const>();

}
}
