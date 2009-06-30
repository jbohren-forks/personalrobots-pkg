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

#include "ros/ros.h"
#include "robot_msgs/PointStamped.h"
#include "robot_msgs/PointCloud.h"
#include "sensor_msgs/CamInfo.h"
#include "message_filters/topic_synchronizer_filter.h"
#include "message_filters/sync_helper.h"
#include "message_filters/consumer.h"
#include "message_filters/msg_cache.h"

using namespace std ;
using namespace message_filters ;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_test") ;

  ros::NodeHandle nh ;

  //TopicSynchronizerFilter<robot_msgs::PointStamped, robot_msgs::PointCloud> filter ;

  // Define the source 'node'
  SyncHelper<sensor_msgs::CamInfo> sync_helper("stereo/left/cam_info/", 10, nh) ;

  Consumer consumer ;
  // Link the consumer to the output of sync_helper
  consumer.subscribe(sync_helper) ;

  MsgCache<sensor_msgs::CamInfo> cache(10) ;
  cache.subscribe(sync_helper) ;

  ros::spin() ;


  return 0 ;
}
