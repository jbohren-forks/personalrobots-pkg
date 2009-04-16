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
 *   * Neither the name of Willow Garage nor the names of its
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


/* Author: Patrick Mihelich */

#ifndef ACTION_DETECT_OUTLET_FINE_H
#define ACTION_DETECT_OUTLET_FINE_H

// ROS Stuff
#include <ros/node.h>

// Msgs
#include <robot_msgs/PointStamped.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_actions/DetectOutletState.h>

// Srvs 
#include <outlet_detection/OutletDetection.h>

// Robot Action Stuff
#include <robot_actions/action.h>


namespace plugs_core{

class DetectOutletFineAction
  : public robot_actions::Action<robot_msgs::PointStamped,
                                 robot_msgs::PoseStamped>
{
public:
  DetectOutletFineAction();

  virtual robot_actions::ResultStatus execute(const robot_msgs::PointStamped& point, robot_msgs::PoseStamped& feedback);

private:
  ros::Node* node_;

  outlet_detection::OutletDetection::Request req_;
  outlet_detection::OutletDetection::Response res_;
};

}
#endif
