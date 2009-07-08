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


//include <labeled_object_detector/planar_object_detector_action.h>
#include <labeled_object_detector/PoseStampedState.h>
#include <robot_actions/action_client.h>
#include <robot_actions/action.h>
#include <ros/ros.h>
#include <robot_msgs/PointStamped.h>
#include <gtest/gtest.h>

using namespace robot_msgs;
using namespace labeled_object_detector;

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "action_test");

  robot_actions::ActionClient<PoseStamped, PoseStampedState, PoseStamped> client("detect_nearest_object");
  ros::Duration duration(10);
  duration.sleep();


  PoseStamped goal;
  PoseStamped feedback;
  // Start the action; it must finish in 1 second

  goal.header.stamp.sec=1246990627;
  goal.header.stamp.nsec=471105000;
  goal.header.frame_id=std::string("map");
  goal.pose.position.x=0;
  goal.pose.position.y=0;
  goal.pose.position.z=0;
  goal.pose.orientation.x=0;
  goal.pose.orientation.y=0;
  goal.pose.orientation.z=0;
  goal.pose.orientation.w=1;

  robot_actions::ResultStatus result = client.execute(goal, feedback, ros::Duration(10));
  //ASSERT_EQ(result, robot_actions::PREEMPTED);

  // Start the action; it must finish in 0.0001 second
  /*result = client.execute(g, f, ros::Duration().fromSec(0.0001));
  ASSERT_EQ(result, robot_actions::PREEMPTED);
  */

  return (0);
}
/* ]--- */

