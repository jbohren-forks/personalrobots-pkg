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
#include "action_tools/action_client.h"
#include "action_tools/MoveBaseGoal.h"
#include "action_tools/MoveBaseResult.h"
#include "action_tools/GoalStatus.h"
#include "robot_msgs/PoseStamped.h"

using namespace action_tools;
using namespace robot_msgs;

typedef ActionClient<MoveBaseGoal, PoseStamped, MoveBaseResult, PoseStamped> MoveBaseClient;

void callback(const TerminalStatuses::TerminalStatus& status, const boost::shared_ptr<const PoseStamped>& result)
{
  ROS_INFO("In ActionClient Callback");
  switch (status)
  {
    case TerminalStatuses::REJECTED:
      ROS_INFO("REJECTED"); break;
    case TerminalStatuses::PREEMPTED:
      ROS_INFO("PREEMPTED"); break;
    case TerminalStatuses::SUCCEEDED:
      ROS_INFO("SUCCEEDED"); break;
    case TerminalStatuses::ABORTED:
      ROS_INFO("ABORTED"); break;
    case TerminalStatuses::TIMED_OUT:
      ROS_INFO("TIMED_OUT"); break;
    case TerminalStatuses::LOST:
      ROS_INFO("LOST"); break;
    case TerminalStatuses::IGNORED:
      ROS_INFO("IGNORED"); break;
    default:
      ROS_INFO("BAD STATUS"); break;
  }

  if (result)
  {
    ROS_INFO("Got Result: [xyz]=(%5.2f, %5.2f, %5.2f)",
             result->pose.position.x,
             result->pose.position.y,
             result->pose.position.z);
  }
}

void spinThread()
{
  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_action_client");

  ros::NodeHandle n;

  boost::thread spinthread = boost::thread(boost::bind(&spinThread)) ;

  ros::Duration sleep_duration(2,0);

  sleep_duration.sleep();
  MoveBaseClient ac("/test_action/move_base", n, true);
  sleep_duration = ros::Duration(10,0);
  sleep_duration.sleep();

  PoseStamped goal_pose;
  goal_pose.pose.position.x = 200;

  ros::Duration runtime_timeout(10,0);
  ros::Duration ack_timeout(3,0);

  ROS_INFO("Call Execute #1");
  //ac.execute(goal_pose, &callback);
  ac.execute(goal_pose);

  sleep(10);

  goal_pose.pose.position.x = 50;
  goal_pose.pose.position.y = 50;

  ROS_INFO("Call Execute #2");
  ac.execute(goal_pose, &callback, runtime_timeout, ack_timeout);

  ROS_INFO("Waiting until done");
  PoseStampedConstPtr result_ptr;
  ac.waitUntilDone(result_ptr);
  ROS_INFO("Done waiting");

  sleep(3);
  goal_pose.pose.position.x = 25;
  goal_pose.pose.position.y = 25;
  ros::Duration preempt_timeout(3,0);

  ROS_INFO("Call Execute #3");
  ac.execute(goal_pose, &callback, runtime_timeout, ack_timeout, preempt_timeout);

  while(n.ok())
    sleep(1);

  return 0;
}
