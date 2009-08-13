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

#include "move_base/MoveBaseAction.h"
#include "actionlib/client/action_client.h"

#include "boost/thread.hpp"

using namespace actionlib;
using namespace geometry_msgs;
using namespace move_base;

typedef ActionClient<MoveBaseAction> MoveBaseClient;

void transitionCallback(ActionClient<MoveBaseAction>::GoalHandle gh)
{
  ROS_DEBUG("In the transition");

  ROS_DEBUG("We have transitioned to: [%s]", gh.getCommState().toString().c_str());

  if (gh.getResult())
    ROS_DEBUG("Got a Result!");
  else
    ROS_DEBUG("NULL Result");
}

void feedbackCallback(ActionClient<MoveBaseAction>::GoalHandle gh, const MoveBaseFeedbackConstPtr& fb)
{
  ROS_INFO("Got Feedback!");
}


void spinThread(ros::NodeHandle* n)
{
  // I can't figure out how to cleanly exit ros::spin(), so I made this hack instead
  while(n->ok())
  {
    ros::spinOnce();
    usleep(10);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_action_client");

  ros::NodeHandle n;

  boost::thread spinthread = boost::thread(boost::bind(&spinThread, &n)) ;

  MoveBaseClient ac("move_base");

  ROS_INFO("Waiting for action server to start");
  if (ac.waitForActionServerToStart( ros::Duration(10,0)))
    ROS_INFO("Connected to action server");
  else
  {
    ROS_ERROR("Timed out waiting for action server");
    n.shutdown();
    spinthread.join();
    return 0;
  }

  MoveBaseGoal goal;

  ActionClient<MoveBaseAction>::GoalHandle gh = ac.sendGoal(goal, &transitionCallback, &feedbackCallback);

  sleep(3.0);

  gh.cancel();

  /*sleep(2.0);

  ROS_INFO("About to send a goal");
  gh.reset();
  gh = ac.sendGoal(goal, &goalCallback, &feedbackCallback, ros::Duration(10.0));

  sleep(2.0);

  ROS_INFO("About to preempt the goal");
  gh.preemptGoal();*/

  //ac.sendGoal(goal);

  while(n.ok())
    sleep(.1);

  n.shutdown();
  spinthread.join();

  return 0;
}
