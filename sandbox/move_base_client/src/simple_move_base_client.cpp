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
#include "actionlib/client/simple_action_client.h"

#include "boost/thread.hpp"

using namespace actionlib;
using namespace robot_msgs;
using namespace move_base;

typedef SimpleActionClient<MoveBaseAction> MoveBaseClient;

void activeCallback()
{
  ROS_DEBUG("In the activeCallback");
}

void doneCallback(const TerminalState& terminal_state, const MoveBaseResultConstPtr& result)
{
  ROS_DEBUG("In the done callback with terminal state=[%s]", terminal_state.toString().c_str());
}

void feedbackCallback(const MoveBaseFeedbackConstPtr& fb)
{
  ROS_INFO("Got Feedback!");
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

  MoveBaseClient ac("move_base");

  sleep(2.0);

  MoveBaseGoal goal;

  ac.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

  ROS_INFO("Blocking until goal finishes");
  ac.waitForGoalToFinish();
  ROS_INFO("Blocking call finished");

  ROS_INFO("Making sure wait for goal doesn't get stuck if we call it again");
  ac.waitForGoalToFinish();
  ROS_INFO("Yay, it worked");

  MoveBaseResultConstPtr result = ac.getResult();
  if (result)
    ROS_INFO("Got A Result!");
  else
    ROS_INFO("Got a NULL Result");

  sleep(2.0);

  ac.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);
  sleep(4.0);
  ac.cancelGoal();

  while(n.ok())
    sleep(.1);

  sleep(3);


  return 0;
}
