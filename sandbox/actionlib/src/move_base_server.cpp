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
#include <ros/ros.h>
#include <actionlib/action_server.h>
#include <actionlib/single_goal_action_server.h>
#include <boost/thread.hpp>
#include <robot_msgs/PoseStamped.h>
#include <actionlib/MoveBaseAction.h>

typedef actionlib::ActionServer<actionlib::MoveBaseActionGoal, actionlib::MoveBaseGoal,
        actionlib::MoveBaseActionResult, actionlib::MoveBaseResult,
        actionlib::MoveBaseActionFeedback, actionlib::MoveBaseFeedback> MoveBaseActionServer;

typedef MoveBaseActionServer::GoalHandle GoalHandle;

std::vector<GoalHandle> goals;

void goalCB(GoalHandle goal){
  ROS_INFO("In goal callback, got a goal with id: %.2f", goal.getGoalID().id.toSec());
  goal.setActive();
  goals.push_back(goal);
}

void preemptCB(GoalHandle goal){
  ROS_INFO("In preempt callback, got a goal with id: %.2f", goal.getGoalID().id.toSec());
  for(unsigned int i = 0; i < goals.size(); ++i){
    if(goals[i] == goal)
      goals[i].setPreempted();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_action");

  ros::NodeHandle n;

  MoveBaseActionServer as(n, "move_base", boost::bind(&goalCB, _1), boost::bind(&preemptCB, _1));

  ros::spin();
}
