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
#include <action_tools/action_server.h>
#include <boost/thread.hpp>
#include <robot_msgs/PoseStamped.h>
#include <action_tools/MoveBaseGoal.h>
#include <action_tools/MoveBaseResult.h>

typedef action_tools::ActionServer<action_tools::MoveBaseGoal, robot_msgs::PoseStamped,
        action_tools::MoveBaseResult, robot_msgs::PoseStamped> MoveBaseActionServer;

typedef MoveBaseActionServer::GoalHandle GoalHandle;

void goalCB(GoalHandle goal_handle){
  ROS_INFO("In goal callback");
}

void preemptCB(){
  ROS_INFO("In preempt callback");
}

void updateLoop(double freq){
  ros::NodeHandle n;
  MoveBaseActionServer as(n, "move_base", 0.5);

  GoalHandle goal_handle;

  ros::Publisher pub = n.advertise<robot_msgs::PoseStamped>("~current_goal", 1);

  as.registerGoalCallback(boost::bind(&goalCB, _1));

  as.registerPreemptCallback(boost::bind(&preemptCB));

  ros::Rate r(freq);
  while(n.ok()){
    r.sleep();
    if(as.isActive()){
      if(!as.isPreempted()){
        if(as.isNewGoalAvailable()){
          ROS_INFO("This action has received a new goal");
          goal_handle = as.acceptNextGoal();
        }

        boost::shared_ptr<const robot_msgs::PoseStamped> goal = goal_handle.getGoal();

        if(goal->pose.position.x > 100.0){
          ROS_INFO("This action aborted");
          as.aborted();
          continue;
        }

        if(goal->pose.position.y > 100.0){
          ROS_INFO("This action succeeded");
          as.succeeded();
          continue;
        }

        pub.publish(*goal);
      }
      else {
        ROS_INFO("This action has been preempted");
        as.preempted();
      }
    }
    else if(as.isNewGoalAvailable()){
      ROS_INFO("This action has received a new goal");
      goal_handle = as.acceptNextGoal();
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "test_action");

  boost::thread* update_thread = new boost::thread(boost::bind(&updateLoop, 10.0));

  ros::spin();

  update_thread->join();
  delete update_thread;
}
