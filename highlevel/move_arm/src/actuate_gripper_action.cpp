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
 ******************************p***************************************/

/* Author: Sachin Chitta */

#include "pr2_robot_actions/ActuateGripperState.h"
#include "std_msgs/Float64.h"

/** Actions and messages */
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>

#include <ros/ros.h>

class ActuateGripperAction : public robot_actions::Action<std_msgs::Float64, std_msgs::Float64>
{
  public: 

    ros::Publisher pub_;


    ActuateGripperAction() : robot_actions::Action<std_msgs::Float64, std_msgs::Float64>("actuate_gripper")
    {
      ros::NodeHandle node;
      pub_ = node.advertise<std_msgs::Float64>("r_gripper_effort_controller/command",10);
    };


    ~ActuateGripperAction()
    {};

    robot_actions::ResultStatus execute(const std_msgs::Float64& goal, std_msgs::Float64& feedback)
    {
      ROS_INFO("ActuateGripperAction: execute");

      // set default feedback
      feedback.data = goal.data;

      std_msgs::Float64 gripper_msg;
      gripper_msg.data = goal.data;
      pub_.publish(gripper_msg);

      ros::Rate r(10.0);
      ros::Time start = ros::Time::now();

      while(ros::Time::now()-start < ros::Duration(10.0))
      {
        if (isPreemptRequested()) {
          gripper_msg.data = 0.0;
          pub_.publish(gripper_msg);
          ROS_INFO("ActuateGripperAction: preempted");
          return robot_actions::PREEMPTED;
        }
        r.sleep();
      }
      ROS_INFO("ActuateGripperAction: Done");
      return robot_actions::SUCCESS;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "actuate_gripper", ros::init_options::AnonymousName);  
    ActuateGripperAction actuate_gripper;
    robot_actions::ActionRunner runner(20.0);
    runner.connect<std_msgs::Float64, pr2_robot_actions::ActuateGripperState,std_msgs::Float64>(actuate_gripper);
    runner.run();
    ros::spin();
    return 0;
}
