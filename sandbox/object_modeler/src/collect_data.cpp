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

// Author Romain Thibaux (thibaux@willowgarage.com)

#include <ros/ros.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_actions/action_client.h>
#include <nav_robot_actions/MoveBaseState.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_modeler");
  //ros::NodeHandle n;
  
  robot_actions::ActionClient<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped> move_client("move_base");

  robot_msgs::Point goal_pos;
  goal_pos.x = 0;
  goal_pos.y = 0;
  goal_pos.z = 0;
  robot_msgs::Quaternion goal_orient;
  goal_orient.x = 0;
  goal_orient.y = 0;
  goal_orient.z = 0;
  goal_orient.w = 0;
  robot_msgs::Pose goal_pose;
  goal_pose.position = goal_pos;
  goal_pose.orientation = goal_orient;
  robot_msgs::PoseStamped goal_pose_stamped;
  goal_pose_stamped.pose = goal_pose;
  robot_msgs::PoseStamped feedback;
  robot_actions::ResultStatus result = move_client.execute(goal_pose_stamped, feedback, ros::Duration(60));
  switch (result) {
  case robot_actions::SUCCESS:
    ROS_INFO("Move successful");
    break;
  case robot_actions::ABORTED:
    ROS_INFO("Move aborted");
    break;
  case robot_actions::PREEMPTED:
    ROS_INFO("Move preempted");
    break;
  }
}
