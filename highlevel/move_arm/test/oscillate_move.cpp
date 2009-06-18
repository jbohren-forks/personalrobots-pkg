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
 *
 *
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ros/ros.h>
#include <robot_actions/action_client.h>

#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>

#include <vector>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_move_arm");
    
    ros::Node hack; // hack
    
    robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm("move_arm");
    
    int32_t                         feedback;
    pr2_robot_actions::MoveArmGoal  goalA;
    pr2_robot_actions::MoveArmGoal  goalB;
    pr2_robot_actions::MoveArmState state;
    
    goalA.goal_constraints.set_pose_constraint_size(1);
    goalA.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
    goalA.goal_constraints.pose_constraint[0].pose.header.frame_id = "torso_lift_link";
    
    goalA.goal_constraints.pose_constraint[0].link_name = "r_wrist_roll_link";
    goalA.goal_constraints.pose_constraint[0].pose.pose.position.x = 0.75;
    goalA.goal_constraints.pose_constraint[0].pose.pose.position.y = -0.188;
    goalA.goal_constraints.pose_constraint[0].pose.pose.position.z = 0;
    
    goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.x = 0;
    goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.y = 0;
    goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.z = 0;
    goalA.goal_constraints.pose_constraint[0].pose.pose.orientation.w = 1;
    goalA.goal_constraints.pose_constraint[0].position_distance = 0.01;
    goalA.goal_constraints.pose_constraint[0].orientation_distance = 0.01;
    goalA.goal_constraints.pose_constraint[0].orientation_importance = 0.1;
    goalA.goal_constraints.pose_constraint[0].type =
	motion_planning_msgs::PoseConstraint::POSITION_XYZ + 
	motion_planning_msgs::PoseConstraint::ORIENTATION_RPY;
    
    
    std::vector<std::string> names(7);
    names[0] = "r_shoulder_pan_joint";
    names[1] = "r_shoulder_lift_joint";
    names[2] = "r_upper_arm_roll_joint";
    names[3] = "r_elbow_flex_joint";
    names[4] = "r_forearm_roll_joint";
    names[5] = "r_wrist_flex_joint";
    names[6] = "r_wrist_roll_joint";
    
    goalB.goal_constraints.joint_constraint.resize(names.size());
    for (unsigned int i = 0 ; i < goalB.goal_constraints.joint_constraint.size(); ++i)
    {
	goalB.goal_constraints.joint_constraint[i].header.stamp = ros::Time::now();
	goalB.goal_constraints.joint_constraint[i].header.frame_id = "base_link";
	goalB.goal_constraints.joint_constraint[i].joint_name = names[i];
	goalB.goal_constraints.joint_constraint[i].value.resize(1);
	goalB.goal_constraints.joint_constraint[i].toleranceAbove.resize(1);
	goalB.goal_constraints.joint_constraint[i].toleranceBelow.resize(1);
	goalB.goal_constraints.joint_constraint[i].value[0] = 0.0;
	goalB.goal_constraints.joint_constraint[i].toleranceBelow[0] = 0.0;
	goalB.goal_constraints.joint_constraint[i].toleranceAbove[0] = 0.0;
    }
    
    goalB.goal_constraints.joint_constraint[0].value[0] = -2.0;
    goalB.goal_constraints.joint_constraint[3].value[0] = -0.1;
    goalB.goal_constraints.joint_constraint[5].value[0] = 0.15;

    ros::NodeHandle nh;
    
    while (nh.ok())
    {
	ros::spinOnce();
	if (move_arm.execute(goalA, feedback, ros::Duration(10.0)) != robot_actions::SUCCESS)
	  ROS_ERROR("failed on goal A");
	  //	    return -1;
	ros::spinOnce();
	if (move_arm.execute(goalB, feedback, ros::Duration(10.0)) != robot_actions::SUCCESS)
	  ROS_ERROR("failed on goal B");
	  //	    return -1;
    }
    
    return 0;
}
