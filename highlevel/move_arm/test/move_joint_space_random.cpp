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

/* Author: Mrinal Kalakrishnan */

#include <ros/ros.h>
#include <robot_actions/action_client.h>

#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>

#include <cstdlib>
#include <vector>
#include <string>

using namespace std;

static double getRandom(double min, double max)
{
  return (((double)rand()/RAND_MAX)*(max-min))+min;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_joint_space_random");
  ros::NodeHandle nh;

  robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm("move_arm");

  int32_t                         feedback;
  pr2_robot_actions::MoveArmGoal  goal;
  pr2_robot_actions::MoveArmState state;

  std::vector<std::string> names(7);
  names[0] = "r_shoulder_pan_joint";
  names[1] = "r_shoulder_lift_joint";
  names[2] = "r_upper_arm_roll_joint";
  names[3] = "r_elbow_flex_joint";
  names[4] = "r_forearm_roll_joint";
  names[5] = "r_wrist_flex_joint";
  names[6] = "r_wrist_roll_joint";

  // joint limits: @TODO - can get rid of this hardcoding!
  std::vector<double> joint_min(7);
  std::vector<double> joint_max(7);
  joint_min[0] = -2.28;
  joint_max[0] = 0.71;
  joint_min[1] = -0.52;
  joint_max[1] = 1.39;
  joint_min[2] = -3.9;
  joint_max[2] = 0.8;
  joint_min[3] = -2.3;
  joint_max[3] = 0.1;
  joint_min[4] = -3.14;
  joint_max[4] = 3.14;
  joint_min[5] = -0.1;
  joint_max[5] = 2.2;
  joint_min[6] = -3.14;
  joint_max[6] = 3.14;

  ros::Time now = ros::Time::now();

  goal.goal_constraints.joint_constraint.resize(names.size());

  while (nh.ok())
  {
    ros::spinOnce();

    // assign a random goal:

    ROS_INFO("Moving to:");
    for (unsigned int i = 0 ; i < goal.goal_constraints.joint_constraint.size(); ++i)
    {
      goal.goal_constraints.joint_constraint[i].header.stamp = now;
      goal.goal_constraints.joint_constraint[i].header.frame_id = "base_link";
      goal.goal_constraints.joint_constraint[i].joint_name = names[i];
      goal.goal_constraints.joint_constraint[i].value.resize(1);
      goal.goal_constraints.joint_constraint[i].toleranceAbove.resize(1);
      goal.goal_constraints.joint_constraint[i].toleranceBelow.resize(1);
      goal.goal_constraints.joint_constraint[i].value[0] =
          getRandom(0.2,0.8)*(joint_max[i]-joint_min[i])+joint_min[i];
      ROS_INFO("\t%25s = %f", names[i].c_str(), goal.goal_constraints.joint_constraint[i].value[0]);
      goal.goal_constraints.joint_constraint[i].toleranceBelow[0] = 0.0;
      goal.goal_constraints.joint_constraint[i].toleranceAbove[0] = 0.0;
    }
    
    if (move_arm.execute(goal, feedback, ros::Duration(10.0)) != robot_actions::SUCCESS)
      ROS_ERROR("failed on random goal");
  }

  return 0;
}
