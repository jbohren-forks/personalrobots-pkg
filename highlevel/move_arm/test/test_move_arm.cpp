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
 * $Id: test_executive.cpp 16122 2009-05-27 00:28:28Z meeussen $
 *
 *********************************************************************/

/* Author: Sachin Chitta */


#include <ros/node.h>
#include <boost/thread/thread.hpp>
#include <robot_actions/action_client.h>

#include <pr2_robot_actions/MoveArmGoal.h>
#include <pr2_robot_actions/MoveArmState.h>
#include <pr2_robot_actions/SwitchControllersState.h>

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv);
    ros::Node node("test_move_arm");
    //  boost::thread* thread;
    
    pr2_robot_actions::SwitchControllers switchlist;
    std_msgs::Empty empty;
    
    Duration timeout_short = Duration().fromSec(2.0);
    Duration timeout_medium = Duration().fromSec(10.0);
    Duration timeout_long = Duration().fromSec(40.0);
    
    //  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty> switch_controllers("switch_controllers");
    robot_actions::ActionClient<pr2_robot_actions::MoveArmGoal, pr2_robot_actions::MoveArmState, int32_t> move_arm("move_arm");
    
    int32_t feedback;
    pr2_robot_actions::MoveArmGoal goal;
    pr2_robot_actions::MoveArmState state;
    
    goal.goal_constraints.set_pose_constraint_size(1);
    goal.goal_constraints.pose_constraint[0].pose.header.stamp = ros::Time::now();
    goal.goal_constraints.pose_constraint[0].pose.header.frame_id = "torso_lift_link";
    
    goal.goal_constraints.pose_constraint[0].pose.pose.position.x = 0.75;
    goal.goal_constraints.pose_constraint[0].pose.pose.position.y = -0.188;
    goal.goal_constraints.pose_constraint[0].pose.pose.position.z = 0;
    
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.x = 0;
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.y = 0;
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.z = 0;
    goal.goal_constraints.pose_constraint[0].pose.pose.orientation.w = 1;
    
    //  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    //  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
    
    //  if(switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
    ROS_INFO("Done switching controllers");
    
    if(move_arm.execute(goal,feedback,timeout_long) != robot_actions::SUCCESS) return -1;
    
    return (0);
}
