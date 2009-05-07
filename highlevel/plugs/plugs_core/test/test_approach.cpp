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
 * $Id: test_executive.cpp 14337 2009-04-23 18:40:13Z meeussen $
 *
 *********************************************************************/

#include <boost/thread/thread.hpp>

#include <ros/node.h>
#include <robot_actions/action_client.h>

// Msgs
#include <robot_msgs/PlugStow.h>
#include <std_msgs/Empty.h>

// State Msgs
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/MoveAndGraspPlugState.h>
#include <pr2_robot_actions/DetectOutletState.h>
#include <pr2_robot_actions/StowPlugState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <pr2_robot_actions/DetectPlugOnBaseState.h>
#include <nav_robot_actions/MoveBaseState.h>

// Actions
#include <safety_core/action_detect_plug_on_base.h>
#include <safety_core/action_tuck_arms.h>
#include <plugs_core/action_untuck_arms.h>
#include <plugs_core/action_move_and_grasp_plug.h>
#include <plugs_core/action_detect_outlet_fine.h>
#include <plugs_core/action_detect_outlet_coarse.h>
#include <plugs_core/action_localize_plug_in_gripper.h>
#include <plugs_core/action_plug_in.h>
#include <plugs_core/action_stow_plug.h>
#include <plugs_core/action_unplug.h>

using namespace ros;
using namespace std;


// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("test_executive");
  boost::thread_group threads_;

  pr2_robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;
  robot_msgs::PlugStow plug_stow;
  robot_msgs::PointStamped point;
  robot_msgs::PoseStamped pose;

  Duration switch_timeout = Duration(4.0);

  robot_actions::ActionClient<robot_msgs::PointStamped, pr2_robot_actions::DetectOutletState, robot_msgs::PoseStamped>
    detect_outlet_coarse("detect_outlet_coarse");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>
    tuck_arm("safety_tuck_arms");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty>
    switch_controllers("switch_controllers");
  robot_actions::ActionClient<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>
    move_base_local("move_base_local");


  detect_outlet_coarse.preempt();
  tuck_arm.preempt();
  switch_controllers.preempt();
  move_base_local.preempt();

  Duration(4.0).sleep();

  // Tuck arms
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, Duration(20.0)) != robot_actions::SUCCESS) return -2;

  // Executes detect outlet (coarse)
  robot_msgs::PointStamped junk;
  junk.header.frame_id = "odom_combined";  // Necessary?
  robot_msgs::PoseStamped coarse_outlet_pose_msg;
  if (detect_outlet_coarse.execute(junk, coarse_outlet_pose_msg, Duration(300.0)) != robot_actions::SUCCESS)
    return -3;

  // Determines the desired base position
  tf::Pose coarse_outlet_pose;
  tf::PoseMsgToTF(coarse_outlet_pose_msg.pose, coarse_outlet_pose);

  tf::Pose desi_offset(tf::Quaternion(0,0,0), tf::Vector3(-0.6, 0.0, 0.0));
  tf::Pose target = coarse_outlet_pose * desi_offset;

  robot_msgs::PoseStamped target_msg;
  target_msg.header.frame_id = coarse_outlet_pose_msg.header.frame_id;
  tf::PoseTFToMsg(target, target_msg.pose);

  // Executes move base
  if (move_base_local.execute(target_msg, target_msg, Duration(500.0)) != robot_actions::SUCCESS) return -4;

  return 0;
}
