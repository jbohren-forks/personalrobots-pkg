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

/* Author: Melonee Wise */


#include <boost/thread/thread.hpp>

#include <ros/node.h>
#include <robot_actions/action_client.h>

#include <safety_core/action_detect_plug_on_base.h>

// Msgs
#include <plugs_msgs/PlugStow.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

// State Msgs
#include <robot_actions/NoArgumentsActionState.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <pr2_robot_actions/DetectOutletState.h>
#include <pr2_robot_actions/DetectPlugOnBaseState.h>
#include <pr2_robot_actions/MoveAndGraspPlugState.h>
#include <pr2_robot_actions/StowPlugState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <pr2_robot_actions/PlugInState.h>


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
  plugs_msgs::PlugStow plug_stow;
  robot_msgs::PointStamped point;
  robot_msgs::PoseStamped pose;

  point.header.frame_id = "torso_lift_link";
  point.point.x=0;
  point.point.y=0;
  point.point.z=0;

  Duration switch_timeout = Duration(4.0);

  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>
    tuck_arm("safety_tuck_arms");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty>
    switch_controllers("switch_controllers");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>
    untuck_arm("plugs_untuck_arms");
  robot_actions::ActionClient<std_msgs::Empty, pr2_robot_actions::DetectPlugOnBaseState, plugs_msgs::PlugStow>
    detect_plug_on_base("detect_plug_on_base");
  robot_actions::ActionClient<plugs_msgs::PlugStow, pr2_robot_actions::MoveAndGraspPlugState, std_msgs::Empty>
    move_and_grasp_plug("move_and_grasp_plug");
  robot_actions::ActionClient<robot_msgs::PointStamped, pr2_robot_actions::DetectOutletState, robot_msgs::PoseStamped>
    detect_outlet_fine("detect_outlet_fine");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>
    localize_plug_in_gripper("localize_plug_in_gripper");
  robot_actions::ActionClient<std_msgs::Int32, pr2_robot_actions::PlugInState, std_msgs::Empty>
    plug_in("plug_in");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>
    unplug("unplug");
  robot_actions::ActionClient< plugs_msgs::PlugStow, pr2_robot_actions::StowPlugState, std_msgs::Empty>
    stow_plug("stow_plug");
  robot_actions::ActionClient<robot_msgs::PointStamped, pr2_robot_actions::DetectOutletState, robot_msgs::PoseStamped>
    detect_outlet_coarse("detect_outlet_coarse");
  robot_actions::ActionClient<robot_msgs::PoseStamped, nav_robot_actions::MoveBaseState, robot_msgs::PoseStamped>
    move_base_local("move_base_local");

  Duration(1.0).sleep();

  tuck_arm.preempt();
  switch_controllers.preempt();
  untuck_arm.preempt();
  detect_plug_on_base.preempt();
  move_and_grasp_plug.preempt();
  detect_outlet_fine.preempt();
  localize_plug_in_gripper.preempt();
  plug_in.preempt();
  unplug.preempt();
  stow_plug.preempt();
  detect_outlet_coarse.preempt();
  move_base_local.preempt();

  Duration(2.0).sleep();

  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("laser_tilt_controller");
  switchlist.stop_controllers.push_back("r_gripper_position_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_wrench_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.stop_controllers.push_back("r_arm_hybrid_controller");
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  switchlist.stop_controllers.push_back("head_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -10;


  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -20;
  if (tuck_arm.execute(empty, empty, Duration(20.0)) != robot_actions::SUCCESS) return -30;


  // Executes detect outlet (coarse)
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("head_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -37;
  robot_msgs::PointStamped guess;
  guess.header.frame_id = "base_link";
  guess.point.x = 2.0;
  guess.point.y = 0.0;
  guess.point.z = 0.4;
  robot_msgs::PoseStamped coarse_outlet_pose_msg;
  int tries = 0;
  while (detect_outlet_coarse.execute(guess, coarse_outlet_pose_msg, Duration(300.0)) != robot_actions::SUCCESS)
  {
    ++tries;
    if (tries > 5)
      return -40;
    Duration(1.0).sleep();
  }

  // Determines the desired base position
  tf::Pose coarse_outlet_pose;
  tf::PoseMsgToTF(coarse_outlet_pose_msg.pose, coarse_outlet_pose);

  tf::Pose desi_offset(tf::Quaternion(0,0,0), tf::Vector3(-0.5, 0.25, 0.0));
  tf::Pose target = coarse_outlet_pose * desi_offset;

  robot_msgs::PoseStamped target_msg;
  target_msg.header.frame_id = coarse_outlet_pose_msg.header.frame_id;
  tf::PoseTFToMsg(target, target_msg.pose);

  // Executes move base
  if (move_base_local.execute(target_msg, target_msg, Duration(500.0)) != robot_actions::SUCCESS) return -50;

  // detect outlet fine
  robot_msgs::PointStamped outlet_pt;
  outlet_pt.header.frame_id = coarse_outlet_pose_msg.header.frame_id;
  outlet_pt.header.stamp = coarse_outlet_pose_msg.header.stamp;
  outlet_pt.point = coarse_outlet_pose_msg.pose.position;
  if (detect_outlet_fine.execute(outlet_pt, pose, Duration(60.0)) != robot_actions::SUCCESS) return -70;

  // untuck arm
  if (untuck_arm.execute(empty, empty, Duration(30.0)) != robot_actions::SUCCESS) return -80;

  // detect plug on base
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -90;
  if (detect_plug_on_base.execute(empty, plug_stow, Duration(120.0)) != robot_actions::SUCCESS) return -100;

  // move and grasp plug
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  switchlist.start_controllers.push_back("r_gripper_position_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_wrench_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -110;
  if (move_and_grasp_plug.execute(plug_stow, empty, Duration(120.0)) != robot_actions::SUCCESS) return -120;

  // localize plug in gripper
  if (localize_plug_in_gripper.execute(empty, empty, Duration(60.0)) != robot_actions::SUCCESS) return -130;

  // plug in
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_wrench_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.start_controllers.push_back("r_arm_hybrid_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -140;
  std_msgs::Int32 outlet_id; outlet_id.data = 0;
  pr2_robot_actions::PlugInState plug_in_state;
  if (plug_in.execute(outlet_id, empty, Duration(10000.0)) != robot_actions::SUCCESS) return -150;

  Duration(8.0).sleep();

  // unplug
  if (unplug.execute(empty, empty, Duration(60.0)) != robot_actions::SUCCESS) return -160;

  // stow plug
   switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_hybrid_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_wrench_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -170;
  if (stow_plug.execute(plug_stow, empty, Duration(60.0)) != robot_actions::SUCCESS) return -180;


  // stop remaining controllers
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("laser_tilt_controller");
  switchlist.stop_controllers.push_back("r_gripper_position_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_wrench_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_twist_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -1;

  printf("DONE!\n");

  return (0);
}
