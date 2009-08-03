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

#include <boost/thread/thread.hpp>

#include <ros/node.h>
#include <robot_actions/action_client.h>
#include <tf/tf.h>

// Msgs
#include <plugs_msgs/PlugStow.h>
#include <std_msgs/Empty.h>

// State Msgs
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/MoveAndGraspPlugState.h>
#include <pr2_robot_actions/DetectOutletState.h>
#include <pr2_robot_actions/StowPlugState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <pr2_robot_actions/DetectPlugOnBaseState.h>
#include <nav_robot_actions/MoveBaseState.h>

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
  geometry_msgs::PointStamped point;
  geometry_msgs::PoseStamped pose;

  Duration switch_timeout = Duration(4.0);

  robot_actions::ActionClient<geometry_msgs::PointStamped, pr2_robot_actions::DetectOutletState, geometry_msgs::PoseStamped>
    detect_outlet_coarse("detect_outlet_coarse");
  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>
    tuck_arm("safety_tuck_arms");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty>
    switch_controllers("switch_controllers");
  robot_actions::ActionClient<geometry_msgs::PoseStamped, nav_robot_actions::MoveBaseState, geometry_msgs::PoseStamped>
    move_base_local("move_base_local");

  Duration(1.0).sleep();

  detect_outlet_coarse.preempt();
  tuck_arm.preempt();
  switch_controllers.preempt();
  move_base_local.preempt();

  Duration(1.0).sleep();

  // Takes down controllers that might already be up
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  switchlist.stop_controllers.push_back("head_controller");
  switchlist.stop_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -1;

  Duration(2.0).sleep();

  // Tuck arms
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  switchlist.start_controllers.push_back("head_controller");
  switchlist.start_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, Duration(20.0)) != robot_actions::SUCCESS) return -2;

  // Executes detect outlet (coarse)
  geometry_msgs::PointStamped guess;
#if 0
  guess.header.frame_id = "odom_combined";
  guess.point.x = 4.0;
  guess.point.y = 0.0;
  guess.point.z = 0.4;
#else
  guess.header.frame_id = "map";
  guess.point.x = 9.899;
  guess.point.y = 24.91;
  guess.point.z = 0.4;
#endif
  geometry_msgs::PoseStamped coarse_outlet_pose_msg;
  int tries = 0;
  while (detect_outlet_coarse.execute(guess, coarse_outlet_pose_msg, Duration(300.0)) != robot_actions::SUCCESS)
  {
    ++tries;
    if (tries > 5)
      return -3;
    Duration(1.0).sleep();
  }

  // Determines the desired base position
  tf::Pose coarse_outlet_pose;
  tf::poseMsgToTF(coarse_outlet_pose_msg.pose, coarse_outlet_pose);

  tf::Pose desi_offset(tf::Quaternion(0,0,0), tf::Vector3(-0.5, 0.25, 0.0));
  tf::Pose target = coarse_outlet_pose * desi_offset;

  geometry_msgs::PoseStamped target_msg;
  target_msg.header.frame_id = coarse_outlet_pose_msg.header.frame_id;
  tf::poseTFToMsg(target, target_msg.pose);

  // Executes move base
  if (move_base_local.execute(target_msg, target_msg, Duration(500.0)) != robot_actions::SUCCESS) return -4;

  return 0;
}
