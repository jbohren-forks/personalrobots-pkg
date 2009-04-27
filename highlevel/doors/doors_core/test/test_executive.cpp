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
 * $Id$
 *
 *********************************************************************/

/* Author: Wim Meeussen */


#include <boost/thread/thread.hpp>
#include <robot_msgs/Door.h>
#include <ros/node.h>
#include <robot_actions/action_client.h>
#include <pr2_robot_actions/Pose2D.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/SwitchControllersState.h>
#include <pr2_robot_actions/MoveBaseStateNew.h>
#include "doors_core/executive_functions.h"


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

  robot_msgs::Door door;
  door.frame_p1.x = 1.0;
  door.frame_p1.y = -0.5;
  door.frame_p2.x = 1.0;
  door.frame_p2.y = 0.5;
  door.rot_dir = -1;
  door.hinge = -1;
  door.header.frame_id = "base_footprint";

  pr2_robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;

  Duration timeout_short = Duration().fromSec(2.0);
  Duration timeout_medium = Duration().fromSec(10.0);
  Duration timeout_long = Duration().fromSec(40.0);

  robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> tuck_arm("doors_tuck_arms");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty> switch_controllers("switch_controllers");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> detect_door("detect_door");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> detect_handle("detect_handle");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> grasp_handle("grasp_handle");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> unlatch_handle("unlatch_handle");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> open_door("open_door");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> release_handle("release_handle");
  robot_actions::ActionClient<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door> move_base_door("move_base_door");
  robot_actions::ActionClient<robot_msgs::PoseStamped, pr2_robot_actions::MoveBaseStateNew, robot_msgs::PoseStamped> move_base_local("move_base_local");

  timeout_medium.sleep();
  robot_msgs::Door tmp_door;


  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  // detect door
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (detect_door.execute(door, door, timeout_long) != robot_actions::SUCCESS) return -1;

  // detect handle
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("head_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  while (detect_handle.execute(door, door, timeout_long) != robot_actions::SUCCESS);

  // approach door
  robot_msgs::PoseStamped goal_msg;
  tf::PoseStampedTFToMsg(getRobotPose(door, 0.6), goal_msg);
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (move_base_local.execute(goal_msg, goal_msg, timeout_long) != robot_actions::SUCCESS) return -1;

  // grasp handle
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  switchlist.start_controllers.push_back("r_gripper_effort_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_wrench_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (grasp_handle.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;

  // unlatch handle
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_tff_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (unlatch_handle.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) return -1;

  // open door in separate thread
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  boost::thread* thread = new boost::thread(boost::bind(&robot_actions::ActionClient<robot_msgs::Door, 
							pr2_robot_actions::DoorActionState, robot_msgs::Door>::execute, 
							&open_door, door, tmp_door, timeout_long));

  // move throught door
  pr2_robot_actions::Pose2D pose2d;
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) {open_door.preempt(); return -1;};
  if (move_base_door.execute(door, tmp_door, timeout_long) != robot_actions::SUCCESS) {open_door.preempt(); return -1;};

  // finish open door
  open_door.preempt();
  thread->join();
  delete thread;

  // release handle
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_cartesian_tff_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.start_controllers.push_back("r_arm_cartesian_twist_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (release_handle.execute(door, door, timeout_long) != robot_actions::SUCCESS) return -1;

  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("r_arm_cartesian_trajectory_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_pose_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_twist_controller");
  switchlist.stop_controllers.push_back("r_arm_cartesian_wrench_controller");
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  // stop remaining controllers
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("laser_tilt_controller");
  switchlist.stop_controllers.push_back("head_controller");
  switchlist.stop_controllers.push_back("r_gripper_effort_controller");
  switchlist.stop_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  return (0);
}
