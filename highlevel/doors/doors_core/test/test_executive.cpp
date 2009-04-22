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


#include <robot_msgs/Door.h>
#include <ros/node.h>
#include <robot_actions/action_client.h>
#include <door_handle_detector/DetectDoorActionStatus.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <robot_actions/SwitchControllersState.h>


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

  robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;

  Duration timeout_short = Duration().fromSec(2.0);
  Duration timeout_medium = Duration().fromSec(10.0);
  Duration timeout_long = Duration().fromSec(20.0);

  //robot_actions::ActionClient<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty> tuck_arm("doors_tuck_arms");
  robot_actions::ActionClient<robot_actions::SwitchControllers, robot_actions::SwitchControllersState,  std_msgs::Empty> switch_controllers("switch_controllers");
  //robot_actions::ActionClient<robot_msgs::Door, door_handle_detector::DetectDoorActionStatus, robot_msgs::Door> detect_door("detect_door");
  //robot_actions::ActionClient<robot_msgs::Door, door_handle_detector::DetectDoorActionStatus, robot_msgs::Door> detect_handle("detect_handle");

  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("r_arm_joint_trajectory_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;
  //if (tuck_arm.execute(empty, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  /*
  // detect door
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("laser_tilt_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (detect_door.execute(door, door, timeout_long) != robot_actions::SUCCESS) return -1;

  // detect handle
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("head_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_short) != robot_actions::SUCCESS) return -1;
  if (detect_handle.execute(door, door, timeout_long) != robot_actions::SUCCESS) return -1;
  */
  return (0);
}
