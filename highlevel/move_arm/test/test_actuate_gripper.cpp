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
 * $Id: test_executive.cpp 18230 2009-07-02 21:27:56Z tfoote $
 *
 *********************************************************************/

/* Author: Sachin Chitta */


#include <ros/node.h>
#include <robot_actions/action_client.h>
#include <pr2_robot_actions/ActuateGripperState.h>
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/SwitchControllers.h>
#include <pr2_robot_actions/SwitchControllersState.h>


using namespace ros;
using namespace std;

int
  main (int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("test_actuate_gripper");

  pr2_robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;

  Duration timeout_short = Duration().fromSec(2.0);
  Duration timeout_medium = Duration().fromSec(10.0);
  Duration timeout_long = Duration().fromSec(40.0);

  robot_actions::ActionClient<std_msgs::Float64, pr2_robot_actions::ActuateGripperState, std_msgs::Float64> actuate_gripper("actuate_gripper");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty> switch_controllers("switch_controllers");


  // tuck arm
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
    switchlist.start_controllers.push_back("r_gripper_effort_controller");
  if (switch_controllers.execute(switchlist, empty, timeout_medium) != robot_actions::SUCCESS) return -1;

  std_msgs::Float64 cmd, fb;
  cmd.data = -20;

  if (actuate_gripper.execute(cmd, fb, timeout_medium) != robot_actions::SUCCESS) return -1;
  sleep(2.0);

  cmd.data = 20;
  if (actuate_gripper.execute(cmd, fb, timeout_medium) != robot_actions::SUCCESS) return -1;
  sleep(2.0);

  return (0);
}
