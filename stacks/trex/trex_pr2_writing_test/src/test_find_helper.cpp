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


#include <ros/node.h>
#include <robot_actions/action_client.h>

// Msgs
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
 
// State Msgs
#include <robot_actions/NoArgumentsActionState.h>
#include <pr2_robot_actions/FindHelperState.h>
#include <pr2_robot_actions/SwitchControllersState.h>

using namespace ros;
using namespace std;


// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("test_find_helper");

  pr2_robot_actions::SwitchControllers switchlist;
  std_msgs::Empty empty;
  geometry_msgs::PoseStamped find_helper_pose_msg;

  Duration switch_timeout = Duration(4.0);

  robot_actions::ActionClient<std_msgs::Empty, pr2_robot_actions::FindHelperState, geometry_msgs::PoseStamped> 
    find_helper("find_helper");
  robot_actions::ActionClient<pr2_robot_actions::SwitchControllers, pr2_robot_actions::SwitchControllersState,  std_msgs::Empty>
    switch_controllers("switch_controllers");


  Duration(1.0).sleep();

  // Preempt all of the actions
  find_helper.preempt();
  switch_controllers.preempt();

  Duration(1.0).sleep();

  // Takes down controllers that might already be up
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.stop_controllers.push_back("head_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -1;

  Duration(2.0).sleep();

  // Find our helper
  switchlist.start_controllers.clear();  switchlist.stop_controllers.clear();
  switchlist.start_controllers.push_back("head_controller");
  if (switch_controllers.execute(switchlist, empty, switch_timeout) != robot_actions::SUCCESS) return -1;
  if (find_helper.execute(empty, find_helper_pose_msg, Duration(100.0)) != robot_actions::SUCCESS) return -2;

  return 0;
}
