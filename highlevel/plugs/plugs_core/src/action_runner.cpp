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
 *********************************************************************/

// Msgs
#include <robot_msgs/PlugStow.h>
#include <std_msgs/Empty.h>

// Actions
#include <plugs_core/action_untuck_arms.h>
#include <plugs_core/action_move_and_grasp_plug.h>
#include <plugs_core/action_detect_outlet_fine.h>
#include <plugs_core/action_detect_outlet_coarse.h>
#include <plugs_core/action_localize_plug_in_gripper.h>
#include <plugs_core/action_plug_in.h>
#include <plugs_core/action_stow_plug.h>

// State Msgs
#include <robot_actions/NoArgumentsActionState.h>
#include <robot_actions/MoveAndGraspPlugState.h>
#include <robot_actions/DetectOutletState.h>
#include <robot_actions/LocalizePlugInGripperState.h>
#include <robot_actions/StowPlugState.h>



#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>

using namespace plugs_core;

// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv);

  ros::Node node("plugs_core_actions");
  std_msgs::Empty empty;
  robot_msgs::PlugStow plug_msg;

  UntuckArmsAction untuck_arms;
  MoveAndGraspPlugAction move_and_grasp;
  DetectOutletFineAction detect_outlet_fine;
  DetectOutletCoarseAction detect_outlet_coarse;
  LocalizePlugInGripperAction localize_plug_in_gripper(node);
  PlugInAction plug_in(node);
  StowPlugAction stow_plug;


  robot_actions::ActionRunner runner(10.0);
  runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(untuck_arms);
  runner.connect<robot_msgs::PlugStow, robot_actions::MoveAndGraspPlugState, std_msgs::Empty>(move_and_grasp);
  runner.connect<robot_msgs::PointStamped, robot_actions::DetectOutletState, robot_msgs::PoseStamped>(detect_outlet_fine);
  runner.connect<robot_msgs::PointStamped, robot_actions::DetectOutletState, robot_msgs::PoseStamped>(detect_outlet_coarse);
  runner.connect<robot_msgs::PoseStamped, robot_actions::LocalizePlugInGripperState, std_msgs::Empty>(localize_plug_in_gripper);
  runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(plug_in);
  runner.connect<robot_msgs::PlugStow, robot_actions::StowPlugState, std_msgs::Empty>(stow_plug);


  runner.run();

  node.spin();
  return 0;
}
