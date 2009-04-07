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

//#include <robot_msgs/PlugStow.h>
#include <std_msgs/Empty.h>
#include <plugs_core/action_untuck_arms.h>
//#include <plugs_core/action_move_and_grasp_plug.h>
//#include <robot_actions/MoveAndGraspPlugActionState.h>
#include <robot_actions/NoArgumentsActionState.h>
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
  //robot_msgs::PlugStow plug_msg;
  //MoveToGraspPlugAction move_to_grasp(node);
  UntuckArmsAction untuck_arms;

  robot_actions::ActionRunner runner(10.0);
  //runner.connect<robot_msgs::PlugStow, robot_actions::MoveAndGraspPlugActionState, std_msgs::Empty>(move_to_grasp);
  runner.connect<std_msgs::Empty, robot_actions::NoArgumentsActionState, std_msgs::Empty>(untuck_arms);

  runner.run();
  // plug_msg.header.frame_id = "torso_lift_link";
  // plug_msg.stowed = 1;
  // plug-msg.plug_centroid.x = 0.24;
  // plug_msg.plug_centroid.y = 0.03;
  // plug_msg.plug_centroid.z = -0.45;
  // move_to_grasp.handleActivate(plug_msg);

  node.spin();
  return 0;
}
