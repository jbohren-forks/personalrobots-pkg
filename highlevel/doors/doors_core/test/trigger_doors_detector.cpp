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

#include "doors_core/action_detect_door.h"
#include "doors_core/action_detect_handle.h"
#include <door_functions/door_functions.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <robot_msgs/Door.h>
#include <ros/node.h>
#include <robot_actions/action_runner.h>

using namespace ros;
using namespace std;
using namespace door_handle_detector;
using namespace door_functions;

// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("trigger_detect_door");

  robot_msgs::Door my_door_;

  my_door_.frame_p1.x = 1.0;
  my_door_.frame_p1.y = -0.5;
  my_door_.frame_p2.x = 1.0;
  my_door_.frame_p2.y = 0.5;
  my_door_.rot_dir = robot_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  my_door_.hinge = robot_msgs::Door::HINGE_P2;
  my_door_.header.frame_id = "base_footprint";

  door_handle_detector::DetectDoorAction door_detector(node);
  door_handle_detector::DetectHandleAction handle_detector(node);
  robot_actions::ActionRunner runner(10.0);
  runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(door_detector);
  runner.connect<robot_msgs::Door, pr2_robot_actions::DoorActionState, robot_msgs::Door>(handle_detector);
  runner.run();

  robot_msgs::Door feedback;
  cout << "door in " << my_door_ << endl;
  door_detector.execute(my_door_, feedback);
  cout << "door out " << feedback << endl;
  //handle_detector.execute(feedback, feedback);

  return (0);
}
