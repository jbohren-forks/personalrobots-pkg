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

#include "doors_core/action_push_door.h"
#include <pr2_robot_actions/DoorActionState.h>
#include <door_msgs/Door.h>
#include <ros/node.h>
#include <robot_actions/action_runner.h>

using namespace ros;
using namespace std;
using namespace door_handle_detector;

// -----------------------------------
//              MAIN
// -----------------------------------

int
  main (int argc, char **argv)
{
  ros::init(argc, argv);

  ros::Node node("name");

  door_msgs::Door door;
  door.frame_p1.x = 1.0;
  door.frame_p1.y = -0.5;
  door.frame_p2.x = 1.0;
  door.frame_p2.y = 0.5;
  door.door_p1.x = 1.0;
  door.door_p1.y = -0.5;
  door.door_p2.x = 1.0;
  door.door_p2.y = 0.5;
  door.travel_dir.x = 1.0;
  door.travel_dir.y = 0.0;
  door.travel_dir.z = 0.0;
  door.rot_dir = door_msgs::Door::ROT_DIR_COUNTERCLOCKWISE;
  door.hinge = door_msgs::Door::HINGE_P2;
  door.header.frame_id = "odom_combined";

  door_handle_detector::PushDoorAction push_door(node);
  robot_actions::ActionRunner runner(10.0);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(push_door);
  runner.run();

  door_msgs::Door feedback;
  push_door.execute(door, feedback);

  return (0);
}
