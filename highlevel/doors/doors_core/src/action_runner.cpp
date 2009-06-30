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

/* Author: Wim Meeussen */

#include <door_msgs/Door.h>
#include "doors_core/action_detect_door.h"
#include "doors_core/action_detect_handle.h"
#include "doors_core/action_check_path.h"
#include "doors_core/action_grasp_handle.h"
#include "doors_core/action_detect_handle_no_camera.h"
#include "doors_core/action_open_door.h"
#include "doors_core/action_push_door.h"
#include "doors_core/action_release_handle.h"
#include "doors_core/action_touch_door.h"
#include "doors_core/action_unlatch_handle.h"


#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_robot_actions/DoorActionState.h>
#include <pr2_robot_actions/CheckPathState.h>

using namespace door_handle_detector;


// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv,"door_domain_action_runer"); 

  ros::Node node("action_runner_door_domain");
  tf::TransformListener tf;

  DetectDoorAction detect_door(tf);
  DetectHandleAction detect_handle(tf);
  CheckPathAction check_path(tf);
  GraspHandleAction grasp(tf);
  DetectHandleNoCameraAction detect_handle_no_camera(tf);
  OpenDoorAction open(tf);
  PushDoorAction push(tf);
  TouchDoorAction touch(tf);
  ReleaseHandleAction release(tf);
  UnlatchHandleAction unlatch(tf);

  robot_actions::ActionRunner runner(10.0);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(detect_door);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(detect_handle);  
  runner.connect<robot_msgs::PoseStamped, pr2_robot_actions::CheckPathState, int8_t>(check_path);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(detect_handle_no_camera);  
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(grasp);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(open);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(push);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(release);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(touch);
  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(unlatch);

  runner.run();
  node.spin();
  return 0;
}
