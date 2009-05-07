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

/* Author: Sachin Chitta */

#include <door_msgs/Door.h>
#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <pr2_robot_actions/DoorActionState.h>

#include <tf/transform_listener.h>
#include <sbpl_door_planner_action/sbpl_door_planner_action.h>
#include <ros/node.h>

// -----------------------------------
//              MAIN
// -----------------------------------

int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  ros::Node node("action_runner_sbpl_door_planner");
  tf::TransformListener tf(node, true, ros::Duration(10));

  SBPLDoorPlanner open(node,tf);

  door_msgs::Door door;
  door_msgs::Door feedback;

  double tmp; int tmp2;
  node.param("~p_door_frame_p1_x", tmp, 0.65); door.frame_p1.x = tmp;
  node.param("~p_door_frame_p1_y", tmp, 0.45); door.frame_p1.y = tmp;
  node.param("~p_door_frame_p2_x", tmp, 0.65); door.frame_p2.x = tmp;
  node.param("~p_door_frame_p2_y", tmp, -0.45); door.frame_p2.y = tmp;
  node.param("~p_door_hinge" , tmp2, 0); door.hinge = tmp2;
  node.param("~p_door_rot_dir" , tmp2, 1); door.rot_dir = tmp2;
  door.header.frame_id = "base_link";
  door.normal.x = 1.0;
  door.normal.y = 0.0;
  door.normal.z = 0.0;
  door.door_p1 = door.frame_p1;
  door.door_p2 = door.frame_p2;

  door.handle.x = 0.65;
  door.handle.y = -0.4;
  door.handle.z = 0.0;

  ros::Time my_time = ros::Time::now();
  door.header.stamp = my_time;
  open.execute(door, feedback);

//  robot_actions::ActionRunner runner(10.0);
//  runner.connect<door_msgs::Door, pr2_robot_actions::DoorActionState, door_msgs::Door>(open);
//  runner.run();
//  node.spin();
  return 0;
}
