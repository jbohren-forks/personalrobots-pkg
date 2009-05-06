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

#include <kdl/frames.hpp>
#include <door_handle_detector/door_functions.h>
#include "doors_core/executive_functions.h"


using namespace KDL;
using namespace ros;
using namespace std;
using namespace tf;
using namespace door_handle_detector;


static const double gripper_height = 0.8;


Stamped<Pose> getRobotPose(const robot_msgs::Door& door, double dist)
{
  Vector x_axis(1,0,0);

  // get hinge point
  Vector hinge, frame_vec;
  if (door.hinge == robot_msgs::Door::HINGE_P1){
    hinge = Vector(door.door_p1.x, door.door_p1.y, door.door_p1.z);
    frame_vec = Vector(door.frame_p2.x - door.frame_p1.x, door.frame_p2.y - door.frame_p1.y, door.frame_p2.z - door.frame_p1.z);
  }
  else if (door.hinge == robot_msgs::Door::HINGE_P2){
    hinge = Vector(door.door_p2.x, door.door_p2.y, door.door_p2.z);
    frame_vec = Vector(door.frame_p1.x - door.frame_p2.x, door.frame_p1.y - door.frame_p2.y, door.frame_p1.z - door.frame_p2.z);
  }
  else
    ROS_ERROR("GetRobotPose: door hinge side not specified");

  // get vector to center of frame

  frame_vec.Normalize();
  double door_width = Vector(door.door_p1.x - door.door_p2.x, door.door_p1.y - door.door_p2.y, door.door_p1.z - door.door_p2.z).Norm();
  Vector frame_center = hinge + (frame_vec * door_width / 2.0);

  // get normal on frame
  Vector normal_door(door.normal.x, door.normal.y, door.normal.z);
  Rotation rot_door_frame = Rotation::RotZ(-getDoorAngle(door));
  Vector normal_frame = rot_door_frame * normal_door;

  // get robot pos
  Vector robot_pos = frame_center + (normal_frame * dist);

  Stamped<Pose> robot_pose;
  robot_pose.frame_id_ = door.header.frame_id;
  robot_pose.stamp_ = door.header.stamp;
  robot_pose.setOrigin( Vector3(robot_pos(0), robot_pos(1), robot_pos(2)));
  robot_pose.setRotation( Quaternion(getVectorAngle(x_axis, normal_frame), 0, 0) ); 

  return robot_pose;  
}


Stamped<Pose> getGripperPose(const robot_msgs::Door& door, double angle, double dist)
{
  Vector x_axis(1,0,0);

  // get hinge point
  Vector hinge, frame_vec;
  if (door.hinge == robot_msgs::Door::HINGE_P1){
    hinge = Vector(door.door_p1.x, door.door_p1.y, door.door_p1.z);
    frame_vec = Vector(door.frame_p2.x - door.frame_p1.x, door.frame_p2.y - door.frame_p1.y, door.frame_p2.z - door.frame_p1.z);
  }
  else if (door.hinge == robot_msgs::Door::HINGE_P2){
    hinge = Vector(door.door_p2.x, door.door_p2.y, door.door_p2.z);
    frame_vec = Vector(door.frame_p1.x - door.frame_p2.x, door.frame_p1.y - door.frame_p2.y, door.frame_p1.z - door.frame_p2.z);
  }
  else
    ROS_ERROR("GetRobotPose: door hinge side not specified");

  // get vector from hinge to goal point
  frame_vec.Normalize();
  frame_vec = frame_vec * dist;
  Rotation rot_frame_angle = Rotation::RotZ(angle);
  frame_vec = rot_frame_angle * frame_vec;

  // get the normal on the resulting door at "angle"
  Vector normal_door(door.normal.x, door.normal.y, door.normal.z);
  Rotation rot_door_res = Rotation::RotZ(-getDoorAngle(door)+angle);
  Vector normal_res = rot_door_res * normal_door;

  // get gripper pos
  Vector gripper_pos = hinge + frame_vec;

  Stamped<Pose> gripper_pose;
  gripper_pose.frame_id_ = door.header.frame_id;
  gripper_pose.stamp_ = door.header.stamp;
  gripper_pose.setOrigin( Vector3(gripper_pos(0), gripper_pos(1), gripper_height));
  gripper_pose.setRotation( Quaternion(getVectorAngle(x_axis, normal_res), 0, 0) ); 

  /*
  cout << "gripper pose is " << gripper_pose.getOrigin()[0] << ", " 
       << gripper_pose.getOrigin()[1] << ", " << gripper_pose.getOrigin()[2] << endl;
  cout << door << endl;
  */

  return gripper_pose;  
}



tf::Vector3 getFrameNormal(const robot_msgs::Door& door)
{
  Vector normal_door(door.normal.x, door.normal.y, door.normal.z);
  Rotation rot_door_frame = Rotation::RotZ(-getDoorAngle(door));
  Vector normal_frame = rot_door_frame * normal_door;
  tf::Vector3 res;
  res[0] = normal_frame(0);
  res[1] = normal_frame(1);
  res[2] = normal_frame(2);
  return res;
}
