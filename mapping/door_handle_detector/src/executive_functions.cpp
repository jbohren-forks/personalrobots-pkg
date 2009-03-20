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

#include <door_handle_detector/executive_functions.h>
#include <kdl/frames.hpp>

using namespace KDL;
using namespace ros;
using namespace std;
using namespace tf;




Stamped<Pose> getRobotPose(const robot_msgs::Door& door, double dist)
{
  Vector normal(door.normal.x, door.normal.y, door.normal.z);
  Vector x_axis(1,0,0);
  double dot      = normal(0) * x_axis(0) + normal(1) * x_axis(1);
  double perp_dot = normal(1) * x_axis(0) - normal(0) * x_axis(1);
  double z_angle = atan2(perp_dot, dot);

  Vector center((door.door_p1.x + door.door_p2.x)/2.0, 
                (door.door_p1.y + door.door_p2.y)/2.0,
                (door.door_p1.z + door.door_p2.z)/2.0);
  Vector robot_pos = center - (normal * dist);

  Stamped<Pose> robot_pose;
  robot_pose.frame_id_ = door.header.frame_id;
  robot_pose.setOrigin( Vector3(robot_pos(0), robot_pos(1), robot_pos(2)));
  robot_pose.setRotation( Quaternion(z_angle, 0, 0) ); 

  return robot_pose;  
}




