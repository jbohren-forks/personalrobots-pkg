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
 *   * Neither the name of the Willow Garage nor the names of its
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

// Author Romain Thibaux (thibaux@willowgarage.com)

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <robot_actions/action_client.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#ifndef BT_EULER_DEFAULT_ZYX
#error It looks like the bullet package was compiled without BT_EULER_DEFAULT_ZYX set
#endif
#include <LinearMath/btQuaternion.h>
//#include <annotated_map_builder/WaitActionState.h>
//#include <annotated_map_builder/WaitActionGoal.h>

using namespace std;

void print_usage() {
  ROS_INFO("collect_data filename\n"
	   "filename contains poses, ie (point, orientation) where point is\n"
	   "a triplet x,y,z and orientation is a quaternion x,y,z,w.\n"
	   "The first pose is the target (the object to look at), the\n"
	   "subsequent poses are positions of the robot. The robot will\n"
	   "keep looking at the target");
}


void read_poses_from_file(vector<geometry_msgs::Point> &result, const string &filename) {
  std::ifstream goal_file(filename.c_str());
  if (goal_file.is_open()) {
    string line;
    while (!goal_file.eof()) {
      getline(goal_file, line);
      istringstream line_stream(line);
      geometry_msgs::Point p;
      geometry_msgs::Quaternion q;
      line_stream >> p.x >> p.y >> p.z >> q.x >> q.y >> q.z >> q.w;
      result.push_back(p);
    }
    goal_file.close();
  }
}

geometry_msgs::Quaternion direction(geometry_msgs::Point position, geometry_msgs::Point target) {
  // By default bullet uses the 'XYZ' convention where rotation around Z is
  // called 'roll', but we compiled the bullet library with the
  // BT_EULER_DEFAULT_ZYX parameter set, so the 'ZYX' convention is used
  // instead. It calls the rotation around Z 'yaw', which better
  // corresponds to the convention used for the robot pose. To ensure
  // unambiguous conversion I do not allow compilation unless BT_EULER_DEFAULT_ZYX
  // is set. It should be because we export this parameter to anything that
  // depends on bullet.
  double vec_x = target.x - position.x;
  double vec_y = target.y - position.y;
  double yaw = atan2(vec_y, vec_x);
  geometry_msgs::Quaternion result;
  btQuaternion dir(yaw, 0, 0);
  result.x = dir.getX();
  result.y = dir.getY();
  result.z = dir.getZ();
  result.w = dir.getW();
  return result;
}



int main(int argc, char** argv)
{
  ROS_DEBUG("Initializing");
  ros::init(argc, argv, "object_modeler");
  ros::NodeHandle n;
  
  ROS_DEBUG("Declaring action clients");
  typedef geometry_msgs::PoseStamped PS;
  robot_actions::ActionClient<PS, nav_robot_actions::MoveBaseState, PS> move_client("move_base");
  //typedef annotated_map_builder::WaitActionState WaitState;
  //typedef annotated_map_builder::WaitActionGoal WaitGoal;
  //robot_actions::ActionClient<WaitGoal, WaitState, WaitState> wait_client("wait_k_messages_action");
  
  ROS_DEBUG("Reading way points");
  vector<geometry_msgs::Point> goal_points;
  read_poses_from_file(goal_points, "poses.txt");
  if (goal_points.size() < 2) {
    print_usage();
    exit(0);
  }

  geometry_msgs::Point target = goal_points[0];

  ROS_DEBUG("Reading way points");
  typedef vector<geometry_msgs::Point>::const_iterator I;
  for (I i = goal_points.begin() + 1; i != goal_points.end(); i++) {
    geometry_msgs::Pose goal_pose;
    goal_pose.position = *i;
    goal_pose.orientation = direction(*i, target);
    PS goal_pose_stamped;
    goal_pose_stamped.pose = goal_pose;
    goal_pose_stamped.header.stamp = ros::Time::now();
    goal_pose_stamped.header.frame_id = "/map";
    PS move_feedback;
    ROS_INFO("Sending move command");
    robot_actions::ResultStatus result_move = move_client.execute(goal_pose_stamped, move_feedback, ros::Duration(60));
    switch (result_move) {
    case robot_actions::SUCCESS:
      ROS_INFO("Move successful");
      break;
    case robot_actions::ABORTED:
      ROS_INFO("Move aborted");
      break;
    case robot_actions::PREEMPTED:
      ROS_INFO("Move preempted");
      break;
      }
    /*
    WaitGoal wait_goal;
    wait_goal.num_events = 3;
    wait_goal.topic_name = "/stereo/raw_stereo_throttled";
    WaitState wait_feedback;
    robot_actions::ResultStatus result_wait = wait_client.execute(wait_goal, wait_feedback, ros::Duration(30));
    switch (result_wait) {
    case robot_actions::SUCCESS:
      ROS_INFO("Wait successful");
      break;
    case robot_actions::ABORTED:
      ROS_INFO("Wait aborted");
      break;
    case robot_actions::PREEMPTED:
      ROS_INFO("Wait preempted");
      break;
      }*/
  
    ROS_INFO("Waiting 3 seconds");
    ros::Duration wait_time;
    wait_time.fromSec(3);
    wait_time.sleep();
    ROS_INFO("Done waiting");
  }
}
