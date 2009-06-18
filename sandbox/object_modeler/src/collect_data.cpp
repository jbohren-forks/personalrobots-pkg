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
#include <robot_msgs/PoseStamped.h>
#include <robot_actions/action_client.h>
#include <nav_robot_actions/MoveBaseState.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <LinearMath/btQuaternion.h>

using namespace std;

typedef robot_msgs::PoseStamped PS;

void print_usage() {
  ROS_INFO("collect_data filename\n"
	   "filename contains poses, ie (point, orientation) where point is\n"
	   "a triplet x,y,z and orientation is a quaternion x,y,z,w.\n"
	   "The first pose is the target (the object to look at), the\n"
	   "subsequent poses are positions of the robot. The robot will\n"
	   "keep looking at the target");
}


void read_poses_from_file(vector<robot_msgs::Point> &result, const string &filename) {
  std::ifstream goal_file(filename.c_str());
  if (goal_file.is_open()) {
    string line;
    while (!goal_file.eof()) {
      getline(goal_file, line);
      istringstream line_stream(line);
      robot_msgs::Point p;
      robot_msgs::Quaternion q;
      line_stream >> p.x >> p.y >> p.z >> q.x >> q.y >> q.z >> q.w;
      result.push_back(p);
    }
    goal_file.close();
  }
}

robot_msgs::Quaternion direction(robot_msgs::Point position, robot_msgs::Point target) {
  double vec_x = target.x - position.x;
  double vec_y = target.y - position.y;
  double yaw = atan2(vec_y, vec_x);
  robot_msgs::Quaternion result;
  btQuaternion dir(yaw, 0, 0);
  result.x = dir.getX();
  result.y = dir.getY();  // WARNING! MAY NOT USE THE SAME CONVENTION
  result.z = dir.getZ();
  result.w = dir.getW();
  return result;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_modeler");
  //ros::NodeHandle n;
  
  robot_actions::ActionClient<PS, nav_robot_actions::MoveBaseState, PS> move_client("move_base");

  vector<robot_msgs::Point> goal_points;
  read_poses_from_file(goal_points, "poses.txt");
  if (goal_points.size() < 2) {
    print_usage();
    exit(0);
  }

  robot_msgs::Point target = goal_points[0];

  typedef vector<robot_msgs::Point>::const_iterator I;
  for (I i = goal_points.begin() + 1; i != goal_points.end(); i++) {
    robot_msgs::Pose goal_pose;
    goal_pose.position = *i;
    goal_pose.orientation = direction(*i, target);
    PS goal_pose_stamped;
    goal_pose_stamped.pose = goal_pose;
    PS feedback;
    robot_actions::ResultStatus result = move_client.execute(goal_pose_stamped, feedback, ros::Duration(30));
    switch (result) {
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
  }
}
