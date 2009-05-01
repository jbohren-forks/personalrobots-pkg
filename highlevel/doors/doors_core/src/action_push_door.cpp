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

#include <door_handle_detector/door_functions.h>
#include <robot_msgs/PoseStamped.h>
#include "doors_core/executive_functions.h"
#include "doors_core/action_push_door.h"

using namespace tf;
using namespace KDL;
using namespace ros;
using namespace std;
using namespace door_handle_detector;

static const string fixed_frame = "odom_combined";
static const double push_dist = 0.65;
static const double push_vel  = 5.0 * M_PI/180.0;  // 5 [deg/sec]



PushDoorAction::PushDoorAction(Node& node) : 
  robot_actions::Action<robot_msgs::Door, robot_msgs::Door>("push_door"), 
  node_(node),
  tf_(node)
{
  node_.advertise<std_msgs::Float64>("r_gripper_effort_controller/command",10);
  node_.advertise<robot_msgs::PoseStamped>("r_arm_cartesian_pose_controller/command",20);
};


PushDoorAction::~PushDoorAction()
{
  node_.unadvertise("r_gripper_effort_controller/command");
  node_.unadvertise("r_arm_cartesian_pose_controller/command");
};



robot_actions::ResultStatus PushDoorAction::execute(const robot_msgs::Door& goal, robot_msgs::Door& feedback)
{
  ROS_INFO("PushDoorAction: execute");
 
  // set default feedback
  feedback = goal;
 
  // transform door message to time fixed frame
  robot_msgs::Door goal_tr;
  if (!transformTo(tf_, fixed_frame, goal, goal_tr, fixed_frame)){
    ROS_ERROR("PushDoorAction: Could not tranform door message from '%s' to '%s' at time %f",
	      goal.header.frame_id.c_str(), fixed_frame.c_str(), goal.header.stamp.toSec());
    return robot_actions::ABORTED;
  }

  // close the gripper 
  std_msgs::Float64 gripper_msg;
  gripper_msg.data = -20.0;
  node_.publish("r_gripper_effort_controller/command", gripper_msg);
  
  // angle step
  Duration sleep_time(0.01);
  double angle_step, angle=0;
  if (goal_tr.rot_dir == robot_msgs::Door::ROT_DIR_CLOCKWISE)
    angle_step = -push_vel*sleep_time.toSec();
  else if (goal_tr.rot_dir == robot_msgs::Door::ROT_DIR_COUNTERCLOCKWISE)
    angle_step = push_vel*sleep_time.toSec();
  else{
    ROS_ERROR("PushDoorAction: door rotation direction not specified");
    return robot_actions::ABORTED;
  }

  // push door
  Stamped<Pose> gripper_pose;
  robot_msgs::PoseStamped gripper_pose_msg;
  while (fabs(angle) < M_PI/2.0){
    // define griper pose
    gripper_pose = getGripperPose(goal_tr, angle, push_dist);
    gripper_pose.stamp_ = Time::now();
    PoseStampedTFToMsg(gripper_pose, gripper_pose_msg);
    node_.publish("r_arm_cartesian_pose_controller/command", gripper_pose_msg);

    // increase angle
    angle += angle_step;

    // check for preemption
    if (isPreemptRequested()) {
      ROS_ERROR("PushDoorAction: preempted");
      return robot_actions::PREEMPTED;
    }

    sleep_time.sleep();
  }

  feedback = goal_tr;
  return robot_actions::SUCCESS;
}


