/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sbpl_arm_executive/pr2_arm_node.h>

using namespace pr2_arm_node;

PR2ArmNode::PR2ArmNode(std::string node_name, std::string arm_name, std::string gripper_name):ros::Node(node_name),tf_(*this),arm_name_(arm_name),gripper_name_(gripper_name)
{
//  param<std::string>("~arm_name",arm_name_, "right_arm");
//  param<std::string>("~gripper_name",arm_name_, "right_gripper");
  param<std::string>("~arm_trajectory_topic_name",trajectory_topic_name_, "/trajectory_controller/trajectory_command");
  param<std::string>("~arm_trajectory_service_start",trajectory_start_name_, "/trajectory_controller/TrajectoryStart");
  param<std::string>("~arm_trajectory_service_query",trajectory_query_name_, "/trajectory_controller/TrajectoryQuery");
  param<std::string>("~effort_command",effort_controller_command_name_, "/effort_controller/set_command");
  param<std::string>("~sbpl_planner_service_name",sbpl_planner_service_name_, "/sbpl_planner/plan_path/GetPath");

  param("use_gripper_effort_controller",use_gripper_effort_controller_,true);

  if(use_gripper_effort_controller_)
  {
    advertise<std_msgs::Float64>(gripper_name_ + effort_controller_command_name_,1);
  }
  else
  {
    advertise<robot_msgs::JointTraj>(gripper_name_ + trajectory_topic_name_,1);
  }
  advertise<robot_msgs::JointTraj>(arm_name_ + trajectory_topic_name_,1);
  advertise<robot_msgs::JointTraj>("head" + trajectory_topic_name_,1);
}

PR2ArmNode::~PR2ArmNode()
{
  if(use_gripper_effort_controller_)
  {
    unadvertise(gripper_name_ + effort_controller_command_name_);
  }
  else
  {
    unadvertise(gripper_name_ + trajectory_topic_name_);
  }
  unadvertise(arm_name_ + trajectory_topic_name_);
}

void PR2ArmNode::pointHead(double yaw, double pitch)
{
  robot_msgs::JointTraj traj;

  traj.set_points_size(1);
  traj.points[0].set_positions_size(2);

  traj.points[0].positions[0] = yaw;
  traj.points[0].positions[1] = pitch;

  sendTrajectory("head",traj);
}

void PR2ArmNode::nodHead()
{
  robot_msgs::JointTraj traj;

  traj.set_points_size(4);
  for(int i=0; i<4; i++)
  {
    traj.points[i].set_positions_size(2);
    traj.points[i].positions[0] = 0.0;
    traj.points[i].positions[1] = pow(-1.0,i)*0.5;
    traj.points[i].time = 0.0;
  }
  sendTrajectory("head",traj);
}

void PR2ArmNode::shakeHead()
{
  robot_msgs::JointTraj traj;

  traj.set_points_size(4);
  for(int i=0; i<4; i++)
  {
    traj.points[i].set_positions_size(2);
    traj.points[i].positions[0] = pow(-1.0,i)*0.5;
    traj.points[i].positions[1] = 0.0;
    traj.points[i].time = 0.0;
  }
  sendTrajectory("head",traj);
}

void PR2ArmNode::openGripper()
{
  actuateGripper(1);
}

void PR2ArmNode::closeGripper()
{
  actuateGripper(0);
}

void PR2ArmNode::actuateGripper(const int &open)
{
  robot_msgs::JointTraj traj;

  traj.set_points_size(1);
  traj.points[0].set_positions_size(1);

  if(open)
    traj.points[0].positions[0] = GRIPPER_OPEN;
  else
    traj.points[0].positions[0] = GRIPPER_CLOSE;

  sendTrajectory(gripper_name_,traj);
}

void PR2ArmNode::openGripperEffort()
{
  actuateGripperEffort(1);
}

void PR2ArmNode::closeGripperEffort()
{
  actuateGripperEffort(0);
}

void PR2ArmNode::actuateGripperEffort(const int &open)
{
  std_msgs::Float64 cmd;
  advertise<std_msgs::Float64>(gripper_name_ + effort_controller_command_name_,1);
  if(open)
    cmd.data = GRIPPER_OPEN_EFFORT;
  else
    cmd.data = GRIPPER_CLOSE_EFFORT;      
  publish(gripper_name_ + effort_controller_command_name_,cmd);
}

void PR2ArmNode::goHome(const std::vector<double> &home_position)
{
  int num_joints = home_position.size();

  robot_msgs::JointTraj traj;

  traj.set_points_size(1);
  traj.points[0].set_positions_size(num_joints);

  for(int i=0; i<num_joints; i++)
  {
    traj.points[0].positions[i] = home_position[i];
  }

  sendTrajectory(arm_name_,traj);
}

bool PR2ArmNode::sendTrajectory(std::string group_name, const robot_msgs::JointTraj &traj)
{
  int trajectory_state = -1;

  pr2_mechanism_controllers::TrajectoryStart::Request  traj_request;
  pr2_mechanism_controllers::TrajectoryStart::Response traj_response;

  pr2_mechanism_controllers::TrajectoryQuery::Request  query_request;
  pr2_mechanism_controllers::TrajectoryQuery::Response query_response;

  traj_request.traj = traj;

  if (ros::service::call(group_name + trajectory_start_name_, traj_request, traj_response))
  {
    ROS_INFO("Done");
  }
  query_request.trajectoryid =  traj_response.trajectoryid;
  while(!(trajectory_state == query_response.State_Done || trajectory_state == query_response.State_Failed))
  {
    if(ros::service::call(group_name + trajectory_query_name_, query_request, query_response))  
    {
      trajectory_state =  query_response.done;
      return true;
    }
    else
    {
      ROS_ERROR("Trajectory query failed");
      return false;
    }
  } 
  return true;
}

void PR2ArmNode::sendTrajectoryOnTopic(std::string group_name,const robot_msgs::JointTraj &traj)
{
  publish(group_name + trajectory_topic_name_,traj);
}

void PR2ArmNode::getCurrentPosition(robot_msgs::JointTrajPoint &current_joint_positions)
{
  int num_joints = 7;
  pr2_mechanism_controllers::TrajectoryQuery::Request  req_traj_query;
  pr2_mechanism_controllers::TrajectoryQuery::Response res_traj_query;

  req_traj_query.trajectoryid = 0;

  ROS_INFO("Calling service with: %s",(arm_name_ + trajectory_query_name_).c_str());
  if(ros::service::call(arm_name_ + trajectory_query_name_, req_traj_query, res_traj_query))  
  {
    current_joint_positions.set_positions_size(7);
    for(int i=0; i < num_joints; i++)
    {
      current_joint_positions.positions[i] = res_traj_query.jointpositions[i];
    }
  }
  else
  {
    ROS_ERROR("Could not get initial joint angles");    
  }
}


bool PR2ArmNode::planSBPLPath(const robot_msgs::JointTrajPoint &joint_start, const std::vector<robot_msgs::Pose> &pose_goals, robot_msgs::JointTraj &planned_path)
{
  sbpl_arm_planner_node::PlanPathSrv::Request  request;
  sbpl_arm_planner_node::PlanPathSrv::Response response;

  request.type.data = "cartesian";
  request.start = joint_start;
  request.cartesian_goals = pose_goals;

  if(ros::service::call(arm_name_ + sbpl_planner_service_name_,request,response))
  {
    planned_path = response.traj;    
    return true;
  }
  else
  {
    ROS_ERROR("Could not get path");
    return false;
  }
}


bool PR2ArmNode::planSBPLPath(const robot_msgs::JointTrajPoint &joint_start, const robot_msgs::JointTrajPoint &joint_goal, robot_msgs::JointTraj &planned_path)
{
  sbpl_arm_planner_node::PlanPathSrv::Request  request;
  sbpl_arm_planner_node::PlanPathSrv::Response response;

  request.type.data = "joint_space";
  request.start = joint_start;
  request.joint_goal = joint_goal;

  if(ros::service::call(arm_name_ + sbpl_planner_service_name_,request,response))
  {
    planned_path = response.traj;    
    return true;
  }
  else
  {
    ROS_ERROR("Could not get path");
    return false;
  }
}

robot_msgs::Pose PR2ArmNode::RPYToTransform(double roll, double pitch, double yaw, double x, double y, double z)
{
  robot_msgs::Pose pose;
  tf::Quaternion quat_trans = tf::Quaternion(yaw,pitch,roll);

  pose.orientation.x = quat_trans.x();
  pose.orientation.y = quat_trans.y();
  pose.orientation.z = quat_trans.z();
  pose.orientation.w = quat_trans.w();

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  return pose;
}
