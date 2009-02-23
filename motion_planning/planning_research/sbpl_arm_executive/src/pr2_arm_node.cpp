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

PR2ArmNode::PR2ArmNode(std::string node_name):ros::Node(node_name),tf_(*this){}

void PR2ArmNode::openGripper(const std::string &gripper_name)
{
  actuateGripper(gripper_name,1);
}

void PR2ArmNode::closeGripper(const std::string &gripper_name)
{
  actuateGripper(gripper_name,0);
}

void PR2ArmNode::actuateGripper(const std::string &gripper_name, const int &open)
{
  robot_msgs::JointTraj traj;

  traj.set_points_size(1);
  traj.points[0].set_positions_size(1);

  if(open)
    traj.points[0].positions[0] = GRIPPER_OPEN;
  else
    traj.points[0].positions[0] = GRIPPER_CLOSE;

  sendTrajectory(gripper_name,traj);
}

void PR2ArmNode::openGripperEffort(const std::string &gripper_name)
{
  actuateGripperEffort(gripper_name,1);
}

void PR2ArmNode::closeGripperEffort(const std::string &gripper_name)
{
  actuateGripperEffort(gripper_name,0);
}

void PR2ArmNode::actuateGripperEffort(const std::string &gripper_name, const int &open)
{
  std_msgs::Float64 cmd;
  advertise<std_msgs::Float64>(gripper_name + "_controller/set_command",1);
  if(open)
    cmd.data = GRIPPER_OPEN_EFFORT;
  else
    cmd.data = GRIPPER_CLOSE_EFFORT;      
  publish(gripper_name + "_controller/set_command",cmd);
}

void PR2ArmNode::goHome(const std::string &arm_name, const std::vector<double> &home_position)
{
  int num_joints = home_position.size();

  robot_msgs::JointTraj traj;

  traj.set_points_size(1);
  traj.points[0].set_positions_size(num_joints);

  for(int i=0; i<num_joints; i++)
  {
    traj.points[0].positions[i] = home_position[i];
  }

  sendTrajectory(arm_name,traj);
}

bool PR2ArmNode::sendTrajectory(const std::string &group_name, const robot_msgs::JointTraj &traj)
{
  int trajectory_state = -1;

  pr2_mechanism_controllers::TrajectoryStart::Request  traj_request;
  pr2_mechanism_controllers::TrajectoryStart::Response traj_response;

  pr2_mechanism_controllers::TrajectoryQuery::Request  query_request;
  pr2_mechanism_controllers::TrajectoryQuery::Response query_response;

  traj_request.traj = traj;

  if (ros::service::call(group_name + "/trajectory_controller/TrajectoryStart", traj_request, traj_response))
  {
    ROS_INFO("Done");
  }
  query_request.trajectoryid =  traj_response.trajectoryid;
  while(!(trajectory_state == query_response.State_Done || trajectory_state == query_response.State_Failed))
  {
    if(ros::service::call(group_name + "/trajectory_controller/TrajectoryQuery", query_request, query_response))  
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
}

void PR2ArmNode::getCurrentPosition(const std::string &group_name, robot_msgs::JointTrajPoint &current_joint_positions)
{
  int num_joints = 7;
  pr2_mechanism_controllers::TrajectoryQuery::Request  req_traj_query;
  pr2_mechanism_controllers::TrajectoryQuery::Response res_traj_query;

  req_traj_query.trajectoryid = 0;

  if(ros::service::call(group_name + "/trajectory_controller/TrajectoryQuery", req_traj_query, res_traj_query))  
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


bool PR2ArmNode::planSBPLPath(const std::string &arm_name, const robot_msgs::JointTrajPoint &joint_start, const std::vector<robot_msgs::Pose> &pose_goals, robot_msgs::JointTraj &planned_path)
{
  sbpl_arm_planner_node::PlanPathSrv::Request  request;
  sbpl_arm_planner_node::PlanPathSrv::Response response;

  request.type.data = "cartesian";
  request.start = joint_start;
  request.cartesian_goals = pose_goals;

  if(ros::service::call(arm_name + "/plan_path/GetPlan",request,response))
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


bool PR2ArmNode::planSBPLPath(const std::string &arm_name, const robot_msgs::JointTrajPoint &joint_start, const robot_msgs::JointTrajPoint &joint_goal, robot_msgs::JointTraj &planned_path)
{
  sbpl_arm_planner_node::PlanPathSrv::Request  request;
  sbpl_arm_planner_node::PlanPathSrv::Response response;

  request.type.data = "joint_space";
  request.start = joint_start;
  request.joint_goal = joint_goal;

  if(ros::service::call(arm_name + "/plan_path/GetPlan",request,response))
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
