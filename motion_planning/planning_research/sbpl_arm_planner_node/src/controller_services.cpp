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

#include <pr2_mechanism_controllers/controller_services.h>


namespace controller_services
{
  void openGripper(const std::string &gripper_name)
  {
    actuateGripper(gripper_name,1);
  }

  void closeGripper(const std::string &gripper_name)
  {
    actuateGripper(gripper_name,0);
  }

  void actuateGripper(const std::string &gripper_name, const int &open)
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

  void goHome(const std:string &arm_name, const std::vector<double> &home_position)
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

  void sendTrajectory(const std::string &group_name, const robot_msgs::JointTraj &traj)
  {
    int trajectory_state = -1;

    robot_msgs::TrajectoryStart::Request  traj_request;
    robot_msgs::TrajectoryStart::Response traj_response;

    robot_msgs::TrajectoryQuery::Request  query_request;
    robot_msgs::TrajectoryQuery::Response query_response;

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
      }
      else
      {
        ROS_ERROR("Trajectory query failed");
      }
    } 
  }


  bool planPath(const std::string &arm_name, const robot_msgs::JointTrajPoint &joint_start, const robot_msgs::Pose &pose_goal, robot_msgs::JointTraj &planned_path)
  {
    robarm3d::PlanPathSrv::Request  request;
    robarm3d::PlanPathSrv::Response response;

    request.type = "cartesian";
    request.start = joint_start;
    request.cartesian_goal = pose_goal;

    if(ros::service::call(arm_name + "/plan_path/GetPlan",request,response))
    {
      planned_path = res_plan_path.traj;    
      return true;
    }
    else
    {
      ROS_ERROR("Could not get path");
      return false;
    }
  }


  bool planPath(const std::string &arm_name, const robot_msgs::JointTrajPoint &joint_start, const robot_msgs::JointTrajPoint &joint_goal, robot_msgs::JointTraj &planned_path)
  {
    robarm3d::PlanPathSrv::Request  request;
    robarm3d::PlanPathSrv::Response response;

    request.type = "joint_space";
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
}
