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
#include <ros/node.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float64.h>

#include <robot_msgs/Pose.h>
#include <robot_msgs/JointTraj.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_msgs/JointTrajPoint.h>

#include <pr2_mechanism_controllers/TrajectoryStart.h>
#include <pr2_mechanism_controllers/TrajectoryQuery.h>

#include <sbpl_arm_planner_node/PlanPathSrv.h>

namespace pr2_arm_node
{
  class PR2ArmNode : public ros::Node
  {
    private:

    tf::TransformListener tf_; 

    public:

      PR2ArmNode(std::string node_name);

      ~PR2ArmNode(){};

      static const double GRIPPER_OPEN  = 0.6;

      static const double GRIPPER_CLOSE = 0.1;

      static const double GRIPPER_OPEN_EFFORT = 6;

      static const double GRIPPER_CLOSE_EFFORT = 2;

      void getCurrentPosition(const std::string &group_name);

      void openGripper(const std::string &gripper_name);

      void closeGripper(const std::string &gripper_name);

      void openGripperEffort(const std::string &gripper_name);

      void closeGripperEffort(const std::string &gripper_name);

      void actuateGripper(const std::string &gripper_name, const int &open);

      void actuateGripperEffort(const std::string &gripper_name, const int &open);

      void goHome(const std::string &arm_name, const std::vector<double> &home_position);

      bool sendTrajectory(const std::string &group_name, const robot_msgs::JointTraj &traj);

      bool planSBPLPath(const std::string &arm_name, const robot_msgs::JointTrajPoint &joint_start, const std::vector<robot_msgs::Pose> &pose_goals, robot_msgs::JointTraj &planned_path);

      bool planSBPLPath(const std::string &arm_name, const robot_msgs::JointTrajPoint &joint_start, const robot_msgs::JointTrajPoint &joint_goal, robot_msgs::JointTraj &planned_path);

      void getCurrentPosition(const std::string &group_name, robot_msgs::JointTrajPoint &current_joint_positions);

      robot_msgs::Pose RPYToTransform(double roll, double pitch, double yaw, double x, double y, double z);

  };
}
