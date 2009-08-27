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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/*
 * Author: Sachin Chitta
 */

#include <algorithm>
#include <pr2_ik/pr2_ik_controller_new.h>

#include "ros/node.h"


using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace pr2_ik {

  PR2IKController::PR2IKController()
      :dimension_(7)
  {
    node_handle_.param<std::string>("~root_name", root_name_, "torso_lift_link");
    node_handle_.param<std::string>("~control_topic_name",control_topic_name_,"/r_arm_joint_trajectory_controller/command");
    node_handle_.param<std::string>("~control_service_name",control_service_name_,"/r_arm_joint_trajectory_controller/TrajectoryStart");
    node_handle_.param<bool>("~free_angle_constraint",free_angle_constraint_,false);
    node_handle_.param<int>("~dimension",dimension_,7);
    invalid_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/invalid",1);
    control_pub_ = node_handle_.advertise<manipulation_msgs::JointTraj>(control_topic_name_,1);
    command_sub_ = node_handle_.subscribe("~command",20,&PR2IKController::command,this);

    if(free_angle_constraint_)
    {
      double free_angle_constraint_min, free_angle_constraint_max;
      node_handle_.param<double>("~free_angle_constraint_min",free_angle_constraint_min,-M_PI);
      node_handle_.param<double>("~free_angle_constraint_max",free_angle_constraint_max,M_PI);
      pr2_ik_solver_.pr2_ik_->min_angles_[pr2_ik_solver_.pr2_ik_->free_angle_] = free_angle_constraint_min;
      pr2_ik_solver_.pr2_ik_->max_angles_[pr2_ik_solver_.pr2_ik_->free_angle_] = free_angle_constraint_max;
    }
    this->init();
  }

  PR2IKController::~PR2IKController()
  {
  }

  bool PR2IKController::init()
  {
    ROS_INFO("Publishing to controller on topic name %s",control_topic_name_.c_str());
    ROS_DEBUG("Initialized IK controller");
    return true;
  }

  void PR2IKController::command(const manipulation_msgs::IKRequestConstPtr &pose_msg)
  {
    manipulation_msgs::IKRequest pose_msg_in = *pose_msg;
    tf::Stamped<tf::Pose> pose_stamped;
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg = pose_msg_in.pose_stamped;
    pose_stamped_msg.header = pose_msg_in.pose_stamped.header;
    pose_stamped_msg.header.stamp = ros::Time();

    poseStampedMsgToTF(pose_stamped_msg, pose_stamped);
    ROS_DEBUG("Converted pose command to tf");

    // convert to reference frame of root link of the chain
    tf_.transformPose(root_name_, pose_stamped, pose_stamped);
    ROS_DEBUG("Converted tf command to root name");

    poseToFrame(pose_stamped, pose_desired_);
    ROS_DEBUG("Converted tf command to KDL");

    poseStampedTFToMsg(pose_stamped,pose_stamped_msg);
    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    if((int)pose_msg_in.positions.size() == dimension_)
    {
      for(int i=0; i < dimension_; i++)
        jnt_pos_in(i) = pose_msg_in.positions[i];
    }
    else
    {
      for(int i=0; i < dimension_; i++)
        jnt_pos_in(i) = -0.2;
    }
    bool ik_valid = (pr2_ik_solver_.CartToJntSearch(jnt_pos_in,pose_desired_,jnt_pos_out,10) >= 0);

    if(ik_valid)
    {
      manipulation_msgs::JointTraj joint_traj;
      joint_traj.set_points_size(1);
      joint_traj.points[0].set_positions_size(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        joint_traj.points[0].positions[i] = jnt_pos_out(i);
        ROS_DEBUG("IK Cmd %d: %f",i,jnt_pos_out(i));
      }
      experimental_controllers::TrajectoryStart srv;
      srv.request.traj = joint_traj;
      ros::ServiceClient control_srv = node_handle_.serviceClient<experimental_controllers::TrajectoryStart>(control_service_name_);
      if(control_srv.call(srv))
      {
        ROS_DEBUG("IK controller service success");
      }
      else
      {
        ROS_DEBUG("IK controller service failed");
      }
//      control_pub_.publish(joint_traj);
      ROS_DEBUG("IK valid: %d", pose_msg_in.pose_stamped.header.seq);   
    }
    else
    {
      invalid_pub_.publish(pose_stamped_msg);
      ROS_ERROR("IK controller solution invalid");   
    }
  }

  void PR2IKController::poseToFrame(const tf::Pose& pose, KDL::Frame& frame)
  {
    frame.p(0) = pose.getOrigin().x();
    frame.p(1) = pose.getOrigin().y();
    frame.p(2) = pose.getOrigin().z();

    double Rz, Ry, Rx;
    pose.getBasis().getEulerZYX(Rz, Ry, Rx);
    frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
  }
} // namespace

int main(int argc, char** argv){
  ros::init(argc, argv,"right_arm_ik_controller");
  pr2_ik::PR2IKController pr2_ik_controller;
  ros::spin();
  return(0);
}
