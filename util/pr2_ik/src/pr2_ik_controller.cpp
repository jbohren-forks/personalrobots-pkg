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
#include <pr2_ik/pr2_ik_controller.h>


using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

namespace pr2_ik {

  PR2IKController::PR2IKController()
      : node_(ros::Node::instance()),
        tf_(*node_),
        command_notifier_(NULL),
        dimension_(7)
  {
    node_->param<std::string>("~root_name", root_name_, "torso_lift_link");
    node_->param<std::string>("~control_topic_name",control_topic_name_,"r_arm_joint_trajectory_controller/command");
    node_->param<bool>("~free_angle_constraint",free_angle_constraint_,false);
    node_->param<int>("~dimension",dimension_,7);
    node_->advertise<robot_msgs::JointTraj>(control_topic_name_, 1);

    if(free_angle_constraint_)
    {
      double free_angle_constraint_min, free_angle_constraint_max;
      node_->param<double>("~free_angle_constraint_min",free_angle_constraint_min,-M_PI);
      node_->param<double>("~free_angle_constraint_max",free_angle_constraint_max,M_PI);
      pr2_ik_solver_.pr2_ik_->min_angles_[pr2_ik_solver_.pr2_ik_->free_angle_] = free_angle_constraint_min;
      pr2_ik_solver_.pr2_ik_->max_angles_[pr2_ik_solver_.pr2_ik_->free_angle_] = free_angle_constraint_max;
    }
    this->init();
  }

  PR2IKController::~PR2IKController()
  {
        node_->unadvertise(control_topic_name_);
  }

  bool PR2IKController::init()
  {
    command_notifier_.reset(new MessageNotifier<pr2_ik::PoseCmd>(&tf_, node_,
                                                                         boost::bind(&PR2IKController::command, this, _1),
                                                                         "~command", root_name_, 10));
    ROS_DEBUG("Initialized IK controller");
    return true;
  }

  void PR2IKController::command(const tf::MessageNotifier<pr2_ik::PoseCmd>::MessagePtr& pose_msg)
  {
    ROS_DEBUG("Got pose command");
    pr2_ik::PoseCmd pose_msg_in = *pose_msg;
    // convert message to transform
    Stamped<Pose> pose_stamped;
    robot_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.pose = pose_msg_in.pose;
    pose_stamped_msg.header = pose_msg_in.header;

    PoseStampedMsgToTF(pose_stamped_msg, pose_stamped);
    ROS_DEBUG("Converted pose command to tf");

    // convert to reference frame of root link of the chain
    tf_.transformPose(root_name_, pose_stamped, pose_stamped);
    ROS_DEBUG("Converted tf command to root name");

    poseToFrame(pose_stamped, pose_desired_);
    ROS_DEBUG("Converted tf command to KDL");

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
      jnt_pos_in(i) = 0.0;
    jnt_pos_in(pr2_ik_solver_.pr2_ik_->free_angle_) = pose_msg_in.free_angle_value;
    bool ik_valid = (pr2_ik_solver_.CartToJntSearch(jnt_pos_in,pose_desired_,jnt_pos_out,10) >= 0);

    if(ik_valid)
    {
      robot_msgs::JointTraj joint_traj;
      joint_traj.set_points_size(1);
      joint_traj.points[0].set_positions_size(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        joint_traj.points[0].positions[i] = jnt_pos_out(i);
        ROS_DEBUG("IK Cmd %d: %f",i,jnt_pos_out(i));
      }
      ROS_DEBUG(" ");
      node_->publish(control_topic_name_,joint_traj);
      ROS_DEBUG("Publishing to controller on topic name %s",control_topic_name_.c_str());
      ROS_INFO("IK valid");   
    }
    else
    {
      ROS_ERROR("IK invalid");   
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
  ros::init(argc, argv);
  ros::Node ros_node("right_arm_ik_controller");
  pr2_ik::PR2IKController pr2_ik_controller;
  ros_node.spin();
  return(0);
}
