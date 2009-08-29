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
 * Author: Wim Meeussen, Mrinal Kalakrishnan
 */



#include <algorithm>
#include <pr2_mechanism_control/mechanism_control.h>
#include "kdl/chainfksolverpos_recursive.hpp"
#include "experimental_controllers/cartesian_pose_twist_controller.h"
#include "kdl/chainfksolvervel_recursive.hpp"


using namespace KDL;
using namespace tf;
using namespace std;


namespace controller {


CartesianPoseTwistController::CartesianPoseTwistController()
: robot_state_(NULL),
  jnt_to_pose_solver_(NULL),
  state_error_publisher_(NULL),
  state_pose_actual_publisher_(NULL),
  state_pose_desired_publisher_(NULL),
  command_notifier_(NULL)
{}

CartesianPoseTwistController::~CartesianPoseTwistController()
{}


bool CartesianPoseTwistController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string tip_name;
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("CartesianPoseTwistController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("CartesianPoseTwistController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state_->model_, root_name_, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_pose_solver_.reset(new ChainFkSolverPos_recursive(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());

  // create solver
  jnt_to_twist_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  jnt_posvel_.resize(kdl_chain_.getNrOfJoints());

  // constructs 3 identical pid controllers: for the x,y and z translations
  control_toolbox::Pid pid_controller;
  if (!pid_controller.init(ros::NodeHandle(node_, "fb_trans"))) return false;
  for (unsigned int i=0; i<3; i++)
    pid_controller_.push_back(pid_controller);

  // constructs 3 identical pid controllers: for the x,y and z rotations
  if (!pid_controller.init(ros::NodeHandle(node_, "fb_rot"))) return false;
  for (unsigned int i=0; i<3; i++)
    pid_controller_.push_back(pid_controller);

  // get a pointer to the wrench controller
  string output;
  if (!node_.getParam("output", output)){
    ROS_ERROR("No ouptut name found on parameter server (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  ros::Duration timeout(20);
  ros::Time start_time = ros::Time::now();
  bool found_controller = false;
  do
  {
    found_controller = getController<CartesianWrenchController>(output, AFTER_ME, wrench_controller_);

    if (!found_controller)
    {
      ros::Duration(0.5).sleep();
    }
  }
  while (!found_controller && ((start_time + timeout) > ros::Time::now()));

  if (!found_controller)
  {
    ROS_ERROR("Could not connect to wrench controller \"%s\"", output.c_str());
    return false;
  }

  node_.getParam("max_position_error", max_position_error_, 0.05);
  node_.getParam("max_orientation_error", max_orientation_error_, 0.05);

  // subscribe to pose+twist commands
  command_notifier_.reset(new MessageNotifier<experimental_controllers::PoseTwistStamped>(tf_,
                                                                       boost::bind(&CartesianPoseTwistController::command, this, _1),
                                                                       node_.getNamespace() + "/command", root_name_, 10));
  // realtime publisher for control state
  state_error_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node_, "state/error", 1));
  state_pose_actual_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_, "state/pose_actual", 1));
  state_pose_desired_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node_, "state/pose_desired", 1));

  return true;
}

bool CartesianPoseTwistController::starting()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pid_controller_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = Twist::Zero();
  pose_desi_ = getPose();
  twist_desi_ = Twist::Zero();
  last_time_ = robot_state_->getTime();

  loop_count_ = 0;
  return true;
}

void CartesianPoseTwistController::update()
{
  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  // get current pose
  pose_meas_ = getPose();


  // get the joint positions and velocities
  chain_.getVelocities(robot_state_->joint_states_, jnt_posvel_);

  // get cartesian twist error
  FrameVel twist;
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, twist);
  twist_meas_ = twist.deriv();
  twist_error_ = twist_meas_ - twist_desi_;
  
  // pose feedback into twist
  pose_error_ = diff(pose_desi_, pose_meas_);


  // pid feedback
  for (unsigned int i=0; i<3; i++)
    wrench_out_.force(i) = pid_controller_[i].updatePid(pose_error_.vel(i), twist_error_.vel(i), dt);

  for (unsigned int i=0; i<3; i++)
    wrench_out_.torque(i) = pid_controller_[i+3].updatePid(pose_error_.rot(i), twist_error_.rot(i), dt);

  // send wrench to wrench controller
  wrench_controller_->wrench_desi_ = wrench_out_;


  if (++loop_count_ % 100 == 0){
    if (state_error_publisher_){
      if (state_error_publisher_->trylock()){
        state_error_publisher_->msg_.linear.x = pose_error_.vel(0);
        state_error_publisher_->msg_.linear.y = pose_error_.vel(1);
        state_error_publisher_->msg_.linear.z = pose_error_.vel(2);
        state_error_publisher_->msg_.angular.x = pose_error_.rot(0);
	state_error_publisher_->msg_.angular.y = pose_error_.rot(1);
        state_error_publisher_->msg_.angular.z = pose_error_.rot(2);
        state_error_publisher_->unlockAndPublish();
      }
    }

    if (state_pose_actual_publisher_){
      if (state_pose_actual_publisher_->trylock()){
	Pose tmp;
	frameToPose(pose_meas_, tmp);
	poseStampedTFToMsg(Stamped<Pose>(tmp, ros::Time::now(), root_name_), state_pose_actual_publisher_->msg_);
        state_pose_actual_publisher_->unlockAndPublish();

      }
    }

    if (state_pose_desired_publisher_){
      if (state_pose_desired_publisher_->trylock()){
        Pose tmp;

        frameToPose(pose_desi_, tmp);
        poseStampedTFToMsg(Stamped<Pose>(tmp, ros::Time::now(), root_name_), state_pose_desired_publisher_->msg_);
        state_pose_desired_publisher_->unlockAndPublish();
      }
    }

  }
}



Frame CartesianPoseTwistController::getPose()
{
  // get the joint positions and velocities
  chain_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // get cartesian pose
  Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);

  return result;
}

void CartesianPoseTwistController::command(const tf::MessageNotifier<experimental_controllers::PoseTwistStamped>::MessagePtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  geometry_msgs::PoseStamped ps_msg;
  ps_msg.header = pose_msg->header;
  ps_msg.pose = pose_msg->pose;
  poseStampedMsgToTF(ps_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  poseToFrame(pose_stamped, pose_desi_);

  // just copy the twist directly
  twist_desi_.vel(0) = pose_msg->twist.linear.x;
  twist_desi_.vel(1) = pose_msg->twist.linear.y;
  twist_desi_.vel(2) = pose_msg->twist.linear.z;
  twist_desi_.rot(0) = pose_msg->twist.angular.x;
  twist_desi_.rot(1) = pose_msg->twist.angular.y;
  twist_desi_.rot(2) = pose_msg->twist.angular.z;

}

void CartesianPoseTwistController::poseToFrame(const Pose& pose, Frame& frame)
{
  frame.p(0) = pose.getOrigin().x();
  frame.p(1) = pose.getOrigin().y();
  frame.p(2) = pose.getOrigin().z();

  double Rz, Ry, Rx;
  pose.getBasis().getEulerZYX(Rz, Ry, Rx);
  frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
}

void CartesianPoseTwistController::frameToPose(const Frame& frame, Pose& pose)
{
  pose.getOrigin()[0] = frame.p(0);
  pose.getOrigin()[1] = frame.p(1);
  pose.getOrigin()[2] = frame.p(2);

  double Rz, Ry, Rx;
  frame.M.GetEulerZYX(Rz, Ry, Rx);
  pose.setRotation( Quaternion(Rz, Ry, Rx));
}

} // namespace

