/*
 * Copyright (c) 2009, Ruben Smits
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
 *     * Neither the name of Ruben Smits nor the names of its
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
 * Author: Ruben Smits
 * based on cartesian_wrench_controller.cpp by Wim Meeussen for Willow Garage Inc.
 */

#include <algorithm>
#include <robot_mechanism_controllers/joint_inverse_dynamics_controller.h>
#include <kdl/kinfam_io.hpp>
#include <filters/transfer_function.h>
#include <filters/filter_chain.h>

using namespace KDL;
using namespace ros;

namespace controller {


ROS_REGISTER_CONTROLLER(JointInverseDynamicsController);

JointInverseDynamicsController::JointInverseDynamicsController()
  :robot_state_(NULL),
   id_solver_(NULL),
   diagnostics_publisher_(node_, "/diagnostics", 2)
{}



JointInverseDynamicsController::~JointInverseDynamicsController()
{}




bool JointInverseDynamicsController::init(mechanism::RobotState *robot_state, const NodeHandle& n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string root_name, tip_name;
  if (!node_.getParam("root_name", root_name)){
    ROS_ERROR("JointInverseDynamicsController: No root name found on parameter server");
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("JointInverseDynamicsController: No tip name found on parameter server");
    return false;
  }

  if (!node_.getParam("Kp_acc", Kp_acc_)){
    ROS_ERROR("JointInverseDynamicsController: No Kp_acc found on parameter server");
    return false;
  }
  if (!node_.getParam("Kv_acc", Kv_acc_)){
    ROS_ERROR("JointInverseDynamicsController: No Kv_acc found on parameter server");
    return false;
  }
  if (!node_.getParam("Kp_tau", Kp_tau_)){
    ROS_ERROR("JointInverseDynamicsController: No Kp_tau found on parameter server");
    return false;
  }
  if (!node_.getParam("Kv_tau", Kv_tau_)){
    ROS_ERROR("JointInverseDynamicsController: No Kv_tau found on parameter server");
    return false;
  }
  if (!node_.getParam("Kf_acc", Kf_acc_)){
    ROS_ERROR("JointInverseDynamicsController: No Kfa found on parameter server");
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state_->model_, root_name, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // create solver with no gravity
  id_solver_.reset(new ChainIdSolver_RNE(kdl_chain_,Vector(0.0,0.0,0.0)));
  //resize the input
  jnt_posvelacc_desi_.resize(kdl_chain_.getNrOfJoints());
  sgmnt_forces_.resize(kdl_chain_.getNrOfSegments());
  //resize the controller variables
  jnt_posvel_meas_.resize(kdl_chain_.getNrOfJoints());
  jnt_posvelacc_control_.resize(kdl_chain_.getNrOfJoints());
  jnt_acc_out_.resize(kdl_chain_.getNrOfJoints());
  jnt_tau_.resize(kdl_chain_.getNrOfJoints());
  //resize the publish variables
  jnt_eff_std_.resize(kdl_chain_.getNrOfJoints());
  jnt_msg_std_.resize(kdl_chain_.getNrOfJoints());

  // diagnostics messages
  diagnostics_.status.resize(1);
  diagnostics_.status[0].name  = "Inverse Dynamics Controller";
  diagnostics_.status[0].values.resize(1);
  diagnostics_.status[0].values[0].value = 3;
  diagnostics_.status[0].values[0].label = "TestValueLabel";

  diagnostics_.status[0].strings.resize(1);
  diagnostics_.status[0].strings[0].value = "TestValue";
  diagnostics_.status[0].strings[0].label = "TestLabel";

  diagnostics_time_ = ros::Time::now();
  diagnostics_interval_ = ros::Duration().fromSec(1.0);


  //advertise our messages:
  pub_pos_desi_ = node_.advertise<std_msgs::Float64MultiArray>("pos_desired",100);
  pub_pos_meas_ = node_.advertise<std_msgs::Float64MultiArray>("pos_meas",100);
  pub_vel_desi_ = node_.advertise<std_msgs::Float64MultiArray>("vel_desired",100);
  pub_vel_meas_ = node_.advertise<std_msgs::Float64MultiArray>("vel_meas",100);
  pub_acc_desi_ = node_.advertise<std_msgs::Float64MultiArray>("acc_desired",100);
  pub_acc_control_ = node_.advertise<std_msgs::Float64MultiArray>("acc_control",100);
  pub_eff_calculated_ = node_.advertise<std_msgs::Float64MultiArray>("effort_calculated",100);
  pub_eff_sent_ = node_.advertise<std_msgs::Float64MultiArray>("effort_send",100);

  jnt_msg_.set_data_size(kdl_chain_.getNrOfJoints());

  return true;
}


bool JointInverseDynamicsController::starting()
{
  // set desired wrench to 0
  SetToZero(jnt_posvelacc_desi_);
  
  last_time_ = robot_state_->hw_->current_time_;
 
   return true;
}



void JointInverseDynamicsController::update()
{
  // check if joints are calibrated
  if (!chain_.allCalibrated(robot_state_->joint_states_)){
    publishDiagnostics(2, "Not all joints are calibrated");
    return;
  }
  publishDiagnostics(0, "No problems detected");

  // get joint positions and velocities
  chain_.getVelocities(robot_state_->joint_states_, jnt_posvel_meas_);
  

  //Calculate qdotdot_out
  SetToZero(jnt_acc_out_);
  //Position feedback
  Subtract(jnt_posvelacc_desi_.q,jnt_posvel_meas_.q,jnt_posvelacc_control_.q);
  if(Kp_acc_!=0.0){
    Multiply(jnt_posvelacc_control_.q,Kp_acc_,jnt_posvelacc_control_.q);
    Add(jnt_acc_out_,jnt_posvelacc_control_.q,jnt_acc_out_);
    Divide(jnt_posvelacc_control_.q,Kp_acc_,jnt_posvelacc_control_.q);
  }
  //Velocity feedback
    Subtract(jnt_posvelacc_desi_.qdot,jnt_posvel_meas_.qdot,jnt_posvelacc_control_.qdot);
  if(Kv_acc_!=0.0){
    Multiply(jnt_posvelacc_control_.qdot,Kv_acc_,jnt_posvelacc_control_.qdot);
    Add(jnt_acc_out_,jnt_posvelacc_control_.qdot,jnt_acc_out_);
    Divide(jnt_posvelacc_control_.qdot,Kv_acc_,jnt_posvelacc_control_.qdot);
  }
  //Acceleration feedforward
  if(Kf_acc_!=0.0){
    Multiply(jnt_posvelacc_desi_.qdotdot,Kf_acc_,jnt_posvelacc_control_.qdotdot);
    Add(jnt_acc_out_,jnt_posvelacc_control_.qdotdot,jnt_acc_out_);
  }

  //Calculate inverse dynamics
  id_solver_->CartToJnt(jnt_posvel_meas_.q,jnt_posvel_meas_.qdot,jnt_acc_out_,sgmnt_forces_,jnt_tau_);

  //Position feedback
  if(Kp_tau_!=0.0){
    Multiply(jnt_posvelacc_control_.q,Kp_tau_,jnt_posvelacc_control_.q);
    Add(jnt_tau_,jnt_posvelacc_control_.q,jnt_tau_);
  }
  //Velocity feedback
  if(Kv_tau_!=0.0){
    Subtract(jnt_posvelacc_desi_.qdot,jnt_posvel_meas_.qdot,jnt_posvelacc_control_.qdot);
    Multiply(jnt_posvelacc_control_.qdot,Kv_tau_,jnt_posvelacc_control_.qdot);
    Add(jnt_tau_,jnt_posvelacc_control_.qdot,jnt_tau_);
  }

  chain_.getEfforts(robot_state_->joint_states_, jnt_eff_std_);

  //publish some resulst, for debugging only
  for(unsigned int i=0;i<kdl_chain_.getNrOfJoints();i++)
    jnt_msg_std_[i]=jnt_tau_(i);
  jnt_msg_.set_data_vec(jnt_msg_std_);
  pub_eff_calculated_.publish(jnt_msg_);
  
  chain_.getEfforts(robot_state_->joint_states_,jnt_msg_std_);
  jnt_msg_.set_data_vec(jnt_eff_std_);
  pub_eff_sent_.publish(jnt_msg_);

  for(unsigned int i=0;i<kdl_chain_.getNrOfJoints();i++)
    jnt_msg_std_[i]=jnt_posvelacc_desi_.qdot(i);
  jnt_msg_.set_data_vec(jnt_msg_std_);
  pub_vel_desi_.publish(jnt_msg_);

  for(unsigned int i=0;i<kdl_chain_.getNrOfJoints();i++)
    jnt_msg_std_[i]=jnt_posvel_meas_.qdot(i);
  jnt_msg_.set_data_vec(jnt_eff_std_);
  pub_vel_meas_.publish(jnt_msg_);
  
  for(unsigned int i=0;i<kdl_chain_.getNrOfJoints();i++)
    jnt_msg_std_[i]=jnt_posvelacc_desi_.qdotdot(i);
  jnt_msg_.set_data_vec(jnt_msg_std_);
  pub_acc_desi_.publish(jnt_msg_);

  for(unsigned int i=0;i<kdl_chain_.getNrOfJoints();i++)
    jnt_msg_std_[i]=jnt_acc_out_(i);
  jnt_msg_.set_data_vec(jnt_eff_std_);
  pub_acc_control_.publish(jnt_msg_);

  // set effort to joints
  chain_.setEfforts(jnt_tau_, robot_state_->joint_states_);
}



bool JointInverseDynamicsController::publishDiagnostics(int level, const std::string& message)
{
  if (diagnostics_time_ + diagnostics_interval_ < ros::Time::now())
  {
    if (!diagnostics_publisher_.trylock())
      return false;

    diagnostics_.status[0].level = level;
    diagnostics_.status[0].message  = message;

    diagnostics_publisher_.unlockAndPublish();
    diagnostics_time_ = ros::Time::now();
    return true;
  }

  return false;
}



}; // namespace
