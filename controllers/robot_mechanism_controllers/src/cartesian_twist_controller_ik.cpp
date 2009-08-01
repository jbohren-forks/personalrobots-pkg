/*
 * Copyright (c) 2008, Ruben Smits and Willow Garage, Inc.
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
 * Author: Wim Meeussen
 *         Ruben Smits
 */

#include <robot_mechanism_controllers/cartesian_twist_controller_ik.h>
#include <algorithm>
#include <mechanism_control/mechanism_control.h>

#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/kinfam_io.hpp>

using namespace KDL;
using namespace ros;

namespace controller {

  ROS_REGISTER_CONTROLLER(CartesianTwistControllerIk)

  CartesianTwistControllerIk::CartesianTwistControllerIk()
  : robot_state_(NULL),
    jnt_to_twist_solver_(NULL),
    twist_to_jnt_solver_(NULL)
  {}



  CartesianTwistControllerIk::~CartesianTwistControllerIk()
  {}



  bool CartesianTwistControllerIk::init(mechanism::RobotState *robot_state, const NodeHandle& n)
  {
    node_ = n;

    // get name of root and tip from the parameter server
    std::string root_name, tip_name;
    double damping,threshold;
    if (!node_.getParam("root_name", root_name)){
      ROS_ERROR("CartesianTwistIkController: No root name found on parameter server");
      return false;
    }
    if (!node_.getParam("tip_name", tip_name)){
      ROS_ERROR("CartesianTwistIkController: No tip name found on parameter server");
      return false;
    }
    if (!node_.getParam("damping", damping)){
      ROS_ERROR("CartesianTwistIkController: No damping found on parameter server");
      return false;
    }
    if (!node_.getParam("threshold", threshold)){
      ROS_ERROR("CartesianTwistIkController: No threshold found on parameter server");
      return false;
    }

    // test if we got robot pointer
    assert(robot_state);
    robot_state_ = robot_state;

    // create robot chain from root to tip
    if (!chain_.init(robot_state->model_, root_name, tip_name))
      return false;
    chain_.toKDL(kdl_chain_);

    // create solver
    jnt_to_twist_solver_.reset(new ChainFkSolverVel_recursive(kdl_chain_));
    twist_to_jnt_solver_.reset(new ChainIkSolverVel_wdls(kdl_chain_,threshold));
    jnt_posvel_.resize(kdl_chain_.getNrOfJoints());

    ((ChainIkSolverVel_wdls*)twist_to_jnt_solver_.get())->setLambda(damping);

    // get pid controller parameters
    control_toolbox::Pid pid_joint;
    if (!pid_joint.init(NodeHandle(node_, "joint"))) return false;

    // constructs 3 identical pid controllers: for the x,y and z translations
    control_toolbox::Pid pid_controller;
    if (!pid_controller.init(NodeHandle(node_, "fb_trans"))) return false;
    for (unsigned int i=0; i<3; i++)
      fb_pid_controller_.push_back(pid_controller);

    // constructs 3 identical pid controllers: for the x,y and z rotations
    if (!pid_controller.init(NodeHandle("fb_rot"))) return false;
    for (unsigned int i=0; i<3; i++)
      fb_pid_controller_.push_back(pid_controller);

    // get a pointer to the id controller
    std::string output;
    if (!node_.getParam("output", output)){
      ROS_ERROR("CartesianTwistController: No output name found on parameter server");
      return false;
    }
    if (!getController<JointInverseDynamicsController>(output, AFTER_ME, id_controller_)){
      ROS_ERROR("CartesianTwistController: could not connect to id controller");
      return false;
    }

    // subscribe to twist commands
    sub_command_ = node_.subscribe("command", 1,
                                   &CartesianTwistControllerIk::command, this);

    return true;
  }



  bool CartesianTwistControllerIk::starting()
  {
    // reset pid controllers
    for (unsigned int i=0; i<6; i++)
      fb_pid_controller_[i].reset();

    // time
    last_time_ = robot_state_->hw_->current_time_;

    // set disired twist to 0
    twist_desi_ = Twist::Zero();

    return true;
  }


  void CartesianTwistControllerIk::update()
  {
    // check if joints are calibrated
    if (!chain_.allCalibrated(robot_state_->joint_states_)) return;

    // get time
    double time = robot_state_->hw_->current_time_;
    double dt = time - last_time_;
    last_time_ = time;

    // get the joint positions
    chain_.getVelocities(robot_state_->joint_states_, jnt_posvel_);

    //Calculate error on the twist
    FrameVel twist;
    jnt_to_twist_solver_->JntToCart(jnt_posvel_, twist);
    twist_meas_ = twist.deriv();
    error = twist_meas_ - twist_desi_;


    // pid feedback on the twist
    for (unsigned int i=0; i<3; i++)
      twist_out_.vel(i) = (twist_desi_.vel(i) * ff_trans_) + fb_pid_controller_[i].updatePid(error.vel(i), dt);

    for (unsigned int i=0; i<3; i++)
      twist_out_.rot(i) = (twist_desi_.rot(i) * ff_rot_) + fb_pid_controller_[i+3].updatePid(error.rot(i), dt);

    // calculate joint velocities
    twist_to_jnt_solver_->CartToJnt(jnt_posvel_.q, twist_desi_, jnt_posvel_.qdot);

    id_controller_->jnt_posvelacc_desi_.qdot=jnt_posvel_.qdot;
  }


  void CartesianTwistControllerIk::command(const robot_msgs::TwistConstPtr& twist_msg)
  {
    // convert to twist command
    twist_desi_.vel(0) = twist_msg->linear.x;
    twist_desi_.vel(1) = twist_msg->linear.y;
    twist_desi_.vel(2) = twist_msg->linear.z;
    twist_desi_.rot(0) = twist_msg->angular.x;
    twist_desi_.rot(1) = twist_msg->angular.y;
    twist_desi_.rot(2) = twist_msg->angular.z;
  }



}; // namespace

