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
 * Author: Wim Meeussen
 */

#include "urdf/parser.h"
#include <algorithm>
#include "robot_kinematics/robot_kinematics.h"
#include "robot_mechanism_controllers/cartesian_tff_controller.h"


using namespace KDL;

static const bool use_constraint_controller = false;


namespace controller {

CartesianTFFController::CartesianTFFController()
  : node_(ros::Node::instance()),
    robot_state_(NULL),
    jnt_to_twist_solver_(NULL),
    mode_(6),
    value_(6),
    twist_to_wrench_(6)
{}



CartesianTFFController::~CartesianTFFController()
{
  if (jnt_to_twist_solver_) delete jnt_to_twist_solver_;
}



bool CartesianTFFController::initialize(mechanism::RobotState *robot_state, const string& root_name, const string& tip_name, const string& controller_name)
{
  cout << "initializing " << controller_name << " between " << root_name << " and " << tip_name << endl;
  controller_name_ = controller_name;

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!robot_.init(robot_state->model_, root_name, tip_name))
    return false;
  robot_.toKDL(chain_);

  // create solver
  num_joints_   = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
  jnt_to_twist_solver_ = new ChainFkSolverVel_recursive(chain_);
  jnt_posvel_.resize(num_joints_);

  // twist to wrench
  double trans, rot;
  node_->param(controller_name+"/twist_to_wrench_trans", trans, 0.0);
  for (unsigned int i=0; i<3; i++)
    twist_to_wrench_[i] = trans;
  node_->param(controller_name+"/twist_to_wrench_trans", rot, 0.0);
  for (unsigned int i=3; i<6; i++)
    twist_to_wrench_[i] = rot;

  // get pid controllers
  control_toolbox::Pid pid_controller;
  pid_controller.initParam(controller_name_+"/vel_trans");
  for (unsigned int i=0; i<3; i++)
    vel_pid_controller_.push_back(pid_controller);

  pid_controller.initParam(controller_name_+"/vel_rot");
  for (unsigned int i=0; i<3; i++)
    vel_pid_controller_.push_back(pid_controller);

  pid_controller.initParam(controller_name_+"/pos_trans");
  for (unsigned int i=0; i<3; i++)
    pos_pid_controller_.push_back(pid_controller);

  pid_controller.initParam(controller_name_+"/pos_rot");
  for (unsigned int i=0; i<3; i++)
    pos_pid_controller_.push_back(pid_controller);

  fprintf(stderr, "pid controllers created\n");

  // create wrench/constraint controller
  if (use_constraint_controller)
    constraint_controller_.initialize(robot_state, root_name, tip_name, controller_name_+"/wrench");
  else
    wrench_controller_.initialize(robot_state, root_name, tip_name, controller_name_+"/wrench");


  return true;
}



bool CartesianTFFController::start()
{
  // time
  last_time_ = robot_state_->hw_->current_time_;

  // set initial modes and values
  for (unsigned int i=0; i<6; i++){
    mode_[i] = tff_msg_.FORCE;
    value_[i] = 0;
  }

  // reset pid controllers
  for (unsigned int i=0; i<6; i++){
    vel_pid_controller_[i].reset();
    pos_pid_controller_[i].reset();
  }

  // set initial position, twist
  FrameVel frame_twist; 
  robot_.getVelocities(robot_state_->joint_states_, jnt_posvel_);
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, frame_twist);
  pose_meas_old_ = frame_twist.value();
  position_ = Twist::Zero();

  if (use_constraint_controller)
    //return constraint_controller_.start();
    return true;
  else
    return wrench_controller_.start();
}


void CartesianTFFController::update()
{
  // get time
  double time = robot_state_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  // get the joint positions and velocities
  robot_.getVelocities(robot_state_->joint_states_, jnt_posvel_);

  // get cartesian twist and pose
  FrameVel frame_twist; 
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, frame_twist);
  pose_meas_ = frame_twist.value();
  twist_meas_ = pose_meas_.M.Inverse() * (frame_twist.deriv());

  // calculate the distance traveled along the axes of the tf
  position_ += pose_meas_.M.Inverse() * diff(pose_meas_old_, pose_meas_);
  pose_meas_old_ = pose_meas_;

  // calculate desired wrench
  wrench_desi_ = Wrench::Zero();
  for (unsigned int i=0; i<6; i++){
    if (mode_[i] == tff_msg_.FORCE)
      wrench_desi_[i] = value_[i];
    else if (mode_[i] ==  tff_msg_.VELOCITY)
      wrench_desi_[i] = twist_to_wrench_[i] * (value_[i] + vel_pid_controller_[i].updatePid(twist_meas_[i] - value_[i], dt));
    else if (mode_[i] == tff_msg_.POSITION)
      wrench_desi_[i] = twist_to_wrench_[i] * (pos_pid_controller_[i].updatePid(position_[i] - value_[i], dt));
  }
  
  // send wrench to wrench controller
  if (use_constraint_controller){
    constraint_controller_.wrench_desired_ = (pose_meas_.M * wrench_desi_);
    constraint_controller_.update();
  }
  else{
    wrench_controller_.wrench_desi_ = (pose_meas_.M * wrench_desi_);
    wrench_controller_.update();
  }
}




void CartesianTFFController::tffCommand(int mode1, double value1, int mode2, double value2, int mode3, double value3,
                                        int mode4, double value4, int mode5, double value5, int mode6, double value6)
{
  mode_[0] = mode1;
  mode_[1] = mode2;
  mode_[2] = mode3;
  mode_[3] = mode4;
  mode_[4] = mode5;
  mode_[5] = mode6;

  value_[0] = value1;
  value_[1] = value2;
  value_[2] = value3;
  value_[3] = value4;
  value_[4] = value5;
  value_[5] = value6;
}





ROS_REGISTER_CONTROLLER(CartesianTFFControllerNode)

CartesianTFFControllerNode::CartesianTFFControllerNode()
: node_(ros::Node::instance())
{}


CartesianTFFControllerNode::~CartesianTFFControllerNode()
{
  node_->unsubscribe(controller_name_ + "/command");
}


bool CartesianTFFControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get the controller name
  controller_name_ = config->Attribute("name");

  // get name of root and tip
  string root_name, tip_name;
  node_->param(controller_name_+"/root_name", root_name, string("no_name_given"));
  node_->param(controller_name_+"/tip_name", tip_name, string("no_name_given"));

  // initialize controller  
  if (!controller_.initialize(robot, root_name, tip_name, controller_name_))
    return false;
  
  // subscribe to tff commands
  node_->subscribe(controller_name_ + "/command", tff_msg_,
		   &CartesianTFFControllerNode::command, this, 1);

  return true;
}

bool CartesianTFFControllerNode::start()
{
  return controller_.start();
}


void CartesianTFFControllerNode::update()
{
  controller_.update();
}


void CartesianTFFControllerNode::command()
{
  // tff command
  controller_.tffCommand(trunc(tff_msg_.mode.vel.x), tff_msg_.value.vel.x,
                         trunc(tff_msg_.mode.vel.y), tff_msg_.value.vel.y,
                         trunc(tff_msg_.mode.vel.z), tff_msg_.value.vel.z,
                         trunc(tff_msg_.mode.rot.x), tff_msg_.value.rot.x,
                         trunc(tff_msg_.mode.rot.y), tff_msg_.value.rot.y,
                         trunc(tff_msg_.mode.rot.z), tff_msg_.value.rot.z);
}



}; // namespace
