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
#include "robot_mechanism_controllers/cartesian_twist_controller.h"


using namespace KDL;

static const std::string controller_name = "cartesian_twist";


namespace controller {

CartesianTwistController::CartesianTwistController()
  : node_(ros::Node::instance()),
    robot_state_(NULL),
    jnt_to_twist_solver_(NULL)
{}



CartesianTwistController::~CartesianTwistController()
{
  if (jnt_to_twist_solver_) delete jnt_to_twist_solver_;
}



bool CartesianTwistController::initialize(mechanism::RobotState *robot_state, const string& root_name, const string& tip_name)
{
  cout << "initializing cartesian twist controller between " << root_name << " and " << tip_name << endl;

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

  // get pid controller
  double p, i, d, i_clamp;
  string name;
  node_->param(controller_name+"/fb_trans_p", p, 0.0) ;
  node_->param(controller_name+"/fb_trans_i", i, 0.0) ;
  node_->param(controller_name+"/fb_trans_d", d, 0.0) ;
  node_->param(controller_name+"/fb_trans_i_clamp", i_clamp, 0.0) ;
  control_toolbox::Pid fb_pid_trans(p, i, d, i_clamp, -i_clamp);
  for (unsigned int i=0; i<3; i++)
    fb_pid_controller_.push_back(fb_pid_trans);

  node_->param(controller_name+"/fb_rot_p", p, 0.0) ;
  node_->param(controller_name+"/fb_rot_i", i, 0.0) ;
  node_->param(controller_name+"/fb_rot_d", d, 0.0) ;
  node_->param(controller_name+"/fb_rot_i_clamp", i_clamp, 0.0) ;
  control_toolbox::Pid fb_pid_rot(p, i, d, i_clamp, -i_clamp);
  for (unsigned int i=0; i<3; i++)
    fb_pid_controller_.push_back(fb_pid_rot);
  fprintf(stderr, "pid controllers created\n");

  node_->param(controller_name+"/ff_trans", ff_trans_, 0.0) ;
  node_->param(controller_name+"/ff_rot", ff_rot_, 0.0) ;

  // time
  last_time_ = robot_state->hw_->current_time_;

  // set disired twist to 0
  twist_desi_ = Twist::Zero();

  // create wrench controller
  wrench_controller_.initialize(robot_state, root_name, tip_name);

  return true;
}






void CartesianTwistController::update()
{
  // check if joints are calibrated
  if (!robot_.allCalibrated(robot_state_->joint_states_)) return;

  // get time
  double time = robot_state_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  // get the joint positions and velocities
  robot_.getVelocities(robot_state_->joint_states_, jnt_posvel_);

  // get cartesian twist error
  FrameVel twist; 
  jnt_to_twist_solver_->JntToCart(jnt_posvel_, twist);
  twist_meas_ = twist.deriv();
  Twist error = twist_meas_ - twist_desi_;

  // pid feedback
  for (unsigned int i=0; i<3; i++)
    wrench_out_.force(i) = (twist_desi_.vel(i) * ff_trans_) + fb_pid_controller_[i].updatePid(error.vel(i), dt);

  for (unsigned int i=0; i<3; i++)
    wrench_out_.torque(i) = (twist_desi_.rot(i) * ff_rot_) + fb_pid_controller_[i+3].updatePid(error.rot(i), dt);

  // send wrench to wrench controller
  wrench_controller_.wrench_desi_ = wrench_out_;
  wrench_controller_.update();
}








ROS_REGISTER_CONTROLLER(CartesianTwistControllerNode)

CartesianTwistControllerNode::CartesianTwistControllerNode()
: node_(ros::Node::instance())
{}


CartesianTwistControllerNode::~CartesianTwistControllerNode()
{
  node_->unsubscribe(controller_name + "/command");
  node_->unsubscribe("spacenav/joy");
}


bool CartesianTwistControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get parameters
  node_->param(controller_name+"/joystick_max_trans", joystick_max_trans_, 0.0);
  node_->param(controller_name+"/joystick_max_rot", joystick_max_rot_, 0.0);

  // get name of root and tip
  string root_name, tip_name;
  node_->param(controller_name+"/root_name", root_name, string("no_name_given"));
  node_->param(controller_name+"/tip_name", tip_name, string("no_name_given"));

  // initialize controller  
  if (!controller_.initialize(robot, root_name, tip_name))
    return false;
  
  // subscribe to twist commands
  node_->subscribe(controller_name + "/command", twist_msg_,
		   &CartesianTwistControllerNode::command, this, 1);

  // subscribe to joystick commands
  node_->subscribe("spacenav/joy", joystick_msg_,
		   &CartesianTwistControllerNode::joystick, this, 1);

  return true;
}


void CartesianTwistControllerNode::update()
{
  controller_.update();
}


void CartesianTwistControllerNode::command()
{
  // convert to twist command
  controller_.twist_desi_.vel(0) = twist_msg_.vel.x;
  controller_.twist_desi_.vel(1) = twist_msg_.vel.y;
  controller_.twist_desi_.vel(2) = twist_msg_.vel.z;
  controller_.twist_desi_.rot(0) = twist_msg_.rot.x;
  controller_.twist_desi_.rot(1) = twist_msg_.rot.y;
  controller_.twist_desi_.rot(2) = twist_msg_.rot.z;
}


void CartesianTwistControllerNode::joystick()
{
  // convert to twist command
  for (unsigned int i=0; i<3; i++){
    controller_.twist_desi_.vel(i)  = joystick_msg_.axes[i]   * joystick_max_trans_;
    controller_.twist_desi_.rot(i)  = joystick_msg_.axes[i+3] * joystick_max_rot_;
  }
}


}; // namespace
