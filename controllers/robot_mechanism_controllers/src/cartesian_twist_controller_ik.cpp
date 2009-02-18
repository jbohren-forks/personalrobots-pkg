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

#include <urdf/parser.h>
#include <algorithm>
#include <robot_kinematics/robot_kinematics.h>
#include "robot_mechanism_controllers/cartesian_twist_controller_ik.h"


using namespace KDL;

static const std::string controller_name = "cartesian_twist_ik";


namespace controller {

CartesianTwistControllerIk::CartesianTwistControllerIk()
  : node_(ros::Node::instance()),
    num_joints_(0),
    robot_state_(NULL),
    jnt_to_twist_solver_(NULL),
    twist_to_jnt_solver_(NULL)
{}



CartesianTwistControllerIk::~CartesianTwistControllerIk()
{
  if (jnt_to_twist_solver_) delete jnt_to_twist_solver_;
  if (twist_to_jnt_solver_) delete twist_to_jnt_solver_;

  for (unsigned int i=0; i<num_joints_; i++)
    if (joint_vel_controllers_[i]) delete joint_vel_controllers_[i];
}



  bool CartesianTwistControllerIk::initialize(mechanism::RobotState *robot_state, const string& root_name, 
					      const string& tip_name,  const string& controller_name)
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
  twist_to_jnt_solver_ = new ChainIkSolverVel_pinv(chain_);
  jnt_pos_.resize(num_joints_);
  jnt_vel_.resize(num_joints_);

  // get pid controller parameters
  control_toolbox::Pid pid_joint;
  pid_joint.initParam(controller_name_+"/joint");

  // time
  last_time_ = robot_state->hw_->current_time_;

  // set disired twist to 0
  twist_desi_ = Twist::Zero();

  // create and initialize joint velocity controllers
  for (unsigned int i=0 ;i<num_joints_; i++)
    joint_vel_controllers_.push_back(new JointVelocityController);
  for (unsigned int i=0 ;i<num_joints_; i++){
    cout << "constructing velocity controller for joint " << robot_.getJointName(i) << endl;
    joint_vel_controllers_[i]->init(robot_state, robot_.getJointName(i), pid_joint);
  }
  return true;
}






void CartesianTwistControllerIk::update()
{
  // check if joints are calibrated
  if (!robot_.allCalibrated(robot_state_->joint_states_)) return;

  // get the joint positions 
  robot_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // calculate joint velocities
  twist_to_jnt_solver_->CartToJnt(jnt_pos_, twist_desi_, jnt_vel_);

  /*
  cout << "vel ";
  for (unsigned int i=0; i<num_joints_; i++)
    cout << jnt_vel_(i) << "  ";
  cout << endl;
  */

  // send joint velocities to joint velocity controllers
  for (unsigned int i=0; i<num_joints_; i++){
    joint_vel_controllers_[i]->setCommand(jnt_vel_(i));
    joint_vel_controllers_[i]->update();
  }

}








ROS_REGISTER_CONTROLLER(CartesianTwistControllerIkNode)

CartesianTwistControllerIkNode::CartesianTwistControllerIkNode()
: node_(ros::Node::instance())
{}


CartesianTwistControllerIkNode::~CartesianTwistControllerIkNode()
{
  node_->unsubscribe(controller_name_ + "/command");
  node_->unsubscribe("spacenav/joy");
}


bool CartesianTwistControllerIkNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get the controller name
  controller_name_ = config->Attribute("name");

  // get parameters
  node_->param(controller_name_+"/joystick_max_trans", joystick_max_trans_, 0.0);
  node_->param(controller_name_+"/joystick_max_rot", joystick_max_rot_, 0.0);

  cout << "joystick max " << joystick_max_trans_ << " " << joystick_max_rot_ << endl;

  // get name of root and tip
  string root_name, tip_name;
  node_->param(controller_name_+"/root_name", root_name, string("no_name_given"));
  node_->param(controller_name_+"/tip_name", tip_name, string("no_name_given"));

  // initialize controller  
  if (!controller_.initialize(robot, root_name, tip_name, controller_name_))
    return false;
  
  // subscribe to twist commands
  node_->subscribe(controller_name_ + "/command", twist_msg_,
		   &CartesianTwistControllerIkNode::command, this, 1);

  // subscribe to joystick commands
  node_->subscribe("spacenav/joy", joystick_msg_,
		   &CartesianTwistControllerIkNode::joystick, this, 1);

  return true;
}


void CartesianTwistControllerIkNode::update()
{
  controller_.update();
}


void CartesianTwistControllerIkNode::command()
{
  // convert to twist command
  controller_.twist_desi_.vel(0) = twist_msg_.vel.x;
  controller_.twist_desi_.vel(1) = twist_msg_.vel.y;
  controller_.twist_desi_.vel(2) = twist_msg_.vel.z;
  controller_.twist_desi_.rot(0) = twist_msg_.rot.x;
  controller_.twist_desi_.rot(1) = twist_msg_.rot.y;
  controller_.twist_desi_.rot(2) = twist_msg_.rot.z;
}


void CartesianTwistControllerIkNode::joystick()
{
  // convert to twist command
  for (unsigned int i=0; i<3; i++){
    controller_.twist_desi_.vel(i)  = joystick_msg_.axes[i]   * joystick_max_trans_;
    controller_.twist_desi_.rot(i)  = joystick_msg_.axes[i+3] * joystick_max_rot_;
  }
}


}; // namespace
