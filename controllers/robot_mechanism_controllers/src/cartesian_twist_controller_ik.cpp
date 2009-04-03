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

#include <algorithm>
#include <robot_kinematics/robot_kinematics.h>
#include <robot_mechanism_controllers/cartesian_twist_controller_ik.h>


using namespace KDL;

static const std::string controller_name = "cartesian_twist_ik";


namespace controller {

CartesianTwistControllerIk::CartesianTwistControllerIk()
  : node_(ros::Node::instance()),
    robot_state_(NULL),
    jnt_to_twist_solver_(NULL),
    twist_to_jnt_solver_(NULL)
{}



CartesianTwistControllerIk::~CartesianTwistControllerIk()
{
  for (unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++)
    if (joint_vel_controllers_[i]) delete joint_vel_controllers_[i];
}



bool CartesianTwistControllerIk::init(mechanism::RobotState *robot_state,
                                      const string& root_name,
                                      const string& tip_name,
                                      const string& controller_name)
{
  controller_name_ = controller_name;

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state->model_, root_name, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_twist_solver_.reset(new ChainFkSolverVel_recursive(kdl_chain_));
  twist_to_jnt_solver_.reset(new ChainIkSolverVel_pinv(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_vel_.resize(kdl_chain_.getNrOfJoints());

  // get pid controller parameters
  control_toolbox::Pid pid_joint;
  if (!pid_joint.initParam(controller_name_+"/joint")) return false;

  // create and initialize joint velocity controllers
  for (unsigned int i=0 ;i<kdl_chain_.getNrOfJoints(); i++)
    joint_vel_controllers_.push_back(new JointVelocityController);
  for (unsigned int i=0 ;i<kdl_chain_.getNrOfJoints(); i++){
    if (!joint_vel_controllers_[i]->init(robot_state, chain_.getJointName(i), pid_joint))
      return false;
  }
  return true;
}



bool CartesianTwistControllerIk::starting()
{
  // time
  last_time_ = robot_state_->hw_->current_time_;

  // set disired twist to 0
  twist_desi_ = Twist::Zero();

  // start joint velocity controllers
  bool res = true;
  for (unsigned int i=0 ;i<kdl_chain_.getNrOfJoints(); i++)
    if (!joint_vel_controllers_[i]->starting()) res = false;

  return res;
}


void CartesianTwistControllerIk::update()
{
  // check if joints are calibrated
  if (!chain_.allCalibrated(robot_state_->joint_states_)) return;

  // get the joint positions
  chain_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // calculate joint velocities
  twist_to_jnt_solver_->CartToJnt(jnt_pos_, twist_desi_, jnt_vel_);

  // send joint velocities to joint velocity controllers
  for (unsigned int i=0; i<kdl_chain_.getNrOfJoints(); i++){
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
}


bool CartesianTwistControllerIkNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get the controller name from xml file
  controller_name_ = config->Attribute("name") ? config->Attribute("name") : "";
  if (controller_name_ == ""){
    ROS_ERROR("CartesianTwistIkControllerNode: No controller name given in xml file");
    return false;
  }

  // get name of root and tip from the parameter server
  std::string root_name, tip_name;
  if (!node_->getParam(controller_name_+"/root_name", root_name)){
    ROS_ERROR("CartesianTwistIkControllerNode: No root name found on parameter server");
    return false;
 }
  if (!node_->getParam(controller_name_+"/tip_name", tip_name)){
    ROS_ERROR("CartesianTwistIkControllerNode: No tip name found on parameter server");
    return false;
  }

  // initialize twist controller
  if (!controller_.init(robot, root_name, tip_name, controller_name_)) return false;

  // subscribe to twist commands
  node_->subscribe(controller_name_ + "/command", twist_msg_,
		   &CartesianTwistControllerIkNode::command, this, 1);

  return true;
}


bool CartesianTwistControllerIkNode::starting()
{
  return controller_.starting();
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



}; // namespace
