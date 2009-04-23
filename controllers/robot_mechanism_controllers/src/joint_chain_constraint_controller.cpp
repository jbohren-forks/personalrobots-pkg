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
 * Author: Melonee Wise
 */

#include "robot_mechanism_controllers/joint_chain_constraint_controller.h"
#include <algorithm>
#include "angles/angles.h"
#include <kdl/chainfksolvervel_recursive.hpp>

using namespace KDL;

namespace controller {


JointChainConstraintController::JointChainConstraintController() :
  node_(ros::Node::instance()),
  list_size_(0),
  robot_state_(NULL),
  jnt_to_jac_solver_(NULL),
  initialized_(false)
{
}



JointChainConstraintController::~JointChainConstraintController()
{
  if (jnt_to_jac_solver_) delete jnt_to_jac_solver_;

}

bool JointChainConstraintController::init(mechanism::RobotState *robot_state,
                                          const std::string& root_name,
                                          const std::string& tip_name,
                                          const std::string& controller_name)
{
  ROS_INFO("Intializing JointChainConstraintController,\"%s\", between \"%s\" and \"%s\".", controller_name.c_str(), root_name.c_str(), tip_name.c_str());

  controller_name_ = controller_name;

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!mechanism_chain_.init(robot_state->model_, root_name, tip_name))
  {
    ROS_ERROR( "Intializing JointChainConstraintController,\"%s\", could not create chain.", controller_name.c_str());
    return false;
  }
  mechanism_chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_jac_solver_ = new ChainJntToJacSolver(kdl_chain_);
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_eff_.resize(kdl_chain_.getNrOfJoints());
  chain_kdl_jacobian_.resize(kdl_chain_.getNrOfJoints());

  wrench_desired_ = Wrench::Zero();

  // create the required matrices
  joint_constraint_torque_ = Eigen::MatrixXf::Zero(kdl_chain_.getNrOfJoints(), 1);
  // TODO: make sure to change this in the future to have mixed constraint conditions on joints
  joint_constraint_jacobian_ = Eigen::MatrixXf::Zero(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
  joint_constraint_null_space_ = Eigen::MatrixXf::Zero(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
  task_torque_ = Eigen::MatrixXf::Zero(kdl_chain_.getNrOfJoints(), 1);
  identity_ = Eigen::MatrixXf::Identity(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
  chain_eigen_jacobian_ = Eigen::MatrixXf::Zero(6, kdl_chain_.getNrOfJoints());
  last_time_ = robot_state->hw_->current_time_;

  // advertise service
  node_->advertiseService(controller_name_ + "/add_constraints", &JointChainConstraintController::addConstraint, this);
  change_constraints_guard_.set(controller_name_ + "/add_constraints");

  return true;
}



void JointChainConstraintController::update()
{
  // check if joints are calibrated
  if (!mechanism_chain_.allCalibrated(robot_state_->joint_states_))
    return;

  // get joint positions
  mechanism_chain_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // get the chain jacobian
  jnt_to_jac_solver_->JntToJac(jnt_pos_, chain_kdl_jacobian_);

  // TODO: Write a function for doing this conversion
  //convert to eigen for easier math
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); j++)
    {
      chain_eigen_jacobian_(i,j) = chain_kdl_jacobian_(i,j);
    }
  }

  // compute constraint torques
  computeConstraintTorques();

  // compute constraint torques
  computeConstraintJacobian();

  // compute constraint torques
  computeConstraintNullSpace();

  // compute the task torque on the joints
  for (unsigned int i=0; i<6; i++)
    task_wrench_(i) = wrench_desired_(i);
  task_torque_ = joint_constraint_null_space_ * chain_eigen_jacobian_.transpose() * task_wrench_;

  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    jnt_eff_(i) = joint_constraint_torque_(i) + task_torque_(i);
    //printf("effort:%lf\n",jnt_eff_(i));
  }
  // set effort to joints
  mechanism_chain_.setEfforts(jnt_eff_, robot_state_->joint_states_);
}


void JointChainConstraintController::computeConstraintTorques()
{
  // clear the constraint torque
  joint_constraint_torque_.setZero();
  // check there are constraints to enforce
  if(list_size_ == 0)
  {
    return;
  }

  double time = robot_state_->hw_->current_time_;
  double error(0);
  std::list<ConstraintState>::iterator it = constraint_list_.begin();
  for(int i = 0 ; i < list_size_; ++i)
  {

    int joint_type = (*it).joint_->type_;

    if(joint_type == mechanism::JOINT_ROTARY)
    {
      double limit_min = (*it).joint_->joint_limit_min_;
      double limit_max = (*it).joint_->joint_limit_max_;

      angles::shortest_angular_distance_with_limits((*it).threshold_start_, jnt_pos_((*it).joint_chain_index_), limit_min, limit_max, error);

    }
    else if(joint_type == mechanism::JOINT_CONTINUOUS)
    {
      error = angles::shortest_angular_distance((*it).threshold_start_, jnt_pos_((*it).joint_chain_index_));
    }
    else //prismatic
    {
      error = jnt_pos_((*it).joint_chain_index_) - (*it).threshold_start_;
    }

    // check the sign of the constraint
    double sign = -((*it).threshold_start_ - (*it).nullspace_start_)/fabs((*it).threshold_start_ - (*it).nullspace_start_);

    (*it).joint_error_ = error;

    if(sign*error > 0)
    {
      double temp_constraint_torque = (*it).pid_.updatePid(error, time - last_time_);
      // apply the limit
      if (fabs(temp_constraint_torque) > (*it).max_constraint_torque_)
        temp_constraint_torque = (*it).max_constraint_torque_ * temp_constraint_torque / fabs(temp_constraint_torque);
      // sum the torques
      joint_constraint_torque_((*it).joint_chain_index_) = joint_constraint_torque_((*it).joint_chain_index_)+ temp_constraint_torque;
    }
    else
    {
      // make error 0 for better accounting
      (*it).joint_error_ = 0;
    }
    it++;
  }
  last_time_ = time;

}


void JointChainConstraintController::computeConstraintJacobian()
{
  // clear the constraint jacobian
  joint_constraint_jacobian_.setZero();
  // check there are constraints to enforce
  if(list_size_ == 0)
  {
    return;
  }
  std::list<ConstraintState>::iterator it = constraint_list_.begin();

  for(int i = 0 ; i < list_size_; ++i)
  {
    // check to turn off control authority
    if((*it).joint_error_ > fabs((*it).threshold_start_ - (*it).nullspace_start_))
    {
      joint_constraint_jacobian_((*it).joint_chain_index_,(*it).joint_chain_index_) = 1;
    }
    it++;
  }

}


void JointChainConstraintController::computeConstraintNullSpace()
{
  // Compute generalized inverse, this is the transpose as long as the constraints are
  // orthonormal to eachother. Will replace with QR method later.
  joint_constraint_null_space_.setZero();
  joint_constraint_null_space_ =  identity_ - joint_constraint_jacobian_ * joint_constraint_jacobian_.transpose();

}


bool JointChainConstraintController::addConstraint(robot_mechanism_controllers::ChangeConstraints::Request &req,
                                                   robot_mechanism_controllers::ChangeConstraints::Response &resp)
{
  ConstraintState temp;
  bool found = false;

  for(unsigned int i =0; i < kdl_chain_.getNrOfJoints(); i++)
  {
    std::string name = mechanism_chain_.getJoint(i)->name_;
    printf("Checking joint %s\n", name.c_str());
    if(req.constraint.joint_name == name)
    {
      temp.joint_chain_index_ = i;
      temp.joint_ = mechanism_chain_.getJoint(i);
      found = true;
      break;
    }
  }

  if (!found) {
    ROS_ERROR("Unable to find joint with ID: \"%s\"", req.constraint.joint_name.c_str());
    resp.add_ok = 0;
    return false;
  }

  temp.pid_.initPid(req.constraint.p,req.constraint.i,req.constraint.d,req.constraint.i_clamp, -req.constraint.i_clamp);

  temp.id_ = req.constraint.id;
  temp.joint_name_ = req.constraint.joint_name;
  temp.threshold_start_ = req.constraint.threshold_start;
  temp.nullspace_start_ = req.constraint.nullspace_start;
  temp.max_constraint_torque_ = req.constraint.max_constraint_torque;
  temp.remove_ = false;
  temp.joint_error_ = 0;

  constraint_list_.push_back(temp);
  list_size_++;
  ROS_INFO("Added constraint on \"%s\" with ID: \"%s\".", req.constraint.joint_name.c_str(),req.constraint.id.c_str());

  resp.add_ok = 1;
  return true;

}






ROS_REGISTER_CONTROLLER(JointChainConstraintControllerNode)

JointChainConstraintControllerNode::JointChainConstraintControllerNode()
: node_(ros::Node::instance())
{}


JointChainConstraintControllerNode::~JointChainConstraintControllerNode()
{
  node_->unsubscribe(controller_name_ + "/command");
}


bool JointChainConstraintControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_=robot;
  // get the controller name
  controller_name_ = config->Attribute("name");

  // get name of root and tip
  std::string root_name, tip_name;
  node_->param(controller_name_+"/root_name", root_name, std::string("no_name_given"));
  node_->param(controller_name_+"/tip_name", tip_name, std::string("no_name_given"));

  // initialize wrench controller
  if (!controller_.init(robot, root_name, tip_name, controller_name_))
  {
    ROS_ERROR( "Failed to initialize JointChainConstraintController,\"%s\".", controller_name_.c_str());
    return false;
  }

  // subscribe to wrench commands
  node_->subscribe(controller_name_ + "/command", wrench_msg_,
		   &JointChainConstraintControllerNode::command, this, 1);


  return true;
}


void JointChainConstraintControllerNode::update()
{
  controller_.update();
}


void JointChainConstraintControllerNode::command()
{
  // convert to wrench command
  controller_.wrench_desired_(0) = wrench_msg_.force.x;
  controller_.wrench_desired_(1) = wrench_msg_.force.y;
  controller_.wrench_desired_(2) = wrench_msg_.force.z;
  controller_.wrench_desired_(3) = wrench_msg_.torque.x;
  controller_.wrench_desired_(4) = wrench_msg_.torque.y;
  controller_.wrench_desired_(5) = wrench_msg_.torque.z;
}


}; // namespace

