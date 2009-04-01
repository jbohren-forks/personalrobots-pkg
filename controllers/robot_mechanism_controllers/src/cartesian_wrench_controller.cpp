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
#include "robot_mechanism_controllers/cartesian_wrench_controller.h"


using namespace KDL;



namespace controller {

CartesianWrenchController::CartesianWrenchController()
: robot_state_(NULL),
  jnt_to_jac_solver_(NULL),
  diagnostics_publisher_("/diagnostics", 2)
{}



CartesianWrenchController::~CartesianWrenchController()
{
  if (jnt_to_jac_solver_) delete jnt_to_jac_solver_;
}



bool CartesianWrenchController::init(mechanism::RobotState *robot_state, 
                                     const std::string& root_name, 
                                     const std::string& tip_name, 
                                     const std::string& controller_name)
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
  num_joints_   = kdl_chain_.getNrOfJoints();
  jnt_to_jac_solver_ = new ChainJntToJacSolver(kdl_chain_);
  jnt_pos_.resize(num_joints_);
  jnt_eff_.resize(num_joints_);
  jacobian_.resize(num_joints_);

  // diagnostics messages
  cout << "Initialize diagnostics !!!!!" << endl;
  diagnostics_.status.resize(1);
  diagnostics_.status[0].name  = "Wrench Controller";
  diagnostics_.status[0].values.resize(1);
  diagnostics_.status[0].values[0].value = 3;
  diagnostics_.status[0].values[0].label = "TestValueLabel";

  diagnostics_.status[0].strings.resize(1);
  diagnostics_.status[0].strings[0].value = "TestValue";
  diagnostics_.status[0].strings[0].label = "TestLabel";

  diagnostics_time_ = ros::Time::now();
  diagnostics_interval_ = ros::Duration().fromSec(1.0);

  return true;
}


bool CartesianWrenchController::starting()
{
  // set desired wrench to 0
  wrench_desi_ = Wrench::Zero();

  return true;
}



void CartesianWrenchController::update()
{


  // check if joints are calibrated
  if (!chain_.allCalibrated(robot_state_->joint_states_)){
    publishDiagnostics(2, "Not all joints are calibrated");
    return;
  }
  publishDiagnostics(0, "No problems detected");

  // get joint positions
  chain_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // get the chain jacobian
  jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // convert the wrench into joint efforts
  for (unsigned int i = 0; i < num_joints_; i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_desi_(j));
  }

  // set effort to joints
  chain_.setEfforts(jnt_eff_, robot_state_->joint_states_);
}



bool CartesianWrenchController::publishDiagnostics(int level, const string& message)
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






ROS_REGISTER_CONTROLLER(CartesianWrenchControllerNode)

CartesianWrenchControllerNode::CartesianWrenchControllerNode()
: node_(ros::Node::instance())
{}


CartesianWrenchControllerNode::~CartesianWrenchControllerNode()
{
  node_->unsubscribe(controller_name_ + "/command");
}


bool CartesianWrenchControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get the controller name from xml file
  controller_name_ = config->Attribute("name") ? config->Attribute("name") : "";
  if (controller_name_ == ""){
    ROS_ERROR("CartesianWrenchControllerNode: No controller name given in xml file");
    return false;
  }

  // get name of root and tip from the parameter server
  std::string root_name, tip_name;
  if (!node_->getParam(controller_name_+"/root_name", root_name)){
    ROS_ERROR("CartesianWrenchControllerNode: No root name found on parameter server");
    return false;
  }
  if (!node_->getParam(controller_name_+"/tip_name", tip_name)){
    ROS_ERROR("CartesianWrenchControllerNode: No tip name found on parameter server");
    return false;
  }

  // initialize wrench controller
  if (!controller_.init(robot, root_name, tip_name, controller_name_)) return false;

  // subscribe to wrench commands
  node_->subscribe(controller_name_ + "/command", wrench_msg_,
		   &CartesianWrenchControllerNode::command, this, 1);

  return true;
}

bool CartesianWrenchControllerNode::starting()
{
  return controller_.starting();
}

void CartesianWrenchControllerNode::update()
{
  controller_.update();
}


void CartesianWrenchControllerNode::command()
{
  // convert to wrench command
  controller_.wrench_desi_.force(0) = wrench_msg_.force.x;
  controller_.wrench_desi_.force(1) = wrench_msg_.force.y;
  controller_.wrench_desi_.force(2) = wrench_msg_.force.z;
  controller_.wrench_desi_.torque(0) = wrench_msg_.torque.x;
  controller_.wrench_desi_.torque(1) = wrench_msg_.torque.y;
  controller_.wrench_desi_.torque(2) = wrench_msg_.torque.z;
}

}; // namespace
