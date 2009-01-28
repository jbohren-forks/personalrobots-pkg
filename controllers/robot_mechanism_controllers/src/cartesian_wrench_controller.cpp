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

static const std::string controller_name = "cartesian_wrench";



namespace controller {

CartesianWrenchController::CartesianWrenchController()
: robot_state_(NULL),
  jnt_to_jac_solver_(NULL)
{}



CartesianWrenchController::~CartesianWrenchController()
{
  if (jnt_to_jac_solver_) delete jnt_to_jac_solver_;
}



bool CartesianWrenchController::initialize(mechanism::RobotState *robot_state, const std::string& root_name, const std::string& tip_name)
{
  cout << "initializing cartesian wrench controller between " << root_name << " and " << tip_name << endl;

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
  jnt_to_jac_solver_ = new ChainJntToJacSolver(chain_);
  jnt_pos_ = JntArray(num_joints_);
  jnt_eff_ = JntArray(num_joints_);
  jacobian_ = Jacobian(num_joints_, num_segments_);

  // set desired wrench to 0
  wrench_desi_ = Wrench::Zero();

  return true;
}




void CartesianWrenchController::update()
{
  // check if joints are calibrated
  if (!robot_.allCalibrated(robot_state_->joint_states_)) return;

  // get joint positions
  robot_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // get the chain jacobian
  jnt_to_jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // convert the wrench into joint efforts
  for (unsigned int i = 0; i < num_joints_; i++){
    jnt_eff_(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_eff_(i) += (jacobian_(j,i) * wrench_desi_(j));
  }

  // add effort to joints
  robot_.setEfforts(jnt_eff_, robot_state_->joint_states_);
}









ROS_REGISTER_CONTROLLER(CartesianWrenchControllerNode)

CartesianWrenchControllerNode::CartesianWrenchControllerNode()
: node_(ros::Node::instance())
{}


CartesianWrenchControllerNode::~CartesianWrenchControllerNode()
{
  node_->unsubscribe(controller_name + "/command");
}


bool CartesianWrenchControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: EndeffectorConstraintController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for EndeffectorConstraintController must specify the root\n");
    return false;
  }
  if (!tip_name)  {
    fprintf(stderr, "Error: Chain element for EndeffectorConstraintController must specify the tip\n");
    return false;
  }

  /*
  // get name of root and tip
  string root_name, tip_name;
  node_->param(controller_name+"/root_name", root_name, string("no_name_given"));
  node_->param(controller_name+"/tip_name", tip_name, string("no_name_given"));
  */

  // initialize wrench controller
  if (!controller_.initialize(robot, root_name, tip_name))
    return false;

  // subscribe to wrench commands
  node_->subscribe(controller_name + "/command", wrench_msg_,
		   &CartesianWrenchControllerNode::command, this, 1);

  return true;
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
