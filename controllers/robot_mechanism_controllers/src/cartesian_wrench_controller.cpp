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


static const double JOYSTICK_MAX_FORCE  = 20.0;
static const double JOYSTICK_MAX_TORQUE = 0.75;


using namespace KDL;

namespace controller {

ROS_REGISTER_CONTROLLER(CartesianWrenchController)

CartesianWrenchController::CartesianWrenchController()
: robot_(NULL),
  jnt_to_jac_solver_(NULL)
{
  printf("CartesianWrenchController constructor\n");
}



CartesianWrenchController::~CartesianWrenchController()
{
  if (jnt_to_jac_solver_) delete jnt_to_jac_solver_;
}



bool CartesianWrenchController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot;

  // set desired wrench to 0
  for (unsigned int i=0; i<3; i++){
    wrench_desi_.force(i) = 0;
    wrench_desi_.torque(i) = 0;
  }

  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: CartesianWrenchController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for CartesianWrenchController must specify the root\n");
    return false;
  }
  if (!tip_name) {
    fprintf(stderr, "Error: Chain element for CartesianWrenchController must specify the tip\n");
    return false;
  }

  if (!chain_.init(robot->model_, root_name, tip_name))
    return false;

  chain_.toKDL(kdl_chain_);
  jnt_to_jac_solver_ = new ChainJntToJacSolver(kdl_chain_);

  return true;
}




void CartesianWrenchController::update()
{
  if (!chain_.allCalibrated(robot_->joint_states_))
    return;

  JntArray jnt_pos(kdl_chain_.getNrOfJoints());
  chain_.positionsToKDL(robot_->joint_states_, jnt_pos);

  // get the chain jacobian
  Jacobian jacobian(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());
  jnt_to_jac_solver_->JntToJac(jnt_pos, jacobian);

  // convert the wrench into joint torques
  JntArray jnt_torq(kdl_chain_.getNrOfJoints());
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
  {
    jnt_torq(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_torq(i) += (jacobian(j,i) * wrench_desi_(j));
  }
  chain_.addEffortsFromKDL(jnt_torq, robot_->joint_states_);
}









ROS_REGISTER_CONTROLLER(CartesianWrenchControllerNode)

CartesianWrenchControllerNode::~CartesianWrenchControllerNode()
{
  ros::Node *node = ros::Node::instance();
  node->unsubscribe(topic_ + "/command");
}

bool CartesianWrenchControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to from xml file
  ros::Node *node = ros::Node::instance();
  topic_ = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic_ == "") {
    ROS_ERROR("No topic given to CartesianWrenchControllerNode");
    return false;
  }

  // initialize controller
  if (!controller_.initXml(robot, config))
    return false;

  // subscribe to wrench commands
  node->subscribe(topic_ + "/command", wrench_msg_,
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
