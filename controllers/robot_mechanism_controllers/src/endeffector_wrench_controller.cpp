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

#define MAX_PRINT_COUNTER  500


#include "urdf/parser.h"
#include <algorithm>
#include "robot_kinematics/robot_kinematics.h"
#include "robot_mechanism_controllers/endeffector_wrench_controller.h"


using namespace KDL;

namespace controller {

ROS_REGISTER_CONTROLLER(EndeffectorWrenchController)


EndeffectorWrenchController::EndeffectorWrenchController()
: print_counter_(0),
  jnt_to_jac_solver_(NULL),
  joints_(0,(mechanism::JointState*)NULL)
{
  printf("EndeffectorWrenchController::EndeffectorWrenchController\n");
}


EndeffectorWrenchController::~EndeffectorWrenchController()
{
  if (jnt_to_jac_solver_) delete jnt_to_jac_solver_;
}



bool EndeffectorWrenchController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // parse robot description from xml file
  ros::node *node = ros::node::instance();
  robot_kinematics::RobotKinematics robot_kinematics ;
  string robot_desc;
  node->param("robotdesc/pr2", robot_desc, string("")) ;
  printf("RobotDesc.length() = %u\n", robot_desc.length());
  robot_kinematics.loadString(robot_desc.c_str()) ;
  robot_kinematics::SerialChain* serial_chain = robot_kinematics.getSerialChain("right_arm");
  if (serial_chain == NULL)  
    fprintf(stderr, "Got NULL Chain\n") ;

  // convert description to KDL chain
  chain_        = serial_chain->chain;
  num_joints_   = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
  printf("Extracted KDL Chain with %u Joints and %u segments\n", num_joints_, num_segments_ );
  jnt_to_jac_solver_ = new ChainJntToJacSolver(chain_);

  // test if we got robot pointer
  assert(robot);

  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: EndeffectorWrenchController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for EndeffectorWrenchController must specify the root\n");
    return false;
  }
  if (!tip_name)  {
    fprintf(stderr, "Error: Chain element for EndeffectorWrenchController must specify the tip\n");
    return false;
  }

  // test if we can get root from robot
  if (!robot->getLinkState(root_name)) {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorWrenchController)\n", root_name);
    return false;
  }

  // get tip from robot
  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)  {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorWrenchController)\n", tip_name);
    return false;
  }

  // Works up the chain, from the tip to the root, and get joints
  while (current->link_->name_ != std::string(root_name)) 
    {
      // get joint from current link
      joints_.push_back(robot->getJointState(current->link_->joint_name_));
      assert(joints_[joints_.size()-1]);
      
      // get parent link
      current = robot->getLinkState(current->link_->parent_name_);
      
      if (!current) {
	  fprintf(stderr, "Error: for EndeffectorWrenchController, tip is not connected to root\n");
	  return false;
	}
    }
  // reverse order of joint vector
  std::reverse(joints_.begin(), joints_.end());

  return true;
}




void EndeffectorWrenchController::update()
{
  //  wrench_desi_.force(0) = 10;
  if (print_counter_ == MAX_PRINT_COUNTER)
    printf("wrench desired %f %f %f %f %f %f\n",
	   wrench_desi_(0), wrench_desi_(1), wrench_desi_(2),wrench_desi_(3), wrench_desi_(4), wrench_desi_(5));
	   

  // check if joints are calibrated
  for (unsigned int i = 0; i < joints_.size(); ++i) {
    if (!joints_[i]->calibrated_)
      return;
  }

  // get the joint positions
  JntArray jnt_pos(num_joints_);
  for (unsigned int i=0; i<num_joints_; i++)
    jnt_pos(i) = joints_[i]->position_;

  // get the chain jacobian
  Jacobian jacobian(num_joints_, num_segments_);
  jnt_to_jac_solver_->JntToJac(jnt_pos, jacobian);

  // convert the wrench into joint torques
  JntArray jnt_torq(num_joints_);
  for (unsigned int i=0; i<num_joints_; i++){
    jnt_torq(i) = 0;
    for (unsigned int j=0; j<6; j++)
      jnt_torq(i) += (jacobian(j,i) * wrench_desi_(j));
    joints_[i]->commanded_effort_ = jnt_torq(i);
    if (print_counter_ == MAX_PRINT_COUNTER)
      printf("joint torque %i set to %f\n",i,jnt_torq(i));
  }




  if (print_counter_ >= MAX_PRINT_COUNTER)
    print_counter_ = 0;
  print_counter_ ++;
}















ROS_REGISTER_CONTROLLER(EndeffectorWrenchControllerNode)


EndeffectorWrenchControllerNode::EndeffectorWrenchControllerNode()
{
  fprintf(stderr, "EndeffectorWrenchControllerNode::EndeffectorWrenchControllerNode\n");
}


EndeffectorWrenchControllerNode::~EndeffectorWrenchControllerNode()
{
}


bool EndeffectorWrenchControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to
  ros::node *node = ros::node::instance();
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "") {
    fprintf(stderr, "No toooopic given to EndeffectorWrenchControllerNode--\n");
    return false;
  }

  // initialize controller  
  if (!controller_.initXml(robot, config))
    return false;
  
  // subscribe to wrench commands
  node->subscribe(topic + "/command", wrench_msg_,
                  &EndeffectorWrenchControllerNode::command, this, 1);
  guard_command_.set(topic + "/command");
  return true;
}


void EndeffectorWrenchControllerNode::update()
{
  controller_.update();
}


void EndeffectorWrenchControllerNode::command()
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
