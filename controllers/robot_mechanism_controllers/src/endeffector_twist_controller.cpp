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
#include "robot_mechanism_controllers/endeffector_twist_controller.h"


using namespace KDL;
namespace controller {


ROS_REGISTER_CONTROLLER(EndeffectorTwistController)


EndeffectorTwistController::EndeffectorTwistController()
: jnt_to_twist_solver_(NULL),
  joints_(0,(mechanism::JointState*)NULL)
{}



EndeffectorTwistController::~EndeffectorTwistController()
{
  if (jnt_to_twist_solver_) delete jnt_to_twist_solver_;
}



bool EndeffectorTwistController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  fprintf(stderr, "initializing twist controller\n");

  // test if we got robot pointer
  assert(robot);
  robot_ = robot;

  // get pid controller
  TiXmlElement *p_trans = config->FirstChildElement("pid_trans");
  control_toolbox::Pid pid_trans;
  pid_trans.initXml(p_trans);
  for (unsigned int i=0; i<3; i++)
    pid_controller_.push_back(pid_trans);

  TiXmlElement *p_rot = config->FirstChildElement("pid_rot");
  control_toolbox::Pid pid_rot;
  pid_rot.initXml(p_rot);
  for (unsigned int i=0; i<3; i++)
    pid_controller_.push_back(pid_rot);
  fprintf(stderr, "pid controllers created\n");


  // time
  last_time_ = robot->hw_->current_time_;

  // set disired twist to 0
  twist_desi_ = Twist::Zero();

  // create wrench controller
  wrench_controller_.initXml(robot, config);

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
  jnt_to_twist_solver_ = new ChainFkSolverVel_recursive(chain_);

  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: EndeffectorTwistController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for EndeffectorTwistController must specify the root\n");
    return false;
  }
  if (!tip_name)  {
    fprintf(stderr, "Error: Chain element for EndeffectorTwistController must specify the tip\n");
    return false;
  }

  // test if we can get root from robot
  if (!robot->getLinkState(root_name)) {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorTwistController)\n", root_name);
    return false;
  }

  // get tip from robot
  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)  {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorTwistController)\n", tip_name);
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
	  fprintf(stderr, "Error: for EndeffectorTwistController, tip is not connected to root\n");
	  return false;
	}
    }
  // reverse order of joint vector
  std::reverse(joints_.begin(), joints_.end());


  fprintf(stderr, "initialized twist controller\n");

  return true;
}






void EndeffectorTwistController::update()
{
  // get current time
  double time = robot_->hw_->current_time_;

  // check if joints are calibrated
  for (unsigned int i = 0; i < joints_.size(); ++i) {
    if (!joints_[i]->calibrated_)
      return;
  }

  // get the joint positions and velocities
  JntArrayVel jnt_posvel(num_joints_);
  for (unsigned int i=0; i<num_joints_; i++){
    jnt_posvel.q(i)    = joints_[i]->position_;
    jnt_posvel.qdot(i) = joints_[i]->velocity_;
  }

  // get endeffector twist error
  FrameVel twist; 
  jnt_to_twist_solver_->JntToCart(jnt_posvel, twist);
  twist_meas_ = twist.deriv();
  Twist error = twist_meas_ - twist_desi_;
  double dt = time - last_time_;

  // pid feedback
  for (unsigned int i=0; i<6; i++)
    wrench_out_(i)  = pid_controller_[i].updatePid(error(i), dt);

  // send wrench to wrench controller
  wrench_controller_.wrench_desi_ = wrench_out_;
  wrench_controller_.update();

  // remember time
  last_time_ = time;
}








ROS_REGISTER_CONTROLLER(EndeffectorTwistControllerNode)

EndeffectorTwistControllerNode::~EndeffectorTwistControllerNode()
{
  ros::node *node = ros::node::instance();

  node->unsubscribe(topic_ + "/command");
  node->unsubscribe(topic_ + "spacenav/joy");
}


bool EndeffectorTwistControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic_ to listen to
  ros::node *node = ros::node::instance();
  topic_ = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic_ == "") {
    fprintf(stderr, "No topic given to EndeffectorTwistControllerNode\n");
    return false;
  }

  // get parameters
  node->param("arm_twist/joystick_max_trans", joystick_max_trans_, 0.0);
  node->param("arm_twist/joystick_max_rot", joystick_max_rot_, 0.0);

  // initialize controller  
  if (!controller_.initXml(robot, config))
    return false;
  
  // subscribe to twist commands
  node->subscribe(topic_ + "/command", twist_msg_,
                  &EndeffectorTwistControllerNode::command, this, 1);
  guard_command_.set(topic_ + "/command");

  // subscribe to joystick commands
  node->subscribe("spacenav/joy", joystick_msg_,
                  &EndeffectorTwistControllerNode::joystick, this, 1);
  guard_command_.set("spacenav/joy");

  return true;
}


void EndeffectorTwistControllerNode::update()
{
  controller_.update();
}


void EndeffectorTwistControllerNode::command()
{
  // convert to twist command
  controller_.twist_desi_.vel(0) = twist_msg_.vel.x;
  controller_.twist_desi_.vel(1) = twist_msg_.vel.y;
  controller_.twist_desi_.vel(2) = twist_msg_.vel.z;
  controller_.twist_desi_.rot(0) = twist_msg_.rot.x;
  controller_.twist_desi_.rot(1) = twist_msg_.rot.y;
  controller_.twist_desi_.rot(2) = twist_msg_.rot.z;
}


void EndeffectorTwistControllerNode::joystick()
{
  // convert to twist command
  for (unsigned int i=0; i<3; i++){
    controller_.twist_desi_.vel(i)  = joystick_msg_.axes[i]   * joystick_max_trans_;
    controller_.twist_desi_.rot(i)  = joystick_msg_.axes[i+3] * joystick_max_rot_;
  }
}


}; // namespace
