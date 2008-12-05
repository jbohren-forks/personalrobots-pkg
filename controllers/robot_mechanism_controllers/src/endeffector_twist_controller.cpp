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

#define SPACENAV_RANGE      400.0
#define SPACENAV_MAX_VEL    10.0
#define SPACENAV_MAX_ROT    2.0
#define MASS_TRANS          2.0
#define MASS_ROT            0.5



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
  // set disired twist to 0
  for (unsigned int i=0; i<3; i++){
    twist_desi_.vel(i) = 0;
    twist_desi_.rot(i) = 0;
  }

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

  // test if we got robot pointer
  assert(robot);

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


  printf("EndeffectorTwistController succesfully initialized\n");
  return true;
}






void EndeffectorTwistController::update()
{
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

  // get endeffector twist
  FrameVel twist; 
  jnt_to_twist_solver_->JntToCart(jnt_posvel, twist);
  twist_meas_ = twist.deriv();

  // twist feedback into wrench
  Twist diff = twist_desi_ - twist_meas_;
  for (unsigned int i=0; i<3; i++){
    wrench_out_.force(i)  = MASS_TRANS * diff.vel(i);
    wrench_out_.torque(i) = MASS_ROT   * diff.rot(i);
  }

  // send wrench to wrench controller
  wrench_controller_.wrench_desi_ = wrench_out_;
  wrench_controller_.update();
}








ROS_REGISTER_CONTROLLER(EndeffectorTwistControllerNode)



bool EndeffectorTwistControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to
  ros::node *node = ros::node::instance();
  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "") {
    fprintf(stderr, "No topic given to EndeffectorTwistControllerNode\n");
    return false;
  }

  // initialize controller  
  if (!controller_.initXml(robot, config))
    return false;
  
  // subscribe to twist commands
  node->subscribe(topic + "/command", twist_msg_,
                  &EndeffectorTwistControllerNode::command, this, 1);
  guard_command_.set(topic + "/command");

  // subscribe to spacenav pos commands
  node->subscribe("spacenav/offset", spacenav_pos_msg_,
                  &EndeffectorTwistControllerNode::spacenavPos, this, 1);
  guard_command_.set("spacenav/offset");

  // subscribe to spacenav rot commands
  node->subscribe("spacenav/rot_offset", spacenav_rot_msg_,
                  &EndeffectorTwistControllerNode::spacenavRot, this, 1);
  guard_command_.set("spacenav/rot_offset");


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

void EndeffectorTwistControllerNode::spacenavPos()
{
  // convert to trans_vel command
  controller_.twist_desi_.vel(0) = spacenav_pos_msg_.x * SPACENAV_MAX_VEL / SPACENAV_RANGE;
  controller_.twist_desi_.vel(1) = spacenav_pos_msg_.y * SPACENAV_MAX_VEL / SPACENAV_RANGE;
  controller_.twist_desi_.vel(2) = spacenav_pos_msg_.z * SPACENAV_MAX_VEL / SPACENAV_RANGE;
}

void EndeffectorTwistControllerNode::spacenavRot()
{
  // convert to rot_vel
  controller_.twist_desi_.rot(0) = spacenav_rot_msg_.x * SPACENAV_MAX_ROT / SPACENAV_RANGE;
  controller_.twist_desi_.rot(1) = spacenav_rot_msg_.y * SPACENAV_MAX_ROT / SPACENAV_RANGE;
  controller_.twist_desi_.rot(2) = spacenav_rot_msg_.z * SPACENAV_MAX_ROT / SPACENAV_RANGE;
}



}; // namespace
