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
#include "robot_mechanism_controllers/endeffector_pose_controller.h"


using namespace KDL;
namespace controller {



ROS_REGISTER_CONTROLLER(EndeffectorPoseController)


EndeffectorPoseController::EndeffectorPoseController()
: jnt_to_pose_solver_(NULL),
  joints_(0,(mechanism::JointState*)NULL)
{}

EndeffectorPoseController::~EndeffectorPoseController()
{
  if (jnt_to_pose_solver_) delete jnt_to_pose_solver_;
}



bool EndeffectorPoseController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  fprintf(stderr, "initializing pose controller\n");

  // test if we got robot pointer
  assert(robot);
  robot_ = robot;

  // get pid controller
  TiXmlElement *p_pose = config->FirstChildElement("pid_pose");
  control_toolbox::Pid pid_pose;
  pid_pose.initXml(p_pose);
  for (unsigned int i=0; i<6; i++)
    pid_controller_.push_back(pid_pose);
  fprintf(stderr, "pid controllers created\n");

  // time
  last_time_ = robot->hw_->current_time_;

  // create twist controller
  twist_controller_.initXml(robot, config);

  // parse robot description from xml file
  ros::Node *node = ros::Node::instance();
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
  jnt_to_pose_solver_ = new ChainFkSolverPos_recursive(chain_);

  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: EndeffectorPoseController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for EndeffectorPoseController must specify the root\n");
    return false;
  }
  if (!tip_name)  {
    fprintf(stderr, "Error: Chain element for EndeffectorPoseController must specify the tip\n");
    return false;
  }

  // test if we can get root from robot
  if (!robot->getLinkState(root_name)) {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorPoseController)\n", root_name);
    return false;
  }

  // get tip from robot
  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)  {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorPoseController)\n", tip_name);
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
	  fprintf(stderr, "Error: for EndeffectorPoseController, tip is not connected to root\n");
	  return false;
	}
    }
  // reverse order of joint vector
  std::reverse(joints_.begin(), joints_.end());

  // set desired pose to current pose
  pose_desi_ = getPose();

  return true;
}






void EndeffectorPoseController::update()
{
  // get current time
  double time = robot_->hw_->current_time_;
  double dt = time - last_time_;

  // get current pose
  pose_meas_ = getPose();

  // pose feedback into twist
  Twist twist_error = diff(pose_desi_, pose_meas_);
  for (unsigned int i=0; i<6; i++)
    twist_out_(i) = pid_controller_[i].updatePid(twist_error(i), dt);

  // send twist to twist controller
  twist_controller_.twist_desi_ = twist_out_;
  twist_controller_.update();

  // remember time
  last_time_ = time;
}



Frame EndeffectorPoseController::getPose()
{
  // check if joints are calibrated
  for (unsigned int i = 0; i < joints_.size(); ++i) {
    if (!joints_[i]->calibrated_)
      fprintf(stderr,"Joint not calibrated\n");
  }

  // get the joint positions 
  JntArray jnt_pos(num_joints_);
  for (unsigned int i=0; i<num_joints_; i++)
    jnt_pos(i) = joints_[i]->position_;

  // get endeffector pose
  Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos, result);

  return result;
}




ROS_REGISTER_CONTROLLER(EndeffectorPoseControllerNode)

EndeffectorPoseControllerNode::~EndeffectorPoseControllerNode()
{
  ros::Node *node = ros::Node::instance();

  node->unsubscribe(topic_ + "/command");
  node->unsubscribe("spacenav/joy");
}


bool EndeffectorPoseControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to
  ros::Node *node = ros::Node::instance();
  topic_ = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic_ == "") {
    fprintf(stderr, "No topic given to EndeffectorPoseControllerNode\n");
    return false;
  }

  // get parameters
  node->param("arm_pose/joystick_max_trans", joystick_max_trans_, 0.0);
  node->param("arm_pose/joystick_max_rot", joystick_max_rot_, 0.0);

  // initialize controller  
  if (!controller_.initXml(robot, config))
    return false;
  
  // subscribe to pose commands
  node->subscribe(topic_ + "/command", pose_msg_,
                  &EndeffectorPoseControllerNode::command, this, 1);

  // subscribe to joystick commands
  node->subscribe("spacenav/joy", joystick_msg_,
                  &EndeffectorPoseControllerNode::joystick, this, 1);

  joystick_twist_ = Twist::Zero();
  return true;
}


void EndeffectorPoseControllerNode::update()
{
  controller_.update();
}


void EndeffectorPoseControllerNode::command()
{
  // convert to pose command
  controller_.pose_desi_.p(0) = pose_msg_.pose.position.x;
  controller_.pose_desi_.p(1) = pose_msg_.pose.position.y;
  controller_.pose_desi_.p(2) = pose_msg_.pose.position.z;
  controller_.pose_desi_.M = Rotation::Quaternion(pose_msg_.pose.orientation.x, pose_msg_.pose.orientation.y,
						  pose_msg_.pose.orientation.z, pose_msg_.pose.orientation.w);
}



void EndeffectorPoseControllerNode::joystick()
{
  // convert to pose command
  for (unsigned int i=0; i<3; i++){
    joystick_twist_.vel(i) = joystick_msg_.axes[i]   * joystick_max_trans_;
    joystick_twist_.rot(i) = joystick_msg_.axes[i+3] * joystick_max_rot_;
  }

  controller_.pose_desi_  = addDelta(controller_.pose_desi_, joystick_twist_);
}


}; // namespace

