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
#include <mechanism_control/mechanism_control.h>
#include "robot_mechanism_controllers/cartesian_pose_controller.h"


using namespace KDL;
using namespace tf;
using namespace std;


namespace controller {

ROS_REGISTER_CONTROLLER(CartesianPoseController)

CartesianPoseController::CartesianPoseController()
: node_(ros::Node::instance()),
  robot_state_(NULL),
  jnt_to_pose_solver_(NULL),
  error_publisher_(NULL),
  tf_(*node_),
  command_notifier_(NULL)
{}

CartesianPoseController::~CartesianPoseController()
{
  if (command_notifier_) delete command_notifier_;
}


bool CartesianPoseController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  // get the controller name from xml file
  controller_name_ = config->Attribute("name") ? config->Attribute("name") : "";
  if (controller_name_ == ""){
    ROS_ERROR("CartesianPoseController: No controller name given in xml file");
    return false;
  }

  // get name of root and tip from the parameter server
  std::string tip_name;
  if (!node_->getParam(controller_name_+"/root_name", root_name_)){
    ROS_ERROR("CartesianPoseController: No root name found on parameter server");
    return false;
  }
  if (!node_->getParam(controller_name_+"/tip_name", tip_name)){
    ROS_ERROR("CartesianPoseController: No tip name found on parameter server");
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!robot_.init(robot_state_->model_, root_name_, tip_name))
    return false;
  robot_.toKDL(chain_);

  // create solver
  jnt_to_pose_solver_.reset(new ChainFkSolverPos_recursive(chain_));
  jnt_pos_.resize(chain_.getNrOfJoints());

  // create 6 identical pid controllers for x, y and z translation, and x, y, z rotation
  control_toolbox::Pid pid_controller;
  if (!pid_controller.initParam(controller_name_)) return false;
  for (unsigned int i=0; i<6; i++)
    pid_controller_.push_back(pid_controller);

  // get a pointer to the twist controller
  MechanismControl* mc;
  if (!MechanismControl::Instance(mc)){
    ROS_ERROR("CartesianPoseController: could not get instance to mechanism control");
    return false;
  }
  if (!mc->getControllerByName<CartesianTwistController>("cartesian_twist", twist_controller_)){
    ROS_ERROR("CartesianTwistController: could not connect to twist controller");
    return false;
  }

  // subscribe to pose commands
  command_notifier_ = new MessageNotifier<robot_msgs::PoseStamped>(&tf_, node_,  
								 boost::bind(&CartesianPoseController::command, this, _1),
								 controller_name_ + "/command", root_name_, 1);
  // realtime publisher for control error
  error_publisher_ = new realtime_tools::RealtimePublisher<robot_msgs::Twist>(controller_name_+"/error", 1);
  loop_count_ = 0;

  return true;
}


bool CartesianPoseController::starting()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pid_controller_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = Twist::Zero();
  pose_desi_ = getPose();
  last_time_ = robot_state_->hw_->current_time_;
}



void CartesianPoseController::update()
{
  // get time
  double time = robot_state_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  // get current pose
  pose_meas_ = getPose();

  // pose feedback into twist
  Twist twist_error = diff(pose_desi_, pose_meas_);
  Twist twist_fb;
  for (unsigned int i=0; i<6; i++)
    twist_fb(i) = pid_controller_[i].updatePid(twist_error(i), dt);

  // send feedback twist and feedforward twist to twist controller
  twist_controller_->twist_desi_ = twist_fb + twist_ff_;
  twist_controller_->update();

  if (++loop_count_ % 100 == 0){
    if (error_publisher_){
      if (error_publisher_->trylock()){
        error_publisher_->msg_.vel.x = twist_error.vel(0);
        error_publisher_->msg_.vel.y = twist_error.vel(1);
        error_publisher_->msg_.vel.z = twist_error.vel(2);
        error_publisher_->msg_.rot.x = twist_error.rot(0);
	error_publisher_->msg_.rot.y = twist_error.rot(1);
        error_publisher_->msg_.rot.z = twist_error.rot(2);
        error_publisher_->unlockAndPublish();
      }
    }
  }

}

Frame CartesianPoseController::getPose()
{
  // get the joint positions and velocities
  robot_.getPositions(robot_state_->joint_states_, jnt_pos_);

  // get cartesian pose
  Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);

  return result;
}

void CartesianPoseController::command(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  PoseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain  
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  TransformToFrame(pose_stamped, pose_desi_);
}

void CartesianPoseController::TransformToFrame(const Transform& trans, Frame& frame)
{
  frame.p(0) = trans.getOrigin().x();
  frame.p(1) = trans.getOrigin().y();
  frame.p(2) = trans.getOrigin().z();

  double Rz, Ry, Rx;
  trans.getBasis().getEulerZYX(Rz, Ry, Rx);
  frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
}



}; // namespace

