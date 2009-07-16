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
#include <mechanism_control/mechanism_control.h>
#include "kdl/chainfksolvervel_recursive.hpp"
#include "robot_mechanism_controllers/cartesian_pose_controller.h"
#include "tf_conversions/tf_kdl.h"


using namespace KDL;
using namespace tf;
using namespace std;


namespace controller {

ROS_REGISTER_CONTROLLER(CartesianPoseController)

CartesianPoseController::CartesianPoseController()
: robot_state_(NULL),
  pids_(6),
  tf_(*ros::Node::instance())
{}

CartesianPoseController::~CartesianPoseController()
{}


bool CartesianPoseController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  // get the controller name from xml file
  std::string controller_name = config->Attribute("name") ? config->Attribute("name") : "";
  if (controller_name == ""){
    ROS_ERROR("CartesianPoseController: No controller name given in xml file");
    return false;
  }

  return init(robot_state, controller_name);
}

bool CartesianPoseController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &n)
{
  node_ = n;

  // get name of root and tip from the parameter server
  std::string tip_name;
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("CartesianPoseController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name)){
    ROS_ERROR("CartesianPoseController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!chain_.init(robot_state_->model_, root_name_, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // create solver
  jnt_to_posevel_solver_.reset(new ChainFkSolverVel_recursive(kdl_chain_));
  jnt_vel.resize(kdl_chain_.getNrOfJoints());

  // Pids

  control_toolbox::Pid temp_pid;

  if (!temp_pid.init(ros::NodeHandle(node_, "fb_trans")))
    return false;
  for (size_t i = 0; i < 3; ++i)
    pids_[i] = temp_pid;

  if (!temp_pid.init(ros::NodeHandle(node_, "fb_rot")))
    return false;
  for (size_t i = 3; i < 6; ++i)
    pids_[i] = temp_pid;

  // Gets a pointer to the wrench controller
  MechanismControl* mc;
  if (!MechanismControl::Instance(mc)){
    ROS_ERROR("CartesianPoseController: could not get instance to mechanism control");
    return false;
  }
  string output;
  if (!node_.getParam("output", output)){
    ROS_ERROR("No ouptut name found on parameter server (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!mc->getControllerByName<CartesianWrenchController>(output, wrench_controller_)){
    ROS_ERROR("Could not connect to wrench controller \"%s\"", output.c_str());
    return false;
  }

  // subscribe to pose commands
  command_notifier_.reset(new MessageNotifier<robot_msgs::PoseStamped>(&tf_, ros::Node::instance(),
                                                                       boost::bind(&CartesianPoseController::command, this, _1),
                                                                       node_.getNamespace() + "/command", root_name_, 10));
  // realtime publisher for control state
  state_error_publisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::Twist>(node_, "state/error", 1));
  state_pose_publisher_.reset(new realtime_tools::RealtimePublisher<robot_msgs::PoseStamped>(node_, "state/pose", 1));

  return true;
}

bool CartesianPoseController::starting()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pids_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = Twist::Zero();
  pose_desi_ = getPose();
  last_time_ = robot_state_->hw_->current_time_;

  loop_count_ = 0;
  return true;
}



void CartesianPoseController::update()
{
  // get time
  double time = robot_state_->hw_->current_time_;
  double dt = time - last_time_;
  last_time_ = time;

  // Gets the current pose and twist of the tip
  KDL::JntArrayVel jnt_vel(kdl_chain_.getNrOfJoints());
  chain_.getVelocities(robot_state_->joint_states_, jnt_vel);
  KDL::FrameVel tip_framevel;
  jnt_to_posevel_solver_->JntToCart(jnt_vel, tip_framevel);
  pose_meas_ = tip_framevel.GetFrame();
  KDL::Twist twist_meas_ = tip_framevel.GetTwist();

  // Filters the measured twist.
  KDL::Twist twist_meas_filtered_ = twist_meas_;

  pose_error_ = diff(pose_desi_, pose_meas_);

  // Determines the desired wrench from the pose error.
  KDL::Wrench wrench_desi;
  for (size_t i = 0; i < pids_.size(); ++i)
    wrench_desi(i) = pids_[i].updatePid(pose_error_(i), twist_meas_filtered_(i), dt);

  // Passes the desired wrench onto the wrench controller.
  wrench_controller_->wrench_desi_ = wrench_desi;

  if (++loop_count_ % 100 == 0){
    if (state_error_publisher_){
      if (state_error_publisher_->trylock()){
        tf::TwistKDLToMsg(pose_error_, state_error_publisher_->msg_);
        state_error_publisher_->unlockAndPublish();
      }
    }
    if (state_pose_publisher_){
      if (state_pose_publisher_->trylock()){
        Stamped<Pose> tmp;
        tf::PoseKDLToTF(pose_meas_, tmp);
        tmp.stamp_ = ros::Time::now();
        tmp.frame_id_ = root_name_;
        tf::poseStampedTFToMsg(tmp, state_pose_publisher_->msg_);
        state_pose_publisher_->unlockAndPublish();
      }
    }
  }
}



Frame CartesianPoseController::getPose()
{
  // get the joint positions and velocities
  chain_.getVelocities(robot_state_->joint_states_, jnt_vel);

  // get cartesian pose
  FrameVel result;
  jnt_to_posevel_solver_->JntToCart(jnt_vel, result);

  return result.GetFrame();
}

void CartesianPoseController::command(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  poseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain
  tf_.transformPose(root_name_, pose_stamped, pose_stamped);
  tf::PoseTFToKDL(pose_stamped, pose_desi_);
}

} // namespace

