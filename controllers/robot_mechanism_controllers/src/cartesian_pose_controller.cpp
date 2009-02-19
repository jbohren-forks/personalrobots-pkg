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
#include "robot_mechanism_controllers/cartesian_pose_controller.h"


using namespace KDL;
using namespace tf;
using namespace std;


namespace controller {



CartesianPoseController::CartesianPoseController()
: node_(ros::Node::instance()),
  robot_state_(NULL),
  jnt_to_pose_solver_(NULL)
{}

CartesianPoseController::~CartesianPoseController()
{
  if (jnt_to_pose_solver_) delete jnt_to_pose_solver_;
}


bool CartesianPoseController::initialize(mechanism::RobotState *robot_state, const string& root_name, const string& tip_name, const string& controller_name)
{
  cout << "initializing " << controller_name << " between " << root_name << " and " << tip_name << endl;
  controller_name_ = controller_name;

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // create robot chain from root to tip
  if (!robot_.init(robot_state_->model_, root_name, tip_name))
    return false;
  robot_.toKDL(chain_);

  // create solver
  num_joints_   = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
  jnt_to_pose_solver_ = new ChainFkSolverPos_recursive(chain_);
  jnt_pos_.resize(num_joints_);

  // get pid controller
  control_toolbox::Pid pid_controller;
  pid_controller.initParam(controller_name_);
  for (unsigned int i=0; i<6; i++)
    pid_controller_.push_back(pid_controller);

  // initialize twist controller
  twist_controller_.initialize(robot_state_, root_name, tip_name, controller_name_+"/twist");

  return true;
}


bool CartesianPoseController::start()
{
  // reset pid controllers
  for (unsigned int i=0; i<6; i++)
    pid_controller_[i].reset();

  // initialize desired pose/twist
  twist_ff_ = Twist::Zero();
  pose_desi_ = getPose();
  last_time_ = robot_state_->hw_->current_time_;

  return twist_controller_.start();
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
  twist_controller_.twist_desi_ = twist_fb + twist_ff_;
  twist_controller_.update();
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








ROS_REGISTER_CONTROLLER(CartesianPoseControllerNode)

CartesianPoseControllerNode::CartesianPoseControllerNode() 
: node_(ros::Node::instance()),
  robot_state_(*node_, true),
  command_notifier_(NULL)
{}


CartesianPoseControllerNode::~CartesianPoseControllerNode()
{
  if (command_notifier_) delete command_notifier_;
}


bool CartesianPoseControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get the controller name
  controller_name_ = config->Attribute("name");

  // get name of root and tip
  string tip_name;
  node_->param(controller_name_+"/root_name", root_name_, string("no_name_given"));
  node_->param(controller_name_+"/tip_name", tip_name, string("no_name_given"));

  // initialize controller  
  if (!controller_.initialize(robot, root_name_, tip_name, controller_name_))
    return false;
  
  // subscribe to pose commands
  command_notifier_ = new MessageNotifier<robot_msgs::PoseStamped>(&robot_state_, node_,  
								 boost::bind(&CartesianPoseControllerNode::command, this, _1),
								 controller_name_ + "/command", root_name_, 1);
  return true;
}

bool CartesianPoseControllerNode::start()
{
  return controller_.start();
}

void CartesianPoseControllerNode::update()
{
  controller_.update();
}


void CartesianPoseControllerNode::command(const tf::MessageNotifier<robot_msgs::PoseStamped>::MessagePtr& pose_msg)
{
  // convert message to transform
  Stamped<Pose> pose_stamped;
  PoseStampedMsgToTF(*pose_msg, pose_stamped);

  // convert to reference frame of root link of the controller chain  
  robot_state_.transformPose(root_name_, pose_stamped, pose_stamped);
  TransformToFrame(pose_stamped, controller_.pose_desi_);
}





void CartesianPoseControllerNode::TransformToFrame(const Transform& trans, Frame& frame)
{
  frame.p(0) = trans.getOrigin().x();
  frame.p(1) = trans.getOrigin().y();
  frame.p(2) = trans.getOrigin().z();

  double Rz, Ry, Rx;
  trans.getBasis().getEulerZYX(Rz, Ry, Rx);
  frame.M = Rotation::EulerZYX(Rz, Ry, Rx);
}



}; // namespace

