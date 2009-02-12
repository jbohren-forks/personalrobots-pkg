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
 * Author: Melonee Wise
 */
#include "angles/angles.h"
#include "urdf/parser.h"
#include <algorithm>
#include "robot_mechanism_controllers/plug_controller.h"

#define DEBUG 0 // easy debugging

static const double JOYSTICK_MAX_FORCE  = 20.0;
static const double JOYSTICK_MAX_TORQUE = 0.75;


using namespace KDL;

namespace controller {

ROS_REGISTER_CONTROLLER(PlugController)

PlugController::PlugController()
: jnt_to_jac_solver_(NULL),
  jnt_to_pose_solver_(NULL),
  initialized_(false),
  outlet_pt_(1, 0, 0),
  outlet_norm_(1,0,0)
{
  constraint_jac_.setZero();
  constraint_wrench_.setZero();
  constraint_force_.setZero();
  printf("PlugController constructor\n");
}



PlugController::~PlugController()
{
}



bool PlugController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    ROS_ERROR("PlugController was not given a chain");
    return false;
  }

  // Gets names for the root and tip of the chain
  root_name_ = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name_) {
    ROS_ERROR("Chain element for PlugController must specify the root");
    return false;
  }
  if (!tip_name)  {
    ROS_ERROR("Chain element for PlugController must specify the tip");
    return false;
  }

  if (!chain_.init(robot->model_, root_name_, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // some parameters
  ros::Node *node = ros::Node::instance();
  assert(node);

  node->param("constraint/upper_arm_limit" , upper_arm_limit , -1.52 ) ; /// upper arm pose limit

  node->param("constraint/f_r_max"      , f_r_max     , 1000.0) ; /// max radial force of line constraint
  node->param("constraint/f_pose_max"   , f_pose_max  , 50.0) ; /// max pose force
  node->param("constraint/f_limit_max"  , f_limit_max  , 20.0) ; /// max upper arm limit force

  // Constructs solvers and allocates matrices.
  unsigned int num_joints   = kdl_chain_.getNrOfJoints();
  jnt_to_jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
  jnt_to_pose_solver_.reset(new ChainFkSolverPos_recursive(kdl_chain_));
  task_jac_ = Eigen::MatrixXf::Zero(6, num_joints);
  identity_ = Eigen::MatrixXf::Identity(6, 6);
  identity_joint_ = Eigen::MatrixXf::Identity(num_joints, num_joints);
  constraint_null_space_ = Eigen::MatrixXf::Zero(6, 6);
  joint_constraint_force_= Eigen::MatrixXf::Zero(num_joints, 1);
  joint_constraint_jac_= Eigen::MatrixXf::Zero(num_joints, 1);
  joint_constraint_null_space_ = Eigen::MatrixXf::Zero(num_joints, num_joints);
  constraint_torq_ = Eigen::MatrixXf::Zero(num_joints, 1);
  task_torq_ = Eigen::MatrixXf::Zero(num_joints, 1);
  task_wrench_ = Eigen::Matrix<float,6,1>::Zero();

  // Sets desired wrench to 0
  for (unsigned int i=0; i<6; i++){
    task_wrench_(i) = 0;
  }

  return true;
}


void PlugController::update()
{
  if (!chain_.allCalibrated(robot_->joint_states_))
    return;

  // Gets the joint positions
  JntArray jnt_pos(kdl_chain_.getNrOfJoints());
  chain_.getPositions(robot_->joint_states_, jnt_pos);

  // Grabs the inital pose of the robot for testing...
  if(!initialized_)
  {
    jnt_to_pose_solver_->JntToCart(jnt_pos, desired_frame_);
    initialized_ = true;
  }

  Jacobian jacobian(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfSegments());
  jnt_to_jac_solver_->JntToJac(jnt_pos, jacobian);

  // TODO: Write a function for doing this conversion
  //convert to eigen for easier math
  for (unsigned int i = 0; i < 6; i++)
  {
    for (unsigned int j = 0; j < kdl_chain_.getNrOfJoints(); j++)
    {
      task_jac_(i,j) = jacobian(i,j);
    }
  }

  // get endeffector pose
  jnt_to_pose_solver_->JntToCart(jnt_pos, endeffector_frame_);

  // compute the constraint jacobian and the constraint force
  computeConstraintJacobian();

  // compute the constraint wrench to apply
  constraint_wrench_ = constraint_jac_ * constraint_force_;

  // compute the constraint null space to project
  computeConstraintNullSpace();

  // convert the wrench into joint torques
  joint_constraint_torq_ = joint_constraint_force_;
  constraint_torq_ = joint_constraint_null_space_ * task_jac_.transpose() * constraint_wrench_;
  task_torq_ = joint_constraint_null_space_ * task_jac_.transpose() * constraint_null_space_ * task_wrench_;



  JntArray jnt_eff(kdl_chain_.getNrOfJoints());
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
    jnt_eff(i) = joint_constraint_torq_(i) + constraint_torq_(i) + task_torq_(i);
  chain_.setEfforts(jnt_eff, robot_->joint_states_);
}


void PlugController::computeConstraintJacobian()
{
  // Clear force vector
  double f_r = 0;
  double f_roll = 0;
  double f_pitch = 0;
  double f_yaw = 0;

  // this will be computed based on the tool position in space
  constraint_jac_(0,0) = 0; // line constraint
  constraint_jac_(1,0) = 0; // line constraint
  constraint_jac_(2,0) = 0; // line constraint
  // the pose constraint is always on
  constraint_jac_(3,1) = 1; // roll
  constraint_jac_(4,2) = 1; // pitch
  constraint_jac_(5,3) = 1; // yaw

  // put the end_effector point into eigen
  Eigen::Vector3f end_effector_pt(endeffector_frame_.p(0), endeffector_frame_.p(1), endeffector_frame_.p(2));

  // Vector from the outlet to the end effector position
  Eigen::Vector3f v = end_effector_pt - outlet_pt_;

  // Vector from the end effector position to (the closest point on ) the line constraint.
  Eigen::Vector3f r_to_line = v.dot(outlet_norm_) * outlet_norm_ - v;

  double dist_to_line = r_to_line.norm();
  r_to_line = r_to_line.normalized();

  // update the jacobian for the line constraint
  if (dist_to_line > 0)
  {
    constraint_jac_(0,0) = r_to_line(0);
    constraint_jac_(1,0) = r_to_line(1);
    constraint_jac_(2,0) = r_to_line(2);
    f_r = dist_to_line * f_r_max; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_r = 0;
  }

  // compute the pose error using a twist
  Twist pose_error = diff(endeffector_frame_, desired_frame_);

  //roll constraint
  if (fabs(pose_error(3)) > 0)
  {
    f_roll = pose_error(3) * f_pose_max; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_roll = 0;
  }

  //pitch constraint
  if (fabs(pose_error(4)) > 0)
  {
    f_pitch = pose_error(4) * f_pose_max; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_pitch = 0;
  }

  //yaw constraint
  if (fabs(pose_error(5)) > 0)
  {
    f_yaw = pose_error(5) * f_pose_max; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_yaw = 0;
  }


  //joint constraint force - stop the upper arm from going past -90 degrees (1.57 rad)
  JntArray jnt_pos(kdl_chain_.getNrOfJoints());
  chain_.getPositions(robot_->joint_states_, jnt_pos);

  double joint_e = angles::shortest_angular_distance(jnt_pos(2), upper_arm_limit);
  if(joint_e < -0.01)
  {
    joint_constraint_jac_(2) = 1;
  }

  if(joint_e < 0)
  {
    joint_constraint_force_(2) = joint_e * f_limit_max;
  }
  else
  {
    joint_constraint_force_(2) = 0;
  }

  constraint_force_(0) = f_r;
  constraint_force_(1) = f_roll;
  constraint_force_(2) = f_pitch;
  constraint_force_(3) = f_yaw;
}

void PlugController::computeConstraintNullSpace()
{
  // Compute generalized inverse, this is the transpose as long as the constraints are
  // orthonormal to eachother. Will replace with QR method later.
  constraint_null_space_ = identity_ - constraint_jac_ * constraint_jac_.transpose();
  joint_constraint_null_space_ =  identity_joint_ - joint_constraint_jac_ * joint_constraint_jac_.transpose();

}



ROS_REGISTER_CONTROLLER(PlugControllerNode)


PlugControllerNode::PlugControllerNode()
: node_(ros::Node::instance()), loop_count_(0), TF(*ros::Node::instance(),false, 10000000000ULL)
{
}


PlugControllerNode::~PlugControllerNode()
{
  node_->unsubscribe(topic_ + "/command");
}

bool PlugControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to from xml file
  topic_ = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic_ == "") {
    fprintf(stderr, "No name given to PlugControllerNode\n");
    return false;
  }

  // initialize controller
  if (!controller_.initXml(robot, config))
    return false;

  assert(node_);
  // subscribe to wrench commands
  node_->subscribe(topic_ + "/command", wrench_msg_,
                  &PlugControllerNode::command, this, 1);
  guard_command_.set(topic_ + "/command");
  // subscribe to outlet pose commands
  node_->subscribe(topic_ + "/outlet_pose", outlet_pose_msg_,
                  &PlugControllerNode::outletPose, this, 1);
  guard_outlet_pose_.set(topic_ + "/outlet_pose");
  return true;
}


void PlugControllerNode::update()
{
  controller_.update();

}
void PlugControllerNode::outletPose()
{

  tf::Stamped<tf::Point> point;
  point.setX(outlet_pose_msg_.point.x);
  point.setY(outlet_pose_msg_.point.y);
  point.setZ(outlet_pose_msg_.point.z);
  point.stamp_ = ros::Time();
  point.frame_id_ = outlet_pose_msg_.header.frame_id;

  tf::Stamped<tf::Point> outlet_pt;

  try
  {
    TF.transformPoint(controller_.root_name_, point, outlet_pt);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }

  controller_.outlet_pt_(0) = outlet_pt.x();
  controller_.outlet_pt_(1) = outlet_pt.y();
  controller_.outlet_pt_(2) = outlet_pt.z();


  tf::Stamped<tf::Vector3> vector;
  vector.setX(outlet_pose_msg_.vector.x);
  vector.setY(outlet_pose_msg_.vector.y);
  vector.setZ(outlet_pose_msg_.vector.z);
  vector.stamp_ = ros::Time();
  vector.frame_id_ = outlet_pose_msg_.header.frame_id;

  tf::Stamped<tf::Vector3> outlet_norm;

  try
  {
    TF.transformVector(controller_.root_name_, vector, outlet_norm);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }

  controller_.outlet_norm_(0) = outlet_norm.x();
  controller_.outlet_norm_(1) = outlet_norm.y();
  controller_.outlet_norm_(2) = outlet_norm.z();

}

void PlugControllerNode::command()
{
  // convert to wrench command
  controller_.task_wrench_(0) = wrench_msg_.force.x;
  controller_.task_wrench_(1) = wrench_msg_.force.y;
  controller_.task_wrench_(2) = wrench_msg_.force.z;
  controller_.task_wrench_(3) = wrench_msg_.torque.x;
  controller_.task_wrench_(4) = wrench_msg_.torque.y;
  controller_.task_wrench_(5) = wrench_msg_.torque.z;

}

}; // namespace
