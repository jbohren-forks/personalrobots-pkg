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

PlugController::PlugController() :
  outlet_pt_(1, 0, 0),
  outlet_norm_(1,0,0),
  jnt_to_jac_solver_(NULL),
  jnt_to_pose_solver_(NULL),
  initialized_(false)
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
  
  controller_name_ = config->Attribute("name");
  
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    ROS_ERROR("PlugController was not given a chain");
    return false;
  }

  // Gets names for the root and tip of the chain
  const char *tip_name = chain->Attribute("tip");
  if (!chain->Attribute("root")) {
    ROS_ERROR("Chain element for PlugController must specify the root");
    return false;
  }
  if (!tip_name)  {
    ROS_ERROR("Chain element for PlugController must specify the tip");
    return false;
  }

  root_name_ = chain->Attribute("root");
  if (!chain_.init(robot->model_, root_name_, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // some parametersjoint_constraint_null_space_ =  identity_joint_ - joint_constraint_jac_ * joint_constraint_jac_.transpose();
  ros::Node *node = ros::Node::instance();
  assert(node);

  node->param(controller_name_+"/upper_arm_limit" , upper_arm_limit , -1.52 ) ; /// upper arm pose limit

  node->param(controller_name_+"/f_r_max"      , f_r_max     , 150.0) ; /// max radial force of line constraint
  node->param(controller_name_+"/f_pose_max"   , f_pose_max  , 40.0) ; /// max pose force
  node->param(controller_name_+"/f_limit_max"  , f_limit_max  , 100.0) ; /// max upper arm limit force
  node->param(controller_name_+"/upper_arm_dead_zone", upper_arm_dead_zone, 0.05);

  roll_pid_.initParam(controller_name_+"/pose_pid"); 
  pitch_pid_.initParam(controller_name_+"/pose_pid"); 
  yaw_pid_.initParam(controller_name_+"/pose_pid"); 
  line_pid_.initParam(controller_name_+"/line_pid"); 
  last_time_ = robot->model_->hw_->current_time_;
  
  
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
    for (unsigned int i = 0; i < 3; ++i)
      outlet_pt_[i] = desired_frame_.p[i];
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
  double time = robot_->model_->hw_->current_time_;
  // Clear force vector
  f_r_ = 0;
  f_roll_ = 0;
  f_pitch_ = 0;
  f_yaw_ = 0;

  // this will be computed based on the tool position in space
  constraint_jac_(0,0) = 0; // line constraint
  constraint_jac_(1,0) = 0; // line constraint
  constraint_jac_(2,0) = 0; // line constraint
  constraint_jac_(0,1) = 0; // line constraint
  constraint_jac_(1,1) = 0; // line constraint
  constraint_jac_(2,1) = 0; // line constraint
  // the pose constraint is always on
  constraint_jac_(3,2) = 1; // roll
  constraint_jac_(4,3) = 1; // pitch
  constraint_jac_(5,4) = 1; // yaw

  joint_constraint_jac_(2) = 0;

  // put the end_effector point into eigen
  Eigen::Vector3f end_effector_pt(endeffector_frame_.p(0), endeffector_frame_.p(1), endeffector_frame_.p(2));

  // Vector from the outlet to the end effector position
  Eigen::Vector3f v = end_effector_pt - outlet_pt_;

  // Vector from the end effector position to (the closest point on ) the line constraint.
  Eigen::Vector3f r_to_line = v.dot(outlet_norm_) * outlet_norm_ - v;
  Eigen::Vector3f other_norm = r_to_line.cross(outlet_norm_);
  other_norm = other_norm.normalized();
  dist_to_line_ = r_to_line.norm();
  r_to_line = r_to_line.normalized();

  // update the jacobian for the line constraint
  if (dist_to_line_ > 0)
  {
    constraint_jac_(0,0) = r_to_line(0);
    constraint_jac_(1,0) = r_to_line(1);
    constraint_jac_(2,0) = r_to_line(2);
    constraint_jac_(0,1) = other_norm(0);
    constraint_jac_(1,1) = other_norm(1);
    constraint_jac_(2,1) = other_norm(2);
    f_r_ = line_pid_.updatePid(-dist_to_line_, time-last_time_);
    if (fabs(f_r_) > f_r_max)
      f_r_ = f_r_max * f_r_ / fabs(f_r_);
  }
  else
  {
    f_r_ = 0;
  }

  // compute the pose error using a twist
  pose_error_ = diff(endeffector_frame_, desired_frame_);

  //roll constraint
  if (fabs(pose_error_(3)) > 0)
  {
    f_roll_ = roll_pid_.updatePid(-pose_error_(3), time-last_time_);
    if (fabs(f_roll_) > f_pose_max)
      f_roll_ = f_pose_max * f_roll_ / fabs(f_roll_);
  }
  else
  {
    f_roll_ = 0;
  }

  //pitch constraint
  if (fabs(pose_error_(4)) > 0)
  {
    f_pitch_ = pitch_pid_.updatePid(-pose_error_(4), time-last_time_);
    if (fabs(f_pitch_) > f_pose_max)
      f_pitch_ = f_pose_max * f_pitch_ / fabs(f_pitch_);
  }
  else
  {
    f_pitch_ = 0;
  }

  //yaw constraint
  if (fabs(pose_error_(5)) > 0)
  {
    f_yaw_ = yaw_pid_.updatePid(-pose_error_(5), time-last_time_);
    if (fabs(f_yaw_) > f_pose_max)
      f_yaw_ = f_pose_max * f_yaw_ / fabs(f_yaw_);
  }
  else
  {
    f_yaw_ = 0;
  }


  //joint constraint force - stop the upper arm from going past -90 degrees (1.57 rad)
  JntArray jnt_pos(kdl_chain_.getNrOfJoints());
  chain_.getPositions(robot_->joint_states_, jnt_pos);

  double joint_e = angles::shortest_angular_distance(jnt_pos(2), upper_arm_limit);
  if(joint_e < upper_arm_dead_zone)
  {
    joint_constraint_jac_(2) = 0;//1;
  }

  if(joint_e < 0)
  {
    joint_constraint_force_(2) =0;// joint_e * f_limit_max;
  }
  else
  {
    joint_constraint_force_(2) = 0;
  }

  constraint_force_(0) = f_r_;
  constraint_force_(1) = 0;
  constraint_force_(2) = f_roll_;
  constraint_force_(3) = f_pitch_;
  constraint_force_(4) = f_yaw_;
  last_time_ = time;
}

void PlugController::computeConstraintNullSpace()
{
  // Compute generalized inverse, this is the transpose as long as the constraints are
  // orthonormal to eachother. Will replace with QR method later.
  constraint_null_space_ = identity_ - constraint_jac_ * constraint_jac_.transpose();
  joint_constraint_null_space_ =  identity_joint_ - joint_constraint_jac_ * joint_constraint_jac_.transpose();

}

void PlugController::setToolOffset(const tf::Transform &tool_offset)
{
  KDL::Chain new_kdl_chain;
  chain_.toKDL(new_kdl_chain);

  KDL::Frame tool_frame;
  mechanism::TransformTFToKDL(tool_offset, tool_frame);
  new_kdl_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), tool_frame));

  kdl_chain_ = new_kdl_chain;

  jnt_to_jac_solver_.reset(new ChainJntToJacSolver(kdl_chain_));
  jnt_to_pose_solver_.reset(new ChainFkSolverPos_recursive(kdl_chain_));
}



ROS_REGISTER_CONTROLLER(PlugControllerNode)


PlugControllerNode::PlugControllerNode()
: node_(ros::Node::instance()), loop_count_(0), TF(*ros::Node::instance(),false,ros::Duration(10.0))
{
  current_frame_publisher_=NULL;
  internal_state_publisher_=NULL;
}


PlugControllerNode::~PlugControllerNode()
{
  current_frame_publisher_->stop();
  delete current_frame_publisher_;
  internal_state_publisher_->stop();
  delete internal_state_publisher_;
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

  if (current_frame_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete current_frame_publisher_ ;
  current_frame_publisher_ = new realtime_tools::RealtimePublisher <robot_msgs::Transform> (topic_ + "/transform", 1) ;
  
  if (internal_state_publisher_ != NULL)// Make sure that we don't memory leak if initXml gets called twice
    delete internal_state_publisher_ ;
  internal_state_publisher_ = new realtime_tools::RealtimePublisher <robot_mechanism_controllers::PlugInternalState> (topic_ + "/internal_state", 1) ;

  node_->advertiseService(topic_ + "/set_tool_frame", &PlugControllerNode::setToolFrame, this);
  guard_set_tool_frame_.set(topic_ + "/set_tool_frame");

  return true;
}


void PlugControllerNode::update()
{
  controller_.update();

  static int count =0;
  if (count % 100 == 0)
  {
    if (current_frame_publisher_->trylock())
    {
      tf::Transform transform;
      mechanism::TransformKDLToTF(controller_.endeffector_frame_, transform);
      tf::TransformTFToMsg(transform, current_frame_publisher_->msg_);
      current_frame_publisher_->unlockAndPublish() ;
     }
    if (internal_state_publisher_->trylock())
    {
      internal_state_publisher_->msg_.line_error = controller_.dist_to_line_;
      internal_state_publisher_->msg_.line_force_cmd = controller_.f_r_;
      internal_state_publisher_->msg_.roll_error = controller_.pose_error_(3);
      internal_state_publisher_->msg_.roll_force_cmd = controller_.f_roll_;
      internal_state_publisher_->msg_.pitch_error = controller_.pose_error_(4);
      internal_state_publisher_->msg_.pitch_force_cmd = controller_.f_pitch_;
      internal_state_publisher_->msg_.yaw_error = controller_.pose_error_(5);
      internal_state_publisher_->msg_.yaw_force_cmd = controller_.f_yaw_;
      internal_state_publisher_->unlockAndPublish() ;
     }
  }

}

void PlugControllerNode::outletPose()
{
  robot_msgs::PoseStamped outlet_in_root_;
  try
  {
    TF.transformPose(controller_.root_name_, outlet_pose_msg_, outlet_in_root_);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }
  tf::Pose outlet;
  tf::PoseMsgToTF(outlet_in_root_.pose, outlet);

  controller_.outlet_pt_(0) = outlet.getOrigin().x();
  controller_.outlet_pt_(1) = outlet.getOrigin().y();
  controller_.outlet_pt_(2) = outlet.getOrigin().z();

  tf::Vector3 norm = quatRotate(outlet.getRotation(), tf::Vector3(1.0, 0.0, 0.0));
  controller_.outlet_norm_(0) = norm.x();
  controller_.outlet_norm_(1) = norm.y();
  controller_.outlet_norm_(2) = norm.z();

  mechanism::TransformTFToKDL(outlet, controller_.desired_frame_);
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

bool PlugControllerNode::setToolFrame(robot_srvs::SetPoseStamped::Request &req,
                                      robot_srvs::SetPoseStamped::Response &resp)
{
  robot_msgs::PoseStamped tool_offset_msg;
  try
  {
    TF.transformPose(controller_.chain_.getLinkName(), req.p, tool_offset_msg);
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return true;
  }

  tf::Transform tool_offset;
  tf::PoseMsgToTF(tool_offset_msg.pose, tool_offset);
  controller_.setToolOffset(tool_offset);
  return true;
}

}; // namespace
