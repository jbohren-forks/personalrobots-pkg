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

#include "urdf/parser.h"
#include <algorithm>
#include "robot_mechanism_controllers/endeffector_constraint_controller.h"

#define DEBUG 0 // easy debugging

static const double JOYSTICK_MAX_FORCE  = 20.0;
static const double JOYSTICK_MAX_TORQUE = 0.75;


using namespace KDL;

namespace controller {

ROS_REGISTER_CONTROLLER(EndeffectorConstraintController)

EndeffectorConstraintController::EndeffectorConstraintController()
: jnt_to_jac_solver_(NULL),
  jnt_to_pose_solver_(NULL),
  initialized_(false)
{
  constraint_jac_.setZero();
  constraint_wrench_.setZero();
  constraint_force_.setZero();
  printf("EndeffectorConstraintController constructor\n");
}



EndeffectorConstraintController::~EndeffectorConstraintController()
{
}



bool EndeffectorConstraintController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  robot_ = robot;

  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    ROS_ERROR("EndeffectorConstraintController was not given a chain");
    return false;
  }

  // Gets names for the root and tip of the chain
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    ROS_ERROR("Chain element for EndeffectorConstraintController must specify the root");
    return false;
  }
  if (!tip_name)  {
    ROS_ERROR("Chain element for EndeffectorConstraintController must specify the tip");
    return false;
  }

  if (!chain_.init(robot->model_, root_name, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // some parameters
  ros::Node *node = ros::Node::instance();
  assert(node);
  node->param("constraint/wall_x"       , wall_x      , 0.8) ; /// location of the wall
  node->param("constraint/threshold_x"  , threshold_x , 0.1 ) ; /// distance within the wall to apply constraint force
  node->param("constraint/wall_r"       , wall_r      , 0.2 ) ; /// cylinder radius
  node->param("constraint/elbow_limit"  , elbow_limit , -1.52 ) ; /// elbow limit

  node->param("constraint/threshold_r"  , threshold_r , 0.1) ; /// radius over with constraint is applied
  node->param("constraint/f_x_max"      , f_x_max     , -1000.0) ; /// max x force
  node->param("constraint/f_r_max"      , f_r_max     , -1000.0) ; /// max r force
  node->param("constraint/f_r_max"      , f_pose_max  , 20.0) ; /// max r force

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


void EndeffectorConstraintController::update()
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
  constraint_torq_ = joint_constraint_null_space_*task_jac_.transpose() * constraint_wrench_;
  task_torq_ = joint_constraint_null_space_ * task_jac_.transpose()* constraint_null_space_ * task_wrench_;


  //ROS_ERROR("%s", chain_.getJointName(3));
  //ROS_ERROR("%f %f %f", jnt_pos(2),jnt_pos(3),jnt_pos(4));
  //ROS_ERROR("%.3f %.3f %.3f",constraint_null_space_(0,0), constraint_null_space_(1,1), constraint_null_space_(2,2));
  //ROS_ERROR("%.3f %.3f %.3f",task_jac_(0,5), task_jac_(1,5), task_jac_(2,5));


  JntArray jnt_eff(kdl_chain_.getNrOfJoints());
  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
    jnt_eff(i) = constraint_torq_(i) + task_torq_(i);
  chain_.setEfforts(jnt_eff, robot_->joint_states_);
}


void EndeffectorConstraintController::computeConstraintJacobian()
{
  // Clear force vector
  double f_x = 0;
  double f_r = 0;
  double f_roll = 0;
  double f_pitch = 0;
  double f_yaw = 0;


  // Compute the end effector angle from the origin of the circle
  //double ee_theta = atan2(endeffector_frame_.p(2), endeffector_frame_.p(1));

  // Constarint for a cylinder centered around the x axis
  constraint_jac_(0,0) = 0; // this is the end of the cylinder
  constraint_jac_(1,1) = 0;
  constraint_jac_(2,1) = 0;
  constraint_jac_(3,2) = 1;
  constraint_jac_(4,3) = 1;
  constraint_jac_(5,4) = 1;

  // X-wall at the end of the cylinder
  double x_dist_to_wall = endeffector_frame_.p(0) - wall_x + threshold_x;
  if (x_dist_to_wall > 0)
  {
    constraint_jac_(0,0) = 0;// 1; // this is the end of the cylinder
    f_x = x_dist_to_wall * f_x_max; /// @todo: FIXME, replace with some exponential function
    if((x_dist_to_wall-threshold_x) > 0 && DEBUG)
    {
      ROS_ERROR("wall x breach! by: %f m\n", (x_dist_to_wall-threshold_x));
    }
  }
  else
  {
    f_x = 0;
  }

  /// yz-force magnitude is a function of endeffector distance from unit circle
  double ee_radius = sqrt( endeffector_frame_.p(1)*endeffector_frame_.p(1) + (endeffector_frame_.p(2))*(endeffector_frame_.p(2)) );
  double r_dist_to_wall = ee_radius - wall_r + threshold_r;

  // assign x-direction constraint force f_x if within range of the wall
  if (r_dist_to_wall > 0)
  {
    constraint_jac_(1,1) = 0;//cos(ee_theta);
    constraint_jac_(2,1) = 0;//sin(ee_theta);
    f_r = r_dist_to_wall * f_r_max; /// @todo: FIXME, replace with some exponential function
    if(r_dist_to_wall > threshold_r && DEBUG)
    {
      ROS_ERROR("wall radius breach! by: %f m\n", (r_dist_to_wall-threshold_r));
    }
  }
  else
  {
    f_r = 0;
  }


  Twist pose_error = diff(endeffector_frame_, desired_frame_);

  //roll constraint
  if (fabs(pose_error(3)) > 0)
  {
    //determine sign
    //constraint_jac_(3,2) = 1;
    f_roll = pose_error(3) * f_pose_max; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_roll = 0;
  }

  //pitch constraint
  if (fabs(pose_error(4)) > 0)
  {
    //determine sign
    //constraint_jac_(4,3) = 1;
    f_pitch = pose_error(4) * f_pose_max*10; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_pitch = 0;
  }

  //yaw constraint
  if (fabs(pose_error(5)) > 0)
  {
    //determine sign
    //constraint_jac_(5,4) = 1;
    f_yaw = pose_error(5)* f_pose_max; /// @todo: FIXME, replace with some exponential function
  }
  else
  {
    f_yaw = 0;
  }

  
  //joint constraint force - stop the elbow from going past -30 degrees (.5235 rad)
  JntArray jnt_pos(kdl_chain_.getNrOfJoints());
  chain_.getPositions(robot_->joint_states_, jnt_pos);
  
  double joint_e = elbow_limit-jnt_pos(2);
  if(joint_e < -0.01) 
  {
    joint_constraint_jac_(2)=1;
  }

  if(joint_e < 0)
  {
    joint_constraint_force_(2)=joint_e*f_limit_max;
  }
  else 
  {
    joint_constraint_force_(0)=0;
  }

  constraint_force_(0) = f_x;
  constraint_force_(1) = f_r;
  constraint_force_(2) = f_roll;
  constraint_force_(3) = f_pitch;
  constraint_force_(4) = f_yaw;
}

void EndeffectorConstraintController::computeConstraintNullSpace()
{
  // Compute generalized inverse, this is the transpose as long as the constraints are
  // orthonormal to eachother. Will replace with QR method later.
  constraint_null_space_ = identity_ - constraint_jac_*constraint_jac_.transpose();
}



ROS_REGISTER_CONTROLLER(EndeffectorConstraintControllerNode)


EndeffectorConstraintControllerNode::EndeffectorConstraintControllerNode()
: node_(ros::Node::instance()), loop_count_(0)
{
}


EndeffectorConstraintControllerNode::~EndeffectorConstraintControllerNode()
{
  node_->unsubscribe(topic_ + "/command");
}

bool EndeffectorConstraintControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to from xml file
  topic_ = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic_ == "") {
    fprintf(stderr, "No name given to EndeffectorConstraintControllerNode\n");
    return false;
  }

  // initialize controller
  if (!controller_.initXml(robot, config))
    return false;

  assert(node_);
  // subscribe to wrench commands
  node_->subscribe(topic_ + "/command", wrench_msg_,
                  &EndeffectorConstraintControllerNode::command, this, 1);
  guard_command_.set(topic_ + "/command");
  node_->advertise<robot_msgs::VisualizationMarker>( "visualizationMarker", 0 );


  return true;
}


void EndeffectorConstraintControllerNode::update()
{
  controller_.update();

  ++loop_count_;
  if (loop_count_ % 100 == 0)
  {
#if 0

    // Debugging code.  Not currently realtime safe

    robot_msgs::VisualizationMarker marker;
    marker.header.frame_id = "torso_lift_link";
    marker.id = 0;
    marker.type = robot_msgs::VisualizationMarker::CUBE;
    marker.action = 0;
    marker.x = 0.6;
    marker.y = 0;
    marker.z = 0;
    marker.yaw = 0;
    marker.pitch = 0;
    marker.roll = 0.0;
    marker.xScale = 0.01;
    marker.yScale = 10;
    marker.zScale = 10;
    marker.alpha = 200;
    marker.r = 0;
    marker.g = 255;
    marker.b = 0;
    node_->publish("visualizationMarker", marker );

    robot_msgs::VisualizationMarker marker;
    marker.header.frame_id = "torso_lift_link";
    marker.id = 1;
    marker.type = robot_msgs::VisualizationMarker::SPHERE;
    marker.action = 0;
    marker.x = 0.6;
    marker.y = 0;
    marker.z = 1;
    marker.yaw = 0;
    marker.pitch = 0;
    marker.roll = 0.0;
    marker.xScale = 0.01;
    marker.yScale = 0.4;
    marker.zScale = 0.4;
    marker.alpha = 200;
    marker.r = 255;
    marker.g = 0;
    marker.b = 0;
    node_->publish("visualizationMarker", marker );
#endif
  }

}


void EndeffectorConstraintControllerNode::command()
{

  // convert to wrench command
  controller_.task_wrench_(0) = wrench_msg_.force.x;
  controller_.task_wrench_(1) = wrench_msg_.force.y;
  controller_.task_wrench_(2) = wrench_msg_.force.z;
  controller_.task_wrench_(3) = wrench_msg_.torque.x;
  controller_.task_wrench_(4) = wrench_msg_.torque.y;
  controller_.task_wrench_(5) = wrench_msg_.torque.z;

  //  ROS_WARN("force magnitude (%f, %f, %f)\n",controller_.wrench_desi_.force(0),controller_.wrench_desi_.force(1),controller_.wrench_desi_.force(1));
}

}; // namespace
