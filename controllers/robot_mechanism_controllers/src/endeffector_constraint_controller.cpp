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
 * Author: John Hsu and Melonee Wise
 */

#include "urdf/parser.h"
#include <algorithm>
#include "robot_mechanism_controllers/endeffector_constraint_controller.h"


static const double JOYSTICK_MAX_FORCE  = 20.0;
static const double JOYSTICK_MAX_TORQUE = 0.75;


using namespace KDL;

namespace controller {

ROS_REGISTER_CONTROLLER(EndeffectorConstraintController)

EndeffectorConstraintController::EndeffectorConstraintController()
: jnt_to_jac_solver_(NULL),
  jnt_to_pose_solver_(NULL),
  joints_(0,(mechanism::JointState*)NULL)
{
  constraint_jac_.setZero();
  constraint_wrench_.setZero();
  printf("EndeffectorConstraintController constructor\n");
}



EndeffectorConstraintController::~EndeffectorConstraintController()
{
  if (jnt_to_jac_solver_) delete jnt_to_jac_solver_;
  if (jnt_to_pose_solver_) delete jnt_to_pose_solver_;
}



bool EndeffectorConstraintController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // set disired wrench to 0
  for (unsigned int i=0; i<3; i++){
    wrench_desi_.force(i) = 0;
    wrench_desi_.torque(i) = 0;
  }

  // parse robot description from xml file
  ros::node *node = ros::node::instance();
  robot_kinematics::RobotKinematics robot_kinematics ;
  string robot_desc;
  node->param("robotdesc/pr2", robot_desc, string("")) ;
  robot_kinematics.loadString(robot_desc.c_str()) ;
  robot_kinematics::SerialChain* serial_chain = robot_kinematics.getSerialChain("right_arm");
  if (serial_chain == NULL)  
    fprintf(stderr, "Got NULL Chain\n") ;

  // some parameters
  node->param("constraint/wall_x"       , wall_x      , 0.75) ; /// location of the wall
  node->param("constraint/threshold_x"  , threshold_x , 0.2 ) ; /// distance within the wall to apply constraint force
  node->param("constraint/wall_r"       , wall_r      , 0.2 ) ; /// cylinder radius
  node->param("constraint/threshold_r"  , threshold_r , 0.1 ) ; /// radius over with constraint is applied
  node->param("constraint/f_x_max"      , f_x_max     , 20.0) ; /// max x force
  node->param("constraint/f_y_max"      , f_y_max     , 20.0) ; /// max y force
  node->param("constraint/f_z_max"      , f_z_max     , 20.0) ; /// max z force

  // convert description to KDL chain
  chain_        = serial_chain->chain;
  num_joints_   = chain_.getNrOfJoints();
  num_segments_ = chain_.getNrOfSegments();
  printf("Extracted KDL Chain with %u Joints and %u segments\n", num_joints_, num_segments_ );
  jnt_to_jac_solver_ = new ChainJntToJacSolver(chain_);
  jnt_to_pose_solver_ = new ChainFkSolverPos_recursive(chain_);

  // get chain
  TiXmlElement *chain = config->FirstChildElement("chain");
  if (!chain) {
    fprintf(stderr, "Error: EndeffectorConstraintController was not given a chain\n");
    return false;
  }

  // get names for root and tip of robot
  const char *root_name = chain->Attribute("root");
  const char *tip_name = chain->Attribute("tip");
  if (!root_name) {
    fprintf(stderr, "Error: Chain element for EndeffectorConstraintController must specify the root\n");
    return false;
  }
  if (!tip_name)  {
    fprintf(stderr, "Error: Chain element for EndeffectorConstraintController must specify the tip\n");
    return false;
  }

  // test if we can get root from robot
  assert(robot);
  if (!robot->getLinkState(root_name)) {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorConstraintController)\n", root_name);
    return false;
  }

  // get tip from robot
  mechanism::LinkState *current = robot->getLinkState(tip_name);
  if (!current)  {
    fprintf(stderr, "Error: link \"%s\" does not exist (EndeffectorConstraintController)\n", tip_name);
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
	fprintf(stderr, "Error: for EndeffectorConstraintController, tip is not connected to root\n");
	return false;
      }
    }
  // reverse order of joint vector
  std::reverse(joints_.begin(), joints_.end());

  // get control parameters



  return true;
}




void EndeffectorConstraintController::update()
{
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
  
  // get endeffector pose
  jnt_to_pose_solver_->JntToCart(jnt_pos, endeffector_frame_);
  
  computeConstraintJacobian();
  // multiply constraint jacobian by constraint wrench 
  constraint_wrench_ = constraint_jac_ * constraint_wrench_;

  // convert the wrench into joint torques
  JntArray jnt_torq(num_joints_);
  for (unsigned int i=0; i<num_joints_; i++){
    jnt_torq(i) = 0;
    for (unsigned int j=0; j<6; j++)
    {
      jnt_torq(i) += (jacobian(j,i) * constraint_wrench_(j));
      jnt_torq(i) += (jacobian(j,i) * wrench_desi_(j)); /// todo: project into null space first before adding
    }
    joints_[i]->commanded_effort_ = jnt_torq(i);
  }
}


void EndeffectorConstraintController::computeConstraintJacobian()
{
  // Constraint equations 
  // r^2=y^2+z^2 with r = 1
  // x=1

  ////////////////////////////////////////////
  //
  // Constraint Jacobian
  //
  ////////////////////////////////////////////
  // endeffector theta and radius from yz = (0,0)
  double ee_theta = atan2( endeffector_frame_.p(2),endeffector_frame_.p(1) );

  double df_dx = -1.0; // we are describing a wall at constant x, x- side is allowed
  double df_dy = -cos(ee_theta); // radial lines toward origin
  double df_dz = -sin(ee_theta); // radial lines toward origin

  // Constraint Jacobian (normals to the constraint surface)
  constraint_jac_(0,0)= df_dx;
  constraint_jac_(1,1)= df_dy;
  constraint_jac_(2,2)= df_dz;

  ////////////////////////////////////////////
  //
  // Contraint Wrench 
  //
  ////////////////////////////////////////////
  // x-direction force is a function of endeffector distance from the wall
  double x_distance = wall_x - endeffector_frame_.p(0);
  double f_x;

  // assign x-direction constraint force f_x if within range of the wall
  if (x_distance >0 && x_distance < threshold_x)
  {
    f_x = (threshold_x-x_distance)/threshold_x * f_x_max; /// @todo: FIXME, replace with some exponential function
  }
  else if (x_distance <= 0)
  {
    f_x = f_x_max;
    ROS_ERROR("wall x breach! by: %f m\n",x_distance);
  }
  else
  {
    f_x = 0;
  }

  /// yz-force magnitude is a function of endeffector distance from unit circle
  double ee_radius = sqrt( endeffector_frame_.p(1)*endeffector_frame_.p(1) + endeffector_frame_.p(2)*endeffector_frame_.p(2) );
  double r_distance = wall_r - ee_radius;
  double f_y, f_z;

  // assign x-direction constraint force f_x if within range of the wall
  if (r_distance > 0 && r_distance < threshold_r)
  {
    f_y = (threshold_r-r_distance)/threshold_r * f_y_max; /// @todo: FIXME, replace with some exponential function
    f_z = (threshold_r-r_distance)/threshold_r * f_z_max; /// @todo: FIXME, replace with some exponential function
  }
  else if (r_distance <= 0)
  {
    f_y = f_y_max;
    f_z = f_z_max;
    ROS_ERROR("wall radius breach! by: %f m\n",r_distance);
  }
  else
  {
    f_y = 0;
    f_z = 0;
  }

  constraint_wrench_(0) = f_x;
  constraint_wrench_(1) = f_y;
  constraint_wrench_(2) = f_z;

  ROS_WARN("ee pos: (%f, %f, %f), force magnitude (%f, %f, %f)\n",endeffector_frame_.p(0),endeffector_frame_.p(1),endeffector_frame_.p(2),f_x,f_y,f_z);

}






ROS_REGISTER_CONTROLLER(EndeffectorConstraintControllerNode)

EndeffectorConstraintControllerNode::~EndeffectorConstraintControllerNode()
{
  ros::node *node = ros::node::instance();
  node->unsubscribe(topic_ + "/command");
}

bool EndeffectorConstraintControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to from xml file
  ros::node *node = ros::node::instance();
  topic_ = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic_ == "") {
    fprintf(stderr, "No topic given to EndeffectorConstraintControllerNode\n");
    return false;
  }

  // initialize controller  
  if (!controller_.initXml(robot, config))
    return false;
  
  // subscribe to wrench commands
  node->subscribe(topic_ + "/command", wrench_msg_,
                  &EndeffectorConstraintControllerNode::command, this, 1);
  guard_command_.set(topic_ + "/command");

  node->advertise<std_msgs::VisualizationMarker>( "visualizationMarker", 0 );

  // visualization not working yet
  std_msgs::VisualizationMarker marker;
  marker.header.frame_id = "base_link";
  marker.id = 0;
  marker.type = 2;
  marker.action = 0;
  marker.x = 0.7;
  marker.y = 0;
  marker.z = 0;
  marker.yaw = 0;
  marker.pitch = 0;
  marker.roll = 0.0;
  marker.xScale = 0.01;
  marker.yScale = 0.3;
  marker.zScale = 0.3;
  marker.alpha = 100;
  marker.r = 0;
  marker.g = 255;
  marker.b = 0;
  node->publish("visualizationMarker", marker );


  return true;
}


void EndeffectorConstraintControllerNode::update()
{
  controller_.update();
}


void EndeffectorConstraintControllerNode::command()
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
