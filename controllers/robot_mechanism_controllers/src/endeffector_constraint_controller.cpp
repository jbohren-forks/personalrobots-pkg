
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
  constraint_force_.setZero();
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
  ros::Node *node = ros::Node::instance();
  robot_kinematics::RobotKinematics robot_kinematics ;
  string robot_desc;
  node->param("robotdesc/pr2", robot_desc, string("")) ;
  robot_kinematics.loadString(robot_desc.c_str()) ;
  robot_kinematics::SerialChain* serial_chain = robot_kinematics.getSerialChain("right_arm");
  if (serial_chain == NULL)
    fprintf(stderr, "Got NULL Chain\n") ;

  // some parameters
  node->param("constraint/wall_x"       , wall_x      , 0.6) ; /// location of the wall
  node->param("constraint/threshold_x"  , threshold_x , 0.2 ) ; /// distance within the wall to apply constraint force
  node->param("constraint/wall_r"       , wall_r      , 0.2 ) ; /// cylinder radius
  node->param("constraint/threshold_r"  , threshold_r , 0.05) ; /// radius over with constraint is applied
  node->param("constraint/f_x_max"      , f_x_max     , -20.0) ; /// max x force
  node->param("constraint/f_y_max"      , f_r_max     , -20.0) ; /// max r force


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

  constraint_wrench_ = constraint_jac_ * constraint_force_;

  // convert the wrench into joint torques
  JntArray constraint_torq(num_joints_);
  for (unsigned int i=0; i<num_joints_; i++)
  {
    constraint_torq(i) = 0;
    for (unsigned int j=0; j<6; j++)
    {
      constraint_torq(i) += (jacobian(j,i) * constraint_wrench_(j));
    }
    joints_[i]->commanded_effort_ = constraint_torq(i);
  }
}


void EndeffectorConstraintController::computeConstraintJacobian()
{
  //clear force vector
  double f_x = 0;
  double f_r = 0;

  double ee_theta = atan2((endeffector_frame_.p(2)-1),endeffector_frame_.p(1) );

  //Constarint for a cylinder centered around the x axis
  constraint_jac_(0,0) = 1; // this is the end of the cylinder
  constraint_jac_(1,1) = cos(ee_theta);
  constraint_jac_(2,1) = sin(ee_theta)+1;

  //Constraint Force Vector
  double x_dist_to_wall = endeffector_frame_.p(0) - wall_x + threshold_x;
  //ROS_ERROR("x_dist_to_wall: %f m\n", (x_dist_to_wall));
  if (x_dist_to_wall > 0)
  {
    f_x = (exp(x_dist_to_wall)-1) * f_x_max; /// @todo: FIXME, replace with some exponential function
    //ROS_ERROR("x_dist_to_wall: %f m f_x: %f N\n", x_dist_to_wall, f_x);
    if((x_dist_to_wall-threshold_x) > 0)
    {
      //ROS_ERROR("wall x breach! by: %f m\n", (x_dist_to_wall-threshold_x));
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
    f_r =0;// pow(r_dist_to_wall,3) * f_r_max;
    if(r_dist_to_wall > threshold_r)
    {
      //ROS_ERROR("wall radius breach! by: %f m\n", (r_dist_to_wall-threshold_r));
    }
  }
  else
  {
    f_r = 0;
  }

  constraint_force_(0) = f_x;
  constraint_force_(1) = f_r;

}




ROS_REGISTER_CONTROLLER(EndeffectorConstraintControllerNode)


EndeffectorConstraintControllerNode::EndeffectorConstraintControllerNode()
: node_(ros::Node::instance())
{
}


EndeffectorConstraintControllerNode::~EndeffectorConstraintControllerNode()
{
  node_->unsubscribe(topic_ + "/command");
}

bool EndeffectorConstraintControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  // get name of topic to listen to from xml file
  topic_ = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic_ == "") {
    fprintf(stderr, "No topic given to EndeffectorConstraintControllerNode\n");
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
  node_->advertise<std_msgs::VisualizationMarker>( "visualizationMarker", 0 );


  return true;
}


void EndeffectorConstraintControllerNode::update()
{
  controller_.update();
  static int count=0;
  count++;
  if (count%100==0)
  {

    std_msgs::VisualizationMarker marker;
    marker.header.frame_id = "torso_lift_link";
    marker.id = 0;
    marker.type = 1;
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
  }

  if (count%100==0)
  {

    std_msgs::VisualizationMarker marker;
    marker.header.frame_id = "torso_lift_link";
    marker.id = 1;
    marker.type = 2;
    marker.action = 0;
    marker.x = 0.6;
    marker.y = 0;
    marker.z = 1;
    marker.yaw = 0;
    marker.pitch = 0;
    marker.roll = 0.0;
    marker.xScale = 0.01;
    marker.yScale = 0.2;
    marker.zScale = 0.2;
    marker.alpha = 200;
    marker.r = 255;
    marker.g = 0;
    marker.b = 0;
    node_->publish("visualizationMarker", marker );
  }



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

  ROS_WARN("force magnitude (%f, %f, %f)\n",controller_.wrench_desi_.force(0),controller_.wrench_desi_.force(1),controller_.wrench_desi_.force(1));
}

}; // namespace
