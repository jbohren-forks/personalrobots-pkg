/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "pr2_mechanism_controllers/base_position_controller.h"
#include "angles/angles.h"                                              // For angular distance error calculation

using namespace pr2_mechanism_controllers ;
using namespace std ;

BasePositionControlUtil::BasePositionControlUtil()
{

}


BasePositionControlUtil::~BasePositionControlUtil()
{
  
  
}

bool BasePositionControlUtil::initXml(TiXmlElement *config)
{
  TiXmlElement *pid_x_elem = config->FirstChildElement("pid_x") ;
  TiXmlElement *pid_y_elem = config->FirstChildElement("pid_y") ;
  TiXmlElement *pid_w_elem = config->FirstChildElement("pid_w") ;

  if (!pid_x_elem || !pid_y_elem || !pid_w_elem)
  {
    printf("BasePositionController:: Error loading XML\n") ;
    return false ;
  }
  
  bool success_x = pid_x_.initXml(pid_x_elem) ;
  bool success_y = pid_y_.initXml(pid_y_elem) ;
  bool success_w = pid_w_.initXml(pid_w_elem) ;
  
  if (!success_x || !success_y || !success_w)
  {
    printf("BasePositionController:: Error loading pid controllers XML\n") ;
    return false ;    
  }
  
  return true ;
}

tf::Vector3 BasePositionControlUtil::updateControl(const tf::Vector3& commanded_pos, const tf::Vector3& actual_pos, double time_elapsed)
{
  double err_x = actual_pos.x() - commanded_pos.x() ;
  double err_y = actual_pos.y() - commanded_pos.y() ;
  double err_w = angles::shortest_angular_distance(commanded_pos.z(), actual_pos.z()) ;
  
  tf::Vector3 velocity_cmd ;
  
  double odom_cmd_x = pid_x_.updatePid(err_x, time_elapsed) ;            // Translation X in the odometric frame
  double odom_cmd_y = pid_y_.updatePid(err_y, time_elapsed) ;            // Translation Y in the odometric frame

  // Rotate the translation commands so that they're in the base frame (instead of the odom frame)
  velocity_cmd.setX( odom_cmd_x*cos(actual_pos.z()) + odom_cmd_y*sin(actual_pos.z())) ;
  velocity_cmd.setY(-odom_cmd_x*sin(actual_pos.z()) + odom_cmd_y*cos(actual_pos.z())) ;

  velocity_cmd.setZ(pid_w_.updatePid(err_w, time_elapsed)) ;             // Rotation command is same is Odom and Base frames
  
  return velocity_cmd ;
}

ROS_REGISTER_CONTROLLER(BasePositionControllerNode)

BasePositionControllerNode::BasePositionControllerNode() : node_(ros::node::instance()), tf_(*node_)
{
  xyt_target_.setX(0) ;
  xyt_target_.setY(0) ;
  xyt_target_.setZ(0) ;
}

BasePositionControllerNode::~BasePositionControllerNode()
{
  
}

bool BasePositionControllerNode::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  ROS_INFO("BasePositionControllerNode:: initXml()") ;
  
  
  bool success ;
  
  last_time_ = robot_state->hw_->current_time_ ;
  
  string service_prefix = config->Attribute("name") ;
  
  robot_state_ = robot_state ;
  
  // Initialize the BasePositionControlUtil
  TiXmlElement *base_pos_control_util_elem = config->FirstChildElement("base_position_control_util") ;
  if (!base_pos_control_util_elem)
  {
    ROS_ERROR("Cannot start BasePositionControllerNode. Cannot find xml element BasePositionControlUtil") ;
    return false ;
  }
  success = base_position_control_util_.initXml(base_pos_control_util_elem) ;  
  if (!success)
  {
    ROS_ERROR("Cannot start BasePositionControllerNode. Error initializing XML for BasePositionControlUtil") ;
    return false ;
  }

  // Initialize the BaseControllerNode
  TiXmlElement *base_controller_node_elem = config->FirstChildElement("base_controller_node") ;
  if (!base_controller_node_elem)
  {
    ROS_ERROR("Cannot start BasePositionControllerNode. Cannot find xml element base_controller_node") ;
    return false ;
  }

  TiXmlElement *base_controller_node_controllers_elem = base_controller_node_elem->FirstChildElement("controller") ;
  //! \todo Add a check to make sure that this XML snippet is for a BaseController
  if (!base_controller_node_controllers_elem)
  {
    ROS_ERROR("Cannot start BasePositionControllerNode. Cannot find xml element 'controller' within 'base_controller_node'") ;
    return false ;
  }
  
  base_controller_node_.initXml(robot_state, base_controller_node_controllers_elem) ;
  
  // Initialize the BasePositionControllerNode
  TiXmlElement *odom_frame_elem = config->FirstChildElement("odometry_frame") ;
  const char* odom_frame = odom_frame_elem->Attribute("name") ;
  if (odom_frame == NULL)
  {
    ROS_ERROR("Cannot start BasePositionControllerNode. odometry_frame element not defined with attribute name") ;
    return false ;    
  }
  odom_frame_name_ = odom_frame ;
  
  node_->subscribe(service_prefix + "/set_pose_command", pose_cmd_, &BasePositionControllerNode::setPoseCommandCallback, this, 1) ;
  guard_set_pose_command_.set(service_prefix + "/set_pose_command") ;

  node_->subscribe(service_prefix + "/set_pose_odom_frame_command", pose_odom_frame_cmd_, &BasePositionControllerNode::setPoseOdomFrameCommandCallback, this, 1) ;
  guard_set_pose_odom_frame_command_.set(service_prefix + "/set_pose_odom_frame_command") ;
  
  return true ;
}

void BasePositionControllerNode::update()
{
  // Grab the current odometric position
  double x, y, w, vx, vy, vw ;
  base_controller_node_.getOdometry(x, y, w, vx, vy, vw) ;
  tf::Vector3 xyt_current(x, y, w) ;
  
  // Determine Elapsed Time
  double cur_time = robot_state_->hw_->current_time_ ;
  double time_elapsed = cur_time - last_time_ ;
  last_time_ = cur_time ;
  
  // Determine next velocity to command
  tf::Vector3 vel_cmd ;
  vel_cmd = base_position_control_util_.updateControl(xyt_target_, xyt_current, time_elapsed) ;
  
  // Command the velocity
  base_controller_node_.setCommand(vel_cmd.x(), vel_cmd.y(), vel_cmd.z()) ;
  
  base_controller_node_.update() ;
  
  //static unsigned int count = 0 ;
  //count++ ;
  //if (count % 500 == 0)
  //{
  //  ROS_INFO("BasePositionControllerNode::Velocity Command: %f %f %f", vel_cmd.x(), vel_cmd.y(), vel_cmd.z()) ;
  //  ROS_INFO("                            Current Position: %f %f %f", x, y, w) ;
  //  ROS_INFO("                            Command Position: %f %f %f", xyt_target_.x(), xyt_target_.y(), xyt_target_.w()) ;
  //}
}

void BasePositionControllerNode::setPoseCommandCallback()
{
  setPoseCommand(pose_cmd_) ;
}

void BasePositionControllerNode::setPoseOdomFrameCommandCallback()
{
  setPoseOdomFrameCommand(pose_odom_frame_cmd_.x, pose_odom_frame_cmd_.y, pose_odom_frame_cmd_.z) ;
}

//! \todo This method has not yet been tested
void BasePositionControllerNode::setPoseCommand(std_msgs::PoseStamped cmd)
{
  std_msgs::PoseStamped pose_odom ;               // Stores the pose in the odometric frame
  cmd.header.stamp = ros::Time(0.0) ;             // Transform using the latest transform
  tf_.transformPose(odom_frame_name_, cmd, pose_odom) ;

  tf::Quaternion orientation_odom ;               // Orientation in the odometric frame
  tf::QuaternionMsgToTF(pose_odom.pose.orientation, orientation_odom) ;

  // For convenience, use 1-Letter names for quaternion terms
  const double& w = orientation_odom.w() ;
  const double& x = orientation_odom.x() ;
  const double& y = orientation_odom.y() ;
  const double& z = orientation_odom.z() ;

  // Compute the Z Rotation from the quaternion
  // Taken from Section 7.4: http://ai.stanford.edu/~diebel/attitude/attitude.pdf
  //! \todo Use a more stable way to compute the z rotation
  double z_rot ;
  if (w > .99999)                       // Calculation becomes unstable near 0 rotation
    z_rot = 2*z ;
  else
    z_rot = 2*acos(w)*z / sqrt(x*x+y*y+z*z) ;
  
  setPoseOdomFrameCommand(pose_odom.pose.position.x,
                          pose_odom.pose.position.y,
                          0.0) ;
                          //z_rot) ;                    // Ignoring the yaw commands for now
}

void BasePositionControllerNode::setPoseOdomFrameCommand(double x, double y, double w)
{
  ROS_INFO("BasePositionControllerNode:: Odom Frame Position Command: %f %f %f\n", x, y, w) ;
  
  //! \todo Mutex this data type
  xyt_target_.setX(x) ;
  xyt_target_.setY(y) ;
  xyt_target_.setZ(w) ;
}
