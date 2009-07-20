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


#include <pr2_mechanism_controllers/head_position_controller.h>
#include <mechanism_control/mechanism_control.h>
#include<cmath>

using namespace KDL;
using namespace tf;
using namespace std;

namespace controller {

ROS_REGISTER_CONTROLLER(HeadPositionController)


HeadPositionController::HeadPositionController()
  : robot_state_(NULL),
    point_head_notifier_(NULL),
    point_frame_on_head_notifier_(NULL)
{}

HeadPositionController::~HeadPositionController()
{
  sub_command_.shutdown();

}

bool HeadPositionController::initXml(mechanism::RobotState *robot_state, TiXmlElement *config)
{
  // get the controller name from xml file
  std::string controller_name = config->Attribute("name") ? config->Attribute("name") : "";
  if (controller_name == ""){
    ROS_ERROR("HeadPositionController: No controller name given in xml file");
    return false;
  }

  return init(robot_state, ros::NodeHandle(controller_name));
}

bool HeadPositionController::init(mechanism::RobotState *robot_state, const ros::NodeHandle &n)
{
  node_ = n;

// get name link names from the param server
  if (!node_.getParam("pan_link_name", pan_link_name_)){
    ROS_ERROR("HeadPositionController: No pan link name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tilt_link_name", tilt_link_name_)){
    ROS_ERROR("HeadPositionController: No tilt link name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // get a pointer to the wrench controller
  MechanismControl* mc;
  if (!MechanismControl::Instance(mc)){
    ROS_ERROR("HeadPositionController: could not get instance of mechanism control");
    return false;
  }
  std::string pan_output;
  if (!node_.getParam("pan_output", pan_output))
  {
    ROS_ERROR("HeadPositionController: No ouptut name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!mc->getControllerByName<JointPositionController>(pan_output, head_pan_controller_))
  {
    ROS_ERROR("HeadPositionController: could not connect to the pan joint controller %s (namespace: %s)",
              pan_output.c_str(), node_.getNamespace().c_str());
    return false;
  }

  std::string tilt_output;
  if (!node_.getParam("tilt_output", tilt_output))
  {
    ROS_ERROR("HeadPositionController: No ouptut name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!mc->getControllerByName<JointPositionController>(tilt_output, head_tilt_controller_))
  {
    ROS_ERROR("HeadPositionController: could not connect to the tilt joint controller %s (namespace: %s)",
              tilt_output.c_str(), node_.getNamespace().c_str());
    return false;
  }

  // subscribe to head commands
  sub_command_ = node_.subscribe<mechanism_msgs::JointStates>("command", 1, &HeadPositionController::command, this);

  point_head_notifier_.reset(new MessageNotifier<robot_msgs::PointStamped>(tf_,
                                                                       boost::bind(&HeadPositionController::pointHead, this, _1),
                                                                       node_.getNamespace() + "/point_head", pan_link_name_, 10));
  point_frame_on_head_notifier_.reset(new MessageNotifier<robot_msgs::PointStamped>(tf_,
                                                                       boost::bind(&HeadPositionController::pointFrameOnHead, this, _1),
                                                                       node_.getNamespace() + "/point_frame_on_head", pan_link_name_, 10));

  return true;
}

bool HeadPositionController::starting()
{
  pan_out_ = head_pan_controller_->joint_state_->position_;
  tilt_out_ = head_tilt_controller_->joint_state_->position_;

  return true;
}

void HeadPositionController::update()
{
  // set position controller commands
  head_pan_controller_->command_ = pan_out_;
  head_tilt_controller_->command_ = tilt_out_;
}

void HeadPositionController::command(const mechanism_msgs::JointStatesConstPtr& command_msg)
{
  ROS_INFO("recieved command_msg");
  assert(command_msg->joints.size() == 2); 
  if(command_msg->joints[0].name == head_pan_controller_->joint_state_->joint_->name_)
  {
    pan_out_ = command_msg->joints[0].position;
    tilt_out_ = command_msg->joints[1].position;
  }
  else 
  {
    pan_out_ = command_msg->joints[1].position;
    tilt_out_ = command_msg->joints[0].position;
  }
  
}

void HeadPositionController::pointHead(const tf::MessageNotifier<robot_msgs::PointStamped>::MessagePtr& point_msg)
{
  std::string pan_parent;
  tf_.getParent( pan_link_name_, ros::Time(), pan_parent);

  // convert message to transform
  Stamped<Point> pan_point;
  pointStampedMsgToTF(*point_msg, pan_point);

  // convert to reference frame of pan link 
  tf_.transformPoint(pan_parent, pan_point, pan_point);

  pan_out_ = atan2(pan_point.y(), pan_point.x());

  Stamped<Point> tilt_point;
  pointStampedMsgToTF(*point_msg, tilt_point);
  tf_.transformPoint(pan_link_name_, tilt_point, tilt_point);

  tilt_out_ = atan2(-tilt_point.z(), tilt_point.x());

}

void HeadPositionController::pointFrameOnHead(const tf::MessageNotifier<robot_msgs::PointStamped>::MessagePtr& point_msg)
{

  Stamped<tf::Transform> frame;
  try
  {
    tf_.lookupTransform(point_msg->header.frame_id, pan_link_name_, ros::Time(), frame);
  }
  catch(TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what());
    return;
  }

  // convert message to transform
  Stamped<Point> pan_point;
  pointStampedMsgToTF(*point_msg, pan_point);
  tf_.transformPoint(pan_link_name_, pan_point, pan_point);

  double radius = pow(pan_point.x(),2)+pow(pan_point.y(),2);
  double x_intercept = sqrt(fabs(radius-pow(frame.getOrigin().y(),2)));
  double delta_theta = atan2(pan_point.y(), pan_point.x())-atan2(frame.getOrigin().y(),x_intercept);
  pan_out_ = head_pan_controller_->joint_state_->position_ + delta_theta;

  // convert message to transform
  Stamped<Point> tilt_point;
  pointStampedMsgToTF(*point_msg, tilt_point);
  tf_.transformPoint(tilt_link_name_, tilt_point, tilt_point);
  
  radius = pow(tilt_point.x(),2)+pow(tilt_point.y(),2);
  x_intercept = sqrt(fabs(radius-pow(frame.getOrigin().z(),2)));
  delta_theta = atan2(-tilt_point.z(), tilt_point.x())-atan2(frame.getOrigin().z(),x_intercept);
  tilt_out_ = head_tilt_controller_->joint_state_->position_ + delta_theta;

}



}//namespace

