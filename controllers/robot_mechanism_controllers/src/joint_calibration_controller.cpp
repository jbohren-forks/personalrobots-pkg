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

#include <robot_mechanism_controllers/joint_calibration_controller.h>
#include <ros/time.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointCalibrationController)

JointCalibrationController::JointCalibrationController()
  : JointManualCalibrationController()
{
}

JointCalibrationController::~JointCalibrationController()
{
}

bool JointCalibrationController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  assert(config);
  robot_ = robot->model_;
  bool base=JointManualCalibrationController::initXml(robot, config);
  if(!base)
    return false;
  TiXmlElement *j = config->FirstChildElement("param");
  if (!j)
  {
    std::cerr<<"JointCalibrationController was not given parameters"<<std::endl;
    return false;
  }

  if(j->QueryDoubleAttribute("velocity", &search_velocity_ ) != TIXML_SUCCESS)
  {
    std::cerr<<"Velocity value was not specified\n";
    return false;
  }

  if(search_velocity_ == 0)
  {
    std::cerr<<"You gave zero velocity\n";
    return false;
  }

  TiXmlElement *v = config->FirstChildElement("controller");
  if(!v)
  {
    std::cerr<<"JointCalibrationController was not given a controller to move the joint."<<std::endl;
    return false;
  }

  return vcontroller_.initXml(robot, v);
}

void JointCalibrationController::update()
{
  assert(joint_state_);
  assert(actuator_);

  const double cur_reading = actuator_->state_.calibration_reading_;
  if(state_ == Begin)
  {
    joint_state_->calibrated_ = false;
    previous_reading_ = cur_reading;
    velocity_cmd_ = cur_reading > 0.5 ? search_velocity_ : -search_velocity_;
    state_ = Search;
  }

  if(std::abs(cur_reading-previous_reading_)>0.5 && state_==Search)
  {
    const double offset_ = offset(actuator_->state_.position_, joint_state_->joint_->reference_position_);
    actuator_->state_.zero_offset_ = offset_;
    state_ = Initialized;
  }

  if(state_ == Initialized)
  {
    joint_state_->calibrated_ = true;
    velocity_cmd_ = 0;
    state_ = Stop;
    state_mutex_.unlock();
  }

  if(state_ == Stop)
  {
    velocity_cmd_ = 0;
  }

  if(state_!=Stop)
  {
    vcontroller_.setCommand(velocity_cmd_);
    vcontroller_.update();
  }
}


ROS_REGISTER_CONTROLLER(JointCalibrationControllerNode)

JointCalibrationControllerNode::JointCalibrationControllerNode()
{
  c_ = new JointCalibrationController();
}

JointCalibrationControllerNode::~JointCalibrationControllerNode()
{
  delete c_;
}

void JointCalibrationControllerNode::update()
{
  c_->update();
}


  bool JointCalibrationControllerNode::calibrateCommand(robot_mechanism_controllers::CalibrateJoint::request &req, robot_mechanism_controllers::CalibrateJoint::response &resp)
{
  c_->beginCalibration();
  ros::Duration d=ros::Duration(0,1000000);
  while(!c_->calibrated())
    d.sleep();
  c_->getOffset(resp.offset);
  return true;
}

bool JointCalibrationControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointCalibrationController\n");
    return false;
  }

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(topic + "/calibrate", &JointCalibrationControllerNode::calibrateCommand, this);
  return true;
}
