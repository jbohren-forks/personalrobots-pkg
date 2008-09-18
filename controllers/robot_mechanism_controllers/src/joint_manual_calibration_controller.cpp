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

ROS_REGISTER_CONTROLLER(JointManualCalibrationController)

JointManualCalibrationController::JointManualCalibrationController()
  : joint_(NULL),
    actuator_(NULL),
    robot_(NULL),
    transmission_(NULL),
    joint_state_(NULL),
    state_(Idle)
{
  std::cout<<"JointManualCalibration created\n";
}

JointManualCalibrationController::~JointManualCalibrationController()
{
  std::cout<<"JointManualCalibration destroyed\n";
}

bool JointManualCalibrationController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  assert(config);
  robot_ = robot->model_;

  TiXmlElement *j = config->FirstChildElement("param");
  if (!j)
  {
    std::cerr<<"JointManualCalibrationController was not given parameters"<<std::endl;
    return false;
  }

  const char *joint_name = j->Attribute("joint_name");
  joint_ = joint_name ? robot_->getJoint(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "JointManualCalibrationController could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  const int i = robot_->getJointIndex(joint_name);
  if(i<0)
  {
    std::cout<<"Could not find joint state\n";
    return false;
  }
  joint_state_ = &(robot->joint_states_[i]);

  const char *act_name = j->Attribute("actuator_name");
  actuator_ = act_name ? robot_->getActuator(act_name) : NULL;
  if(!actuator_)
  {
    std::cout<<"JointManualCalibrationController could not find an actuator called "<<act_name<<std::endl;
    return false;
  }

  const char *trans_name = j->Attribute("transmission_name");
  transmission_ = trans_name ? robot_->getTransmission(trans_name) : NULL;
  if(!transmission_)
  {
    std::cout<<"JointManualCalibrationController could not find a transmission called "<<trans_name<<std::endl;
    return false;
  }

  return true;
}

void JointManualCalibrationController::beginCalibration()
{
  // Upward exploration
  if(state_mutex_.trylock())
  {
    state_ = Begin;
    std::cout<<"Starting calibration sequence "<<std::endl;
  }
  else
    std::cerr<<"JointManualCalibrationController"<<joint_->name_<<" : You tried to find the offset while it is already looking for it.\n";
}

void JointManualCalibrationController::endCalibration()
{
  if(state_ != Search)
  {
    std::cerr<<"You tried to stop the calibration procedure while it is not searching\n";
  }
  state_ = Stop;
}

void JointManualCalibrationController::update()
{
  assert(joint_state_);
  assert(actuator_);

  if(state_ == Begin)
  {
    joint_state_->calibrated_ = false;
    min_ = actuator_->state_.position_;
    max_ = actuator_->state_.position_;
    state_ = Search;
  }

  if(state_ == Search)
  {
    min_= std::min(min_, actuator_->state_.position_);
    max_= std::max(max_, actuator_->state_.position_);
  }

  if(state_ == Stop)
  {
    // Compute the offset:
    std::cout<<">>"<<min_<<" "<<max_<<std::endl;
    const double offset_min = offset(min_, joint_->joint_limit_min_);
    const double offset_max = offset(max_, joint_->joint_limit_max_);
    const double offset_avg = 0.5*(offset_min+offset_max);
    std::cout<<"Offset found: "<<offset_min<<'\t'<<offset_max<<'\t'<<offset_avg<<std::endl;
    actuator_->state_.zero_offset_=offset_avg;
    state_ = Initialized;

  }

  if(state_ == Initialized)
  {
    joint_state_->calibrated_ = true;
    state_mutex_.unlock();
  }
}

bool JointManualCalibrationController::getOffset(double & value)
{
  value = actuator_->state_.zero_offset_;
  return state_ == Initialized;
}

double JointManualCalibrationController::offset(double act_pos, double joint_ref_pos)
{
  std::cout<<"act_pos = "<<act_pos<<'\n';
  std::cout<<"ref_pos = "<<joint_ref_pos<<'\n';
  assert(transmission_);

  Actuator act;
  mechanism::JointState jState;
  std::vector<Actuator *> acts;
  std::vector<mechanism::JointState*> jStates;
  acts.push_back(&act);
  jStates.push_back(&jState);

  jState.position_ = joint_ref_pos;

  transmission_->propagatePositionBackwards(jStates, acts);
  std::cout<<"ref_pos (enc) = "<<act.state_.position_<<'\n';
  std::cout<<"computed offset = "<<act_pos - act.state_.position_<<'\n';
  return act_pos - act.state_.position_;
}


ROS_REGISTER_CONTROLLER(JointManualCalibrationControllerNode)
    JointManualCalibrationControllerNode::JointManualCalibrationControllerNode()
{
  c_ = new JointManualCalibrationController();
}

JointManualCalibrationControllerNode::~JointManualCalibrationControllerNode()
{
  delete c_;
}

void JointManualCalibrationControllerNode::update()
{
  c_->update();
}

  bool JointManualCalibrationControllerNode::beginCalibrationCommand(robot_mechanism_controllers::CalibrateJoint::request &req, robot_mechanism_controllers::CalibrateJoint::response &resp)
{
  c_->beginCalibration();
  return true;
}

  bool JointManualCalibrationControllerNode::endCalibrationCommand(robot_mechanism_controllers::CalibrateJoint::request &req, robot_mechanism_controllers::CalibrateJoint::response &resp)
{
  c_->endCalibration();
  ros::Duration d=ros::Duration(0,1000000);
  while(!c_->calibrated())
    d.sleep();
  c_->getOffset(resp.offset);
  return true;
}

bool JointManualCalibrationControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointManualCalibrationController\n");
    return false;
  }

  if (!c_->initXml(robot, config))
    return false;
  node->advertise_service(topic + "/begin_manual_calibration", &JointManualCalibrationControllerNode::beginCalibrationCommand, this);
  node->advertise_service(topic + "/end_manual_calibration", &JointManualCalibrationControllerNode::endCalibrationCommand, this);
  return true;
}
