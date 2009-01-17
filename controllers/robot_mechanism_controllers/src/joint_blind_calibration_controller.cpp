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

#include <robot_mechanism_controllers/joint_blind_calibration_controller.h>
#include <ros/time.h>

using namespace std;
using namespace controller;

ROS_REGISTER_CONTROLLER(JointBlindCalibrationController)

JointBlindCalibrationController::JointBlindCalibrationController()
: state_(INITIALIZED), joint_(NULL)
{
}

JointBlindCalibrationController::~JointBlindCalibrationController()
{
}

bool JointBlindCalibrationController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  assert(config);

  TiXmlElement *cal = config->FirstChildElement("calibrate");
  if (!cal)
  {
    std::cerr<<"JointBlindCalibrationController was not given calibration parameters"<<std::endl;
    return false;
  }

  if(cal->QueryDoubleAttribute("velocity", &search_velocity_) != TIXML_SUCCESS)
  {
    std::cerr<<"Velocity value was not specified\n";
    return false;
  }

  const char *joint_name = cal->Attribute("joint");
  joint_ = joint_name ? robot->getJointState(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "Error: JointBlindCalibrationController could not find joint \"%s\"\n",
            joint_name);
    return false;
  }

  const char *actuator_name = cal->Attribute("actuator");
  actuator_ = actuator_name ? robot->model_->getActuator(actuator_name) : NULL;
  if (!actuator_)
  {
    fprintf(stderr, "Error: JointBlindCalibrationController could not find actuator \"%s\"\n",
            actuator_name);
    return false;
  }

  const char *transmission_name = cal->Attribute("transmission");
  transmission_ = transmission_name ? robot->model_->getTransmission(transmission_name) : NULL;
  if (!transmission_)
  {
    fprintf(stderr, "Error: JointBlindCalibrationController could not find transmission \"%s\"\n",
            transmission_name);
    return false;
  }

  control_toolbox::Pid pid;
  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: JointBlindCalibrationController was not given a pid element.\n");
    return false;
  }
  if (!pid.initXml(pid_el))
    return false;

  if (!vc_.init(robot, joint_name, pid))
    return false;

  return true;
}

void JointBlindCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  static int count = 0;
  static double joint_max_raw, joint_min_raw;  // Joint angles at the bumps

  switch (state_)
  {
  case INITIALIZED:
    return;
  case BEGINNING:
    state_ = STARTING_UP;
    count = 0;
    joint_->calibrated_ = false;
    actuator_->state_.zero_offset_ = 0.0;
    break;
  case STARTING_UP:
    vc_.setCommand(search_velocity_);
    if (++count > 500)
    {
      count = 0;
      state_ = MOVING_UP;
    }
    break;
  case MOVING_UP:
    if (fabs(joint_->velocity_) < 0.001)
    {
      joint_max_raw = joint_->position_;
      state_ = STARTING_DOWN;
      count = 0;
    }
    break;
  case STARTING_DOWN:
    vc_.setCommand(-search_velocity_);
    if (++count > 500)
    {
      state_ = MOVING_DOWN;
      count = 0;
    }
    break;
  case MOVING_DOWN:
    if (fabs(joint_->velocity_) < 0.001)
    {
      joint_min_raw = joint_->position_;

      // Sets the desired joint zero based on where we bumped the limits.
      double joint_zero = ((joint_max_raw - joint_->joint_->joint_limit_max_) +
                           (joint_min_raw - joint_->joint_->joint_limit_min_)) / 2.0;
      std::vector<Actuator*> fake_a;
      std::vector<mechanism::JointState*> fake_j;
      fake_a.push_back(new Actuator);
      fake_j.push_back(new mechanism::JointState);
      fake_j[0]->position_ = joint_zero;
      transmission_->propagatePositionBackwards(fake_j, fake_a);
      actuator_->state_.zero_offset_ = fake_a[0]->state_.position_;

      state_ = BACKING_OFF;
      count = 0;
    }
    break;
  case BACKING_OFF:
    vc_.setCommand(search_velocity_);
    if (++count > 1000)
    {
      joint_->calibrated_ = true;
      state_ = DONE;
    }
    break;
  case DONE:
    vc_.setCommand(0.0);
    break;
  }

  vc_.update();
}


ROS_REGISTER_CONTROLLER(JointBlindCalibrationControllerNode)

JointBlindCalibrationControllerNode::JointBlindCalibrationControllerNode()
{
  c_ = new JointBlindCalibrationController();
}

JointBlindCalibrationControllerNode::~JointBlindCalibrationControllerNode()
{
  delete c_;
}

void JointBlindCalibrationControllerNode::update()
{
  c_->update();
}


  bool JointBlindCalibrationControllerNode::calibrateCommand(robot_mechanism_controllers::CalibrateJoint::request &req, robot_mechanism_controllers::CalibrateJoint::response &resp)
{
  c_->beginCalibration();
  ros::Duration d = ros::Duration(0,1000000);
  while(!c_->calibrated())
    d.sleep();
  resp.offset = 0.0;
  return true;
}

bool JointBlindCalibrationControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::Node *node = ros::Node::instance();

  std::string topic = config->Attribute("topic") ? config->Attribute("topic") : "";
  if (topic == "")
  {
    fprintf(stderr, "No topic given to JointBlindCalibrationController\n");
    return false;
  }

  if (!c_->initXml(robot, config))
    return false;

  node->advertiseService(topic + "/calibrate", &JointBlindCalibrationControllerNode::calibrateCommand, this);
  guard_calibrate_.set(topic + "/calibrate");
  return true;
}
