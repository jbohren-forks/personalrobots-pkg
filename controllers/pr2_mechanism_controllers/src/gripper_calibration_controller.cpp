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

#include "pr2_mechanism_controllers/gripper_calibration_controller.h"
#include <ros/time.h>

using namespace std;
using namespace controller;

GripperCalibrationController::GripperCalibrationController()
: state_(INITIALIZED), joint_(NULL)
{
}

GripperCalibrationController::~GripperCalibrationController()
{
}

bool GripperCalibrationController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  assert(robot);
  assert(config);

  TiXmlElement *cal = config->FirstChildElement("calibrate");
  if (!cal)
  {
    std::cerr<<"GripperCalibrationController was not given calibration parameters"<<std::endl;
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
    fprintf(stderr, "Error: GripperCalibrationController could not find joint \"%s\"\n",
            joint_name);
    return false;
  }

  const char *actuator_name = cal->Attribute("actuator");
  actuator_ = actuator_name ? robot->model_->getActuator(actuator_name) : NULL;
  if (!actuator_)
  {
    fprintf(stderr, "Error: GripperCalibrationController could not find actuator \"%s\"\n",
            actuator_name);
    return false;
  }

  control_toolbox::Pid pid;
  TiXmlElement *pid_el = config->FirstChildElement("pid");
  if (!pid_el)
  {
    fprintf(stderr, "Error: GripperCalibrationController was not given a pid element.\n");
    return false;
  }
  if (!pid.initXml(pid_el))
    return false;

  if (!vc_.init(robot, joint_name, pid))
    return false;

  return true;
}

void GripperCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  switch (state_)
  {
  case INITIALIZED:
    state_ = BEGINNING;
    return;
  case BEGINNING:
    count_ = 0;
    joint_->calibrated_ = false;
    actuator_->state_.zero_offset_ = 0.0;
    vc_.setCommand(search_velocity_);
    state_ = STARTING;
    break;
  case STARTING:
    // Makes sure we start moving for a bit before checking if we've stopped.
    if (++count_ > 500)
    {
      count_ = 0;
      state_ = CLOSING;
    }
    break;
  case CLOSING:
    if (fabs(joint_->velocity_) < 0.001)
    {
      actuator_->state_.zero_offset_ = actuator_->state_.position_;
      state_ = CALIBRATED;
      joint_->calibrated_ = true;
      vc_.setCommand(0);
    }
    break;
  case CALIBRATED:
    break;
  }

  if (state_ != CALIBRATED)
    vc_.update();
}


ROS_REGISTER_CONTROLLER(GripperCalibrationControllerNode)

GripperCalibrationControllerNode::GripperCalibrationControllerNode()
: robot_(NULL)
{
}

GripperCalibrationControllerNode::~GripperCalibrationControllerNode()
{
}

void GripperCalibrationControllerNode::update()
{
  c_.update();

  if (c_.calibrated())
  {
    if (last_publish_time_ + 0.5 < robot_->hw_->current_time_)
    {
      assert(pub_calibrated_);
      if (pub_calibrated_->trylock())
      {
        last_publish_time_ = robot_->hw_->current_time_;
        pub_calibrated_->unlockAndPublish();
      }
    }
  }
}

bool GripperCalibrationControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  robot_ = robot;
  ros::node *node = ros::node::instance();

  std::string topic = config->Attribute("name") ? config->Attribute("name") : "";
  if (topic == "")
  {
    fprintf(stderr, "No name given to GripperCalibrationController\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  pub_calibrated_ = new misc_utils::RealtimePublisher<std_msgs::Empty>(topic + "/calibrated", 1);

  return true;
}
