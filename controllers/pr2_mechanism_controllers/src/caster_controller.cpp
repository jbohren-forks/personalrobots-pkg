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
 * Author: Stuart Glaser
 */

#include "pr2_mechanism_controllers/caster_controller.h"

namespace controller {

const double CasterController::WHEEL_RADIUS = 0.079;
const double CasterController::WHEEL_OFFSET = 0.049;

CasterController::CasterController()
  : steer_velocity_(0), drive_velocity_(0)
{
}

CasterController::~CasterController()
{
}

bool CasterController::init(
  mechanism::RobotState *robot,
  const std::string &caster_joint,
  const std::string &wheel_l_joint, const std::string &wheel_r_joint,
  const control_toolbox::Pid &caster_pid, const control_toolbox::Pid &wheel_pid)
{
  caster_ = robot->getJointState(caster_joint);
  if (!caster_)
  {
    fprintf(stderr, "Error: Caster joint \"%s\" does not exist\n", caster_joint.c_str());
    return false;
  }

  if (!caster_vel_.init(robot, caster_joint, caster_pid))
    return false;
  if (!wheel_l_vel_.init(robot, wheel_l_joint, wheel_pid))
    return false;
  if (!wheel_r_vel_.init(robot, wheel_r_joint, wheel_pid))
    return false;

  return true;
}

bool CasterController::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  TiXmlElement *jel = config->FirstChildElement("joints");
  if (!jel)
  {
    fprintf(stderr, "Error: CasterController could not find joints element\n");
    return false;
  }

  const char *caster_joint = jel->Attribute("caster");
  const char *wheel_l_joint = jel->Attribute("wheel_l");
  const char *wheel_r_joint = jel->Attribute("wheel_r");
  if (!caster_joint)
  {
    fprintf(stderr, "Error: CasterController was not given a caster joint\n");
    return false;
  }
  if (!wheel_l_joint)
  {
    fprintf(stderr, "Error: CasterController was not given a wheel_l joint\n");
    return false;
  }
  if (!wheel_r_joint)
  {
    fprintf(stderr, "Error: CasterController was not given a wheel_r joint\n");
    return false;
  }

  control_toolbox::Pid caster_pid, wheel_pid;
  TiXmlElement *cpel = config->FirstChildElement("caster_pid");
  TiXmlElement *wpel = config->FirstChildElement("wheel_pid");
  if (!cpel)
  {
    fprintf(stderr, "Error: CasterController was not given a caster_pid element\n");
    return false;
  }
  if (!caster_pid.initXml(cpel))
    return false;
  if (!wpel)
  {
    fprintf(stderr, "Error: CasterController was not given a wheel_pid element\n");
    return false;
  }
  if (!wheel_pid.initXml(wpel))
    return false;

  return init(robot, caster_joint, wheel_l_joint, wheel_r_joint, caster_pid, wheel_pid);
}

void CasterController::update()
{
  caster_vel_.setCommand(steer_velocity_);

  double wd = drive_velocity_ / WHEEL_RADIUS;  // Angular velocity due to driving
  double ws = (WHEEL_RADIUS / WHEEL_OFFSET) * steer_velocity_;  // Angular velocity due to steering
  wheel_r_vel_.setCommand(wd + ws);
  wheel_l_vel_.setCommand(wd - ws);

  caster_vel_.update();
  wheel_l_vel_.update();
  wheel_r_vel_.update();
}



ROS_REGISTER_CONTROLLER(CasterControllerNode);

CasterControllerNode::CasterControllerNode()
{
}

CasterControllerNode::~CasterControllerNode()
{
}

bool CasterControllerNode::initXml(mechanism::RobotState *robot, TiXmlElement *config)
{
  ros::node *node = ros::node::instance();
  assert(node);

  std::string name = config->Attribute("name") ? config->Attribute("name") : "";
  if (name == "")
  {
    fprintf(stderr, "Error: No name given for CasterControllerNode\n");
    return false;
  }

  if (!c_.initXml(robot, config))
    return false;

  node->subscribe(name + "/steer_velocity", steer_velocity_msg_,
                  &CasterControllerNode::setSteerVelocity, this, 2);
  guard_steer_velocity_.set(name + "/steer_velocity");
  node->subscribe(name + "/drive_velocity", drive_velocity_msg_,
                  &CasterControllerNode::setDriveVelocity, this, 2);
  guard_drive_velocity_.set(name + "/drive_velocity");

  return true;
}

void CasterControllerNode::update()
{
  if (!c_.caster_->calibrated_)
    return;
  c_.update();
}

}
