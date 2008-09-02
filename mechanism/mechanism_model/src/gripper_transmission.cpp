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

#include "mechanism_model/gripper_transmission.h"
#include <algorithm>
#include <numeric>

namespace mechanism {

ROS_REGISTER_TRANSMISSION(GripperTransmission)

bool GripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  actuator_ = actuator_name ? robot->getActuator(actuator_name) : NULL;
  if (!actuator_)
  {
    fprintf(stderr, "GripperTransmission could not find actuator named \"%s\"\n", actuator_name);
    return false;
  }

  for (TiXmlElement *j = config->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint"))
  {
    const char *joint_name = j->Attribute("name");
    Joint* joint = joint_name ? robot->getJoint(joint_name) : NULL;
    if (!joint)
    {
      fprintf(stderr, "GripperTransmission could not find joint named \"%s\"\n", joint_name);
      return false;
    }
    joints_.push_back(joint);

    const char *joint_red = j->Attribute("reduction");
    if (!joint_red)
    {
      fprintf(stderr, "GripperTransmission's joint \"%s\" was not given a reduction.\n", joint_name);
      return false;
    }
    reductions_.push_back(atof(joint_red));
  }

  pids_.resize(joints_.size());
  TiXmlElement *pel = config->FirstChildElement("pid");
  if (pel)
  {
    Pid pid;
    pid.initXml(pel);
    for (unsigned int i = 0; i < pids_.size(); ++i)
      pids_[i] = pid;
  }

  motor_torque_constant_ = atof(config->FirstChildElement("motorTorqueConstant")->GetText()),
  pulses_per_revolution_ = atof(config->FirstChildElement("pulsesPerRevolution")->GetText());

  return true;
}

void GripperTransmission::propagatePosition()
{
  assert(actuator_);
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    double mr = reductions_[i];
    joints_[i]->position_ = actuator_->state_.encoder_count_ * 2 * M_PI / (mr * pulses_per_revolution_);
    joints_[i]->velocity_ = actuator_->state_.encoder_velocity_ * 2 * M_PI / (mr * pulses_per_revolution_);
    joints_[i]->applied_effort_ = actuator_->state_.last_measured_current_ * mr * motor_torque_constant_;
  }
}

void GripperTransmission::propagatePositionBackwards()
{
  double mean_encoder = 0.0;
  double mean_encoder_v = 0.0;
  double mean_current = 0.0;
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    double mr = reductions_[i];
    mean_encoder += joints_[i]->position_ * mr * pulses_per_revolution_ / (2 * M_PI);
    mean_encoder_v += joints_[i]->velocity_ * mr * pulses_per_revolution_ / (2 * M_PI);
    mean_current += joints_[i]->applied_effort_ / (mr * motor_torque_constant_);
  }
  actuator_->state_.encoder_count_ = mean_encoder / joints_.size();
  actuator_->state_.encoder_velocity_ = mean_encoder_v / joints_.size();
  actuator_->state_.last_measured_current_ = mean_current / joints_.size();
}

void GripperTransmission::propagateEffort()
{
  double strongest = 0.0;
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    if (fabs(joints_[i]->commanded_effort_ / (reductions_[i])) > fabs(strongest))
      strongest = joints_[i]->commanded_effort_ / (reductions_[i] * motor_torque_constant_);
  }
  actuator_->command_.current_ = strongest;
}

void GripperTransmission::propagateEffortBackwards()
{
  std::vector<double> scaled_positions(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    scaled_positions[i] = joints_[i]->position_ * reductions_[i];

  double mean = std::accumulate(scaled_positions.begin(), scaled_positions.end(), 0.0)
    / scaled_positions.size();

  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    double err = scaled_positions[i] - mean;
    double pid_effort = pids_[i].updatePid(err, 0.001);

    joints_[i]->commanded_effort_ =
      pid_effort / reductions_[i] +
      actuator_->command_.current_ * reductions_[i] * motor_torque_constant_;
  }
}

} // namespace mechanism
