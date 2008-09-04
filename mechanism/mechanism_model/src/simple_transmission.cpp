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
/*
 * Author: Stuart Glaser
 */
#include <math.h>
#include "mechanism_model/robot.h"
#include "mechanism_model/simple_transmission.h"

using namespace mechanism;

ROS_REGISTER_TRANSMISSION(SimpleTransmission)

bool SimpleTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  if (!joint_name || robot->getJointIndex(joint_name) < 0)
  {
    fprintf(stderr, "SimpleTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }
  joint_names_.push_back(joint_name);

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || robot->getActuatorIndex(actuator_name) < 0)
  {
    fprintf(stderr, "SimpleTransmission could not find actuator named \"%s\"\n", actuator_name);
    return false;
  }
  actuator_names_.push_back(actuator_name);

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText()),
  motor_torque_constant_ = atof(elt->FirstChildElement("motorTorqueConstant")->GetText()),
  pulses_per_revolution_ = atof(elt->FirstChildElement("pulsesPerRevolution")->GetText());
  return true;
}

void SimpleTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  js[0]->position_ = ((double)as[0]->state_.encoder_count_*2*M_PI)/(pulses_per_revolution_ * mechanical_reduction_);
  js[0]->velocity_ = ((double)as[0]->state_.encoder_velocity_*2*M_PI)/(pulses_per_revolution_ * mechanical_reduction_);
  js[0]->applied_effort_ = as[0]->state_.last_measured_current_ * (motor_torque_constant_ * mechanical_reduction_);
}

void SimpleTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  as[0]->state_.encoder_count_ = (int)(js[0]->position_ * pulses_per_revolution_ * mechanical_reduction_ / (2*M_PI));
  as[0]->state_.encoder_velocity_ = js[0]->velocity_ * pulses_per_revolution_ * mechanical_reduction_ / (2*M_PI);
  as[0]->state_.last_measured_current_ = js[0]->applied_effort_ / (motor_torque_constant_ * mechanical_reduction_);
}

void SimpleTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  as[0]->command_.current_ = js[0]->commanded_effort_/(motor_torque_constant_ * mechanical_reduction_);
  as[0]->command_.enable_ = true;
}

void SimpleTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  assert(as.size() == 1);
  assert(js.size() == 1);
  js[0]->commanded_effort_ = as[0]->command_.current_ * motor_torque_constant_ * mechanical_reduction_;
}
