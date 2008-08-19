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
#include <math.h>
#include <ros/node.h>
#include <mechanism_model/robot.h>

using namespace mechanism;

ROS_REGISTER_TRANSMISSION(SimpleTransmission)

SimpleTransmission::SimpleTransmission(Joint *joint, Actuator *actuator,
  double mechanical_reduction, double motor_torque_constant,
  double pulses_per_revolution)
{
  actuator_ = actuator;
  mechanical_reduction_ = mechanical_reduction;
  motor_torque_constant_ = motor_torque_constant;
  pulses_per_revolution_ = pulses_per_revolution;
  joint_ = joint;
}

bool SimpleTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel->Attribute("name") : NULL;
  joint_ = joint_name ? robot->getJoint(joint_name) : NULL;
  if (!joint_)
  {
    fprintf(stderr, "SimpleTransmission could not find joint named \"%s\"\n", joint_name);
    return false;
  }

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  actuator_ = actuator_name ? robot->getActuator(actuator_name) : NULL;
  if (!actuator_)
  {
    fprintf(stderr, "SimpleTransmission could not find actuator named \"%s\"\n", actuator_name);
    return false;
  }

  mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText()),
  motor_torque_constant_ = atof(elt->FirstChildElement("motorTorqueConstant")->GetText()),
  pulses_per_revolution_ = atof(elt->FirstChildElement("pulsesPerRevolution")->GetText());
  return true;
}

void SimpleTransmission::initTransmission(std::string transmission_name,std::string joint_name,std::string actuator_name,double mechanical_reduction,double motor_torque_constant,double pulses_per_revolution, Robot *robot)
{
  name_     =    transmission_name;
  joint_name_ = joint_name;
  actuator_name_ = actuator_name;
  joint_    = robot->getJoint(   joint_name);
  actuator_ = robot->getActuator(actuator_name);
  mechanical_reduction_  = mechanical_reduction ;
  motor_torque_constant_ = motor_torque_constant;
  pulses_per_revolution_ = pulses_per_revolution;
}

void SimpleTransmission::propagatePosition()
{
  assert(joint_);  assert(actuator_);
  joint_->position_ = ((double)actuator_->state_.encoder_count_*2*M_PI)/(pulses_per_revolution_ * mechanical_reduction_);
  joint_->velocity_ = ((double)actuator_->state_.encoder_velocity_*2*M_PI)/(pulses_per_revolution_ * mechanical_reduction_);
  joint_->applied_effort_ = actuator_->state_.last_measured_current_ * (motor_torque_constant_ * mechanical_reduction_);

}

void SimpleTransmission::propagatePositionBackwards()
{
  assert(joint_);  assert(actuator_);
  actuator_->state_.encoder_count_ = (int)(joint_->position_ * pulses_per_revolution_ * mechanical_reduction_ / (2*M_PI));
  actuator_->state_.encoder_velocity_ = joint_->velocity_ * pulses_per_revolution_ * mechanical_reduction_ / (2*M_PI);
  actuator_->state_.last_measured_current_ = joint_->applied_effort_ / (motor_torque_constant_ * mechanical_reduction_);
}

void SimpleTransmission::propagateEffort()
{
  assert(joint_);  assert(actuator_);
  actuator_->command_.current_ = joint_->commanded_effort_/(motor_torque_constant_ * mechanical_reduction_);
  actuator_->command_.enable_ = true;

}

void SimpleTransmission::propagateEffortBackwards()
{
  assert(joint_);  assert(actuator_);
  joint_->commanded_effort_ = actuator_->command_.current_ * motor_torque_constant_ * mechanical_reduction_;
}
