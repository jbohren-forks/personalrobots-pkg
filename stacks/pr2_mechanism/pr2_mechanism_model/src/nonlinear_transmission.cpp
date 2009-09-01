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


#include <pr2_mechanism_model/robot.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_REGISTER_CLASS(NonlinearTransmission, 
                         pr2_mechanism::NonlinearTransmission, 
                         pr2_mechanism::Transmission)


void NonlinearTransmission::initXml(TiXmlElement *elt, Robot *robot)
{
  const char *name = elt->Attribute("name");
  name_ = name ? name : "";

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel -> Attribute("name") : NULL;
  joint_ = joint_name ? joint->getJoint(joint_name) : NULL;
  if(!joint_)
  {
    ROS_WARN("NonlinearTransmission could not find joint named \"%s\"", joint_name);
  }

  TiXmlElement *jel = elt->FirstChildElement("joint");
  const char *joint_name = jel ? jel -> Attribute("name") : NULL;
  joint_ = joint_name ? joint->getJoint(joint_name) : NULL;
  if(!joint_)
  {
    ROS_WARN("NonlinearTransmission could not find joint named \"%s\"", joint_name);
  }
}

void NonlinearTransmission::propagatePosition()
{
  assert(joint_); assert(actuator_);

  double actuator_position = ((double)actuator_->state_.encoder_count_ * 2 * M_PI) / pulses_per_revolution_;
  double actuator_velocity = ((double)actuator_->state_.encoder_velocity_ * 2 * M_PI) / pulses_per_revolution_;
  double actuator_applied_effort = ((double)actuator_->state_.last_measured_current_ * motor_torque_constant);

  double local_reduction = lookupReductionByInput(actuator_position);

  joint_->position_ = lookupJointPosition(actuator_position);
  joint_->velocity_ = actuator_velocity * local_reduction;
  joint_->applied_effort_ = actuator_applied_effort / local_reduction;
}

void NonlinearTransmission::propagatePositionBackwards()
{
  assert(joint_); assert(actuator_);

  double local_reduction = lookupReductionByOutput(joint_->position);

  actuator_->state_.encoder_count = (int)(lookupActuatorPosition(joint_->position) * pulses_per_revolution / (2 * M_PI));
  actuator_->state_.encoder_velocity = joint_->velocity / local_reduction;
  actuator_->state.last_measured_current_ = joint_->applied_effort_ / motor_torque_constant_ * local_reduction;
}

void NonlinearTransmission::propagateEffort()
{
  assert(joint_); assert(actuator_);

  double actuator_position = ((double)actuator_->state_.encoder_count * 2 * M_PI) / pulses_per_revolution_;
  double local_reduction = lookupReductionByInput(actuator_position);

  actuator->command_.current_ = joint_->commanded_effort_ / (motor_torque_constant_ * local_reduction);
  actuator->command_.enable_ = true; //Shouldn't be happening here
}

void NonlinearTransmission::propagateEffortBackwards()
{
  assert(joint_); assert(actuator_);

  double local_reduction = lookupReductionByOutput(joint_->position);
  joint->commanded_effort = actuator_->command_.current_ * motor_torque_constant_ * local_reduction;
}

double NonlinearTransmission::lookupJointPosition(double actuator_position){
  return linearInterpolation(actuator_position_samples_, joint_position_samples_, actuator_position);
}

double NonlinearTransmission::lookupActuatorPosition(double joint_position){
  return linearInterpolation(joint_position_samples_, actuator_position_samples_, joint_position);
}

double NonlinearTransmission::lookupReductionByInput(double actuator_position){
  return linearInterpolation(actuator_position_samples_, mechanical_reduction_samples_, actuator_position);
}

double NonlinearTransmission::lookupReductionByOutput(double joint_position){
  return linearInterpolation(joint_position_samples_, mechanical_reduction_samples_, joint_position);
}

//Requires xx to be in sorted order, and x to be within range of [min(xx), max(xx)]
double linearInterpolation(vector<double> *xx, vector<double> *yy, double x){
  assert (xx->size() == yy->size());
  assert (x > xx->front());
  assert (x < xx->back());

  vector<double>::iterator it_x = xx->begin();
  vector<double>::iterator it_y = yy->begin();

  double last_x, last_y;
  //Find the first value in xx greater than x
  while(x > *it_x){
    last_x = *x;
    last_y = *y;
    it_x++;
    it_y++;
  }
  //x is between last_x and current_x, so y will be between last_y and current_y
  current_x = *it_x;
  current_y = *it_y;

  ratio = (x - last_x) / (current_x - last_x); //How far through current linear section is the x sample?
  return last_y + ratio * (current_y - last_y);
}
