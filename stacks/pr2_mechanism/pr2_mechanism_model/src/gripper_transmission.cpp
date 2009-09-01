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

#include "pr2_mechanism_model/gripper_transmission.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <numeric>

namespace pr2_mechanism {

PLUGINLIB_REGISTER_CLASS(GripperTransmission, 
                         pr2_mechanism::GripperTransmission, 
                         pr2_mechanism::Transmission)

bool GripperTransmission::initXml(TiXmlElement *config, Robot *robot)
{
  const char *name = config->Attribute("name");
  name_ = name ? name : "";

  const char *pA = config->Attribute("A");
  if( pA != NULL )
      A_ = atof(pA);
  const char *pB = config->Attribute("B");
  if( pB != NULL )
      B_ = atof(pB);

  TiXmlElement *ael = config->FirstChildElement("actuator");
  const char *actuator_name = ael ? ael->Attribute("name") : NULL;
  if (!actuator_name || !robot->getActuator(actuator_name))
  {
    ROS_ERROR("GripperTransmission could not find actuator named \"%s\"", actuator_name);
    return false;
  }
  robot->getActuator(actuator_name)->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);

  for (TiXmlElement *j = config->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint"))
  {
    const char *joint_name = j->Attribute("name");
    if (!joint_name || !robot->getJoint(joint_name))
    {
      ROS_ERROR("GripperTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }
    joint_names_.push_back(joint_name);

    const char *joint_pred = j->Attribute("preduction");
    if (!joint_pred)
    {
      ROS_ERROR("GripperTransmission's joint \"%s\" was not given a reduction.", joint_name);
      return false;
    }
    preductions_.push_back(atof(joint_pred));

    const char *joint_ered = j->Attribute("ereduction");
    if (!joint_ered)
    {
      ROS_ERROR("GripperTransmission's joint \"%s\" was not given a reduction.", joint_name);
      return false;
    }
    ereductions_.push_back(atof(joint_ered));
  }

  return true;
}

void GripperTransmission::propagatePosition(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == preductions_.size());

  // sin(pos*reduction) = A*motor+B
  // reduction * cos(pos*reduction) * dpos/dt = A * dmotor/dt
  double sang = as[0]->state_.position_*A_+B_;
  double ang;
  if( sang <= -1 ) 
      ang = -M_PI/2;
  else if( sang >= 1 )
      ang = M_PI/2;
  else
      ang = asin(sang);

  for (unsigned int i = 0; i < js.size(); ++i)
  {
    js[i]->position_ = ang / preductions_[i];
    js[i]->velocity_ = A_*as[0]->state_.velocity_ / (cos(ang)*preductions_[i]);
    js[i]->applied_effort_ = as[0]->state_.last_measured_effort_ * ereductions_[i];
  }
}

void GripperTransmission::propagatePositionBackwards(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == preductions_.size());

  double mean_position = 0.0;
  double mean_velocity = 0.0;
  double mean_effort = 0.0;
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    mean_position += (sin(js[i]->position_ * preductions_[i])-B_)/A_;
    mean_velocity += js[i]->velocity_ * preductions_[i] * cos(js[i]->position_*preductions_[i]) / A_;
    mean_effort += js[i]->applied_effort_ / (ereductions_[i]);
  }
  
  as[0]->state_.position_ = mean_position / js.size();
  as[0]->state_.velocity_ = mean_velocity / js.size();
  as[0]->state_.last_measured_effort_ = mean_effort / js.size();
}

void GripperTransmission::propagateEffort(
  std::vector<JointState*>& js, std::vector<Actuator*>& as)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == ereductions_.size());

  double strongest = 0.0;
  for (unsigned int i = 0; i < js.size(); ++i)
  {
    if (fabs(js[i]->commanded_effort_ / (ereductions_[i])) > fabs(strongest))
      strongest = js[i]->commanded_effort_ / ereductions_[i];
  }
  
  as[0]->command_.effort_ = strongest;
}

void GripperTransmission::propagateEffortBackwards(
  std::vector<Actuator*>& as, std::vector<JointState*>& js)
{
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == ereductions_.size());
  ROS_ASSERT(js.size() == preductions_.size());

  std::vector<double> scaled_positions(js.size());
  for (unsigned int i = 0; i < js.size(); ++i)
    scaled_positions[i] = (sin(js[i]->position_ * preductions_[i])-B_)/A_;

  //double mean = std::accumulate(scaled_positions.begin(), scaled_positions.end(), 0.0)
  //  / scaled_positions.size();

  for (unsigned int i = 0; i < js.size(); ++i)
  {
    //double err = scaled_positions[i] - mean;

    js[i]->commanded_effort_ =
        /*pid_effort / ereductions_[i] + */as[0]->command_.effort_ * ereductions_[i];
  }
}

} // namespace pr2_mechanism
