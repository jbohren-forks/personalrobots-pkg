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

#include <mechanism_model/joint.h>
#include <map>
#include <string>
#include <vector>
#include <cfloat>
#include "urdf/parser.h"

using namespace std;
using namespace mechanism;

static const pair<string, int> types[] = {
  pair<string, int>("none", JOINT_NONE),
  pair<string, int>("revolute", JOINT_ROTARY),
  pair<string, int>("prismatic", JOINT_PRISMATIC),
  pair<string, int>("fixed", JOINT_FIXED),
  pair<string, int>("planar", JOINT_PLANAR),
};

static map<string, int> g_type_map(types, types + sizeof(types)/sizeof(types[0]));

void Joint::enforceLimits(JointState *s)
{
  double vel_high, vel_low;
  double effort_high, effort_low;

  if (!has_safety_limits_)
    return;

  if(s->calibrated_ && s->joint_->type_ != JOINT_CONTINUOUS)
  {
    // Computes the position bounds based on the safety lengths.
    double pos_high = joint_limit_max_ - safety_length_max_;
    double pos_low = joint_limit_min_ + safety_length_min_;

    // Computes the velocity bounds based on the absolute limit and the
    // proximity to the joint limit.
    vel_high = min(velocity_limit_,
                   -k_position_limit_ * (s->position_ - pos_high));
    vel_low = max(-velocity_limit_,
                  -k_position_limit_ * (s->position_ - pos_low));
  }
  else
  {
    vel_high = velocity_limit_;
    vel_low = -velocity_limit_;
  }

  // Computes the effort bounds based on the velocity bounds.
  if (velocity_limit_ >= 0.0)
  {
    effort_high = min(effort_limit_,
                      -k_velocity_limit_ * (s->velocity_ - vel_high));
    effort_low = max(-effort_limit_,
                     -k_velocity_limit_ * (s->velocity_ - vel_low));
  }
  else
  {
    effort_high = effort_limit_;
    effort_low = -effort_limit_;
  }

  s->commanded_effort_ =
    min( max(s->commanded_effort_, effort_low), effort_high);
}

bool Joint::initXml(TiXmlElement *elt)
{
  const char *name = elt->Attribute("name");
  if (!name)
  {
    fprintf(stderr, "Error: unnamed joint found\n");
    return false;
  }
  name_ = name;

  const char *type = elt->Attribute("type");
  if (!type)
  {
    fprintf(stderr, "Error: Joint \"%s\" has no type.\n", name_.c_str());
    return false;
  }
  type_ = g_type_map[type];
  TiXmlElement *limits;

  if (type_ != JOINT_PLANAR && type_ != JOINT_FIXED)
  {
    limits = elt->FirstChildElement("limit");
    if (!limits)
    {
      fprintf(stderr, "Error: Joint \"%s\" has no limits specified.\n", name_.c_str());
      return false;
    }

    if (limits->QueryDoubleAttribute("effort", &effort_limit_) != TIXML_SUCCESS)
    {
      fprintf(stderr, "Error: no effort limit specified for joint \"%s\"\n", name_.c_str());
      return false;
    }

    if (limits->QueryDoubleAttribute("velocity", &velocity_limit_) != TIXML_SUCCESS)
      velocity_limit_ = -1.0;
    else
    {
      if (limits->QueryDoubleAttribute("k_velocity", &k_velocity_limit_) != TIXML_SUCCESS)
      {
        fprintf(stderr, "No k_velocity for joint %s\n", name_.c_str());
        return false;
      }
    }


    TiXmlElement *calibration = elt->FirstChildElement("calibration");
    if(calibration)
    {
      if(calibration->QueryDoubleAttribute("reference_position", &reference_position_) == TIXML_SUCCESS)
        std::cout<<"Found reference point at "<<reference_position_<<std::endl;
    }


    int min_ret = limits->QueryDoubleAttribute("min", &joint_limit_min_);
    int max_ret = limits->QueryDoubleAttribute("max", &joint_limit_max_);

    if (type_ == JOINT_ROTARY && min_ret == TIXML_NO_ATTRIBUTE && max_ret == TIXML_NO_ATTRIBUTE)
    {
      type_ = JOINT_CONTINUOUS;
    }
    else if (min_ret == TIXML_NO_ATTRIBUTE || max_ret == TIXML_NO_ATTRIBUTE)
    {
      fprintf(stderr, "Error: no min and max limits specified for joint \"%s\"\n", name_.c_str());
      return false;
    }

    if (type_ == JOINT_ROTARY || type_ == JOINT_PRISMATIC)
    {
      if (limits->QueryDoubleAttribute("k_position", &k_position_limit_) != TIXML_SUCCESS)
        fprintf(stderr, "No k_position for joint %s\n", name_.c_str());
      if (limits->QueryDoubleAttribute("safety_length_min", &safety_length_min_) != TIXML_SUCCESS)
        fprintf(stderr, "No safety_length_min_ for joint %s\n", name_.c_str());
      if (limits->QueryDoubleAttribute("safety_length_max", &safety_length_max_) != TIXML_SUCCESS)
        fprintf(stderr, "No safety_lenght_max_ for joint %s\n", name_.c_str());

      has_safety_limits_ = true;
    }
  }


  // Parses out the joint properties, this is done for all joints by default
  TiXmlElement *prop_el = elt->FirstChildElement("joint_properties");
  if (!prop_el)
  {
    fprintf(stderr, "Warning: Joint \"%s\" did not specify any joint properties, default to 0.\n", name_.c_str());
    joint_damping_coefficient_ = 0.0;
    joint_friction_coefficient_ = 0.0;
  }
  else
  {
    if (prop_el->QueryDoubleAttribute("damping", &joint_damping_coefficient_) != TIXML_SUCCESS)
      fprintf(stderr,"damping is not specified\n");
    if (prop_el->QueryDoubleAttribute("friction", &joint_friction_coefficient_) != TIXML_SUCCESS)
      fprintf(stderr,"friction is not specified\n");
  }

  if (type_ == JOINT_ROTARY || type_ == JOINT_CONTINUOUS || type_ == JOINT_PRISMATIC)
  {
    // Parses out the joint axis
    TiXmlElement *axis_el = elt->FirstChildElement("axis");
    if (!axis_el)
    {
      fprintf(stderr, "Error: Joint \"%s\" did not specify an axis\n", name_.c_str());
      return false;
    }
    std::vector<double> axis_pieces;
    urdf::queryVectorAttribute(axis_el, "xyz", &axis_pieces);
    if (axis_pieces.size() != 3)
    {
      fprintf(stderr, "Error: The axis for joint \"%s\" must have 3 value\n", name_.c_str());
      return false;
    }
    axis_[0] = axis_pieces[0];
    axis_[1] = axis_pieces[1];
    axis_[2] = axis_pieces[2];
    axis_.normalize();
  }
  return true;
}

tf::Vector3 JointState::getTranslation()
{
  switch (joint_->type_)
  {
  case JOINT_PRISMATIC:
    return position_ * joint_->axis_;
  default:
    return tf::Vector3(0, 0, 0);
  }
}

tf::Quaternion JointState::getRotation()
{
  switch (joint_->type_)
  {
  case JOINT_CONTINUOUS:
  case JOINT_ROTARY:
    return tf::Quaternion(joint_->axis_, position_);
  default:
    return tf::Quaternion(0, 0, 0, 1);
  }
}

tf::Vector3 JointState::getTransVelocity()
{
  switch (joint_->type_)
  {
  case JOINT_PRISMATIC:
    return velocity_ * joint_->axis_;
  default:
    return tf::Vector3(0, 0, 0);
  }
}

tf::Vector3 JointState::getRotVelocity()
{
  switch (joint_->type_)
  {
  case JOINT_CONTINUOUS:
  case JOINT_ROTARY:
    return velocity_ * joint_->axis_;
  default:
    return tf::Vector3(0, 0, 0);
  }
}

tf::Transform JointState::getTransform()
{
  switch (joint_->type_)
  {
  case JOINT_CONTINUOUS:
  case JOINT_ROTARY:
    return tf::Transform(getRotation());
  case JOINT_PRISMATIC:
    return tf::Transform(tf::Quaternion(0,0,0,1), getTranslation());
  default:
    tf::Transform t;
    t.setIdentity();
    return t;
  }
}
