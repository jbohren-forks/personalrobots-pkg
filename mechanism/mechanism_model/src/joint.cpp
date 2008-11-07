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
  s->commanded_effort_ = min(max(s->commanded_effort_, -effort_limit_), effort_limit_);

  if( !(has_safety_limits_ && s->calibrated_) )
    return;

  //TODO: add velocity control

  double upper_limit = joint_limit_max_ - safety_length_max_;
  double lower_limit = joint_limit_min_ + safety_length_min_;

  if (s->position_ > upper_limit)
  {
    // Damping
    if (s->velocity_ > 0)
      s->commanded_effort_ += -damping_constant_max_ * s->velocity_;

    // Spring
    double offset = s->position_ - upper_limit;
    s->commanded_effort_ += -spring_constant_max_ * offset;
  }
  else if(s->position_ < lower_limit)
  {
    // Damping
    if (s->velocity_ < 0)
      s->commanded_effort_ += -damping_constant_min_ * s->velocity_;

    // Spring
    double offset = s->position_ - lower_limit;
    s->commanded_effort_ += -spring_constant_min_ * offset;
  }

  // One more time, just in case there are bugs in the safety limit code
  s->commanded_effort_ = min(max(s->commanded_effort_, -effort_limit_), effort_limit_);
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
      velocity_limit_ = 0.0;

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
  }

  // Safety limit code
  if (type_ == JOINT_ROTARY || type_ == JOINT_PRISMATIC)
  {
    // Loads safety limits
    TiXmlElement *safetyMinElt =elt->FirstChildElement("safety_limit_min");

    if(!safetyMinElt && SAFETY_LIMS_STRICTLY_ENFORCED)
      return false;
    else if (safetyMinElt)
    {
      if(safetyMinElt->QueryDoubleAttribute("spring_constant", &spring_constant_min_) != TIXML_SUCCESS)
        return false;
      if(safetyMinElt->QueryDoubleAttribute("damping_constant", &damping_constant_min_) != TIXML_SUCCESS)
        return false;
      if(safetyMinElt->QueryDoubleAttribute("safety_length", &safety_length_min_) != TIXML_SUCCESS)
        return false;

      TiXmlElement *safetyMaxElt =elt->FirstChildElement("safety_limit_max");
      if(!safetyMaxElt)
        return SAFETY_LIMS_STRICTLY_ENFORCED == false;

      if(safetyMaxElt->QueryDoubleAttribute("spring_constant", &spring_constant_max_) != TIXML_SUCCESS)
        return false;
      if(safetyMaxElt->QueryDoubleAttribute("damping_constant", &damping_constant_max_) != TIXML_SUCCESS)
        return false;
      if(safetyMaxElt->QueryDoubleAttribute("safety_length", &safety_length_max_) != TIXML_SUCCESS)
        return false;

      std::cout<<"Loaded safety limit code for joint "<<name_<<std::endl;
      std::cout<<spring_constant_min_<<" "<<damping_constant_min_<<" "<<safety_length_min_<<"\n";

      assert(safety_length_max_ > 0);
      assert(safety_length_min_ > 0);
      assert(joint_limit_max_ > joint_limit_min_);
      const double midpoint = 0.5*(joint_limit_max_+joint_limit_min_);
      assert(joint_limit_max_ - safety_length_max_ > midpoint);
      assert(joint_limit_min_ + safety_length_min_ < midpoint);

      has_safety_limits_ = true;
    }
  }

  // Parses out the joint properties, this is done for all joints as default
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
