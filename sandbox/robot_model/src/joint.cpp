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

/* Author: John Hsu */

#include <robot_model/joint.h>

namespace robot_model{

bool JointProperties::initXml(TiXmlElement* config)
{
  this->clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL)
    std::cout << "INFO: joint_properties: no damping" << std::endl;
  else
    this->damping_ = atof(damping_str);

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL)
    std::cout << "INFO: joint_properties: no friction" << std::endl;
  else
    this->friction_ = atof(friction_str);

  if (damping_str == NULL && friction_str == NULL)
  {
    std::cerr << "ERROR: joint_properties element specified with no damping and no friction" << std::endl;
    return false;
  }
  else
    return true;
}

bool JointLimits::initXml(TiXmlElement* config)
{
  this->clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL)
    std::cout << "INFO: joint limit: no lower" << std::endl;
  else
    this->lower_ = atof(lower_str);

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL)
    std::cout << "INFO: joint limit: no upper" << std::endl;
  else
    this->upper_ = atof(upper_str);

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL)
    std::cout << "INFO: joint limit: no effort" << std::endl;
  else
    this->effort_ = atof(effort_str);

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL)
    std::cout << "INFO: joint limit: no velocity" << std::endl;
  else
    this->velocity_ = atof(velocity_str);

  if (lower_str == NULL && upper_str == NULL && effort_str == NULL && velocity_str == NULL)
  {
    std::cerr << "ERROR: joint limit element specified with no readable attributes" << std::endl;
    return false;
  }
  else
    return true;
}

bool JointSafety::initXml(TiXmlElement* config)
{
  this->clear();

  // Get soft_lower_limit joint limit
  const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    std::cout << "INFO: joint safety: no soft_lower_limit, using default value" << std::endl;
    this->soft_lower_limit_ = 0;
  }
  else
    this->soft_lower_limit_ = atof(soft_lower_limit_str);

  // Get soft_upper_limit joint limit
  const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    std::cout << "INFO: joint safety: no soft_upper_limit, using default value" << std::endl;
    this->soft_upper_limit_ = 0;
  }
  else
    this->soft_upper_limit_ = atof(soft_upper_limit_str);

  // Get k_p_ safety "position" gain - not exactly position gain
  const char* k_p_str = config->Attribute("k_p");
  if (k_p_str == NULL)
  {
    std::cout << "INFO: joint safety: no k_p, using default value" << std::endl;
    this->k_p_ = 0;
  }
  else
    this->k_p_ = atof(k_p_str);

  // Get k_v_ safety velocity gain
  const char* k_v_str = config->Attribute("k_v");
  if (k_v_str == NULL)
  {
    std::cout << "INFO: joint safety: no k_v, using default value" << std::endl;
    this->k_v_ = 0;
  }
  else
    this->k_v_ = atof(k_v_str);

  return true;
}

bool JointCalibration::initXml(TiXmlElement* config)
{
  this->clear();

  // Get reference_position
  const char* reference_position_str = config->Attribute("reference_position");
  if (reference_position_str == NULL)
  {
    std::cout << "INFO: joint calibration: no reference_position, using default value" << std::endl;
    this->reference_position_ = 0;
  }
  else
    this->reference_position_ = atof(reference_position_str);

  return true;
}

bool Joint::initXml(TiXmlElement* config)
{
  this->clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    std::cerr << "ERROR: unnamed joint found" << std::endl;
    return false;
  }
  this->name_ = name;

  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("name");
    if (!pname)
      std::cout << "WARN: parent name for Joint link: "
                << this->name_ << ", this might be the root?" << std::endl;
    else
    {
      this->parent_link_name_ = pname;

      // Get transform from Parent Link to Joint Frame
      TiXmlElement *origin_xml = parent_xml->FirstChildElement("origin");
      this->origin_.reset(new Pose());
      if (!origin_xml)
      {
        std::cerr << "INFO: Joint " << this->name_
                  << " requires an origin tag under parent"
                  << " describing transform from Parent Link to Joint Frame."
                  << std::endl;
        this->origin_.reset();
        return false;
      }
      else
      {
        this->parent_origin_.reset(new Pose());
        if (!this->parent_origin_->initXml(origin_xml))
        {
          std::cerr << "ERROR: Malformed parent origin element for joint:"
                    << this->name_ << std::endl;
          this->parent_origin_.reset();
          return false;
        }
      }
    }
  }

  // Get Joint type
  const char* type = config->Attribute("type");
  if (!type)
  {
    std::cout << "WARN: joint " << name_
              << " has no type, check to see if it's a reference." << std::endl;
    return false;
  }
  std::string type_str = type;
  if (type_str == "planar")
    type_ = PLANAR;
  else if (type_str == "floating")
    type_ = FLOATING;
  else if (type_str == "revolute")
    type_ = REVOLUTE;
  else if (type_str == "continuous")
    type_ = CONTINUOUS;
  else if (type_str == "prismatic")
    type_ = PRISMATIC;
  else if (type_str == "fixed")
    type_ = FIXED;
  else
  {
    std::cerr << "ERROR: Joint " << this->name_ << " has no known type: " << type_str << std::endl;
    return false;
  }

  // Get Joint Axis
  if (this->type_ != FLOATING)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (axis_xml)
    {
      this->axis_.reset(new Vector3());
      if (!axis_xml->Attribute("xyz"))
        std::cout << "WARN: no xyz attribute for axis element for Joint link: "
                  << this->name_ << ", using default values" << std::endl;
      else
      {
        if (!this->axis_->init(axis_xml->Attribute("xyz")))
        {
          if (this->type_ == PLANAR)
            std::cout << "INFO: PLANAR Joint " << this->name_
                      << " will require an axis tag in the future"
                      << " which indicates the surface normal of the plane."
                      << std::endl;
          else
          {
            std::cerr << "ERROR: Malformed axis element for joint:"
                      << this->name_ << std::endl;
            this->axis_.reset();
            return false;
          }
        }
      }
    }
  }

  // Get transform from Link Frame to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    std::cerr << "ERROR: Joint " << this->name_
              << " requires an origin tag"
              << " describing transform from Link Frame to Joint Frame."
              << std::endl;
    return false;
  }
  else
  {
    this->origin_.reset(new Pose());
    if (!this->origin_->initXml(origin_xml))
    {
      std::cerr << "ERROR: Malformed origin element for joint:"
                << this->name_ << std::endl;
      this->origin_.reset();
      return false;
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    joint_limits_.reset(new JointLimits);
    if (!joint_limits_->initXml(limit_xml))
    {
      std::cerr << "ERROR: Could not parse limit element for joint:"
                << this->name_ << std::endl;
      joint_limits_.reset();
    }
  }

  // Get safety
  TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    joint_safety_.reset(new JointSafety);
    if (!joint_safety_->initXml(safety_xml))
    {
      std::cerr << "ERROR: Could not parse safety element for joint:"
                << this->name_ << std::endl;
      joint_safety_.reset();
    }
  }

  // Get calibration
  TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    joint_calibration_.reset(new JointCalibration);
    if (!joint_calibration_->initXml(calibration_xml))
    {
      std::cerr << "ERROR: Could not parse calibration element for joint:"
                << this->name_ << std::endl;
      joint_calibration_.reset();
    }
  }

  // Get properties
  TiXmlElement *prop_xml = config->FirstChildElement("joint_properties");
  if (prop_xml)
  {
    joint_properties_.reset(new JointProperties);
    if (!joint_properties_->initXml(prop_xml))
    {
      std::cerr << "ERROR: Could not parse joint_properties element for joint:"
                << this->name_ << std::endl;
      joint_properties_.reset();
    }
  }

  return true;
}



}
