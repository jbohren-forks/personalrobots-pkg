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

#include <rdf_parser/joint.h>

using namespace std;

namespace rdf_parser{

bool JointProperties::initXml(TiXmlElement* config)
{
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

  // Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

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
  // Get min joint limit
  const char* min_str = config->Attribute("min");
  if (min_str == NULL)
    std::cout << "INFO: joint limit: no min" << std::endl;
  else
    this->min_ = atof(min_str);

  // Get min joint limit
  const char* max_str = config->Attribute("max");
  if (max_str == NULL)
    std::cout << "INFO: joint limit: no max" << std::endl;
  else
    this->max_ = atof(max_str);

  // Get min joint limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL)
    std::cout << "INFO: joint limit: no effort" << std::endl;
  else
    this->effort_ = atof(effort_str);

  // Get min joint limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL)
    std::cout << "INFO: joint limit: no velocity" << std::endl;
  else
    this->velocity_ = atof(velocity_str);

  // Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  if (min_str == NULL && max_str == NULL && effort_str == NULL && velocity_str == NULL)
  {
    std::cerr << "ERROR: joint limit element specified with no readable attributes" << std::endl;
    return false;
  }
  else
    return true;
}

const std::string& Joint::getName() const
{
  return name_;
}

bool Joint::initXml(TiXmlElement* config)
{
  const char *name = config->Attribute("name");
  if (!name)
  {
    std::cerr << "ERROR: unnamed joint found" << std::endl;
    return false;
  }
  this->name_ = name;

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
      if (!axis_xml->Attribute("xyz"))
        std::cout << "WARN: no xyz attribute for axis element for Joint link: "
                  << this->name_ << ", using default values" << std::endl;
      else if (!this->axis_.init(axis_xml->Attribute("xyz")))
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
          return false;
        }
      }
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

  // Get Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  return true;
}



}
