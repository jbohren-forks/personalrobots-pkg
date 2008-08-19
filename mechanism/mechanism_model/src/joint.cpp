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
#include <cfloat>

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

void Joint::enforceLimits()
{
  commanded_effort_ = min(max(commanded_effort_, -effort_limit_), effort_limit_);
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

  TiXmlElement *limits = elt->FirstChildElement("limit");
  if (limits)
  {
    if (limits->QueryDoubleAttribute("effort", &effort_limit_) != TIXML_SUCCESS)
      effort_limit_ = 0.0;
    if (limits->QueryDoubleAttribute("velocity", &velocity_limit_) != TIXML_SUCCESS)
      velocity_limit_ = 0.0;


    int min_ret = limits->QueryDoubleAttribute("min", &joint_limit_min_);
    int max_ret = limits->QueryDoubleAttribute("max", &joint_limit_max_);

    if (type_ == JOINT_ROTARY && min_ret == TIXML_NO_ATTRIBUTE && max_ret == TIXML_NO_ATTRIBUTE)
    {
      type_ = JOINT_CONTINUOUS;
    }
  }
  else
  {
    if (type_ == JOINT_ROTARY)
      type_ = JOINT_CONTINUOUS;
    effort_limit_ = DBL_MAX;
  }

  return true;
}

