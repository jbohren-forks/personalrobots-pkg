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

#include <map>
#include <string>
#include <mechanism_model/joint.h>

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
  // TODO: enforce the limits so the joint operates safely
}

void Joint::initXml(TiXmlElement *elt)
{
  TiXmlElement *min = elt->FirstChildElement("limitMin");
  TiXmlElement *max = elt->FirstChildElement("limitMax");
  joint_limit_min_ = min ? atof(min->GetText()) : 0;
  joint_limit_max_ = max ? atof(max->GetText()) : 0;
  effort_limit_ = atof(elt->FirstChildElement("effortLimit")->GetText());
  velocity_limit_ = atof(elt->FirstChildElement("velocityLimit")->GetText());

  type_ = g_type_map[elt->Attribute("type")];
  // If type is revolute and limits aren't set, then it is continuous
  if (type_ == JOINT_ROTARY && min == NULL && max == NULL)
  {
    type_ = JOINT_CONTINUOUS;
  }
}

