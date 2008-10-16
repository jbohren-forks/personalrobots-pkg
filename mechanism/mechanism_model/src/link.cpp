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

#include "mechanism_model/link.h"
#include <map>
#include <string>
#include "mechanism_model/robot.h"
#include "urdf/parser.h"

namespace mechanism {

Link::Link()
  : origin_xyz_(0,0,0), origin_rpy_(0,0,0)
{
}

bool Link::initXml(TiXmlElement *config, Robot *robot)
{
  const char *name = config->Attribute("name");
  if (!name)
  {
    fprintf(stderr, "Error: No name given for the link.\n");
    return false;
  }
  name_ = std::string(name);

  // Parent
  TiXmlElement *p = config->FirstChildElement("parent");
  const char *parent_name = p ? p->Attribute("name") : NULL;
  if (!parent_name)
  {
    fprintf(stderr, "Error: No parent name given for link \"%s\"\n", name_.c_str());
    return false;
  }
  parent_name_ = std::string(parent_name);

  // Joint
  TiXmlElement *j = config->FirstChildElement("joint");
  const char *joint_name = j ? j->Attribute("name") : NULL;
  if (!joint_name || !robot->getJoint(joint_name))
  {
    fprintf(stderr, "Error: Link \"%s\" could not find the joint named \"%s\"\n",
            name_.c_str(), joint_name);
    return false;
  }
  joint_name_ = std::string(joint_name);

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    fprintf(stderr, "Error: Link \"%s\" has no origin element\n", name_.c_str());
    return false;
  }
  std::vector<double> xyz, rpy;
  if (!urdf::queryVectorAttribute(o, "xyz", &xyz))
  {
    fprintf(stderr, "Error: Link \"%s\"'s origin has no xyz attribute\n", name_.c_str());
    return false;
  }
  if (!urdf::queryVectorAttribute(o, "rpy", &rpy))
  {
    fprintf(stderr, "Error: Link \"%s\"'s origin has no rpy attribute\n", name_.c_str());
    return false;
  }
  if (xyz.size() != 3)
  {
    fprintf(stderr, "Error: Link \"%s\"'s xyz origin is malformed\n", name_.c_str());
    return false;
  }
  if (rpy.size() != 3)
  {
    fprintf(stderr, "Error: Link \"%s\"'s rpy origin is malformed\n", name_.c_str());
    return false;
  }
  for (int i = 0; i < 3; ++i)
  {
    origin_xyz_[i] = xyz[i];
    origin_rpy_[i] = rpy[i];
  }

  return true;
}

// j[oint] connects this and p[arent]
void LinkState::propagateFK(LinkState *p, JointState *j)
{
  if (p == NULL && j == NULL)
  {
    rel_frame_.setIdentity();
    abs_position_.setValue(0, 0, 0);
    abs_orientation_.setValue(0, 0, 0, 1);
    abs_velocity_.setValue(0, 0, 0);
    abs_rot_velocity_.setValue(0, 0, 0);
  }
  else
  {
    assert(p);  assert(j);

    abs_position_ =
      p->abs_position_
      + quatRotate(p->abs_orientation_, link_->origin_xyz_)
      + quatRotate(p->abs_orientation_, j->getTranslation());

    tf::Quaternion rel_or(link_->origin_rpy_[2], link_->origin_rpy_[1], link_->origin_rpy_[0]);
    abs_orientation_ = p->abs_orientation_ * j->getRotation() * rel_or;
    abs_orientation_.normalize();

    abs_velocity_ =
      p->abs_velocity_
      + cross(p->abs_rot_velocity_, quatRotate(p->abs_orientation_,
                                               link_->origin_xyz_))
      + quatRotate(p->abs_orientation_, j->getTransVelocity());

    abs_rot_velocity_ = p->abs_rot_velocity_ + quatRotate(p->abs_orientation_, j->getRotVelocity());


    // Computes the relative frame transform
    rel_frame_.setIdentity();
    rel_frame_ *= tf::Transform(tf::Quaternion(0,0,0), link_->origin_xyz_);
    rel_frame_ *= j->getTransform();
    rel_frame_ *= tf::Transform(tf::Quaternion(link_->origin_rpy_[2],
                                               link_->origin_rpy_[1],
                                               link_->origin_rpy_[0]));

    tf::Vector3 jo = j->getTransform().getOrigin();
  }
}

} // namespace mechanism
