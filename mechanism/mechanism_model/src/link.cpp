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
#include <memory>
#include "mechanism_model/robot.h"
#include "urdf/parser.h"


namespace mechanism {

bool parseOrigin(TiXmlElement *o, tf::Vector3 &xyz_out, tf::Vector3 &rpy_out)
{
  if (!o)
  {
    ROS_ERROR("Origin element did not exist");
    return false;
  }
  std::vector<double> xyz, rpy;
  if (!urdf::queryVectorAttribute(o, "xyz", &xyz))
  {
    ROS_ERROR("Origin has no xyz element");
    return false;
  }
  if (!urdf::queryVectorAttribute(o, "rpy", &rpy))
  {
    ROS_ERROR("Origin has no rpy element");
    return false;
  }
  if (xyz.size() != 3)
  {
    ROS_ERROR("Origin's xyz is malformed");
    return false;
  }
  if (rpy.size() != 3)
  {
    ROS_ERROR("Origin's rpy is malformed");
    return false;
  }
  for (int i = 0; i < 3; ++i)
  {
    xyz_out[i] = xyz[i];
    rpy_out[i] = rpy[i];
  }

  return true;
}

Geometry *parseGeometry(TiXmlElement *g)
{
  if (!g) return NULL;
  std::auto_ptr<Geometry> geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    ROS_ERROR("Geometry tag contains no shape");
    return NULL;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
    geom.reset(new Sphere);
  else if (type_name == "box")
    geom.reset(new Box);
  else if (type_name == "cylinder")
    geom.reset(new Cylinder);
  else if (type_name == "mesh")
    geom.reset(new Mesh);
  else
  {
    ROS_ERROR("Unknown geometry type: %s", type_name.c_str());
    return NULL;
  }

  if (!geom->initXml(shape))
    return NULL;

  return geom.release();
}

Link::Link()
  : origin_xyz_(0,0,0), origin_rpy_(0,0,0)
{
}

bool Link::initXml(TiXmlElement *config, Robot *robot)
{
  const char *name = config->Attribute("name");
  if (!name)
  {
    ROS_ERROR("No name given for the link.");
    return false;
  }
  name_ = std::string(name);

  // Parent
  TiXmlElement *p = config->FirstChildElement("parent");
  const char *parent_name = p ? p->Attribute("name") : NULL;
  if (!parent_name)
  {
    ROS_ERROR("No parent name given for link \"%s\"", name_.c_str());
    return false;
  }
  parent_name_ = std::string(parent_name);

  // Joint
  TiXmlElement *j = config->FirstChildElement("joint");
  const char *joint_name = j ? j->Attribute("name") : NULL;
  if (!joint_name || !robot->getJoint(joint_name))
  {
    ROS_ERROR("Link \"%s\" could not find the joint named \"%s\"",
              name_.c_str(), joint_name);
    return false;
  }
  joint_name_ = std::string(joint_name);

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!parseOrigin(o, origin_xyz_, origin_rpy_))
  {
    ROS_ERROR("The origin tag for link \"%s\" is malformed", name_.c_str());
    return false;
  }

  // Visual
  TiXmlElement *v = config->FirstChildElement("visual");
  if (v)
  {
    visual_.reset(new Visual);
    if (!visual_->initXml(v))
    {
      ROS_ERROR("Could not parse visual element for link \"%s\"", name_.c_str());
      visual_.reset();
    }
  }

  // Collision
  TiXmlElement *col = config->FirstChildElement("collision");
  if (v)
  {
    collision_.reset(new Collision);
    if (!collision_->initXml(col))
    {
      ROS_ERROR("Could not parse collision element for link \"%s\"", name_.c_str());
      collision_.reset();
    }
  }

  maps_.initXml(config);

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

Visual::Visual()
  : origin_xyz_(0,0,0), origin_rpy_(0,0,0)
{
}

bool Visual::initXml(TiXmlElement *config)
{
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!parseOrigin(o, origin_xyz_, origin_rpy_))
  {
    ROS_ERROR("Visual has a malformed origin tag");
    return false;
  }

  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry_.reset(parseGeometry(geom));
  if (!geometry_)
  {
    ROS_ERROR("Malformed geometry for Visual element");
    return false;
  }

  maps_.initXml(config);

  return true;
}

Collision::Collision()
  : origin_xyz_(0,0,0), origin_rpy_(0,0,0)
{
}

bool Collision::initXml(TiXmlElement* config)
{
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!parseOrigin(o, origin_xyz_, origin_rpy_))
  {
    ROS_ERROR("Collision has a malformed origin tag");
    return false;
  }

  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry_.reset(parseGeometry(geom));
  if (!geometry_)
  {
    ROS_ERROR("Malformed geometry for Collision element");
    return false;
  }

  maps_.initXml(config);

  return true;
}

bool Sphere::initXml(TiXmlElement *c)
{
  if (!c->Attribute("radius"))
  {
    ROS_ERROR("Sphere shape must have a radius attribute");
    return false;
  }

  radius_ = atof(c->Attribute("radius"));
  return false;
}

bool Box::initXml(TiXmlElement *c)
{
  std::vector<double> dim;
  if (!urdf::queryVectorAttribute(c, "size", &dim))
  {
    ROS_ERROR("Box shape has no size attribute");
    return false;
  }
  if (dim.size() != 3)
  {
    ROS_ERROR("Malformed xml for box");
    return false;
  }

  for (unsigned int i = 0; i < 3; ++i)
    dim_[i] = dim[i];
  return true;
}

bool Cylinder::initXml(TiXmlElement *c)
{
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    ROS_ERROR("Cylinder shape must have both length and radius attributes");
    return false;
  }

  length_ = atof(c->Attribute("length"));
  radius_ = atof(c->Attribute("radius"));
  return true;
}

bool Mesh::initXml(TiXmlElement *c)
{
  if (!c->Attribute("filename"))
  {
    ROS_ERROR("Mesh must contain a filename attribute");
    return false;
  }

  filename_ = c->Attribute("filename");
  return true;
}

} // namespace mechanism
