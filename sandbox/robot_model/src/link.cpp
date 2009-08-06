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

/* Author: Wim Meeussen */


#include "robot_model/link.h"

namespace robot_model{

Geometry *parseGeometry(TiXmlElement *g)
{
  if (!g) return NULL;
  std::auto_ptr<Geometry> geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    std::cerr << "ERROR: Geometry tag contains no child element." << std::endl;
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
    std::cerr << "ERROR: Unknown geometry type: " << type_name << std::endl;
    return NULL;
  }

  if (!geom->initXml(shape))
    return NULL;

  return geom.release();
}

bool Inertial::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!o)
  {
    std::cout << "WARN: origin tag not present for inertial element, checking old RobotModel format com: " << std::endl;
    o = config->FirstChildElement("com");
    if (!o)
    {
      std::cerr << "ERROR: Inertial missing origin or com tag" << std::endl;
      return false;
    }
  }
  this->origin_.reset(new Pose());
  if (!this->origin_->initXml(o))
  {
    std::cerr << "ERROR: Inertial has a malformed origin tag" << std::endl;
    this->origin_.reset();
    return false;
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    std::cerr << "ERROR: Inertial element must have mass element" << std::endl;
    return false;
  }
  if (!mass_xml->Attribute("value"))
  {
    std::cerr << "ERROR: Inertial: mass element must have value attributes" << std::endl;
    return false;
  }
  mass_ = atof(mass_xml->Attribute("value"));

  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    std::cerr << "ERROR: Inertial element must have inertia element" << std::endl;
    return false;
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    std::cerr << "ERROR: Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes" << std::endl;
    return false;
  }
  ixx_  = atof(inertia_xml->Attribute("ixx"));
  ixy_  = atof(inertia_xml->Attribute("ixy"));
  ixz_  = atof(inertia_xml->Attribute("ixz"));
  iyy_  = atof(inertia_xml->Attribute("iyy"));
  iyz_  = atof(inertia_xml->Attribute("iyz"));
  izz_  = atof(inertia_xml->Attribute("izz"));

  return true;
}

bool Visual::initXml(TiXmlElement *config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  this->origin_.reset(new Pose());
  if (!this->origin_->initXml(o))
  {
    std::cerr << "ERROR: Visual has a malformed origin tag" << std::endl;
    this->origin_.reset();
    return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry_.reset(parseGeometry(geom));
  if (!geometry_)
  {
    std::cerr << "ERROR: Malformed geometry for Visual element" << std::endl;
    return false;
  }

  return true;
}

bool Collision::initXml(TiXmlElement* config)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  this->origin_.reset(new Pose());
  if (!this->origin_->initXml(o))
  {
    std::cerr << "ERROR: Collision has a malformed origin tag" << std::endl;
    this->origin_.reset();
    return false;
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  geometry_.reset(parseGeometry(geom));
  if (!geometry_)
  {
    std::cerr << "ERROR: Malformed geometry for Collision element" << std::endl;
    return false;
  }

  return true;
}

bool Sphere::initXml(TiXmlElement *c)
{
  this->clear();

  this->type_ = SPHERE;
  if (!c->Attribute("radius"))
  {
    std::cerr << "ERROR: Sphere shape must have a radius attribute" << std::endl;
    return false;
  }

  radius_ = atof(c->Attribute("radius"));
  return false;
}

bool Box::initXml(TiXmlElement *c)
{
  this->clear();

  this->type_ = BOX;
  if (!c->Attribute("size"))
  {
    std::cerr << "ERROR: Box shape has no size attribute" << std::endl;
    return false;
  }
  dim_.reset(new Vector3());
  if (!dim_->init(c->Attribute("size")))
  {
    std::cerr << "ERROR: Box shape has malformed size attribute" << std::endl;
    dim_.reset();
    return false;
  }
  return true;
}

bool Cylinder::initXml(TiXmlElement *c)
{
  this->clear();

  this->type_ = CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    std::cerr << "ERROR: Cylinder shape must have both length and radius attributes" << std::endl;
    return false;
  }

  length_ = atof(c->Attribute("length"));
  radius_ = atof(c->Attribute("radius"));
  return true;
}

bool Mesh::initXml(TiXmlElement *c)
{
  this->clear();

  this->type_ = MESH;
  if (!c->Attribute("filename"))
  {
    std::cerr << "ERROR: Mesh must contain a filename attribute" << std::endl;
    return false;
  }

  filename_ = c->Attribute("filename");

  if (c->Attribute("scale"))
  {
    this->scale_.reset(new Vector3());
    if (!this->scale_->init(c->Attribute("scale")))
    {
      std::cerr << "ERROR: Mesh scale was specified, but could not be parsed" << std::endl;
      this->scale_.reset();
      return false;
    }
  }

  return true;
}


bool Link::initXml(TiXmlElement* config)
{
  this->clear();

  const char *name = config->Attribute("name");
  if (!name)
  {
    std::cerr << "ERROR: No name given for the link." << std::endl;
    return false;
  }
  name_ = std::string(name);

  // Inertial
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    inertial_.reset(new Inertial);
    if (!inertial_->initXml(i))
    {
      std::cerr << "ERROR: Could not parse inertial element for Link:"
                << this->name_ << std::endl;
      inertial_.reset();
    }
  }

  // Visual
  TiXmlElement *v = config->FirstChildElement("visual");
  if (v)
  {
    visual_.reset(new Visual);
    if (!visual_->initXml(v))
    {
      std::cerr << "ERROR: Could not parse visual element for Link:"
                << this->name_ << std::endl;
      visual_.reset();
    }
  }

  // Collision
  TiXmlElement *col = config->FirstChildElement("collision");
  if (col)
  {
    collision_.reset(new Collision);
    if (!collision_->initXml(col))
    {
      std::cerr << "ERROR: Could not parse collision element for Link:"
                << this->name_ << std::endl;
      collision_.reset();
    }
  }

  // Joint
  TiXmlElement *joi = config->FirstChildElement("joint");
  if (joi)
  {
    parent_joint_.reset(new Joint);
    if (!parent_joint_->initXml(joi))
    {
      std::cerr << "ERROR: Could not parse joint element for Link:"
                << this->name_ << std::endl;
      parent_joint_.reset();
    }
    else
    {
      parent_joint_->link_ = (boost::shared_ptr<Link>)this;
      parent_joint_->link_name_ = name_;
    }
  }

  return true;
}

void Link::setParent(boost::shared_ptr<Link> parent)
{
  this->parent_link_ = parent;
  std::cout << "INFO: set parent Link: " << parent->name_ << " for Link: " << this->name_ << std::endl;
}

void Link::addChild(boost::shared_ptr<Link> child)
{
  this->child_links_.push_back(child);
  std::cout << "INFO: added child Link: " << child->name_ << " to Link: " << this->name_ << std::endl;
}

void Link::addChildJoint(boost::shared_ptr<Joint> child)
{
  this->child_joints_.push_back(child);
  std::cout << "INFO: added child Joint " << child->name_ << " to Link: " << this->name_ << std::endl;
}



}

