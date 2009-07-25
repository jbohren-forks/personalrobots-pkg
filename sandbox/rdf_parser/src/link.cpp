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


#include "rdf_parser/link.h"

using namespace std;


namespace rdf_parser{

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
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!this->origin_.initXml(o))
  {
    std::cerr << "ERROR: Inertial has a malformed origin tag" << std::endl;
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

  // Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  return true;
}

bool Visual::initXml(TiXmlElement *config)
{
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!this->origin_.initXml(o))
  {
    std::cerr << "ERROR: Visual has a malformed origin tag" << std::endl;
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

  // Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  return true;
}

bool Collision::initXml(TiXmlElement* config)
{
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (!this->origin_.initXml(o))
  {
    std::cerr << "ERROR: Collision has a malformed origin tag" << std::endl;
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

  // Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  return true;
}

bool Sphere::initXml(TiXmlElement *c)
{
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
  if (!dim_.init(c->Attribute("size")))
  {
    std::cerr << "ERROR: Box shape has no size attribute" << std::endl;
    return false;
  }
  return true;
}

bool Cylinder::initXml(TiXmlElement *c)
{
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
  if (!c->Attribute("filename"))
  {
    std::cerr << "ERROR: Mesh must contain a filename attribute" << std::endl;
    return false;
  }

  filename_ = c->Attribute("filename");

  if (c->Attribute("scale"))
  {
    if (!this->scale_.init(c->Attribute("scale")))
    {
      std::cerr << "ERROR: Mesh scale was specified, but could not be parsed" << std::endl;
      return false;
    }
  }

  return true;
}


bool Link::initXml(TiXmlElement* config)
{
  this->children_.clear();
  this->child_joints_.clear();
  this->maps_.clear();

  const char *name = config->Attribute("name");
  if (!name)
  {
    std::cerr << "ERROR: No name given for the link." << std::endl;
    return false;
  }
  name_ = std::string(name);

  // set parent Joint name
  TiXmlElement *j = config->FirstChildElement("joint");
  const char *parent_joint_name = j ? j->Attribute("name") : NULL;
  if (!parent_joint_name)
  {
    // in proposed new URDF links are to have no joints, but the other way around
    std::cerr << "ERROR: Invalid parent joint name: " << parent_joint_name << " for Link:"
              << this->name_ << std::endl;
    return false;
  }
  parent_joint_name_ = std::string(parent_joint_name);

  // Parent - this is to be moved to Joint
  //   fill in parent_name_ and parent_
  TiXmlElement *p = config->FirstChildElement("parent");
  const char *parent_name = p ? p->Attribute("name") : NULL;
  if (!parent_name)
  {
    // in proposed new URDF, parent is specified in joint, joints connect to parent and child links
    std::cerr << "ERROR: No parent name given for Link:"
              << this->name_ << std::endl;
    return false;
  }
  parent_name_ = std::string(parent_name);

  // Origin - this is to be moved to Joint
  //   fill in origin_xml_
  origin_xml_ = config->FirstChildElement("origin");
  if (!origin_xml_)
  {
    // in proposed new URDF, origin is specified in joint, for both parent and child
    std::cerr << "ERROR: No origin tag for Link:"
              << this->name_ << std::endl;
    return false;
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

  // Maps
  for (TiXmlElement* map_xml = config->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  return true;
}


const std::string& Link::getName() const
{
  return name_;
}

Link* Link::getParent()
{
  return parent_;
}

std::vector<Link*>* Link::getChildren()
{
  return &this->children_;
}

std::vector<Joint*>* Link::getChildrenJoint()
{
  return &this->child_joints_;
}

void Link::setParent(Link* parent)
{
  this->parent_ = parent;
  cout << "INFO: set parent Link: " << parent->getName() << " for Link: " << getName() << endl;
}

void Link::addChild(Link* child)
{
  this->children_.push_back(child);
  cout << "INFO: added child Link: " << child->getName() << " to Link: " << getName() << endl;
}

const std::string& Link::getParentJointName()
{
  return this->parent_joint_name_;
}

void Link::setParentJoint(Joint* parent)
{
  this->parent_joint_ = parent;
  cout << "INFO: set parent Joint " << parent->getName() << " for Link: " << getName() << endl;
}

void Link::addChildJoint(Joint* child)
{
  this->child_joints_.push_back(child);
  cout << "INFO: added child Joint " << child->getName() << " to Link: " << getName() << endl;
}



}

