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

#ifndef RDF_PARSER_LINK_H
#define RDF_PARSER_LINK_H

#include <string>
#include <vector>
#include <tinyxml/tinyxml.h>
#include <boost/scoped_ptr.hpp>

#include <rdf_parser/joint.h>

using namespace std;

namespace rdf_parser{

class Geometry
{
public:
  virtual bool initXml(TiXmlElement *) = 0;

  enum {SPHERE, BOX, CYLINDER, MESH} type_;

  std::vector<TiXmlElement*> maps_;
};

class Sphere : public Geometry
{
public:
  Sphere() { type_ = SPHERE; }
  bool initXml(TiXmlElement *);
  double radius_;
};

class Box : public Geometry
{
public:
  Box() { type_ = BOX; }
  bool initXml(TiXmlElement *);
  Vector3 dim_;
};

class Cylinder : public Geometry
{
public:
  Cylinder() { type_ = CYLINDER; }
  bool initXml(TiXmlElement *);
  double length_;
  double radius_;
};

class Mesh : public Geometry
{
public:
  Mesh() { type_ = MESH; }
  bool initXml(TiXmlElement *);
  std::string filename_;
  Vector3 scale_;
};

class Inertial
{
public:
  bool initXml(TiXmlElement* config);
private:
  std::vector<TiXmlElement*> maps_;
  Pose origin_;
  double mass_;
  double ixx_,ixy_,ixz_,iyy_,iyz_,izz_;
};

class Visual
{
public:
  Visual(void) {};
  bool initXml(TiXmlElement* config);
private:
  std::vector<TiXmlElement*> maps_;
  Pose origin_;
  boost::scoped_ptr<Geometry> geometry_;

};

class Collision
{
public:
  bool initXml(TiXmlElement* config);
private:
  std::vector<TiXmlElement*> maps_;
  Pose origin_;
  boost::scoped_ptr<Geometry> geometry_;

};


class Link
{
public:

  bool initXml(TiXmlElement* config);

  /// returns the name of the link
  const std::string& getName() const;

  /// returns the parent link. The root link does not have a parent
  Link* getParent();

  /// returns the parent link name
  const std::string& getParentName() {return parent_name_;};

  /// returns the parent joint name
  const std::string& getParentJointName();

  /// returns children of the link
  std::vector<Link*> getChildren();

  /// returns joint attaching link to parent
  Joint* getParentJoint();

  /// returns joints attaching link to children
  std::vector<Joint*> getChildrenJoint();

  /// inertial element
  boost::scoped_ptr<Inertial> inertial_;

  /// visual element
  boost::scoped_ptr<Visual> visual_;

  /// collision element
  boost::scoped_ptr<Collision> collision_;

private:
  std::string name_;

  Link* parent_;
  std::vector<Link*> children_;

  Joint* parent_joint_;
  std::vector<Joint*> child_joints_;

  std::vector<TiXmlElement*> maps_;

  // store parent Link/Joint and origin, these are to be moved to joint
  std::string parent_joint_name_;
  std::string parent_name_;
  TiXmlElement* origin_xml_;

public:
  void setParent(Link* parent);
  void addChild(Link* child);

  void setParentJoint(Joint* parent);
  void addChildJoint(Joint* child);
};




}

#endif
