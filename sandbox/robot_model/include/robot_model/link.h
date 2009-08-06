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

#ifndef RobotModel_PARSER_LINK_H
#define RobotModel_PARSER_LINK_H

#include <string>
#include <vector>
#include <tinyxml/tinyxml.h>
#include <boost/shared_ptr.hpp>

#include <robot_model/joint.h>

namespace robot_model{

class Geometry
{
public:
  enum {SPHERE, BOX, CYLINDER, MESH} type_;

  virtual bool initXml(TiXmlElement *) = 0;

};

class Sphere : public Geometry
{
public:
  Sphere() { this->clear(); };
  double radius_;
protected:
  void clear()
  {
    radius_ = 0;
  };
  bool initXml(TiXmlElement *);

  friend class Link;
};

class Box : public Geometry
{
public:
  Box() { this->clear(); };
  boost::shared_ptr<Vector3> dim_;
protected:
  void clear()
  {
    dim_.reset();
  };
  bool initXml(TiXmlElement *);

  friend class Link;
};

class Cylinder : public Geometry
{
public:
  Cylinder() { this->clear(); };
  double length_;
  double radius_;
protected:
  void clear()
  {
    length_ = 0;
    radius_ = 0;
  };
  bool initXml(TiXmlElement *);

  friend class Link;
};

class Mesh : public Geometry
{
public:
  Mesh() { this->clear(); };
  std::string filename_;
  boost::shared_ptr<Vector3> scale_;
protected:
  void clear()
  {
    filename_.clear();
    scale_.reset();
  };
  bool initXml(TiXmlElement *);

  friend class Link;
};

class Inertial
{
public:
  Inertial() { this->clear(); };
  boost::shared_ptr<Pose> origin_;
  double mass_;
  double ixx_,ixy_,ixz_,iyy_,iyz_,izz_;
protected:
  void clear()
  {
    origin_.reset();
    mass_ = 0;
    ixx_ = ixy_ = ixz_ = iyy_ = iyz_ = izz_ = 0;
  };
  bool initXml(TiXmlElement* config);

  friend class Link;
};

class Visual
{
public:
  Visual() { this->clear(); };
  boost::shared_ptr<Pose> origin_;
  boost::shared_ptr<Geometry> geometry_;
protected:
  void clear()
  {
    origin_.reset();
    geometry_.reset();
  };
  bool initXml(TiXmlElement* config);

  friend class Link;
};

class Collision
{
public:
  Collision() { this->clear(); };
  boost::shared_ptr<Pose> origin_;
  boost::shared_ptr<Geometry> geometry_;
protected:
  void clear()
  {
    origin_.reset();
    geometry_.reset();
  };
  bool initXml(TiXmlElement* config);

  friend class Link;
};


class Link
{
public:
  Link() { this->clear(); };

  std::string name_;

  /// inertial element
  boost::shared_ptr<Inertial> inertial_;

  /// visual element
  boost::shared_ptr<Visual> visual_;

  /// collision element
  boost::shared_ptr<Collision> collision_;

  /// Parent Joint element
  ///   explicitly stating "parent" because we want directional-ness for tree structure
  ///   every link can have one parent
  boost::shared_ptr<Joint> parent_joint_;

  /// Get Parent Link throught the Parent Joint
  boost::shared_ptr<Link> parent_link_;

  std::vector<boost::shared_ptr<Joint> > child_joints_;
  std::vector<boost::shared_ptr<Link> > child_links_;

protected:
  bool initXml(TiXmlElement* config);
  void setParent(boost::shared_ptr<Link> parent);

private:
  void clear()
  {
    this->name_.clear();
    this->inertial_.reset();
    this->visual_.reset();
    this->collision_.reset();
    this->parent_joint_.reset();
    this->parent_link_.reset();
    this->child_joints_.clear();
    this->child_links_.clear();
  };
  void addChild(boost::shared_ptr<Link> child);
  void addChildJoint(boost::shared_ptr<Joint> child);

  friend class RobotModel;
};




}

#endif
