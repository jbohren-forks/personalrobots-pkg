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
#ifndef MECHANISM_LINK_H
#define MECHANISM_LINK_H

#include "mechanism_model/joint.h"
#include <vector>
#include "boost/scoped_ptr.hpp"
#include "libTF/Pose3D.h"
#include "tf/transform_datatypes.h"


namespace mechanism {


// Only slightly more sane than the previous version
class Maps
{
public:
  typedef std::map<std::pair<std::string,std::string>,std::string> M;
  void put(const std::string &mapn, const std::string &key, const std::string &value)
  {
    map_.insert(std::make_pair(std::make_pair(mapn, key), value));
  }
  std::string get(const std::string &mapn, const std::string &key, const std::string &def = "")
  {
    M::iterator it = map_.find(std::make_pair(mapn, key));
    return it == map_.end() ? def : it->second;
  }
  M map_;

  void initXml(TiXmlElement *xml)
  {
    TiXmlElement *map = xml->FirstChildElement("map");
    for (; map; map = map->NextSiblingElement("map"))
    {
      TiXmlElement *elem = map->FirstChildElement("elem");
      for (; elem; elem = elem->NextSiblingElement("elem"))
      {
        if (elem->Attribute("key") && elem->Attribute("value"))
          put(map->Attribute("name"), elem->Attribute("key"), elem->Attribute("value"));
      }
    }
    // TODO: verbatim
  }
};


class Robot;
class Visual;
class Geometry;
class Collision;

class Link
{
public:
  Link();
  ~Link() {}

  bool initXml(TiXmlElement *config, Robot *robot);

  std::string name_;
  std::string parent_name_;
  std::string joint_name_;

  tf::Vector3 origin_xyz_;
  tf::Vector3 origin_rpy_;

  tf::Transform getOffset()
  {
    return tf::Transform(tf::Quaternion(0,0,0), origin_xyz_);
  }

  tf::Transform getRotation()
  {
    return tf::Transform(tf::Quaternion(origin_rpy_[2], origin_rpy_[1], origin_rpy_[0]));
  }

  boost::scoped_ptr<Visual> visual_;
  boost::scoped_ptr<Collision> collision_;

  Maps maps_;
};

class LinkState
{
public:
  Link *link_;

  tf::Transform rel_frame_;  // Relative transform to the parent link's frame.
  tf::Vector3 abs_position_;  // Absolute position (in the robot frame)
  tf::Quaternion abs_orientation_;  // Absolute orientation (in the robot frame)
  tf::Vector3 abs_velocity_;
  tf::Vector3 abs_rot_velocity_;

  void propagateFK(LinkState *parent, JointState *joint);

  LinkState() : link_(NULL) {}
  LinkState(const LinkState &s) : link_(s.link_), rel_frame_(s.rel_frame_) {}
};



class Visual
{
public:
  Visual();
  ~Visual() {}

  bool initXml(TiXmlElement *config);

  tf::Vector3 origin_xyz_;
  tf::Vector3 origin_rpy_;
  tf::Vector3 scale_;  // TODO
  boost::scoped_ptr<Geometry> geometry_;

  Maps maps_;
};

class Collision
{
public:
  Collision();
  ~Collision() {}

  bool initXml(TiXmlElement *config);

  tf::Vector3 origin_xyz_, origin_rpy_;
  boost::scoped_ptr<Geometry> geometry_;

  Maps maps_;
};

class Geometry
{
public:
  virtual ~Geometry() {}

  virtual bool initXml(TiXmlElement *) = 0;

  enum {SPHERE, BOX, CYLINDER, MESH} type_;
  // TODO: Map
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
  tf::Vector3 dim_;
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
};

} // namespace mechanism

#endif
