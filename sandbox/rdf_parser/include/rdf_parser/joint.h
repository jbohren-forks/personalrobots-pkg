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

#ifndef RDF_PARSER_JOINT_H
#define RDF_PARSER_JOINT_H

#include <string>
#include <vector>
#include <tinyxml/tinyxml.h>
#include <boost/scoped_ptr.hpp>

#include <rdf_parser/pose.h>

using namespace std;

namespace rdf_parser{

class Link;

class JointProperties
{
public:
  JointProperties()
  {
    damping_ = 0;
    friction_ = 0;
  };
  virtual ~JointProperties(void) {};
  bool initXml(TiXmlElement* config);
  double damping_;
  double friction_;
};

class JointLimits
{
public:
  JointLimits()
  {
    min_ = 0;
    max_ = 0;
    effort_ = 0;
    velocity_ = 0;
  };
  virtual ~JointLimits(void) {};
  bool initXml(TiXmlElement* config);
  double min_;
  double max_;
  double effort_;
  double velocity_;

};

class Joint
{
public:
  virtual ~Joint(void) {};
  bool initXml(TiXmlElement* xml);

  /// returns the name of the joint
  const std::string& getName() const;

  Link* getParentLink() {return this->parent_link_;};
  Link* getChildLink() {return this->child_link_;};

private:
  TiXmlElement* xml_;
  std::string name_;

  enum
  {
    REVOLUTE, PRISMATIC, FLOATING, PLANAR, FIXED
  } type_;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3 axis_;

  Link* parent_link_;
  Pose  parent_pose_;

  Link* child_link_;
  Pose  child_pose_;

  /// Joint Properties
  boost::scoped_ptr<JointProperties> joint_properties_;

  /// Joint Limits
  boost::scoped_ptr<JointLimits> joint_limits_;

  /// \brief A list of maps
  std::vector<TiXmlElement*> maps_;

};

}

#endif
