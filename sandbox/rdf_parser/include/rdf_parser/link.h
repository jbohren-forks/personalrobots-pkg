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

#include <rdf_parser/joint.h>
#include <rdf_parser/geometry.h>

using namespace std;

namespace rdf_parser{

class Link
{
public:

  bool initXml(TiXmlElement* xml);

  /// returns the name of the link
  const std::string& getName() const;

  /// returns the parent link. The root link does not have a parent
  Link* getParent();

  /// returns children of the link
  std::vector<Link*> getChildren();

  /// returns joint attaching link to parent
  Joint* getParentJoint();

  /// returns joints attaching link to children
  std::vector<Joint*> getChildrenJoint();

  class Inertial
  {
  public:
    virtual ~Inertial(void) {};
    bool initXml(TiXmlElement* xml);
  private:
    std::vector<TiXmlElement*> maps_;
    Pose origin_;
    double mass_;
    double ixx_,ixy_,ixz_,iyy_,iyz_,izz_;
  };

  class Visual
  {
  public:
    virtual ~Visual(void) {};
    bool initXml(TiXmlElement* xml);
  private:
    std::vector<TiXmlElement*> maps_;
    Pose origin_;
    Geometry geometry_;

  };

  class Collision
  {
  public:
    virtual ~Collision(void) {};
    bool initXml(TiXmlElement* xml);
  private:
    std::vector<TiXmlElement*> maps_;
    Pose origin_;
    Geometry geometry_;

  };


private:
  std::string name_;
  std::vector<TiXmlElement*> maps_;

  Link* parent_;
  std::vector<Link*> children_;

  std::vector<Joint*> joints_;

  Joint* parent_joint_;
  std::vector<Joint*> children_joint_;

};




}

#endif
