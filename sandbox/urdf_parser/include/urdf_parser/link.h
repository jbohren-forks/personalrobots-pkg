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

#ifndef LINK_URDF_PARSER_H
#define LINK_URDF_PARSER_H

#include <string>
#include <vector>
#include <tinyxml/tinyxml.h>

using namespace std;

namespace urdf_parser{

class Link
{
public:
  Link(const std::string &name, 
       Link* parent = NULL, 
       TiXmlElement *joint = NULL, 
       TiXmlElement *origin = NULL,
       TiXmlElement *visual = NULL,
       TiXmlElement *collision = NULL,
       TiXmlElement *geometry = NULL,
       TiXmlElement *inertia = NULL);

  /// returns the name of the link
  const std::string& getName();

  /// returns the parent link. The root link does not have a parent
  bool getParent(Link& link);

  /// returns the number of children of the link
  unsigned int getNrOfChildren();

  /// retuns child 
  bool getChild(unsigned int nr, Link& link);

  /// the xml elements of the link
  TiXmlElement *joint_, *origin_, *visual_, *collision_, *geometry_, *inertia_;

private:
  void addChild(Link* child);

  std::string name_;
  Link* parent_;
  std::vector<Link*> children_;
};

}

#endif
