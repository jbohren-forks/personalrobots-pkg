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

#ifndef RDF_PARSER_H
#define RDF_PARSER_H

#include <string>
#include <map>
#include <tinyxml/tinyxml.h>
#include <boost/function.hpp>
#include "rdf_parser/link.h"

using namespace std;

namespace rdf_parser{

class RdfParser
{
public:
  RdfParser(){};

  bool initXml(TiXmlElement *xml);
  Link* getRoot();

private:
  bool checkRotInertia(TiXmlElement *rot_inertia_xml);
  bool checkInertia(TiXmlElement *inertia_xml);
  bool checkJoint(TiXmlElement *joint_xml, std::string& joint_name);
  bool checkCollision(TiXmlElement *collision_xml);
  bool checkGeometry(TiXmlElement *geometry_xml);
  bool getLink(TiXmlElement *link_xml, Link& link);
  void addChildren(Link* p);
  bool findElements(const std::string& element_type, 
                    TiXmlElement* robot_xml, 
                    std::map<std::string, TiXmlElement*>& elements, 
                    boost::function<bool (RdfParser*, TiXmlElement*, std::string&)> checkfunction);


  std::map<std::string, Link*> links_;
  std::map<std::string, Joint*> joints_;

  std::string root_name_;

  /// keep a map of link names and their parent names
  map<string, string> link_parent;

};

}

#endif
