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

#include <boost/algorithm/string.hpp>
#include <vector>
#include "rdf_parser/rdf.h"
#include "rdf_parser/rdf_tools.h"

using namespace std;

namespace rdf_parser{


RDF::RDF()
{
  links_.clear();
  joints_.clear();
  root_link_ = NULL;
  link_parent_.clear();
}


bool RDF::initXml(TiXmlElement *robot_xml)
{
  cout << "Parsing robot xml" << endl;
  if (!robot_xml) return false;


  /// parsing strategy:
  ///   1.  get all Joint elements, store in map<name,Joint*>
  ///   2.  get all Link elements, store in map<name,Link*>
  ///   3.  call perspective initXml() to create Link elements and Joint elements

  /// Because old URDF allows us to define Joint elements inside/outside of Link elements, we have to do this
  /// @todo: give deprecation warning and phase out defining Joint elements inside Link elements
  // Get Joint elements defined outside of Link elements
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    Joint* joint = new Joint();
    if (joint->initXml(joint_xml))
    {
      joints_.insert(make_pair(joint->getName(),joint));
      std::cout << "successfully added a new joint (" << joint->getName() << ")" << std::endl;
    }
    else
    {
      std::cerr << "Joint element is a malformed xml" << std::endl;
      delete joint;
    }
  }
  // Get Joint elements defined inside Link elements
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
    for (TiXmlElement* joint_xml = link_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
    {
      Joint* joint = new Joint();
      if (joint->initXml(joint_xml))
      {
        joints_.insert(make_pair(joint->getName(),joint));
        std::cout << "successfully added a new joint (" << joint->getName() << ")" << std::endl;
      }
      else
      {
        std::cerr << "Joint element is either malformed or is referenced inside of Link element but undefined elsewhere." << std::endl;
        delete joint;
      }
    }


  // Get all links
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    std::cout << "new link " << std::endl;
    Link* link = new Link();
    if (link->initXml(link_xml))
    {
      links_.insert(make_pair(link->getName(),link));
      std::cout << "successfully added a new link (" << link->getName() << ")" << std::endl;
    }
    else
    {
      std::cout << "link xml is not initialized correctly" << std::endl;
      delete link;
    }
  }

  // find the root link
  std::string root_name = std::string("not found");
  for (map<string, string>::const_iterator p=link_parent_.begin(); p!=link_parent_.end(); p++){
    if (link_parent_.find(p->second) == link_parent_.end()){
      if (root_name != "not found")
      {
        cerr << "Two root links found: " << root_name << " and " << p->second << endl;
        return false;
      }
      else
        root_name = p->second;
    }
  }
  if (root_name == "not found")
  {cerr << "No root link found. The robot xml contains a graph instead of a tree." << endl; return false;}
  cout << root_name << " is root link " << endl;


  // Start building tree
  //addChildren(links_.find(root_name)->second);

  return true;
}

void RDF::addChildren(Link* p)
{
  // find links that have parent 'p'
  for (map<string, string>::const_iterator c=link_parent_.begin(); c!=link_parent_.end(); c++){
    if (c->second == p->getName()){
      //addChildren(links_.find(c->first)->second);
    }
  }
}


}

