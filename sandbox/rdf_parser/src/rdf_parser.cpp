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
#include "rdf_parser/rdf_parser.h"

using namespace std;

namespace rdf_parser{


Link* RdfParser::getRoot()
{
  map<string, Link*>::iterator root_it = links_.find(root_name_);
  if (root_it != links_.end()){
    return root_it->second;
  }
  return NULL;
}


bool RdfParser::initXml(TiXmlElement *robot_xml)
{
  cout << "Parsing robot xml" << endl;
  if (!robot_xml) return false;

  // get all joints
  map<string, TiXmlElement*> joints;
  findElements(string("joint"), robot_xml, joints, &RdfParser::checkJoint);

  // construct the links
  link_parent.clear();

  //link_origin.clear();
  //link_visual.clear();
  //link_geometry.clear();
  //link_collision.clear();
  //link_inertia.clear();
  //link_joint.clear();


  TiXmlElement *link_xml = NULL;
  for (link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link")){

    // add a new link
    Link* link = new Link();
    link->initXml(link_xml);

    // get link name
    string link_name;
    if (!getAtribute(link_xml, "name", link_name))
    {cerr << "Link does not have name" << endl; return false;}
    if (link_parent.find(link_name) != link_parent.end())
    {cerr << "Link " << link_name << " is specified twice" << endl; return false;}

    // get link parent
    string parent_name;
    if (!getAtribute(link_xml->FirstChildElement("parent"), "name", parent_name))
    {cerr << "Link " << link_name << " does not have parent" << endl; return false;}
    link_parent[link_name] = parent_name;

    // get link origin
    if (!checkOrigin(link_xml->FirstChildElement("origin")))
    {cerr << "Link " << link_name << " has invalid origin" << endl; return false;}
    link_origin[link_name] = link_xml->FirstChildElement("origin");

    // get link visual
    if (link_xml->FirstChildElement("visual")){
      if (!checkGeometry(link_xml->FirstChildElement("visual")))
      {cerr << "Link " << link_name << " has invalid visual" << endl; return false;}
      link_visual[link_name] = link_xml->FirstChildElement("geometry");
    }

    // get link geometry
    if (link_xml->FirstChildElement("geometry")){
      if (!checkGeometry(link_xml->FirstChildElement("geometry")))
      {cerr << "Link " << link_name << " has invalid geometry" << endl; return false;}
      link_geometry[link_name] = link_xml->FirstChildElement("geometry");
    }

    // get link collision
    if (link_xml->FirstChildElement("collision")){
      if (!checkCollision(link_xml->FirstChildElement("collision")))
      {cerr << "Link " << link_name << " has invalid collision" << endl; return false;}
      link_collision[link_name] = link_xml->FirstChildElement("collision");
    }

    // get link inertia
    if (link_xml->FirstChildElement("inertial")){
      if (!checkInertia(link_xml->FirstChildElement("inertial")))
      {cerr << "Link " << link_name << " has invalid inertial" << endl; return false;}
      link_inertia[link_name] = link_xml->FirstChildElement("inertial");
    }

    // get link joint
    string joint_name;
    if (!getAtribute(link_xml->FirstChildElement("joint"), "name", joint_name))
    {cerr << "Link " << link_name << " does not specify joint name" << endl; return false;}
    if (joints.find(joint_name) == joints.end())
    {cerr << "Link " << link_name << " specifies joint " << joint_name << " but this joint was not defined or had invalid syntax" << endl; return false;}
    link_joint[link_name] = joints[joint_name];
  }



  // find the root link
  root_name_="not found";
  for (map<string, string>::const_iterator p=link_parent.begin(); p!=link_parent.end(); p++){
    if (link_parent.find(p->second) == link_parent.end()){
      if (root_name_ != "not found")
      {cerr << "Two root links found: " << root_name_ << " and " << p->second << endl; return false;}
      else
        root_name_ = p->second;
    }
  }
  if (root_name_ == "not found")
  {cerr << "No root link found. The robot xml contains a graph instead of a tree." << endl; return false;}
  cout << root_name_ << " is root link " << endl;


  // Start building tree
  links_.clear();
  joints_.clear();
  links_.insert(make_pair(root_name_, new Link(root_name_)));
  addChildren(links_.find(root_name_)->second);

  return true;
}

void RdfParser::addChildren(Link* p)
{
  // find links that have parent 'p'
  for (map<string, string>::const_iterator c=link_parent.begin(); c!=link_parent.end(); c++){
    if (c->second == p->getName()){
      links_.insert(make_pair(c->first, 
                              new Link(c->first, p, 
                                       link_joint[c->first],
                                       link_origin[c->first],
                                       link_visual[c->first],
                                       link_collision[c->first],
                                       link_geometry[c->first],
                                       link_inertia[c->first])));
      addChildren(links_.find(c->first)->second);
    }
  }
}



bool RdfParser::getLink(TiXmlElement *link_xml, Link& link)
{
  if (!link_xml) return false;
  // get link name
  string link_name;
  if (!getAtribute(link_xml, "name", link_name)) 
  {cerr << "Link does not have name" << endl; return false;}

  // get mandetory frame
  if (!checkFrame(link_xml->FirstChildElement("origin"))) 
  {cerr << "Link does not have origin" << endl; return false;}

  // get mandetory joint 
  string joint_name;
  if (!getAtribute(link_xml->FirstChildElement("joint"), "name", joint_name)) 
  {cerr << "Link does not specify joint name" << endl; return false;}

  // get optional inertia
  checkInertia(link_xml->FirstChildElement("inertial"));

  return true;
}






bool RdfParser::checkNumber(const char& c)
{
  return (c=='1' || c=='2' ||c=='3' ||c=='4' ||c=='5' ||c=='6' ||c=='7' ||c=='8' ||c=='9' ||c=='0' ||c=='.' ||c=='-' ||c==' ');
}

bool RdfParser::checkNumber(const std::string& s)
{
  for (unsigned int i=0; i<s.size(); i++)
    if (!checkNumber(s[i])) return false;
  return true;
}


bool RdfParser::getAtribute(TiXmlElement *xml, const string& name, string& attr)
{
  if (!xml) return false;
  const char *attr_char = xml->Attribute(name.c_str());
  if (!attr_char) return false;
  attr = string(attr_char);
  return true;
}

bool RdfParser::checkVector(TiXmlElement *vector_xml, const string& field)
{
  if (!vector_xml) return false;
  string vector_str;
  if (!getAtribute(vector_xml, field, vector_str))
    return false;

  std::vector<std::string> pieces;
  boost::split( pieces, vector_str, boost::is_any_of(" "));
  unsigned int pos=0;
  for (unsigned int i = 0; i < pieces.size(); ++i){
    if (pieces[i] != ""){
      if (pos < 3){
        if (!checkNumber(pieces[i]))
        {cerr << "This is not a valid number: '" << pieces[i] << "'" << endl; return false;}
      }
      pos++;
    }
  }

  if (pos != 3) {
    cerr << "Vector did not contain 3 pieces:" << endl; 
    pos = 1;
    for (unsigned int i = 0; i < pieces.size(); ++i){
      if (pieces[i] != ""){
        cerr << "  " << pos << ": '" << pieces[i] << "'" << endl;
        pos++;
      }
    }
    return false;
  }

  return true;
}

bool RdfParser::checkValue(TiXmlElement *value_xml, const string& field)
{
  if (!value_xml) return false;
  string value_str;
  if (!getAtribute(value_xml, field, value_str)) return false;

  if (!checkNumber(value_str))
  {cerr << "This is not a valid number: '" << value_str << "'" << endl; return false;}

  return true;
}


bool RdfParser::checkOrigin(TiXmlElement *origin_xml)
{
  if (!origin_xml) return false;

  if (!checkVector(origin_xml, "xyz"))
  {cerr << "Origin does not have xyz" << endl; return false;}
  if (!checkVector(origin_xml, "rpy"))
  {cerr << "Origin does not have rpy" << endl; return false;}

  return true;
}


bool RdfParser::checkFrame(TiXmlElement *frame_xml)
{
  if (!frame_xml) return false;

  if (!checkVector(frame_xml, "xyz")) 
  {cerr << "Frame does not have xyz" << endl; return false;}
  if (!checkVector(frame_xml, "rpy")) 
  {cerr << "Frame does not have rpy" << endl; return false;}

  return true;
}


bool RdfParser::checkRotInertia(TiXmlElement *rot_inertia_xml)
{
  if (!rot_inertia_xml) return false;
  if (!checkValue(rot_inertia_xml, "ixx")) return false;
  if (!checkValue(rot_inertia_xml, "iyy")) return false;
  if (!checkValue(rot_inertia_xml, "izz")) return false;
  if (!checkValue(rot_inertia_xml, "ixy")) return false;
  if (!checkValue(rot_inertia_xml, "ixz")) return false;
  if (!checkValue(rot_inertia_xml, "iyz")) return false;

  return true;
}


bool RdfParser::checkInertia(TiXmlElement *inertia_xml)
{
  if (!inertia_xml) return false;
  if (!checkVector(inertia_xml->FirstChildElement("com"), "xyz")) 
  {cerr << "Inertia does not specify center of gravity" << endl; return false;}
  if (!checkValue(inertia_xml->FirstChildElement("mass"), "value")) 
  {cerr << "Inertia does not specify mass" << endl; return false;}
  if (!checkRotInertia(inertia_xml->FirstChildElement("inertia"))) 
  {cerr << "Inertia does not specify rotational inertia" << endl; return false;}

  return true;
}



bool RdfParser::checkJoint(TiXmlElement *joint_xml, string& joint_name)
{
  if (!joint_xml) return false;
  // get joint name
  if (!getAtribute(joint_xml, "name", joint_name)) 
  {cerr << "Joint does not have name" << endl; joint_name ="name not found"; return false;}

  // get joint type
  string joint_type;
  if (!getAtribute(joint_xml, "type", joint_type)) return false;

  if (joint_type == "revolute"){
    // mandatory axis
    if (!checkVector(joint_xml->FirstChildElement("axis"), "xyz")) 
    {cerr << "Revolute joint does not spacify axis" << endl; return false;}
    // optional origin
    checkVector(joint_xml->FirstChildElement("anchor"), "xyz");
  }
  else if (joint_type == "prismatic"){
    // mandatory axis
    if (!checkVector(joint_xml->FirstChildElement("axis"), "xyz"))
    {cerr << "Prismatic joint does not spacify axis" << endl; return false;};
    // optional origin
    checkVector(joint_xml->FirstChildElement("anchor"), "xyz");
  }
  else if (joint_type == "fixed"){}
  else if (joint_type == "planar"){}
  else{
    cerr << "Unknown joint type '" << joint_type << endl;
  }

  return true;
}



bool RdfParser::checkCollision(TiXmlElement *collision_xml)
{
  if (!collision_xml) return false;

  return true;
}


bool RdfParser::checkGeometry(TiXmlElement *geometry_xml)
{
  if (!geometry_xml) return false;

  return true;
}



bool RdfParser::findElements(const string& element_type, 
                              TiXmlElement* robot_xml, 
                              map<string, TiXmlElement*>& elements, 
                              boost::function<bool (RdfParser*, TiXmlElement*, std::string&)> checkfunction)
{
  TiXmlElement* element_xml = NULL, * link_xml = NULL;

  // Get all joints defined outside of links
  string element_name;
  for (element_xml = robot_xml->FirstChildElement(element_type); element_xml; element_xml = element_xml->NextSiblingElement(element_type)){
    if (checkfunction(this, element_xml, element_name)){
      if (elements.find(element_name) != elements.end())
      {cerr << element_type << " " << element_name << " is defined twice" << endl; return false;}
      elements[element_name] = element_xml;
    }
  }
  // Get all joints defined inside links
  for (link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link")){
    element_xml = link_xml->FirstChildElement(element_type);
    if (checkfunction(this, element_xml, element_name)){
      if (elements.find(element_name) != elements.end())
      {cerr << element_type << " " << element_name << " is defined twice" << endl; return false;}
      elements[element_name] = element_xml;
    }
  }
  return true;
}

}

