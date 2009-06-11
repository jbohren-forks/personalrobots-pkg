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

#include <tinyxml/tinyxml.h>
#include <boost/algorithm/string.hpp>
#include "kdl_parser/tree_parser.hpp"

using namespace std;

namespace KDL{




bool getAtribute(TiXmlElement *xml, const string& name, string& attr)
{
  if (!xml) return false;
  const char *attr_char = xml->Attribute(name.c_str());
  if (!attr_char){
    cout << "No " << name << " found in xml" << endl;
    return false;
  }
  attr = string(attr_char);
  return true;
}


bool getVector(TiXmlElement *vector_xml, const string& field, Vector& vector)
{
  string vector_str;
  if (!getAtribute(vector_xml, field, vector_str))
    return false;

  std::vector<std::string> pieces;
  boost::split( pieces, vector_str, boost::is_any_of(" "));
  unsigned int pos=0;
  for (unsigned int i = 0; i < pieces.size(); ++i){
    if (pieces[i] != ""){
      if (pos < 3)
        vector(pos) = atof(pieces[i].c_str());
      pos++;
    }
  }

  if (pos != 3) {
    cout << "Vector did not contain 3 pieces:" << endl; 
    pos = 1;
    for (unsigned int i = 0; i < pieces.size(); ++i){
      if (pieces[i] != ""){
        cout << "  " << pos << ": '" << pieces[i] << "'" << endl;
        pos++;
      }
    }
    return false;
  }

  return true;
}

bool getValue(TiXmlElement *value_xml, const string& field, double& value)
{
  string value_str;
  if (!getAtribute(value_xml, field, value_str)) return false;

  value = atof(value_str.c_str());

  return true;
}


static bool getFrame(TiXmlElement *frame_xml, Frame& frame)
{
  Vector origin, rpy;
  if (!getVector(frame_xml, "xyz", origin)) 
  {cout << "Frame does not have xyz" << endl; return false;}
  if (!getVector(frame_xml, "rpy", rpy)) 
  {cout << "Frame does not have rpy" << endl; return false;}

  frame = Frame(Rotation::RPY(rpy(0), rpy(1), rpy(2)), origin);
  return true;
}


bool getRotInertia(TiXmlElement *rot_inertia_xml, RotationalInertia& rot_inertia)
{
  double Ixx=0, Iyy=0, Izz=0, Ixy=0, Ixz=0, Iyz=0;
  if (!getValue(rot_inertia_xml, "ixx", Ixx)) return false;
  if (!getValue(rot_inertia_xml, "iyy", Iyy)) return false;
  if (!getValue(rot_inertia_xml, "izz", Izz)) return false;
  if (!getValue(rot_inertia_xml, "ixy", Ixy)) return false;
  if (!getValue(rot_inertia_xml, "ixz", Ixz)) return false;
  if (!getValue(rot_inertia_xml, "iyz", Iyz)) return false;

  rot_inertia = RotationalInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
  return true;
}


bool getInertia(TiXmlElement *inertia_xml, RigidBodyInertia& inertia)
{
  Vector cog;
  if (!getVector(inertia_xml->FirstChildElement("com"), "xyz", cog)) 
  {cout << "Inertia does not specify center of gravity" << endl; return false;}
  double mass = 0.0;
  if (!getValue(inertia_xml->FirstChildElement("mass"), "value", mass)) 
  {cout << "Inertia does not specify mass" << endl; return false;}
  RotationalInertia rot_inertia;
  if (!getRotInertia(inertia_xml->FirstChildElement("inertia"), rot_inertia)) 
  {cout << "Inertia does not specify rotational inertia" << endl; return false;}
  inertia = RigidBodyInertia(mass, cog, rot_inertia);
  return true;
}



bool getJoint(TiXmlElement *joint_xml, Joint& joint)
{
  // get joint type
  string joint_type;
  if (!getAtribute(joint_xml, "type", joint_type)) 
  {cout << "Joint does not have type" << endl; return false;}

  if (joint_type == "revolute"){
    Vector axis, origin;
    // mandatory axis
    if (!getVector(joint_xml->FirstChildElement("axis"), "xyz", axis)) 
    {cout << "Revolute joint does not spacify axis" << endl; return false;}
    // optional origin
    if (!getVector(joint_xml->FirstChildElement("anchor"), "xyz", origin)) 
      origin = Vector::Zero();
    joint = Joint(origin, axis, Joint::RotAxis);
  }
  else if (joint_type == "prismatic"){
    Vector axis, origin;
    // mandatory axis
    if (!getVector(joint_xml->FirstChildElement("axis"), "xyz", axis))
    {cout << "Prismatic joint does not spacify axis" << endl; return false;};
    // optional origin
    if (!getVector(joint_xml->FirstChildElement("anchor"), "xyz", origin)) 
      origin = Vector::Zero();
    joint = Joint(origin, axis, Joint::TransAxis);
  }
  else if (joint_type == "fixed"){
    joint = Joint(Joint::None);
  }
  else{
    cout << "Unknown joint type '" << joint_type << "'. Using fixed joint instead" << endl;
    joint = Joint(Joint::None);
  }

  return true;
}


bool getSegment(TiXmlElement *segment_xml, map<string, Joint>& joints, Segment& segment)
{
  // get mandetory frame
  Frame frame;
  if (!getFrame(segment_xml->FirstChildElement("origin"), frame)) 
  {cout << "Segment does not have origin" << endl; return false;}

  // get mandetory joint
  string joint_name;
  if (!getAtribute(segment_xml->FirstChildElement("joint"), "name", joint_name)) 
  {cout << "Segment does not specify joint name" << endl; return false;}
  map<string, Joint>::iterator it = joints.find(joint_name);
  if (it == joints.end()) 
  {cout << "Could not find joint " << joint_name << " in segment" << endl; return false;}
  Joint joint = it->second;
  if (joint.getType() != Joint::None)
    joint = Joint(frame*(joint.JointOrigin()), joint.JointAxis(), joint.getType());

  // get optional inertia
  RigidBodyInertia inertia(0.0);
  getInertia(segment_xml->FirstChildElement("inertial"), inertia);

  segment = Segment(joint, frame, inertia);
  return true;
}



bool getTree(TiXmlElement *robot, Tree& tree)
{
  // empty tree
  tree = Tree();

  // Constructs the joints
  TiXmlElement *joint_xml = NULL;
  Joint joint;
  map<string, Joint> joints;
  for (joint_xml = robot->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")){
    // get joint name
    string joint_name;
    if (!getAtribute(joint_xml, "name", joint_name)) 
    {cout << "Joint does not have name" << endl; return false;}

    // build joint
    if (!getJoint(joint_xml, joint)) 
    {cout << "Constructing joint " << joint_name << " failed" << endl; return false;}
    joints[joint_name] = joint;
  }

  // Constructs the segments
  TiXmlElement *segment_xml = NULL;
  Segment segment;
  for (segment_xml = robot->FirstChildElement("link"); segment_xml; segment_xml = segment_xml->NextSiblingElement("link")){

    // get segment name
    string segment_name;
    if (!getAtribute(segment_xml, "name", segment_name)) 
    {cout << "Segment does not have name" << endl; return false;}

    // get segment parent
    string segment_parent;
    if (!getAtribute(segment_xml->FirstChildElement("parent"), "name", segment_parent)) 
    {cout << "Segment " << segment_name << " does not have parent" << endl; return false;}
    if (tree.getNrOfSegments() == 0 && segment_parent != "root"){
      cout << "Adding first segment to tree. Changing parent name from " << segment_parent << " to root" << endl;
      segment_parent = "root";
    }

    // build segment
    if (!getSegment(segment_xml, joints, segment)) 
    {cout << "Constructing segment " << segment_name << " failed" << endl; return false;}
    
    // add segment to tree
    if (!tree.addSegment(segment, segment_name, segment_parent))
    {cout << "Failed to add segment to tree" << endl; return false;}

    cout << "Added segment " << segment_name << " to " << segment_parent << endl;
  }
  cout << "Tree has " << tree.getNrOfJoints() << " joints and " << tree.getNrOfSegments() << " segments" << endl;

  return true;
}


bool treeFromFile(const string& file, Tree& tree)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(file);
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  if (!root){
    cout << "Could not parse the xml" << endl;
    return false;
  }
  return getTree(root, tree);
}


bool treeFromString(const string& xml, Tree& tree)
{
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml.c_str());
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  if (!root){
    cout << "Could not parse the xml" << endl;
    return false;
  }
  return getTree(root, tree);
}
}

