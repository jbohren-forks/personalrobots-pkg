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
#include "kdl_parser/tree_parser.hpp"

using namespace std;

namespace KDL{




bool getAtribute(TiXmlElement *xml, const string& name, string& attr)
{
  if (!xml) return false;
  const char *attr_char = xml->Attribute(name.c_str());
  if (!attr_char){
    cerr << "No " << name << " found in xml" << endl;
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
  {cerr << "Frame does not have xyz" << endl; return false;}
  if (!getVector(frame_xml, "rpy", rpy)) 
  {cerr << "Frame does not have rpy" << endl; return false;}

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
  {cerr << "Inertia does not specify center of gravity" << endl; return false;}
  double mass = 0.0;
  if (!getValue(inertia_xml->FirstChildElement("mass"), "value", mass)) 
  {cerr << "Inertia does not specify mass" << endl; return false;}
  RotationalInertia rot_inertia;
  if (!getRotInertia(inertia_xml->FirstChildElement("inertia"), rot_inertia)) 
  {cerr << "Inertia does not specify rotational inertia" << endl; return false;}
  inertia = RigidBodyInertia(mass, cog, rot_inertia);
  return true;
}



bool getJoint(TiXmlElement *joint_xml, Joint& joint)
{
  // get joint type
  string joint_type;
  if (!getAtribute(joint_xml, "type", joint_type)) 
  {cerr << "Joint does not have type" << endl; return false;}

  if (joint_type == "revolute"){
    Vector axis, origin;
    // mandatory axis
    if (!getVector(joint_xml->FirstChildElement("axis"), "xyz", axis)) 
    {cerr << "Revolute joint does not spacify axis" << endl; return false;}
    // optional origin
    if (!getVector(joint_xml->FirstChildElement("anchor"), "xyz", origin)) 
      origin = Vector::Zero();
    joint = Joint(origin, axis, Joint::RotAxis);
  }
  else if (joint_type == "prismatic"){
    Vector axis, origin;
    // mandatory axis
    if (!getVector(joint_xml->FirstChildElement("axis"), "xyz", axis))
    {cerr << "Prismatic joint does not spacify axis" << endl; return false;};
    // optional origin
    if (!getVector(joint_xml->FirstChildElement("anchor"), "xyz", origin)) 
      origin = Vector::Zero();
    joint = Joint(origin, axis, Joint::TransAxis);
  }
  else if (joint_type == "fixed"){
    joint = Joint(Joint::None);
  }
  else{
    cerr << "Unknown joint type '" << joint_type << "'. Using fixed joint instead" << endl;
    joint = Joint(Joint::None);
  }

  return true;
}


bool getSegment(TiXmlElement *segment_xml, map<string, Joint>& joints, Segment& segment, string& joint_name)
{
  // get mandetory frame
  Frame frame;
  if (!getFrame(segment_xml->FirstChildElement("origin"), frame)) 
  {cerr << "Segment does not have origin" << endl; return false;}

  // get mandetory joint
  if (!getAtribute(segment_xml->FirstChildElement("joint"), "name", joint_name)) 
  {cerr << "Segment does not specify joint name" << endl; return false;}

  Joint joint;
  map<string, Joint>::iterator it = joints.find(joint_name);
  if (it != joints.end())
    joint = it->second;
  else if (getJoint(segment_xml->FirstChildElement("joint"), joint));
  else {cerr << "Could not find joint " << joint_name << " in segment" << endl; return false;}

  if (joint.getType() != Joint::None)
    joint = Joint(frame*(joint.JointOrigin()), joint.JointAxis(), joint.getType());

  // get optional inertia
  RigidBodyInertia inertia(0.0);
  getInertia(segment_xml->FirstChildElement("inertial"), inertia);

  segment = Segment(joint, frame, inertia);
  return true;
}


void addChildrenToTree(const string& root, const map<string, Segment>& segments, const map<string, string>& parents, Tree& tree)
{
  // add root segments
  if (tree.addSegment(segments.find(root)->second, root, parents.find(root)->second)){
    cout << "Added segment " << root << " to " << parents.find(root)->second << endl;

    // find children
    for (map<string, string>::const_iterator p=parents.begin(); p!=parents.end(); p++)
      if (p->second == root) addChildrenToTree(p->first, segments, parents, tree);
  }
  else {cerr << "Failed to add segment to tree" << endl;}
}


bool getTree(TiXmlElement *robot, Tree& tree, map<string, string>& joint_segment_mapping)
{
  // empty tree
  tree = Tree();
  joint_segment_mapping.clear();

  // Constructs the joints
  TiXmlElement *joint_xml = NULL;
  Joint joint;
  map<string, Joint> joints;
  for (joint_xml = robot->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")){
    // get joint name
    string joint_name;
    if (!getAtribute(joint_xml, "name", joint_name)) 
    {cerr << "Joint does not have name" << endl; return false;}

    // build joint
    if (!getJoint(joint_xml, joint)) 
    {cerr << "Constructing joint " << joint_name << " failed" << endl; return false;}
    joints[joint_name] = joint;
  }

  // Constructs the segments
  TiXmlElement *segment_xml = NULL;
  Segment segment;
  map<string, Segment> segments;
  map<string, string> parents;
  for (segment_xml = robot->FirstChildElement("link"); segment_xml; segment_xml = segment_xml->NextSiblingElement("link")){

    // get segment name
    string segment_name;
    if (!getAtribute(segment_xml, "name", segment_name)) 
    {cerr << "Segment does not have name" << endl; return false;}

    // get segment parent
    string segment_parent;
    if (!getAtribute(segment_xml->FirstChildElement("parent"), "name", segment_parent)) 
    {cerr << "Segment " << segment_name << " does not have parent" << endl; return false;}

    // build segment
    string joint_name;
    if (!getSegment(segment_xml, joints, segment, joint_name)) 
    {cerr << "Constructing segment " << segment_name << " failed" << endl; return false;}
    joint_segment_mapping[joint_name] = segment_name;
    segments[segment_name] = segment;
    parents[segment_name] = segment_parent;
  }

  // fail if no segments were found
  if (segments.empty()){
    cerr << "Did not find any segments" << endl;
    return false;
  }
  
  // find the root segment: the parent segment that does not exist
  string root;
  for (map<string, string>::const_iterator p=parents.begin(); p!=parents.end(); p++){
    for (map<string, Segment>::const_iterator s=segments.begin(); s!=segments.end(); s++){
      if (p->second == s->first) break;
      if (s == --(segments.end())) root = p->first;
    }
  }
  cout << root << " is root segment " << endl;
  cout << "renaming parent of root segment '" << root << "' from " << parents[root] << " to root" << endl;
  parents[root] = "root";

  // add all segments to tree
  addChildrenToTree(root, segments, parents, tree);

  cout << "Tree has " << tree.getNrOfJoints() << " joints and " << tree.getNrOfSegments() << " segments" << endl;

  return true;
}


bool treeFromFile(const string& file, Tree& tree, map<string, string>& joint_segment_mapping)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(file);
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  if (!root){
    cerr << "Could not parse the xml" << endl;
    return false;
  }
  return getTree(root, tree, joint_segment_mapping);
}


bool treeFromString(const string& xml, Tree& tree, map<string, string>& joint_segment_mapping)
{
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml.c_str());
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  if (!root){
    cerr << "Could not parse the xml" << endl;
    return false;
  }
  return getTree(root, tree, joint_segment_mapping);
}


bool treeFromXml(TiXmlElement *root, Tree& tree, map<string, string>& joint_segment_mapping)
{
  return getTree(root, tree, joint_segment_mapping);
}

}

