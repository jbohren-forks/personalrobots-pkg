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
  cout << "INFO: Parsing robot xml" << endl;
  if (!robot_xml) return false;


  /// current parsing strategy:
  ///   1.  get all Joint elements, store in map<name,Joint*>
  ///   2.  get all Link elements, store in map<name,Link*>
  ///   3.  build tree - fill in pointers for parents, children, etc
  ///
  ///
  /// When we switch to the new RDF structure, we'll have to change the steps to below:
  ///   1.  get all Link elements, store in map<nam,Link*>
  ///   2.  get all Joint elements, store in map<nam,Joint*> AND
  ///         fill in parent/child information in Joint::initXml() for both Joint and Link elements.
  ///



  /// Because old URDF allows us to define Joint elements inside/outside of Link elements, we have to do this
  /// @todo: give deprecation warning and phase out defining Joint elements inside Link elements
  // Get Joint elements defined outside of Link elements
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    Joint* joint = new Joint();
    if (joint->initXml(joint_xml))
    {
      joints_.insert(make_pair(joint->getName(),joint));
      std::cout << "INFO: successfully added a new joint (" << joint->getName() << ")" << std::endl;
    }
    else
    {
      std::cerr << "ERROR: Joint element is a malformed xml" << std::endl;
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
        std::cout << "INFO: successfully added a new joint (" << joint->getName() << ")" << std::endl;
      }
      else
      {
        if (this->joints_.find(joint->getName()) == this->joints_.end())
        {
          const char* link_name = link_xml->Attribute("name");
          std::cerr << "ERROR: Joint: " << joint->getName() << " is not defined anywhere, only referenced by name by Link: " << link_name << std::endl;
        }
        else
        {
          std::cout << "INFO: Joint: " << joint->getName() << " is already loaded." << std::endl;
        }
        delete joint;
      }
    }

  // Get all Link elements, connectivity information stored as
  //   parent link name
  //   parent joint name
  //   parent link origin
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    Link* link = new Link();
    if (link->initXml(link_xml))
    {
      links_.insert(make_pair(link->getName(),link));
      std::cout << "INFO: successfully added a new link (" << link->getName() << ")" << std::endl;
    }
    else
    {
      std::cout << "INFO: link xml is not initialized correctly" << std::endl;
      delete link;
    }
  }


  // Start building tree
  // find link/parent mapping
  for (std::map<std::string,Link*>::iterator link = this->links_.begin();link != this->links_.end(); link++)
  {
    std::string parent_name = link->second->getParentName();
    std::cout << "INFO: build tree: " << link->first << " is a child of " << parent_name << std::endl;

    Link* parent_link = this->getLink(parent_name);
    if (!parent_link)
    {
      if (parent_name == "world")
        std::cout << "INFO: parent link: " << parent_name << " is a special case." << std::endl;
      else
      {
        std::cerr << "ERROR: parent link: " << parent_name << " is not found!" << std::endl;
        return false;
      }
    }
    else
    {
      // fill in child/parent string map
      this->link_parent_[link->second->getName()] = parent_name;

      // set parent_ link for child Link element
      link->second->setParent( parent_link);

      // add child link in parent Link element
      parent_link->addChild(link->second);
      //std::cout << "INFO: now Link: " << parent_link->getName() << " has " << parent_link->getChildren()->size() << " children" << std::endl;


      //--------------------------------------------
      // setup parent Joint element for Link element
      //--------------------------------------------
      std::map<std::string,Joint*>::iterator parent_joint = this->joints_.find(link->second->getParentJointName());
      if (parent_joint == this->joints_.end())
      {
        std::cerr << "ERROR: link: " << link->first << " parent joint: " << link->second->getParentJointName()<< " not found in joint list" << std::endl;
        return false;
      }
      else
      {
        // set parent Joint for Link element
        link->second->setParentJoint(parent_joint->second);


        //-------------------------------------------------------
        // setup connectivity information for parent Link element
        //-------------------------------------------------------
        // add links to parent Link element for Joint elements
        parent_joint->second->setParentLink(parent_link);

        // set transform from parent Link to parent Joint frame for Link element
        Pose parent_pose;
        parent_pose.initXml(link->second->getOriginXml());
        parent_joint->second->setParentPose(parent_pose);

        // since we're using old URDF format, set origins for Joint elements using origin_xml_ in links
        parent_joint->second->setChildLink(link->second);

        parent_joint->second->setChildPose(Pose());
      }
    }

  }
      
  // find the root link
  std::string* root_name = NULL;
  for (map<string, string>::const_iterator p=link_parent_.begin(); p!=link_parent_.end(); p++)
  {
    if (link_parent_.find(p->second) == link_parent_.end())
    {
      if (root_name)
      {
        std::cout << "INFO: child: " << p->first << " parent: " << p->second << " root: " << *root_name << std::endl;
        if (*root_name != p->second)
        {
          cerr << "ERROR: Two root links found: " << *root_name << " and " << p->second << endl;
          return false;
        }
      }
      else
        root_name = &(std::string)(p->second);
    }
  }
  if (root_name == NULL)
  {cerr << "ERROR: No root link found. The robot xml contains a graph instead of a tree." << endl; return false;}
  cout << "INFO: Link: " << *root_name << " is the root Link " << endl;
  this->root_link_ = getLink(*root_name); // set root link

  // Get Maps
  for (TiXmlElement* map_xml = robot_xml->FirstChildElement("map"); map_xml; map_xml = map_xml->NextSiblingElement("map"))
    this->maps_.push_back(map_xml);

  return true;
}

void RDF::addChildren(Link* p)
{
  // find links that have parent 'p'
  for (map<string, string>::const_iterator c=link_parent_.begin(); c!=link_parent_.end(); c++)
  {
    if (c->second == p->getName())
    {
      //addChildren(links_.find(c->first)->second);
    }
  }
}

Link* RDF::getLink(const std::string& name)
{
  if (this->links_.find(name) == this->links_.end())
    return NULL;
  else
    return this->links_.find(name)->second;
}

}

