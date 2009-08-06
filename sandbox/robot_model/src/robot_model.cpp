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
#include "robot_model/robot_model.h"

namespace robot_model{


RobotModel::RobotModel()
{
  this->clear();
}

void RobotModel::clear()
{
  name_.clear();
  links_.clear();
  root_link_.reset();
  link_parent_.clear();
}


bool RobotModel::initFile(const std::string& filename)
{
  TiXmlDocument urdf_xml;
  urdf_xml.LoadFile(filename);
  TiXmlElement *robot_xml = urdf_xml.FirstChildElement("robot");
  if (!robot_xml)
  {std::cerr << "Could not parse the xml" << std::endl; return false;}

  return initXml(robot_xml);
}


bool RobotModel::initXml(TiXmlElement *robot_xml)
{
  links_.clear();
  root_link_.reset();
  link_parent_.clear();

  std::cout << "INFO: Parsing robot xml" << std::endl;
  if (!robot_xml) return false;

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    std::cerr << "ERROR: No name given for the robot." << std::endl;
    return false;
  }
  this->name_ = std::string(name);

  // Get all Link elements, connectivity information stored as
  //   parent link name
  //   parent joint name
  //   parent link origin
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link(new Link());

    if (link->initXml(link_xml))
    {
      links_.insert(make_pair(link->name_,link));
      std::cout << "INFO: successfully added a new link (" << link->name_ << ")" << std::endl;
    }
    else
    {
      std::cout << "INFO: link xml is not initialized correctly" << std::endl;
      link.reset();
    }
  }


  // building tree: name mapping
  if (!this->initTree())
  {
    std::cerr << "failed to find build tree" << std::endl;
    return false;
  }
      
  // find the root link
  if (!this->initRoot())
  {
    std::cerr << "failed to find root link" << std::endl;
    return false;
  }

  return true;
}

bool RobotModel::initTree()
{
  // find link/parent mapping
  for (std::map<std::string,boost::shared_ptr<Link> >::iterator link = this->links_.begin();link != this->links_.end(); link++)
  {
    std::string parent_link_name = link->second->parent_joint_->parent_link_name_;
    std::cout << "INFO: build tree: " << link->first << " is a child of " << parent_link_name << std::endl;

    if (parent_link_name.c_str() == NULL)
    {
        std::cerr << "ERROR: parent link name is not valid!" << std::endl;
        return false;
    }
    else
    {

      boost::shared_ptr<Link> parent_link = this->getLink(parent_link_name);

      if (!parent_link)
      {
        if (parent_link_name == "world")
        {
          std::cout << "INFO: parent link: " << parent_link_name << " is a special case." << std::endl;
        }
        else
        {
          std::cerr << "ERROR: parent link: " << parent_link_name << " is not found!" << std::endl;
          return false;
        }
      }
      else
      {
        // fill in child/parent string map
        this->link_parent_[link->second->name_] = parent_link_name;

        // set parent_ link for child Link element
        link->second->setParent(parent_link);

        // add child link in parent Link element
        parent_link->addChild(link->second);
        std::cout << "INFO: now Link: " << parent_link->name_ << " has " << parent_link->child_links_.size() << " children" << std::endl;


      }
    }

  }

  return true;
}



bool RobotModel::initRoot()
{
  std::string root_name = "NoRootSpecified";
  for (std::map<std::string, std::string>::const_iterator p=link_parent_.begin(); p!=link_parent_.end(); p++)
  {
    if (link_parent_.find(p->second) == link_parent_.end())
    {
      if (root_name != "NoRootSpecified")
      {
        std::cout << "INFO: child: " << p->first << " parent: " << p->second << " root: " << root_name << std::endl;
        if (root_name != p->second)
        {
          std::cerr << "ERROR: Two root links found: " << root_name << " and " << p->second << std::endl;
          return false;
        }
      }
      else
        root_name = p->second;
    }
  }
  if (root_name == "NoRootSpecified")
  {std::cerr << "ERROR: No root link found. The robot xml contains a graph instead of a tree." << std::endl; return false;}
  std::cout << "INFO: Link: " << root_name << " is the root Link " << std::endl;
  this->root_link_ = getLink(root_name); // set root link

  return true;
}

const boost::shared_ptr<Link> RobotModel::getLink(const std::string& name) const
{
  if (this->links_.find(name) == this->links_.end())
    return boost::shared_ptr<Link>();
  else
    return this->links_.find(name)->second;
}

}

