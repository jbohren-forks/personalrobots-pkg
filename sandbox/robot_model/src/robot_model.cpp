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
#include <ros/ros.h>
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
  joints_.clear();
  root_link_.reset();
}


bool RobotModel::initFile(const std::string& filename)
{
  TiXmlDocument xml_doc;
  xml_doc.LoadFile(filename);

  return initXml(&xml_doc);
}


bool RobotModel::initString(const std::string& xml_string)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  return initXml(&xml_doc);
}


bool RobotModel::initXml(TiXmlDocument *xml_doc)
{
  if (!xml_doc)
  {
    ROS_ERROR("Could not parse the xml");
    return false;
  }

  TiXmlElement *robot_xml = xml_doc->FirstChildElement("robot");
  if (!robot_xml)
  {
    ROS_ERROR("Could not find the 'robot' element in the xml file");
    return false;
  }
  return initXml(robot_xml);
}

bool RobotModel::initXml(TiXmlElement *robot_xml)
{
  this->clear();

  ROS_DEBUG("Parsing robot xml");
  if (!robot_xml) return false;

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    ROS_ERROR("No name given for the robot.");
    return false;
  }
  this->name_ = std::string(name);

  // Get all Link elements, connectivity information stored as
  //   parent link name
  //   parent joint name
  //   parent link origin
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (link->initXml(link_xml))
    {
      if (this->getLink(link->name))
      {
        ROS_ERROR("link '%s' is not unique.", link->name.c_str());
        link.reset();
        return false;
      }
      else
      {
        links_.insert(make_pair(link->name,link));
        ROS_DEBUG("successfully added a new link '%s'", link->name.c_str());
        // add list of joints to a vector
        if (link->parent_joint)
        {
          joints_.insert(make_pair(link->parent_joint->name,link->parent_joint));
          ROS_DEBUG("successfully added a new joint '%s'", link->parent_joint->name.c_str());
        }
      }
    }
    else
    {
      ROS_ERROR("link xml is not initialized correctly");
      link.reset();
    }
  }


  /// for convenience keep a map of link names and their parent names
  std::map<std::string, std::string> link_tree;
  link_tree.clear();

  // building tree: name mapping
  if (!this->initTree(link_tree))
  {
    ROS_ERROR("failed to build tree");
    return false;
  }
      
  // find the root link
  if (!this->initRoot(link_tree))
  {
    ROS_ERROR("failed to find root link");
    return false;
  }

  return true;
}

bool RobotModel::initTree(std::map<std::string, std::string> &link_tree)
{
  // find link/parent mapping
  for (std::map<std::string,boost::shared_ptr<Link> >::iterator link = this->links_.begin();link != this->links_.end(); link++)
  {
    std::string parent_link_name = link->second->parent_joint->parent_link_name;
    ROS_DEBUG("build tree: '%s' is a child of '%s'", link->first.c_str(), parent_link_name.c_str());

    if (parent_link_name.c_str() == NULL)
    {
      ROS_ERROR("parent link name is not valid!");
      return false;
    }
    else
    {

      boost::shared_ptr<Link> parent_link;
      this->getLink(parent_link_name, parent_link);

      if (!parent_link)
      {
        if (parent_link_name == "world")
        {
          ROS_DEBUG("parent link '%s' is a special case", parent_link_name.c_str());
        }
        else
        {
          ROS_ERROR("parent link '%s' is not found", parent_link_name.c_str());
          return false;
        }
      }
      else
      {
        // fill in child/parent string map
        link_tree[link->second->name] = parent_link_name;

        // set parent_ link for child Link element
        link->second->setParent(parent_link);

        // add child link in parent Link element
        parent_link->addChild(link->second);
        ROS_DEBUG("now Link '%s' has %i children ", parent_link->name.c_str(), parent_link->child_links.size());
      }
    }

  }

  return true;
}



bool RobotModel::initRoot(std::map<std::string, std::string> &link_tree)
{
  std::string root_name = "NoRootSpecified";
  for (std::map<std::string, std::string>::iterator p=link_tree.begin(); p!=link_tree.end(); p++)
  {
    if (link_tree.find(p->second) == link_tree.end())
    {
      if (root_name != "NoRootSpecified")
      {
        ROS_DEBUG("child '%s', parent '%s', root '%s'", p->first.c_str(), p->second.c_str(), root_name.c_str());
        if (root_name != p->second)
        {
          ROS_ERROR("Two root links found: '%s' and '%s'", root_name.c_str(), p->second.c_str());
          return false;
        }
      }
      else
        root_name = p->second;
    }
  }
  if (root_name == "NoRootSpecified")
  {
    ROS_ERROR("No root link found. The robot xml contains a graph instead of a tree.");
    return false;
  }
  ROS_DEBUG("Link '%s' is the root link", root_name.c_str());
  getLink(root_name,this->root_link_); // set root link

  return true;
}

boost::shared_ptr<const Link> RobotModel::getLink(const std::string& name) const
{
  boost::shared_ptr<const Link> ptr;
  if (this->links_.find(name) == this->links_.end())
    ptr.reset();
  else
    ptr = this->links_.find(name)->second;
  return ptr;
}

void RobotModel::getLink(const std::string& name,boost::shared_ptr<Link> &link) const
{
  boost::shared_ptr<Link> ptr;
  if (this->links_.find(name) == this->links_.end())
    ptr.reset();
  else
    ptr = this->links_.find(name)->second;
  link = ptr;
}

boost::shared_ptr<const Joint> RobotModel::getJoint(const std::string& name) const
{
  boost::shared_ptr<const Joint> ptr;
  if (this->joints_.find(name) == this->joints_.end())
    ptr.reset();
  else
    ptr = this->joints_.find(name)->second;
  return ptr;
}

}

