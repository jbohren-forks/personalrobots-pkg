/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#include "mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"

namespace mechanism {

bool Robot::initXml(TiXmlElement *root)
{
  TiXmlElement *xit = NULL;

  // Constructs the joints.
  for (xit = root->FirstChildElement("joint"); xit;
       xit = xit->NextSiblingElement("joint"))
  {
    Joint *j = new Joint;
    if (j->initXml(xit))
      joints_.push_back(j);
    else
      delete j;
  }

  // Constructs the transmissions.
  for (xit = root->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    const char *type = xit->Attribute("type");
    Transmission *t = type ? TransmissionFactory::instance().create(type) : NULL;
    if (!t)
      fprintf(stderr, "Unknown transmission type: \"%s\"\n", type);
    else if (!t->initXml(xit, this))
      delete t;
    else // Success!
      transmissions_.push_back(t);
  }

  // Constructs the links.
  for (xit = root->FirstChildElement("link"); xit;
       xit = xit->NextSiblingElement("link"))
  {
    Link *link = new Link;
    if (link->initXml(xit, this))
      links_.push_back(link);
    else
      delete link;
  }
  for (unsigned int i = 0; i < links_.size(); ++i)
  {
    links_[i]->createTreePointers(this);
  }
  printLinkTree();

#if 0
  // Constructs the kinematic chains.
  for (xit = root->FirstChildElement("chain"); xit;
       xit = xit->NextSiblingElement("chain"))
  {
    kinematics::Chain *c = new kinematics::Chain;
    if (c->initXml(xit, this))
      chains_.push_back(c);
    else
      delete c;
  }
#endif

  return true;
}

template <class T>
T* findByName(std::vector<T*>& v, const std::string &name)
{
  typename std::vector<T*>::iterator it;
  for (it = v.begin(); it != v.end(); ++it)
  {
    if ((*it) && (*it)->name_ == name)
      return *it;
  }
  return NULL;
}

Joint* Robot::getJoint(const std::string &name)
{
  // Yeah, it's a linear search.  Deal with it.
  return findByName(joints_, name);
}

Actuator* Robot::getActuator(const std::string &name)
{
  // Yeah, it's a linear search.  Deal with it.
  return findByName(hw_->actuators_, name);
}

Link* Robot::getLink(const std::string &name)
{
  return findByName(links_, name);
}

void printLinkTreeHelper(Link *link, int depth = 0)
{
  for (int i = 0; i < depth; ++i)
    printf("  ");

  for (unsigned int i = 0; i < link->children_.size(); ++i)
    printLinkTreeHelper(link->children_[i], depth + 1);
}
void Robot::printLinkTree()
{
  Link *root = NULL;
  for (unsigned int i = 0; i < links_.size(); ++i)
  {
    if (links_[i]->parent_name_ == "world")
    {
      root = links_[i];
      break;
    }
  }
  if (!root)
  {
    fprintf(stderr, "Could not print the link tree because there's no link connected to the world\n");
    return;
  }

  printLinkTreeHelper(root, 0);
}

} // namespace mechanism
