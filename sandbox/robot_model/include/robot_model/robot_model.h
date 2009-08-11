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

#ifndef ROBOT_MODEL_PARSER_H
#define ROBOT_MODEL_PARSER_H

#include <string>
#include <map>
#include <tinyxml/tinyxml.h>
#include <boost/function.hpp>
#include "robot_model/link.h"


namespace robot_model{


class RobotModel
{
public:
  RobotModel();

  bool initXml(TiXmlElement *xml);
  bool initFile(const std::string& filename);
  bool initString(const std::string& xmlstring);

  const boost::shared_ptr<Link> getRoot(void) const{return (const boost::shared_ptr<Link>)this->root_link_;};
  const boost::shared_ptr<Link> getLink(const std::string& name) const;
  const std::string& getName() const {return name_;};

private:
  void clear();

  std::string name_;

  /// in initXml(), onece all links are loaded,
  /// it's time to build a tree
  bool initTree();

  /// in initXml(), onece tree is built,
  /// it's time to find the root Link
  bool initRoot();

  /// Every Robot Description File can be described as a
  ///   list of Links and Joints
  /// The connection between links(nodes) and joints(edges)
  ///   should define a tree (i.e. 1 parent link, 0+ children links)
  /// RobotModel currently do not support 
  std::map<std::string, boost::shared_ptr<Link> > links_;

  /// RobotModel is restricted to a tree for now, which means there exists one root link
  ///  typically, root link is the world(inertial).  Where world is a special link
  /// or is the root_link_ the link attached to the world by PLANAR/FLOATING joint?
  ///  hmm...
  boost::shared_ptr<Link> root_link_;

  /// for convenience keep a map of link names and their parent names
  std::map<std::string, std::string> link_parent_;

};

}

#endif
