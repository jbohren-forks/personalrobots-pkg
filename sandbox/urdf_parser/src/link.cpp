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


#include "urdf_parser/link.h"

using namespace std;


namespace urdf_parser{

Link::Link(const std::string &name, Link* parent, 
           TiXmlElement *joint, 
           TiXmlElement *origin,
           TiXmlElement *visual,
           TiXmlElement *collision,
           TiXmlElement *geometry,
           TiXmlElement *inertia)
  :  joint_(joint),
     origin_(origin),
     visual_(visual),
     collision_(collision),
     geometry_(geometry),
     inertia_(inertia),
     name_(name),
     parent_(parent)
{
  if (parent_ != NULL)
    parent_->addChild(this);
}

const std::string& 
Link::getName()
{
  return name_;
}

void 
Link::addChild(Link* child)
{
  children_.push_back(child);
  cout << "added child " << child->getName() << " to " << getName() << endl;
}


bool
Link::getParent(Link& link)
{
  if (parent_ != NULL){
    link = *parent_;
    return true;
  }
  return false;
}


unsigned int 
Link::getNrOfChildren()
{
  return children_.size();
}


bool 
Link::getChild(unsigned int nr, Link& link)
{
  if ((int)nr >=0 && nr <getNrOfChildren()){
    link = *(children_[nr]);
    return true;
  }
  return false;
}

}

