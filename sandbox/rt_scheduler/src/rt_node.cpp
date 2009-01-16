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

//! \author Vijay Pradeep

#include "rt_scheduler/rt_node.h"
#include "rt_scheduler/scheduler.h"

using namespace std ;
using namespace rt_scheduler ;

RtNode::RtNode(Scheduler* parent, std::string name) : name_(name), parent_(parent)
{
  if (parent_)
    parent_->add(this) ;
}

string RtNode::getFullName()
{
  if (parent_ == NULL)
    return name_ ;
  else
    return parent_->getFullName() + "/" + name_ ;
}


void RtNode::getAncestry(vector<RtNode*>& ancestry) const
{
  ancestry.clear() ;

  vector<RtNode*> rev_ancestry ;

  //rev_ancestry.push_back(this) ;
  RtNode* cur_node = parent_ ;
  while (cur_node != NULL)
  {
    rev_ancestry.push_back(cur_node) ;
    cur_node = cur_node->parent_ ;
  }

  const unsigned int N = rev_ancestry.size() ;

  ancestry.resize(N) ;
  for (unsigned int i=0; i<N; i++)
  {
    ancestry[i] = rev_ancestry[N-i-1] ;
  }
}

void RtNode::getAncestry(std::list<RtNode*>& ancestry) const
{
  ancestry.clear() ;

  RtNode* cur_node = parent_ ;
  while (cur_node != NULL)
  {
    ancestry.push_front(cur_node) ;
    cur_node = cur_node->parent_ ;
  }
}

std::string RtNode::getRelativeName(RtNode* parent)
{
  if (parent == parent_)
    return name_ ;

  if (parent_ == NULL)
    return "" ;

  std::string partial = parent_->getRelativeName(parent) ;
  if (partial == "")
    return "" ;
  return partial + "/" + name_ ;
}

bool RtNode::addDependency(void* link, RtNode* dep_node)
{
  // Make sure we're not double adding a dependency. This could cause data inconsistency
  printf("RtNode::addDependency::Looking for repeat in deps_ (size=%u)\n", deps_.size()) ;
  for (unsigned int i=0; i<deps_.size(); i++)
  {
    if (deps_[i].link_ == link)
      return false ;
  }
  deps_.push_back(DepElem(link, dep_node)) ;
  printf("Adding dep: %s->%s\n", dep_node->name_.c_str(), name_.c_str()) ;
  return true ;
}

bool RtNode::removeDependency(void* link)
{
  for (std::vector<DepElem>::iterator it=deps_.begin(); it<deps_.end(); ++it)
  {
    if ( it->link_ == link)
    {
      deps_.erase(it) ;
      return true ;
    }
  }
  return false ;    // We got through the whole list, but we couldn't find out link. This implies there's some data inconsistency somewhere
}
