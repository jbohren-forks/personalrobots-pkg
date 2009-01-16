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

#ifndef RT_SCHEDULER_SCHEDULER_H_
#define RT_SCHEDULER_SCHEDULER_H_



#include <vector>
#include <utility>

#include "rt_scheduler/rt_node.h"



namespace rt_scheduler
{

class Scheduler : public rt_scheduler::RtNode
{
public:

  //Scheduler(Scheduler* parent, Oracle* oracle, std::string name) : RtNode(parent, oracle, name)
  Scheduler(Scheduler* parent, std::string name) : RtNode(parent, name)
  {
    active_ = true ;
  }

  ~Scheduler()
  {  }

  void add(RtNode* node)
  {
    if (active_)
    {
      std::string rel_name = node->getRelativeName(this) ;
      node_list_.push_back(NodeListElem(rel_name, node )) ;
      printf("Adding %s to scheduler %s\n", rel_name.c_str(), name_.c_str() ) ;
    }
    else
      parent_->add(node) ;
  }

  bool update()
  {
    if (active_)
    {
      bool success = true ;
      for (unsigned int i=0; i < exec_order_.size(); i++)
      {
        printf("Updating Node #%u: %s\n", exec_order_[i], node_list_[exec_order_[i]].node_->name_.c_str()) ;
        success = success && node_list_[exec_order_[i]].node_->update() ;
      }
      return success ;
    }
    return true ;
  }

  // A super dumb, dangerous scheduler
  void updateExecutionOrder()
  {
    printf("Dependencies:\n") ;
    for (unsigned int i=0; i<node_list_.size(); i++)
    {
      for (unsigned int j=0; j<node_list_[i].node_->deps_.size(); j++)
        printf("  %s -> %s\n", node_list_[i].node_->name_.c_str(), node_list_[i].node_->deps_[j].dep_node_->name_.c_str()) ;
    }

    exec_order_.resize(node_list_.size()) ;
    for (unsigned int i=0; i<exec_order_.size(); i++)
      exec_order_[i] = i ;
  }

  /*static Scheduler* findNextActive(std::list<Scheduler*>::iterator it, const std::list<Scheduler*>::iterator& end)
  {
    while( it!=end && !(*it)->active_ )
      ++it ;
    if (it == end)
      return NULL ;
    else
      return *it ;
  }*/

  static Scheduler* findNextActive(unsigned int start, const std::vector<Scheduler*>& scheduler_list)
  {
    unsigned int i=start ;
    while( i < scheduler_list.size() && !scheduler_list[i]->active_ )
      i++ ;
    if (i == scheduler_list.size())
      return NULL ;
    else
      return scheduler_list[i] ;
  }

  bool active_ ;
private:
  class NodeListElem
  {
  public:
    NodeListElem(std::string name, RtNode* node) : name_(name), node_(node) { } ;
    std::string name_ ;
    RtNode* node_ ;
  } ;

  std::vector<NodeListElem> node_list_ ;
  std::vector<unsigned int> exec_order_ ;
} ;

}

#endif /* RT_SCHEDULER_SCHEDULER_H_ */
