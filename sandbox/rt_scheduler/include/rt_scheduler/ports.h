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

#ifndef RT_SCHEDULER_PORTS_H_
#define RT_SCHEDULER_PORTS_H_

#include <vector>
#include <list>

#include "rt_scheduler/rt_node.h"
#include "rt_scheduler/scheduler.h"

using namespace std ;

namespace rt_scheduler
{

template<class T> class Link ;

// Data generated in a RtNode that needs to be sent somewhere should go in an output port.
template<class T>
class OutputPort
{
public:
  std::vector< Link<T>* > links_ ;
  T data_ ;
  RtNode* owner_ ;

  OutputPort(RtNode* owner) { owner_ = owner ; }

  bool addLink(Link<T>* link)
  {
    // Do some error checking. We shouldn't ever add a link twice
    //std::vector< Link<T>* >::iterator it ;

    for(unsigned int i=0; i<links_.size(); i++)
    {
      if (links_[i] == link)
        return false ;
    }

    links_.push_back(link) ;
    return true ;
  }

  bool removeLink(Link<T>* link)
  {
    //std::vector<Link<T>*>::iterator it ;
    typename std::vector<Link<T>*>::iterator it ;
    for (it=links_.begin(); it<links_.end(); ++it)
    {
      if (*it == link)
      {
        links_.erase(it) ;
        return true ;
      }
    }
    return false ; // We got through the whole list, but we couldn't find out link. Implies some inconsistency
  }


//private
} ;

// RtNodes should instantiate InputPorts when they need to get data from an output port.
template<class T>
class InputPort
{
public:



  InputPort(RtNode* owner)
  {
    owner_ = owner ;
    link_ = NULL ;
  }

  bool getData(T& data)
  {
    if (link_)
      return link_->getData(data) ;
    else
      return false ;
  }

  Link<T>* link_ ;
  RtNode* owner_ ;
} ;


class LinkHandle
{
public:
  LinkHandle() { } ;
  ~LinkHandle() { } ;
  virtual bool breakLink() = 0 ;
} ;

// Links connect output ports to input ports.
template<class T>
class Link : public LinkHandle
{
public:
  OutputPort<T>* out_ ;         // The output port associated with the link
  InputPort<T>* in_ ;           // The input port associated with the link
  RtNode* dep_out_ ;            // The node to which a dependency was added because of the output port
  RtNode* dep_in_ ;             // The node to which a dependency was added because of the input port

  Link() : out_(NULL), in_(NULL), dep_out_(NULL), dep_in_(NULL) { }
  ~Link() { }

  // Grab data from the output port
  bool getData(T& val)
  {
    if (!out_)
      return false ;
    val = out_->data_ ;
    return true ;
  }

  bool makeLink(OutputPort<T>* output_port, InputPort<T>* input_port)
  {
    // Make sure the link hasn't been created already
    if (out_ || in_)
    {
      printf("A\n") ;
      return false ;
    }
    // Make sure that the input port isn't already linked to something
    if (input_port->link_)
    {
      printf("B\n") ;
      return false ;
    }
    // Make the link
    if(!output_port->addLink(this))
    {
      printf("C\n") ;
      return false ;
    }

    input_port ->link_ = this ;
    out_ = output_port ;
    in_ = input_port ;

    // Find and add the dependency
    bool result ;
    result = addDependency(output_port->owner_, input_port->owner_) ;
    return result ;
  }

  bool addDependency(RtNode* out_node, RtNode* in_node)
  {
    // Can't add a dependency to yourself (forms a cycle)
    if (out_node == in_node)
      return false ;
    std::vector<Scheduler*> out_ancestry ;
    std::vector<Scheduler*> in_ancestry ;
      //! \todo REALLY REALLY UGLY. Need to fix this ASAP.
    out_node->getAncestry( *((std::vector<RtNode*>*) &out_ancestry)) ;
    in_node->getAncestry(*((std::vector<RtNode*>*) &in_ancestry)) ;

    //printf("out_ancestry: size=%u\n", out_ancestry.size()) ;
    //printf("in_ancestry: size=%u\n", in_ancestry.size()) ;

    // Both ancestries must be nonzero
    if (out_ancestry.size() == 0 || in_ancestry.size() == 0)
      return false ;

    // Both RtNodes must be in the same tree, otherwise it doesn't make sense to add a dependency
    if (out_ancestry[0] != in_ancestry[0])
      return false ;

    // Find the first difference between the ancestries
    unsigned int i=0 ;
    while(i<out_ancestry.size() &&
          i<in_ancestry.size()  &&
          out_ancestry[i] == in_ancestry[i])
    {
      i++ ;
    }

    // Do a quick sanity check
    if (i == out_ancestry.size() || i == in_ancestry.size())
    {
      if (out_ancestry.size() != in_ancestry.size())
        printf("ports.h: Something is seriously wrong\n") ;
      //assert( out_result ) ;  // Sanity Check
      //assert( in_result ) ;   // Sanity Check
    }


    if (i == out_ancestry.size() && i  == in_ancestry.size())
    {
      // If both ancestries match, then add the original dependency
      dep_out_ = out_node ;
      dep_in_ = in_node ;
    }
    else
    {
      // Find the first in & out scheduler to add this dependency to
      dep_out_ = Scheduler::findNextActive(i, out_ancestry) ;

      if ( dep_out_ == NULL )           // If there is no sub-scheduler, then simply add the original node
        dep_out_ = out_node ;

      dep_in_ = Scheduler::findNextActive(i, (std::vector<Scheduler*>)in_ancestry) ;

      if ( dep_in_ == NULL )
        dep_in_ = in_node ;
    }

    return dep_in_->addDependency(this, dep_out_) ;
  }

  // Removes an exisiting dependency
  bool removeDependency()
  {
    if (dep_out_ == NULL || dep_in_ == NULL)
      return false ;
    return dep_in_->removeDependency(this) ;
  }

  bool breakLink()
  {
    bool result ;
    if (in_ && out_)
      result = removeDependency() ;

    if (!result)
      return false ;

    // Make sure that we have a valid link
    if (in_)
    {
      in_->link_ = NULL ;
      in_ = NULL ;
    }
    if (out_)
    {
      out_->removeLink(this) ;
      out_ = NULL ;
    }
    return true ;
  }

} ;

}



#endif /* RT_SCHEDULER_PORTS_H_ */
