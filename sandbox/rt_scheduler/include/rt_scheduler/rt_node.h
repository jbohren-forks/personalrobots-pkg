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

#ifndef RT_SCHEDULER_RT_NODE_H_
#define RT_SCHEDULER_RT_NODE_H_

#include <string>
#include <vector>
#include <list>
#include <utility>

namespace rt_scheduler
{

class Scheduler ;

class RtNode
{
public:
  RtNode(Scheduler* parent, std::string name) ;
  virtual ~RtNode() { } ;
  virtual bool update() = 0 ;

//  unsigned int generateID()
//  {
//    unsigned int next_id = deps_.size() ;
//    deps_.push_back(NULL) ;
//    printf("%s: Adding port ID #%u\n", name_.c_str(), next_id) ;
//    return next_id ;
//  }

  //! Get the RtNode's fullly scoped name
  std::string getFullName() ;

  void getAncestry(std::vector<RtNode*>& ancestry) const ;
  void getAncestry(std::list<RtNode*>& ancestry) const ;

  //! Get the RtNode's prefix relative to an ancestor. This will be <= the prefix from getPrefix(). Returns "" on error
  std::string getRelativeName(RtNode* parent) ;

  bool addDependency(void* link, RtNode* dep_node) ;

  bool removeDependency(void* link) ;

  std::string name_ ;

  Scheduler* parent_ ;
  //Oracle* oracle_ ;

//protected:
  class DepElem
  {
  public:
    DepElem(void* link, RtNode* node) : link_(link), dep_node_(node) { }
    void* link_ ;
    RtNode* dep_node_ ;
  };
  std::vector< DepElem > deps_ ;
private:
  //std::vector<unsigned int> exec_order_ ;

} ;

}



#endif /* RT_SCHEDULER_RT_NODE_H_ */
