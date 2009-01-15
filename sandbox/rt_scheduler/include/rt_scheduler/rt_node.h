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
#include <utility>

namespace rt_scheduler
{

class RtNode
{
public:
  RtNode() { }
  RtNode(std::string name) : name_(name) { } ;
  virtual ~RtNode() { } ;
  virtual bool update() = 0 ;

  unsigned int generateID()
  {
    unsigned int next_id = deps_.size() ;
    deps_.push_back(NULL) ;
    printf("%s: Adding port ID #%u\n", name_.c_str(), next_id) ;
    return next_id ;
  }

  void updateDep(unsigned int id, RtNode* dep_node)
  {
    deps_[id] = dep_node ;
  }

  std::string name_ ;

//private:
  std::vector< RtNode* > deps_ ;
  std::vector<unsigned int> exec_order_ ;

} ;

}



#endif /* RT_SCHEDULER_RT_NODE_H_ */
