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

#include "rt_scheduler/rt_node.h"

namespace rt_scheduler
{

template<class T>
class OutputPort
{
public:
  OutputPort(RtNode* owner) { owner_ = owner ; }

//private
  T data_ ;
  RtNode* owner_ ;
} ;

template<class T>
class InputPort
{
public:
  InputPort(RtNode* owner)
  {
    owner_ = owner ;
    output_port_ = NULL ;
    id_ = owner_->generateID() ;
  }

  bool getData(T& data)
  {
    if (output_port_)
    {
      data = output_port_->data_ ;
      return true ;
    }
    else
      return false ;
  }

  void Link(OutputPort<T>* output_port)
  {
    output_port_ = output_port ;
    owner_->updateDep(id_, output_port_->owner_) ;
  }

//private
  OutputPort<T>* output_port_ ;
  RtNode* owner_ ;
  unsigned int id_ ;
} ;

}



#endif /* RT_SCHEDULER_PORTS_H_ */
