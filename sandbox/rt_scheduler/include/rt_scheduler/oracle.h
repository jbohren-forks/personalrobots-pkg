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

#ifndef RT_SCHEDULER_ORACLE_H_
#define RT_SCHEDULER_ORACLE_H_

#include <vector>
#include <utility>
#include <string>

#include "rt_scheduler/rt_node.h"
#include "rt_scheduler/ports.h"

namespace rt_scheduler
{

#define REGISTER_RT_TYPE(T) \
  void registerPort(OutputPort<T>* port, const std::string& name) \
  { \
    std::pair< std::string, OutputPort<T>* > entry(name, port) ; \
    ports_out_##T.push_back(entry) ; \
  } \
  \
  void find(OutputPort<T>* &port, const std::string& name) \
  { \
    port = NULL ; \
    for (unsigned int i=0; i < ports_out_##T.size(); i++) \
    { \
      if (ports_out_##T[i].first == name) \
        port = ports_out_##T[i].second ; \
    } \
  } \
  void registerPort(InputPort<T>* port, const std::string& name) \
  { \
    std::pair< std::string, InputPort<T>* > entry(name, port) ; \
    ports_in_##T.push_back(entry) ; \
  } \
  \
  void find(InputPort<T>* &port, const std::string& name) \
  { \
    port = NULL ; \
    for (unsigned int i=0; i < ports_in_##T.size(); i++) \
    { \
      if (ports_in_##T[i].first == name) \
        port = ports_in_##T[i].second ; \
    } \
  } \
  \
  std::vector< std::pair<std::string, OutputPort<T>* > > ports_out_##T ; \
  std::vector< std::pair<std::string, InputPort<T>* > > ports_in_##T

class Oracle
{
public:
  Oracle()
  {  }

  ~Oracle()
  {  }


  LinkHandle* newLink(std::string output_name, std::string input_name, std::string type)
  {
    if(false) ;
    else if (type == "int")
    {
      OutputPort<int>* out ;
      InputPort<int>* in ;
      find(out, output_name) ;
      find(in, input_name) ;
      if (out==NULL || in==NULL)
        return NULL ;
      Link<int>* link = new Link<int> ;
      bool success = link->makeLink(out, in) ;
      if (!success)
      {
        delete link ;
        return NULL ;
      }
      return link ;
    }

    return NULL ;
  }

  bool deleteLink(LinkHandle* &handle)
  {
    if (handle == NULL)
      return false ;
    delete handle ;
    handle = NULL ;
    return true ;
  }


  REGISTER_RT_TYPE(int) ;

private:
} ;

}

#endif /* RT_SCHEDULER_ORACLE_H_ */
