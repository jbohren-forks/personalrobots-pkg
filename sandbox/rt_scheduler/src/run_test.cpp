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

#include <cstdio>

#include "rt_scheduler/rt_node.h"
#include "rt_scheduler/ports.h"
#include "rt_scheduler/scheduler.h"
#include "rt_scheduler/oracle.h"
using namespace rt_scheduler ;

class Talker : public rt_scheduler::RtNode
{
public:
  Talker(Scheduler* parent, std::string name, Oracle* oracle, int start, int inc) : RtNode(parent, name), out_(this)
  {
    state_ = start ;
    inc_ = inc ;
  }

  ~Talker()
  { }

  bool update()
  {
    state_ += inc_ ;
    out_.data_ = state_ ;
    printf("%s: out = %i\n", name_.c_str(), out_.data_) ;
    return true ;
  }

  OutputPort<int> out_ ;
  int state_ ;
  int inc_ ;
} ;

class Adder : public rt_scheduler::RtNode
{
public:
  Adder(Scheduler* parent, std::string name, Oracle* oracle) : RtNode(parent, name), in_A_(this),
                                                                in_B_(this), out_(this)
  {
    //oracle.registerPort(&out_, name+"/out") ;
  }

  ~Adder()
  { }

  bool update()
  {
    int a,b ;
    if (in_A_.getData(a))
      printf("  Got data from Port A: %i\n", a) ;
    else
    {
      printf("  Error accessing Port A\n") ;
      return false ;
    }
    if (in_B_.getData(b))
      printf("  Got data from Port B: %i\n", b) ;
    else
    {
      printf("  Error accessing Port B\n") ;
      return false ;
    }

    int result = a+b ;

    out_.data_ = result ;

    printf("  Result: %i\n", result) ;

    return true ;
  }

  InputPort<int> in_A_ ;
  InputPort<int> in_B_ ;
  OutputPort<int> out_ ;
} ;

class Doubler : public RtNode
{
public:
  Doubler(Scheduler* parent, std::string name) : RtNode(parent, name), in_(this)
  {

  }

  ~Doubler()
  { }

  bool update()
  {
    int a ;
    if (in_.getData(a))
      printf("  Got data from Port: %i\n", a) ;
    else
    {
      printf("  Error accessing Port\n") ;
      return false ;
    }

    int result = 2*a ;


    printf("  Result: %i\n", result) ;

    return true ;
  }

  InputPort<int> in_ ;
} ;


int main(int argc, char** argv)
{
  // Initialize
  Oracle oracle ;
  Scheduler controllers(NULL, "controllers") ;
  Scheduler group_A(&controllers, "Group_A") ;
  group_A.active_ = false ;

  Talker talker_A(&group_A, string("talkerA"), &oracle, 5,2) ;
  Talker talker_B(&group_A, string("talkerB"), &oracle, 10,-1) ;
  Adder adder_A(&group_A, string("AdderA"), &oracle) ;

  Talker talker_C(&controllers, string("talkerC"), &oracle, 0,100) ;
  Adder adder_B(&controllers, string("AdderB"), &oracle) ;
  Doubler doubler(&controllers, string("Doubler")) ;

  // Connect ports together
  Link<int> link_1, link_2, link_3 ;
  Link<int> link_4, link_5 ;

  printf("Atempting link1...\n") ;
  if(!link_1.makeLink(&talker_A.out_, &adder_A.in_A_))
    printf("Error making link 1\n") ;
  else
    printf("Success\n") ;

  printf("Atempting link2...\n") ;
  if(!link_2.makeLink(&talker_B.out_, &adder_A.in_B_))
    printf("Error making link 2\n") ;
  else
    printf("Success\n") ;

  printf("Atempting link3...\n") ;
  if(!link_3.makeLink(&adder_A.out_, &adder_B.in_A_))
    printf("Error making link 3\n") ;
  else
    printf("Success\n") ;

  if(!link_4.makeLink(&talker_C.out_, &adder_B.in_B_))
    printf("Error making link 4\n") ;
  if(!link_5.makeLink(&adder_B.out_, &doubler.in_))
    printf("Error making link 5\n") ;

  controllers.updateExecutionOrder() ;
  group_A.updateExecutionOrder() ;

  while(true)
  {
    controllers.update() ;
    getchar() ;
  }

  return 0 ;
}
