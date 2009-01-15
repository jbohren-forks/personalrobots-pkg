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
  Talker(int start, int inc) : out_(this)
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
  Adder(Oracle& oracle, std::string name) : rt_scheduler::RtNode(name), in_A_(this),
                                            in_B_(this), out_(this)
  {
    oracle.registerPort(&out_, name+"/out") ;
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

class Doubler : public rt_scheduler::RtNode
{
public:
  Doubler(std::string name) : rt_scheduler::RtNode(name), in_(this)
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

  Talker talker_A(5,2) ;
  Talker talker_B(10,-1) ;
  Adder adder(oracle, "Adder") ;
  Doubler doubler("Doubler") ;

  talker_A.name_ = "TalkerA" ;
  talker_B.name_ = "TalkerB" ;

  // Connect ports together
  adder.in_A_.Link(&talker_A.out_) ;
  adder.in_B_.Link(&talker_B.out_) ;

  OutputPort<int>* port ;
  oracle.find(port, "Adder/out") ;
  if(port != NULL)
    doubler.in_.Link(port) ;
  else
    printf("Oracle could not find port") ;

  // Add to the scheduler
  Scheduler scheduler ;
  scheduler.add(&talker_A) ;
  scheduler.add(&talker_B) ;
  scheduler.add(&adder) ;
  scheduler.add(&doubler) ;

  scheduler.updateExecutionOrder() ;

  while(true)
  {
    scheduler.update() ;
    getchar() ;
  }

  return 0 ;
}
