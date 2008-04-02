///////////////////////////////////////////////////////////////////////////////
// The ipdcmot package provides a library that talks to the FMOD IP-based 
// motor controller. I just have their single-channel 1.5A box, but perhaps
// some of this code will be useful on their other boxes as well.
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include "ros/ros_slave.h"
#include "ros/ros_flowpair.h"
#include "common_flows/FlowInt32.h"
#include "common_flows/FlowFloat64.h"
#include "common_flows/FlowEmpty.h"

class ServoStepper : public ROS_Slave
{
public:
  FlowFloat64 *setpos_request, *getpos_result;
  FlowEmpty *getpos_request;
  FlowInt32 *setpos_result;
  double start_pos, end_pos, step;
  ROS_FlowPair *setpos_pair, *getpos_pair;
  bool setpos_ok;
  double last_pos;

  ServoStepper() : ROS_Slave()
  {
    register_source(setpos_request = new FlowFloat64("setpos_request"));
    register_source(getpos_request = new FlowEmpty("getpos_request"));
    register_sink(setpos_result = new FlowInt32("setpos_result"), ROS_CALLBACK(ServoStepper, setpos_result_callback));
    register_sink(getpos_result = new FlowFloat64("getpos_result"), ROS_CALLBACK(ServoStepper, getpos_result_callback));
    setpos_pair = new ROS_FlowPair(setpos_request, setpos_result);
    getpos_pair = new ROS_FlowPair(getpos_request, getpos_result);
    // TODO: pull start_pos, end_pos, step from master parameter dictionary
    start_pos = -10;
    end_pos = 10;
    step = 0.05;
  }
  virtual ~ServoStepper() { }
  bool move_motor(double target, double *actual)
  {
    setpos_request->data = target;
    setpos_pair->publish_and_block(30.0);
    if (!setpos_ok)
    {
      printf("setpos not OK\n");
      return false;
    }
    if (actual)
    {
      getpos_pair->publish_and_block(5.0);
      *actual = last_pos;
    }
    return true;
  }
  void step_cycle()
  {
    if (step == 0)
      step = 1;
    if (start_pos < end_pos && step < 0)
      step *= -1;
    else if (start_pos > end_pos && step > 0)
      step *= -1;
    for (double pos = start_pos; pos < end_pos; pos += step)
    {
      double actual;
      if (!move_motor(pos, &actual))
      {
        printf("error moving to %f\n", pos);
        break;
      }
      printf("commanded %f actual %f\n", pos, actual);
    }
  }
  void setpos_result_callback()
  {
    setpos_ok = (setpos_result->data == 1);
  }
  void getpos_result_callback()
  {
    last_pos = getpos_result->data;
  }
};

int main(int argc, char **argv)
{
  ServoStepper s;
  while(1)
  {
    char c = getchar();
    printf("you pressed '%c'\n", c);
    if (c == 'q')
      break;
    if (c == 'g')
    {
      printf("starting a stepper cycle\n");
      s.step_cycle();
    }
  }
  
  return 0;
}

