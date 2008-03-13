///////////////////////////////////////////////////////////////////////////////
// The vacuum package provides a catch-all node that will write any ROS stream
// to a file. It tries to be smart about some types of streams.
//
// Copyright (C) 2008  Morgan Quigley
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

class VacuumFlow : public ROS_Flow
{
public:
  FILE *log;
  // uses an empty datatype string to bypass sanity check on flow connect
  VacuumFlow(string name) : ROS_Flow(name, "", 1)
  {
    log = NULL;
  }
  ~VacuumFlow()
  {
    if (log)
      fclose(log);
    log = NULL;
  }
  virtual void deserialize()
  {
    printf("vacuum received a %d-byte atom\n", get_atom_len());
  }
  virtual void serialize() { }
};

class Vacuum : public ROS_Slave
{
public:
  Vacuum() : ROS_Slave()
  {
    register_with_master();
  }
  virtual ~Vacuum() { }
  void hose_cb() { }
  virtual ROS_Flow *sinkConnectFlow_findFlow(string flowname)
  {
    printf("vacuum creating flow for [%s]\n", flowname.c_str());
    VacuumFlow *vac = new VacuumFlow(flowname);
    vac->flow_dir = ROS_Flow::SINK;
    add_flow(vac);
    return vac;
  }
};

int main(int argc, char **argv)
{
  Vacuum v;
  v.spin();
  return 0;
}

