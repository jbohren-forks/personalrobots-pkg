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

#include <sys/stat.h>
#include <errno.h>
#include "ros/ros_slave.h"

class VacuumFlow : public ROS_Flow
{
public:
  FILE *log;
  // an asterisk for the datatype to bypass sanity check on flow connect
  VacuumFlow(string name, string incoming_flowname, string incoming_datatype, string log_dir) : 
    ROS_Flow(name, "*", 1)
  {
    if (incoming_datatype.length() > 0)
    {
      char filename[500];
      snprintf(filename, sizeof(filename),
        "%s/%s_-_%s.dump", log_dir.c_str(), incoming_flowname.c_str(), 
        incoming_datatype.c_str());
      ROS_Slave::ros_slave_instance->log(ROS::INFO, "creating dump of flow [%s] at [%s]", name.c_str(), filename);
      log = fopen(filename, "wb");
      if (!log)
      {
        printf("woah! couldn't create log file\n");
        ROS_Slave::ros_slave_instance->log(ROS::ERROR, "woah! couldn't create log file [%s]", filename);
      }
    }
    else
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
    printf("vacuum hose [%s] received a %d-byte atom\n", 
      name.c_str(), get_atom_len());
    if (log)
    {
      uint32_t atom_len = get_atom_len();
      fwrite(&atom_len, 1, 4, log);
      fwrite(read_ptr, 1, get_atom_len(), log);
    }
  }
  virtual void serialize() { }
};

class Vacuum : public ROS_Slave
{
public:
  string log_dir;

  Vacuum() : ROS_Slave(), log_dir("log")
  {
    register_sink(new VacuumFlow("hose","","",log_dir), ROS_CALLBACK(Vacuum, hose_cb));
    register_with_master();
    get_string_param("dir", log_dir);
    string pkg_dir = get_my_package_path();
    printf("pkg_dir = [%s]\n", pkg_dir.c_str());
    string log_path = pkg_dir + "/" + log_dir;
    int suffix = 1;
    char cstr_suffix[100] = "";
    while (mkdir((log_path + cstr_suffix).c_str(), 0755))
    {
      if (errno != EEXIST)
      {
        log(ROS::ERROR, "woah! couldn't make directory [%s]", 
          (log_path + string(cstr_suffix)).c_str());
        break;
      }
      snprintf(cstr_suffix, sizeof(cstr_suffix), "_%d", suffix);
      if (suffix++ > 100000) // something is really wrong
      {
        log(ROS::ERROR, "woah! I tried 100000 times and still couldn't create the log directory");
        break;
      }
    }
    log_dir = log_path + cstr_suffix;
    printf("log dir = [%s]\n", log_dir.c_str());
    log(ROS::INFO, "using [%s] as the log directory\n", log_dir.c_str());
  }
  virtual ~Vacuum() { }
  void hose_cb() { }
  virtual ROS_Flow *sinkConnectFlow_findFlow(const string &flowname, const string &source)
  {
    vector<ROS_Slave::flow_descrip> graph_flows;
    get_namespace_flows(graph_flows);
    string flow_datatype;
    for (vector<ROS_Slave::flow_descrip>::iterator i = graph_flows.begin(); i != graph_flows.end(); ++i)
    {
      if (i->name == source)
      {
        flow_datatype = i->type;
        log(ROS::INFO, "flow %s has datatype %s", source.c_str(), i->type.c_str());
        break;
      }
    }
    if (!flow_datatype.length())
    {
      log(ROS::WARNING, "couldn't find the datatype for incoming flow [%s]", source.c_str());
      return NULL; // bogus.
    }
    log(ROS::INFO,"vacuum creating flow to receive from [%s]", source.c_str());
    vector<string> splat;
    split(source, splat, ":");
    if (splat.size() != 2)
    {
      log(ROS::ERROR, "vacuum received a source locator which couldn't be split into a NAME:FLOW pair");
      return NULL; // toast
    }
    string sanitized_flowname = splat[0] + string("_-_") + splat[1];
    log(ROS::INFO, "sanitized inflow name: [%s]", sanitized_flowname.c_str());

    split(flow_datatype, splat, "/");
    if (splat.size() != 2)
    {
      log(ROS::ERROR, "vacuum couldn't split an incoming datatype [%s] into a package/flowdef pair",
        flow_datatype.c_str());
      return NULL; // see ya
    }
    string sanitized_datatype = splat[0] + string("_-_") + splat[1];
    log(ROS::INFO, "sanitized datatype: [%s]", sanitized_datatype.c_str());

    VacuumFlow *vac = new VacuumFlow(flowname, sanitized_flowname, sanitized_datatype, log_dir);
    vac->flow_dir = ROS::SINK;
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

