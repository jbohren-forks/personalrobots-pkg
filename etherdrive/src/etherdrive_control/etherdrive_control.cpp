///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
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
#include "unstable_flows/FlowActuator.h"
#include "etherdrive/etherdrive.h"
#include <sstream>

class EtherDrive_Control : public ROS_Slave
{
public:
  FlowActuator* mot[6];
  FlowActuator* mot_cmd[6];

  string host;
  EtherDrive *ed;
  int frame_id;

  float last_mot_val[6];
  double pulse_per_rad[6];

  ROS_Mutex control_mutex;

  EtherDrive_Control() : ROS_Slave(), ed(NULL)
  {

    for (int i = 0;i < 6;i++) {
      ostringstream oss;
      oss << "mot" << i;
      register_source(mot[i] = new FlowActuator(oss.str().c_str()));

      oss << "_cmd";
      register_sink(mot_cmd[i] = new FlowActuator(oss.str().c_str()), ROS_CALLBACK(EtherDrive_Control, mot_callback));

      last_mot_val[i] = 0;
    }

    register_with_master();

    if (!get_string_param(".host", host))
      host = "10.0.0.151";
    printf("EtherDrive host set to [%s]\n", host.c_str());

    if (!get_int_param(".frame_id", &frame_id))
      frame_id = -1;
    printf("EtherDrive frame_id set to [%d]\n", frame_id);

    for (int i = 0;i < 6;i++) {
      ostringstream oss;
      oss << ".pulse_per_rad" << i;
      if (!get_double_param(oss.str().c_str(), &(pulse_per_rad[i])))
	pulse_per_rad[i] = 1591.54943;
    }

    ed = new EtherDrive();
    ed->init(host);
    ed->motors_on();
  }

  virtual ~EtherDrive_Control()
  { 
    if (ed) 
      delete ed; 
  }

  bool do_tick()
  {

    int tmp_mot_cmd[6];

    for (int i = 0; i < 6; i++) {
    mot_cmd[i]->lock_atom();
      if (mot_cmd[i]->valid) {
	if (mot_cmd[i]->rel) {
	  last_mot_val[i] = mot[i]->val + mot_cmd[i]->val;
	} else {
	  last_mot_val[i] = mot_cmd[i]->val;
	}
      }
      mot_cmd[i]->valid = false;  // set to invalid so we don't re-use

      tmp_mot_cmd[i] = (int)(last_mot_val[i] * pulse_per_rad[i]);

      mot_cmd[i]->unlock_atom();
    }

    control_mutex.lock();

    printf("Commanding: %g -- %d\n", last_mot_val[1], tmp_mot_cmd[1]);

    ed->drive(6,tmp_mot_cmd);

    control_mutex.unlock();

    int val[6];
    
    if (!ed->tick(6,val)) {
      return false;
    }

    for (int i = 0; i < 6; i++) {
      mot[i]->val = val[i] / pulse_per_rad[i];
      mot[i]->rel = false;
      mot[i]->valid = true;
      mot[i]->publish();
    }
    return true;
  }

  void mot_callback() { }

};

int main(int argc, char **argv)
{
  EtherDrive_Control a;
  while (a.happy()) {
    if (!a.do_tick())
    {
      a.log(ROS::ERROR,"Etherdrive tick failed.");
      break;
    }
  }
  return 0;
}

