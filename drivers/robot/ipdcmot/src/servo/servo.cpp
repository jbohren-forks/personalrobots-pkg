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

#include <csignal>
#include "ros/ros_slave.h"
#include "ipdcmot/ipdcmot.h"
#include "common_flows/FlowInt32.h"
#include "common_flows/FlowFloat64.h"
#include "common_flows/FlowEmpty.h"
#include "common_flows/FlowString.h"
#include "ipdcmot/FlowPatrol.h"

class Servo : public ROS_Slave
{
public:
  FlowFloat64 *setpos_blocking, *getpos_result;
  FlowEmpty *getpos_blocking;
  FlowInt32 *setpos_result;
  FlowPatrol *patrol;
  FlowFloat64 *pos_f;
  FlowString *status_f;
  string ipdcmot_host;
  IPDCMOT *mot;
  double pos_send_freq;

  Servo() : ROS_Slave(), mot(NULL), ipdcmot_host("192.168.1.123"),
    pos_send_freq(1.0)
  {
    register_sink(setpos_blocking = new FlowFloat64("setpos_blocking"), ROS_CALLBACK(Servo, setpos_blocking_cb));
    register_source(setpos_result = new FlowInt32("setpos_result"));

    register_sink(getpos_blocking = new FlowEmpty("getpos_blocking"), ROS_CALLBACK(Servo, getpos_blocking_cb));
    register_source(getpos_result = new FlowFloat64("getpos_result"));

    register_sink(patrol = new FlowPatrol("patrol"), ROS_CALLBACK(Servo, patrol_cb));
    register_source(pos_f = new FlowFloat64("pos"));
    register_source(status_f = new FlowString("status"));

    register_with_master();
    if (!get_string_param(".host", ipdcmot_host))
      printf("ipdcmot_host parameter not specified... defaulting to [%s]\n", ipdcmot_host.c_str());
    printf("ipdcmot host set to [%s]\n", ipdcmot_host.c_str());
    get_double_param(".pos_send_freq", &pos_send_freq);
    printf("position send frequency set to %f\n", pos_send_freq);
    mot = new IPDCMOT(ipdcmot_host, 75.0);
    status_f->data = "ready";
    status_f->publish();
  }
  virtual ~Servo()
  { 
    if (mot) 
      delete mot; 
    mot = NULL;
  }
  void patrol_cb()
  {
    printf("set patrol: (%f, %f, %f, %d)\n", 
      patrol->stop1, patrol->stop2, patrol->speed, patrol->init_dir);
    mot->set_patrol(patrol->stop1, patrol->stop2, patrol->speed, patrol->init_dir);
  }
  void setpos_blocking_cb()
  {
    //printf("received new position target: %f\n", setpos_blocking->data);
    if (mot->set_pos_deg_blocking(setpos_blocking->data))
      setpos_result->data = 1;
    else
      setpos_result->data = 0;
    setpos_result->publish();
  }
  void getpos_blocking_cb()
  {
    if (mot->get_pos_blocking(&getpos_result->data, NULL))
      getpos_result->publish();
    else
      printf("woah! couldn't get the motor position.\n");
  }
  void send_pos()
  {
    mot->get_pos_blocking(&pos_f->data, NULL, 1);
    pos_f->data *= 3.1415926 / 180.0; // motor library is in radians
    pos_f->publish();
  }
};

Servo *g_s = NULL;

void safe_term(int)
{
  if (g_s)
    delete g_s;
  g_s = NULL;
}

int main(int argc, char **argv)
{
  signal(SIGTERM, safe_term);
  signal(SIGINT, safe_term);
  signal(SIGQUIT, safe_term);
  signal(SIGHUP, safe_term);
  Servo s;
  g_s = &s;
  int sleep_usecs = (int)(1000000.0 / s.pos_send_freq);
  while (s.happy())
  {
    usleep(sleep_usecs);
    s.send_pos();
  }
  g_s = NULL;
  return 0;
}

