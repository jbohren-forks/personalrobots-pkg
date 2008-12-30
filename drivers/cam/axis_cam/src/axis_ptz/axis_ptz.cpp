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

#include "ros/node.h"
#include "axis_cam/PTZActuatorCmd.h"
#include "axis_cam/PTZActuatorState.h"
#include "axis_cam/axis_cam.h"

class Axis_PTZ_node : public ros::node
{
public:
  axis_cam::PTZActuatorCmd   ptz_cmd;
  axis_cam::PTZActuatorState ptz_state;

  string axis_host;
  AxisCam *cam;

  bool cmd_updated;

  axis_cam::PTZActuatorCmd ptz_cmd_saved;

  boost::mutex control_mutex;

  Axis_PTZ_node() : node("axis_ptz"), cam(NULL), cmd_updated(false)
  {
    advertise<axis_cam::PTZActuatorState>("ptz_state",1);

    subscribe("ptz_cmd", ptz_cmd, &Axis_PTZ_node::ptz_callback, this, 1);

    param("~host", axis_host, string("192.168.0.90"));
    printf("axis_cam host set to [%s]\n", axis_host.c_str());

    cam = new AxisCam(axis_host);
  }

  virtual ~Axis_PTZ_node()
  {
    if (cam)
      delete cam;
  }

  void set_actuator_pos(std_msgs::ActuatorState& act, float  pos) {
    act.pos = pos;
    act.pos_valid = 1;
    act.vel = 0;
    act.vel_valid = 0;
    act.torque = 0;
    act.torque_valid = 0;
  }

  bool get_and_send_ptz()
  {
    if (cam->query_params()) {
      return false;
    }

    set_actuator_pos(ptz_state.pan, cam->last_pan);
    set_actuator_pos(ptz_state.tilt, cam->last_tilt);
    set_actuator_pos(ptz_state.zoom, cam->last_zoom);
    set_actuator_pos(ptz_state.pan, cam->last_pan);

    if (cam->last_autofocus_enabled)
      set_actuator_pos(ptz_state.focus, -1.0);
    else
      set_actuator_pos(ptz_state.focus, cam->last_focus);

    if (cam->last_autoiris_enabled)
      set_actuator_pos(ptz_state.iris, -1.0);
    else
      set_actuator_pos(ptz_state.iris, cam->last_iris);

    publish("ptz_state", ptz_state);
    return true;
  }


  void ptz_callback()
  {
    control_mutex.lock();

    ptz_cmd_saved = ptz_cmd;

    cmd_updated = true;

    control_mutex.unlock();
  }

  bool do_ptz_control() {

    bool retval = true;

    control_mutex.lock();

    if (cmd_updated)
    {

      ostringstream oss;

      if (ptz_cmd_saved.pan.valid)
        oss << (ptz_cmd_saved.pan.rel ? "r" : "") << string("pan=") << ptz_cmd_saved.pan.cmd << string("&");
      if (ptz_cmd_saved.tilt.valid)
        oss << (ptz_cmd_saved.tilt.rel ? "r" : "") << string("tilt=") << ptz_cmd_saved.tilt.cmd << string("&");
      if (ptz_cmd_saved.zoom.valid)
        oss << (ptz_cmd_saved.zoom.rel ? "r" : "") << string("zoom=") << ptz_cmd_saved.zoom.cmd << string("&");

      if (ptz_cmd_saved.focus.valid) {
        if (!ptz_cmd_saved.focus.rel && ptz_cmd_saved.focus.cmd <= 0)
          oss << string("autofocus=on&");
        else
          oss << string("autofocus=off&") << (ptz_cmd_saved.focus.rel ? "r" : "") << string("focus=") << ptz_cmd_saved.focus.cmd << string("&");
      }

      if (ptz_cmd_saved.iris.valid) {
        if (!ptz_cmd_saved.iris.rel && ptz_cmd_saved.iris.cmd <= 0)
          oss << string("autoiris=on&");
        else
          oss << string("autoiris=off&") << (ptz_cmd_saved.iris.rel ? "r" : "") << string("iris=") << ptz_cmd_saved.iris.cmd << string("&");
      }

      printf("Sending cmd: %s\n", oss.str().c_str());

      if (oss.str().size() > 0)
        if (cam->send_params(oss.str()))
        {
          retval = false;
        } else {
          cmd_updated = false;
        }
    }

    control_mutex.unlock();

    return retval;
  }


  bool spin()
  {
    while (ok())
    {
      if (!get_and_send_ptz())
      {
        ROS_ERROR("Couldn't acquire ptz info.");
        usleep(1000000);
        param("~host", axis_host, string("192.168.0.90"));
        cam->set_host(axis_host);
      }
      if (!do_ptz_control())
      {
        ROS_ERROR("Couldn't command ptz.");
        usleep(1000000);
        param("~host", axis_host, string("192.168.0.90"));
        cam->set_host(axis_host);
      }
    }
    return true;
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv);

  Axis_PTZ_node a;

  a.spin();

  ros::fini();

  return 0;
}

