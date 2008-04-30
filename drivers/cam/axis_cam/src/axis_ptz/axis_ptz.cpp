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
#include "unstable_flows/FlowPTZActuatorNoSub.h"
#include "axis_cam/axis_cam.h"

class Axis_PTZ : public ROS_Slave
{
public:
  FlowPTZActuatorNoSub *ptz;
  FlowPTZActuatorNoSub *ptz_control;

  string axis_host;
  AxisCam *cam;
  int frame_id;

  float pan;
  bool pan_rel;
  bool pan_valid;

  float tilt;
  bool tilt_rel;
  bool tilt_valid;

  float zoom;
  bool zoom_rel;
  bool zoom_valid;

  float focus;
  bool focus_rel;
  bool focus_valid;

  float iris;
  bool iris_rel;
  bool iris_valid;

  ROS_Mutex control_mutex;

  Axis_PTZ() : ROS_Slave(), cam(NULL), frame_id(0)
  {
    register_source(ptz = new FlowPTZActuatorNoSub("ptz"));
    register_sink(ptz_control = new FlowPTZActuatorNoSub("ptz_control"), ROS_CALLBACK(Axis_PTZ, ptz_callback));
    register_with_master();
    if (!get_string_param(".host", axis_host))
    {
      printf("host parameter not specified; defaulting to 192.168.0.90\n");
      axis_host = "192.168.0.90";
    }
    printf("axis_cam host set to [%s]\n", axis_host.c_str());
    get_int_param(".frame_id", &frame_id);
    cam = new AxisCam(axis_host);
    printf("package path is [%s]\n", get_my_package_path().c_str());
  }

  virtual ~Axis_PTZ()
  { 
    if (cam) 
      delete cam; 
  }

  bool get_and_send_ptz()
  {
    if (!cam->query_params()) {
      return false;
    }

    ptz->frame_id = frame_id;

    ptz->pan_val = cam->last_pan;
    ptz->pan_rel = false;
    ptz->pan_valid = true;

    ptz->tilt_val = cam->last_tilt;
    ptz->tilt_rel = false;
    ptz->tilt_valid = true;

    ptz->lens_zoom_val = cam->last_zoom;
    ptz->lens_zoom_rel = false;
    ptz->lens_zoom_valid = true;

    ptz->lens_focus_val = cam->last_focus;
    if (cam->last_autofocus_enabled)
      ptz->lens_focus_val = -1;
    ptz->lens_focus_rel = false;
    ptz->lens_focus_valid = true;

    ptz->lens_iris_val = cam->last_iris;
    if (cam->last_autoiris_enabled == true)
      ptz->lens_iris_val = -1;
    ptz->lens_iris_rel = false;
    ptz->lens_iris_valid = true;

    ptz->publish();
    return true;
  }

  void ptz_callback()
  {
    control_mutex.lock();

    if (ptz_control->pan_valid) {
      pan = ptz_control->pan_val;
      pan_rel = ptz_control->pan_rel;
      pan_valid = true;
    }

    if (ptz_control->tilt_valid) {
      tilt = ptz_control->tilt_val;
      tilt_rel = ptz_control->tilt_rel;
      tilt_valid = true;
    }

    if (ptz_control->lens_zoom_valid) {
      zoom = ptz_control->lens_zoom_val;
      zoom_rel = ptz_control->lens_zoom_rel;
      zoom_valid = true;
    }

    if (ptz_control->lens_focus_valid) {
      focus = ptz_control->lens_focus_val;
      focus_rel = ptz_control->lens_focus_rel;
      focus_valid = true;
    }

    if (ptz_control->lens_focus_valid) {
      focus = ptz_control->lens_focus_val;
      focus_rel = ptz_control->lens_focus_rel;
      focus_valid = true;
    }

    if (ptz_control->lens_iris_valid) {
      iris = ptz_control->lens_iris_val;
      iris_rel = ptz_control->lens_iris_rel;
      iris_valid = true;
    }

    control_mutex.unlock();
  }

  bool do_ptz_control() {

    control_mutex.lock();

    ostringstream oss;
    
    if (pan_valid)
      oss << (pan_rel ? "r" : "") << string("pan=") << pan << string("&");
    if (tilt_valid)
      oss << (tilt_rel ? "r" : "") << string("tilt=") << tilt << string("&");
    if (zoom_valid)
      oss << (zoom_rel ? "r" : "") << string("zoom=") << zoom << string("&");
    if (focus_valid) {
      if (!focus_rel && focus <= 0)
	oss << string("autofocus=on&");
      else 
	oss << string("autofocus=off&") << (focus_rel ? "r" : "") << string("focus=") << focus << string("&");
    }
    if (iris_valid) {
      if (!iris_rel && iris <= 0)
	oss << string("autoiris=on&");
      else 
	oss << string("autoiris=off&") << (iris_rel ? "r" : "") << string("iris=") << iris << string("&");
    }

    if (oss.str().size() > 0) {
      if (!cam->send_params(oss.str()))
	return false;
      pan_valid = tilt_valid = zoom_valid = focus_valid = iris_valid = false;
    }

    control_mutex.unlock();
    
    return true;
  }


};

int main(int argc, char **argv)
{
  Axis_PTZ a;
  while (a.happy()) {
    if (!a.get_and_send_ptz())
    {
      a.log(ROS::ERROR,"Couldn't acquire ptz info.");
      break;
    }
    if (!a.do_ptz_control())
    {
      a.log(ROS::ERROR,"Couldn't command ptz.");
      break;
    }
  }
  return 0;
}

