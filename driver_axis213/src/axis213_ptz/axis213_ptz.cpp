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

#include "ros/ros_slave.h"
#include "image_flows/FlowImage.h"
#include "common_flows/FlowEmpty.h"
#include "axis213/axis213.h"
#include "driver_axis213/FlowPTZPosition.h"

#include "ros/ros_mutex.h"

class Axis213_ptz : public ROS_Slave
{
public:
  FlowPTZPosition *ptz_control;
  FlowPTZPosition *ptz_observe_all;
  FlowPTZPosition *ptz_observe_polled;
  FlowEmpty *shutter;

  string axis_host;
  Axis213 *cam;

  float pan;
  float tilt;
  float zoom;
  float focus;
  bool relative;
  bool sent;

  int last_count;

  ROS_Mutex control_mutex;

  Axis213_ptz() : ROS_Slave(), cam(NULL)
  {
    register_sink(ptz_control = new FlowPTZPosition("ptz_control"), ROS_CALLBACK(Axis213_ptz, ptz_control_callback));

    register_source(ptz_observe_all = new FlowPTZPosition("ptz_observe_all"));
    register_source(ptz_observe_polled = new FlowPTZPosition("ptz_observe_polled"));
    register_sink(shutter = new FlowEmpty("shutter"), ROS_CALLBACK(Axis213_ptz, shutter_callback));

    register_with_master();

    print_param_names();

    if (!get_string_param("axis213_ptz.host", axis_host))
    {
      printf("axis_host parameter not specified; defaulting to 10.0.0.150\n");
      axis_host = "10.0.0.150";
    }
    printf("axis host set to [%s]\n", axis_host.c_str());
    cam = new Axis213(axis_host);

    cam->get_ptz(&ptz_observe_all->pan, 
		 &ptz_observe_all->tilt, 
		 &ptz_observe_all->zoom, 
		 &ptz_observe_all->focus);
    relative = false;
    sent = true;

    last_count = 0;
  }
  virtual ~Axis213_ptz()
  { 
    if (cam) 
      delete cam; 
  }

  void ptz_control_callback()
  {
    control_mutex.lock();
    if (sent || !ptz_control->relative) {
      pan = ptz_control->pan;
      tilt = ptz_control->tilt;
      zoom = ptz_control->zoom;
      focus = ptz_control->focus;
      relative = ptz_control->relative;
    } else {
      pan += ptz_control->pan;
      tilt += ptz_control->tilt;
      zoom += ptz_control->zoom;
      focus += ptz_control->focus;
    }
    sent = false;
    control_mutex.unlock();
  }

  void do_ptz_control() {
    if (!sent) {
      control_mutex.lock();
      cam->ptz(pan, tilt, zoom, focus, relative);
      sent = true;
      control_mutex.unlock();
    }
  }

  bool fetch_and_send_position()
  {
    if (!cam->get_ptz(&ptz_observe_all->pan, 
		      &ptz_observe_all->tilt, 
		      &ptz_observe_all->zoom, 
		      &ptz_observe_all->focus))
      return false;
    ptz_observe_all->relative = false;
    ptz_observe_all->publish();
    return true;
  }

  void shutter_callback()
  {
    ptz_observe_all->lock_atom();
    ptz_observe_polled->pan = ptz_observe_all->pan;
    ptz_observe_polled->tilt = ptz_observe_all->tilt;
    ptz_observe_polled->zoom = ptz_observe_all->zoom;
    ptz_observe_polled->focus = ptz_observe_all->focus;
    ptz_observe_polled->relative = ptz_observe_all->relative;
    ptz_observe_all->unlock_atom();
    ptz_observe_polled->publish();
  }

};

int main(int argc, char **argv)
{
  Axis213_ptz a;
  while (a.happy()) {
    if (!a.fetch_and_send_position())
      {
	printf("couldn't get position.\n");
	break;
      }
    a.do_ptz_control();
  }
  return 0;
}

