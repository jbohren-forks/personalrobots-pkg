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
#include "common_flows/FlowPointCloudFloat32.h"
#include "common_flows/FlowEmpty.h"
#include "cloud_viewer/cloud_viewer.h"
#include <SDL/SDL.h>
#include "math.h"

class Cloud_Node : public ROS_Slave
{
public:
  FlowPointCloudFloat32 *cloud;
  FlowEmpty *shutter;
  CloudViewer *cloud_viewer;
  

  int count;

  Cloud_Node() : ROS_Slave(), cloud_viewer(NULL)
  {
    register_sink(cloud = new FlowPointCloudFloat32("cloud"), ROS_CALLBACK(Cloud_Node, cloud_callback));
    register_sink(shutter = new FlowEmpty("shutter"), ROS_CALLBACK(Cloud_Node, shutter_callback));

    cloud_viewer =  new CloudViewer;

    count = 0;

    register_with_master();
  }

  virtual ~Cloud_Node()
  { 
    if (cloud_viewer)
      delete cloud_viewer;
  }


  void refresh() {
    // SDL (and Xlib) don't handle threads well, so we do both of these
    check_keyboard();
    display_image();
  }

  void check_keyboard() {
    SDL_Event event;
    while(SDL_PollEvent(&event))
      {
	switch(event.type)
	  {
	  case SDL_MOUSEMOTION:
	    cloud_viewer->mouse_motion(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel);
	    break;
	  case SDL_MOUSEBUTTONDOWN:
	  case SDL_MOUSEBUTTONUP:
	    {
	      int button = -1;
	      switch(event.button.button)
		{
		case SDL_BUTTON_LEFT: button = 0; break;
		case SDL_BUTTON_MIDDLE: button = 1; break;
		case SDL_BUTTON_RIGHT: button = 2; break;
		}
	      cloud_viewer->mouse_button(event.button.x, event.button.y,
					button, (event.type == SDL_MOUSEBUTTONDOWN ? true : false));
	      break;
	    }
	  case SDL_KEYDOWN:
	    cloud_viewer->keypress(event.key.keysym.sym);
	    break;
	  }
      }
  }

  void display_image() {
    cloud->lock_atom();     // I've commandeered the cloud mutex for my own nefarious purposes
    cloud_viewer->render();
    SDL_GL_SwapBuffers();
    cloud->unlock_atom();
  }


  bool sdl_init()
  {
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   24);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    const int w = 640, h = 480;
    if (SDL_SetVideoMode(w, h, 32, SDL_OPENGL | SDL_HWSURFACE) == 0)  {
      fprintf(stderr, "setvideomode failed: %s\n", SDL_GetError());
      return false;
    }

    cloud_viewer->set_opengl_params(w,h);

    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);//People expect Key Repeat

    return true;
  }

  void cloud_callback()
  {
    for (int i = 0; i < cloud->get_x_size(); i++) {
      cloud_viewer->add_point(cloud->x[i], cloud->y[i], cloud->z[i],
			      255,255,255);
    }
  }

  void shutter_callback()
  {
    cloud->lock_atom();
    cloud_viewer->clear_cloud();
    cloud->unlock_atom();
  }



};

int main(int argc, char **argv)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "SDL initialization error: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  Cloud_Node cn;

  if (!cn.sdl_init())
  {
    fprintf(stderr, "SDL video startup error: %s\n", SDL_GetError());
    exit(1);
  }

  while (cn.happy()) {
    cn.refresh();
    usleep(1000);
  }
  return 0;
}

