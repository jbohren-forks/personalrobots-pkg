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

#include "ros/node.h"
#include "std_msgs/MsgLaserScan.h"
#include "cloud_viewer/cloud_viewer.h"
#include <SDL.h>
#include "math.h"

class Laser_Viewer : public ros::node
{
public:
  MsgLaserScan laser;
  CloudViewer cloud_viewer;

  Laser_Viewer() : ros::node("laser_viewer")
  {
    subscribe("scan", laser, &Laser_Viewer::scans_callback);
  }

  virtual ~Laser_Viewer()
  { 
  }


  void refresh() {
    SDL_Event event;
    event.type = SDL_USEREVENT;
    SDL_PushEvent(&event);
  }

  void check_events() {
    SDL_Event event;
    if(SDL_WaitEvent(&event))
      {
	switch(event.type)
	  {
	  case SDL_MOUSEMOTION:
	    laser.lock();
	    cloud_viewer.mouse_motion(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel);
	    laser.unlock();
	    refresh();
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
	      laser.lock();
	      cloud_viewer.mouse_button(event.button.x, event.button.y,
					button, (event.type == SDL_MOUSEBUTTONDOWN ? true : false));
	      laser.unlock();
	      refresh();
	      break;
	    }
	  case SDL_KEYDOWN:
	    laser.lock();
	    cloud_viewer.keypress(event.key.keysym.sym);
	    laser.unlock();
	    refresh();
	    break;
	  case SDL_USEREVENT:
	    laser.lock();
	    cloud_viewer.render();
	    laser.unlock();
	    SDL_GL_SwapBuffers();
	    break;
	  }

      }
  }

  bool sdl_init()
  {
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   24);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    const int w = 1024, h = 768;
    if (SDL_SetVideoMode(w, h, 32, SDL_OPENGL | SDL_HWSURFACE) == 0)  {
      fprintf(stderr, "setvideomode failed: %s\n", SDL_GetError());
      return false;
    }

    cloud_viewer.set_opengl_params(w,h);

    SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

    return true;
  }

  void scans_callback()
  {
    cloud_viewer.clear_cloud();
    for (int i = 0; i < laser.get_ranges_size(); i++) {
      cloud_viewer.add_point( -sin(laser.angle_min + i*laser.angle_increment) * laser.ranges[i], 
			      -cos(laser.angle_min + i*laser.angle_increment) * laser.ranges[i],
			      0, 
			      255,255,255);
    }
    refresh();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv);
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "SDL initialization error: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  Laser_Viewer lv;

  if (!lv.sdl_init())
  {
    fprintf(stderr, "SDL video startup error: %s\n", SDL_GetError());
    exit(1);
  }

  while (lv.ok()) {
    lv.check_events();
  }
  return 0;
}

