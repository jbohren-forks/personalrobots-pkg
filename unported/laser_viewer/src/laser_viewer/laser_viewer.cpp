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

/**

@mainpage

@htmlinclude manifest.html

@b laser_viewer is a GUI for displaying laser scans, in 3-D.

<hr>

@section usage Usage
@verbatim
$ laser_viewer  [standard ROS args]
@endverbatim

@par Example

@verbatim
$ laser_viewer 
@endverbatim

@par GUI controls

GL-style mouse-based manipulation.

@todo Actually document the controls.

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "scan"/LaserScan : scans to display

Publishes to (name / type):
- None

<hr>

@section parameters ROS parameters

- None

 **/
#include "ros/node.h"
#include "std_msgs/LaserScan.h"
#include "cloud_viewer/cloud_viewer.h"
#include <SDL.h>
#include "math.h"
#include <fstream>
#include <sstream>
#include <sys/stat.h>

using namespace std;

class Laser_Viewer : public ros::Node
{
public:
  std_msgs::LaserScan laser;
  vector< vector<float> > scans;
  vector<CloudViewerPoint> saved_cloud;
  CloudViewer cloud_viewer;
  int count;

  char dir_name[256];

  int scan_cnt;

  bool save_next;

  Laser_Viewer() : ros::Node("laser_viewer"), count(0), scan_cnt(0), save_next(false)
  {
    subscribe("scan", laser, &Laser_Viewer::scans_callback);

    time_t rawtime;
    struct tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    
    sprintf(dir_name, "scans/%.2d%.2d%.2d_%.2d%.2d%.2d", timeinfo->tm_mon + 1, timeinfo->tm_mday,timeinfo->tm_year - 100,timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    
    if (mkdir(dir_name, 0755)) {
      printf("Failed to make directory: %s", dir_name);
    }
  }

  virtual ~Laser_Viewer()
  { 
  }


  void refresh() {
    SDL_Event event;
    event.type = SDL_USEREVENT;
    SDL_PushEvent(&event);
    scans.reserve(100);
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
	    if (event.key.keysym.sym == SDLK_RETURN) {
	      save_next = true;
	    }
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
    if (count > 100) {
            
      saved_cloud.clear();
      cloud_viewer.clear_cloud();
      
      for (int j = 0; j < scans[0].size(); j++) {
	double val = 0.0;
	double valsq = 0.0;
	int cnt = 0;
	for (int i = 0; i < scans.size(); i++) {
	  if (scans[i][j] < 20.0  && scans[i][j] > 0.1) {
	    val += scans[i][j];
	    valsq += pow(scans[i][j],2.0);
            cnt++;
	  }
	}
	
	if (cnt > 0) {
	  double mean = val / cnt;
	  double std = sqrt( (valsq - cnt*pow(mean, 2.0))/(cnt - 1));
	  
	  cloud_viewer.add_point( -sin(laser.angle_min + j*laser.angle_increment) * (mean + std), 
				  -cos(laser.angle_min + j*laser.angle_increment) * (mean + std),
				  0, 
				  255,0,0);
	  cloud_viewer.add_point( -sin(laser.angle_min + j*laser.angle_increment) * mean, 
				  -cos(laser.angle_min + j*laser.angle_increment) * mean,
				  0, 
				  0,255,0);
	  cloud_viewer.add_point( -sin(laser.angle_min + j*laser.angle_increment) * (mean - std),
				  -cos(laser.angle_min + j*laser.angle_increment) * (mean - std),
				  0, 
				  255,0,0);
	  saved_cloud.push_back(CloudViewerPoint(cos(laser.angle_min + j*laser.angle_increment) * (mean - std),
						 sin(laser.angle_min + j*laser.angle_increment) * (mean - std),
						 0,
						 0, 0, 0));
	}
      }
      refresh();

      if (save_next) {
	save();
	save_next = false;
      }
      
      count = 0;
      scans.clear();
    } else {
      vector<float> tmp;
      for (int i = 0; i < laser.get_ranges_size(); i++) {
	tmp.push_back(laser.ranges[i]);
      }
      scans.push_back(tmp);
      count++;
    }
  }
    
    
  void save () {
    scan_cnt++;
    {
      std::ostringstream oss;
      oss << dir_name << "/Scan" << scan_cnt << ".xy";    
      ofstream out(oss.str().c_str());
      
      out.setf(ios::fixed, ios::floatfield);
      out.setf(ios::showpoint);
      out.precision(4);
      
      for (int i = 0; i < saved_cloud.size(); i++) {
	out << saved_cloud[i].x << " " << saved_cloud[i].y << endl;
      }
    
      out.close();
    }

    {
      std::ostringstream oss;
      oss << dir_name << "/Scan" << scan_cnt << ".rng";    
      ofstream out(oss.str().c_str());
      
      out.setf(ios::fixed, ios::floatfield);
      out.setf(ios::showpoint);
      out.precision(4);
      
      for (int i = 0; i < scans.size(); i++) {
	for (int j = 0; j < scans[i].size(); j++) {
	  out << scans[i][j] << " ";
	}
	out << endl;
      }
      
      out.close();
    }
  }
};

void 
quit(int sig) 
{
  SDL_Quit();
  ros::basic_sigint_handler(sig);
}

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

  signal(SIGINT, quit);

  while (lv.ok()) {
    lv.check_events();
  }

  ros::fini();

  return 0;
}

