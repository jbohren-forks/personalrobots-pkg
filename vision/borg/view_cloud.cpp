///////////////////////////////////////////////////////////////////////////////
// This file provides a handy little cloud viewer
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

#include <math.h>
#include <SDL.h>
#include <cstdio>
#include <unistd.h>
#include <string.h>
#include "cloud_viewer/cloud_viewer.h"
#include <ctime>
#include <time.h>
#include <vector>
#include "ros/time.h"
#include "borg.h"

using namespace borg;
using std::vector;

static const double D2R = 3.1415926 / 180.0;

int main(int argc, char **argv)
{
  srand(time(NULL));
  double frac_to_show = 1.0;
  if (argc < 2)
  {
    printf("please give the cloudfile as the first parameter\n");
    return 0;
  }
  /*
  if (argc >= 3)
    frac_to_show = atof(argv[2]);
    */

  printf("opening [%s]\n", argv[1]);
  FILE *f = fopen(argv[1], "r");
  if (!f)
  {
    printf("couldn't open [%s]\n", argv[1]);
    return -1;
  }
  Borg borg(0);
  vector<Borg::SensedPoint> extraction;
  vector<Borg::ProjectedPoint> projection;
  if (argc >= 3)
  {
    borg.loadExtractionFile(argv[2], extraction);
    borg.project(extraction, projection);
  }
  CloudViewer cloud_viewer;
  int line_num = 1;
  double tilt = borg.get_tilt() * D2R;
  printf("tilt = %f\n", tilt);
  while (!feof(f))
  {
    double x, y, z;
    int r, g, b;
    if (6 != fscanf(f, "%lf %lf %lf %d %d %d\n", &x, &y, &z, &r, &g, &b))
    //if (10 != fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &x, &y, &z, &nx, &ny, &nz, &r, &g, &b, &dummy))
    {
      printf("bad syntax on line %d\n", line_num);
      break;
    }
    const double blend_factor = 0.3;
    g = (int)(g * (1.0 - blend_factor) + 255 * blend_factor); 
    // rotate about x so that the floor is level in the rendering
    double ry =  y * cos(tilt) + z * sin(tilt);
    double rz = -y * sin(tilt) + z * cos(tilt);
    y = ry; z = rz;

    if (line_num == 1)
    {
      printf("%f %f %f\n", x, y, z);
    }
    if (frac_to_show < 1.0)
    {
      if (rand() % 1000 <= (int)(frac_to_show * 1000))
        cloud_viewer.add_point(x, y, z, r, g, b);
    }
    else
      cloud_viewer.add_point(x, y, z, r, g, b);
    line_num++;
  }
  printf("read %d points\n", line_num);

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "video init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   24);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  const int w = 1100, h = 700;
  if (SDL_SetVideoMode(w, h, 32, SDL_OPENGL | SDL_HWSURFACE) == 0)
  {
    fprintf(stderr, "setvideomode failed: %s\n", SDL_GetError());
    return 2;
  }

  cloud_viewer.set_opengl_params(w,h);
  // choose a look target by sampling a bunch of points and choosing one 
  // closest to the mean
  srand(time(NULL));
  double x_mean = 0, y_mean = 0, z_mean = 0;
  vector<CloudViewerPoint> *pts = &cloud_viewer.points;
  for (size_t i = 0; i < pts->size(); i++)
  {
    x_mean += pts->at(i).x;
    y_mean += pts->at(i).y;
    z_mean += pts->at(i).z;
  }
  x_mean /= pts->size();
  y_mean /= pts->size();
  z_mean /= pts->size();
  // now, grab some samples and use the closest one
  int c = 0; // c is for closest
  double closest_dist = 1e100;
  for (int i = 0; i < 1000; i++)
  {
    int j = rand() % pts->size();
    double dx = pts->at(j).x - x_mean;
    double dy = pts->at(j).y - y_mean;
    double dz = pts->at(j).z - z_mean;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      c = j;
    }
  }
  cloud_viewer.set_look_tgt((*pts)[c].x, (*pts)[c].y, (*pts)[c].z-.2);
  cloud_viewer.render();
  SDL_GL_SwapBuffers();
  ros::Time last_render(ros::Time::now());
  double rho_min =  0.5, rho_max = 2.0, rho_step = 0.01;
  double azi_min = -2.4, azi_max = -0.7, azi_step = 0.003;
  double ele_min = -1.0, ele_max = -0.1, ele_step = 0.005;
  cloud_viewer.cam_rho = 0.5 * (rho_min + rho_max);
  cloud_viewer.cam_azi = 0.5 * (azi_min + azi_max);
  cloud_viewer.cam_ele = 0.5 * (ele_min + ele_max);
  bool auto_move = true;//false;
  double rho_dir = -1, azi_dir = 1, ele_dir = 1;

  bool done = false, reproject = false;
  while(!done)
  {
    usleep(1000);
    ros::Time t(ros::Time::now());
    if ((t - last_render).to_double() > 0.0333)
    {
      last_render = t;
      cloud_viewer.render();
      SDL_GL_SwapBuffers();
      // pan the camera for next time
      if (auto_move)
      {
        cloud_viewer.cam_rho += rho_dir * rho_step;
        if ((rho_dir > 0 && cloud_viewer.cam_rho > rho_max) || 
            cloud_viewer.cam_rho < rho_min)
          rho_dir *= -1;
        cloud_viewer.cam_azi += azi_dir * azi_step;
        if ((azi_dir > 0 && cloud_viewer.cam_azi > azi_max) || 
            cloud_viewer.cam_azi < azi_min)
          azi_dir *= -1;
        cloud_viewer.cam_ele += ele_dir * ele_step;
        if ((ele_dir > 0 && cloud_viewer.cam_ele > ele_max) || 
            cloud_viewer.cam_ele < ele_min)
          ele_dir *= -1;
      }
    }
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_MOUSEMOTION:
          cloud_viewer.mouse_motion(event.motion.x, event.motion.y, 
                                    event.motion.xrel, event.motion.yrel);
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
          cloud_viewer.mouse_button(event.button.x, event.button.y,
            button, (event.type == SDL_MOUSEBUTTONDOWN ? true : false));
          break;
        }
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
            done = true;
          else if (event.key.keysym.sym == SDLK_m)
            auto_move = !auto_move;
          else if (event.key.keysym.sym == SDLK_1)
          {
            reproject = true;
            borg.tx += 0.001 * (event.key.keysym.mod & SDLK_RSHIFT ? 1 : -1);
          }
          else if (event.key.keysym.sym == SDLK_2)
          {
            reproject = true;
            borg.tz += 0.001 * (event.key.keysym.mod & SDLK_RSHIFT ? 1 : -1);
          }
          else if (event.key.keysym.sym == SDLK_3)
          {
            reproject = true;
            borg.enc_offset += 0.01 * (event.key.keysym.mod & SDLK_RSHIFT ? 1 : -1);
          }
          else if (event.key.keysym.sym == SDLK_4)
          {
            reproject = true;
            borg.laser_roll += 0.1 * (event.key.keysym.mod & SDLK_RSHIFT ? 1 : -1);
          }
          else
            cloud_viewer.keypress(event.key.keysym.sym);
          //cloud_viewer.render();
          //SDL_GL_SwapBuffers();
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
    if (reproject)
    {
      printf("tx = %f\ntz = %f\nenc offset = %f\nlaser_roll = %f\n",
             borg.tx, borg.tz, borg.enc_offset, borg.laser_roll);
      borg.project(extraction, projection);
      cloud_viewer.points.clear();
      for (vector<Borg::ProjectedPoint>::iterator p = projection.begin();
           p != projection.end(); ++p)
        cloud_viewer.add_point(p->x, p->y, p->z, p->r, p->g, p->b);
      reproject = false;
        //printf("%.3f %.3f %.3f %d %d %d\n", 
        //       p->x, p->y, p->z, p->r, p->g, p->b);

    }
  }
  SDL_Quit();

  return 0;
}
