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

#include <cstdio>
#include <cassert>
#include "dc1394_cam/dc1394_cam.h"
#include "SDL/SDL.h"


int main(int argc, char **argv)
{

  bool colorize = false;
  bool rectify = false;

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error [%s]\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  SDL_Surface *surf = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE);
  assert(surf);

  char winName[256];

  dc1394_cam::init();

  int num = dc1394_cam::numCams();

  if (num > 0)
  {
    printf("Found %d cameras\n", num);
  } else {
    printf("No cameras found.");
    return 1;
  }

  int ind = 0;

  uint64_t guid = dc1394_cam::getGuid(ind % num);
  dc1394_cam::Cam* c = new dc1394_cam::Cam(guid,
                                           DC1394_ISO_SPEED_400,
                                           DC1394_VIDEO_MODE_640x480_MONO8,
                                           DC1394_FRAMERATE_30, 40);

  printf("Using camera with guid: %llx\n", guid);
  snprintf(winName, 256, "%llx", guid);
  SDL_WM_SetCaption(winName, winName);


  dc1394_camera_print_info(c->dcCam, stdout);


  bool done = false;
  while (!done)
  {

    dc1394_cam::FrameSet fs = c->getFrames();

    if (fs.size() > 0)
    {      
      uint8_t *in = fs[0].getFrame()->image, *out = (uint8_t *)surf->pixels;

      uint32_t height = fs[0].getFrame()->size[0];
      uint32_t width = fs[0].getFrame()->size[1];
      dc1394color_coding_t coding = fs[0].getFrame()->color_coding;


      if (coding == DC1394_COLOR_CODING_RGB8)
      {
        for (uint32_t row = 0; row < height; row++)
          for (uint32_t col = 0; col < width; col++)
          {
            *(out  ) = *(in+2);
            *(out+1) = *(in+1);
            *(out+2) = *(in  );
            in += 3;
            out += 4;
          }
      }
      else if (coding == DC1394_COLOR_CODING_MONO8)
      {
        for (uint32_t row = 0; row < height; row++)
          for (uint32_t col = 0; col < width; col++)
          {
            *(out++) = *in;
            *(out++) = *in;
            *(out++) = *in;
            *(out++);
            in++;
          }
      }
      fs[0].releaseFrame();
    }

    SDL_UpdateRect(surf, 0, 0, 640, 480);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_KEYDOWN:
          switch(event.key.keysym.sym)
          {
          case SDLK_ESCAPE: done = true; break;
            
          case SDLK_SPACE: 

            ind++;

            delete c;
            
            guid = dc1394_cam::getGuid(ind % num);
            c = new dc1394_cam::Cam(guid);
            
            printf("Using camera with guid: %llx\n", guid);
            snprintf(winName, 256, "%llx", guid);
            SDL_WM_SetCaption(winName, winName);
            
            break;
            
          case SDLK_c: 
            colorize = !colorize;

            if (colorize)
              c->enableColorization();
            else
              c->disableColorization();
            break;

          case SDLK_r: 
            rectify = !rectify;
            if (rectify)
            {
              c->enableRectification(500, 500, 320, 240, -.3, 0.16, 0.0, 0.0, 0.0);
            } else {
              c->disableRectification();
            }

            break;
            
          case SDLK_RETURN:
            
            dc1394featureset_t features;
            
            dc1394_feature_get_all(c->dcCam, &features);
            dc1394_feature_print_all(&features, stdout);

            default: break;

          }
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }

  delete c;

  dc1394_cam::fini();

  return 0;
}
