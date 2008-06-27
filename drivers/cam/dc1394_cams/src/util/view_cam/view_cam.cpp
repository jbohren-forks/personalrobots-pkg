///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008, Willow Garage
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


#include <cstdio>
#include <cassert>
#include "dc1394/dc1394.h"
#include "SDL/SDL.h"


void teardown_cam(dc1394camera_t* camera) {
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
}

void cleanup_and_exit(dc1394camera_t *camera)
{
  teardown_cam(camera);
  exit(1);
}

int setup_cam(dc1394_t *d, int ind, dc1394camera_t** camera) {
  dc1394error_t err;

  dc1394camera_list_t * list;
  err=dc1394_camera_enumerate (d, &list);
  DC1394_ERR_RTN(err,"Failed to enumerate cameras");

  if (list->num <= 0) {
    printf("No cameras found: %d\n", list->num);
    return 1;
  } else {
    ind = ind % list->num;
    printf("Found %d cameras.  Using camera %d with guid %llx\n", list->num, ind, list->ids[ind].guid);
  }

  *camera = dc1394_camera_new(d, list->ids[ind].guid);

  if (!*camera) {
    dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[ind].guid);
    return 1;
  }

  dc1394_camera_free_list (list);

  dc1394_reset_bus(*camera);

  err=dc1394_video_set_iso_speed(*camera, DC1394_ISO_SPEED_400);
  DC1394_ERR_CLN_RTN(err,cleanup_and_exit(*camera),"Could not set iso speed");
  
  err=dc1394_video_set_mode(*camera, DC1394_VIDEO_MODE_640x480_MONO8);
  DC1394_ERR_CLN_RTN(err,cleanup_and_exit(*camera),"Could not set video mode");
  
  err=dc1394_video_set_framerate(*camera, DC1394_FRAMERATE_30);
  DC1394_ERR_CLN_RTN(err,cleanup_and_exit(*camera),"Could not set framerate");
  
  err=dc1394_capture_setup(*camera,45, DC1394_CAPTURE_FLAGS_DEFAULT );
  DC1394_ERR_CLN_RTN(err,cleanup_and_exit(*camera),"Could not setup camera-\nmake sure that the video mode and framerate are\nsupported by your camera");

  err=dc1394_video_set_transmission(*camera, DC1394_ON);
  DC1394_ERR_CLN_RTN(err,cleanup_and_exit(*camera),"Could not start camera iso transmission");

  char name[100];
  sprintf(name, "%llx", (*camera)->guid);
  SDL_WM_SetCaption(name,name);
}

int main(int argc, char **argv)
{

  bool colorize = false;

  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error [%s]\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);

  SDL_Surface *surf = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE);
  assert(surf);

  dc1394error_t err;
  dc1394_t *d = dc1394_new ();

  int ind = 0;
  dc1394camera_t* camera;
  setup_cam(d, ind, &camera);
  ind++;

  bool done = false;
  while (!done)
  {

    dc1394video_frame_t *in_frame;
    dc1394video_frame_t *frame;

    err=dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &in_frame);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not capture f frame");

    if (colorize)
    {
      frame = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
      in_frame->color_filter =  DC1394_COLOR_FILTER_GBRG;

      err=dc1394_debayer_frames(in_frame, frame, DC1394_BAYER_METHOD_BILINEAR);
      DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not debayer frame");
    } else {
      frame = in_frame;
    }

    uint8_t *in = frame->image, *out = (uint8_t *)surf->pixels;
    for (uint32_t row = 0; row < frame->size[0]; row++)
      for (uint32_t col = 0; col < frame->size[1]; col++)
        if (frame->color_coding == DC1394_COLOR_CODING_RGB8)
        {
          *(out  ) = *(in+2);
          *(out+1) = *(in+1);
          *(out+2) = *(in  );
          in += 3;
          out += 4;
        }
        else if (frame->color_coding == DC1394_COLOR_CODING_MONO8)
        {
          *(out++) = *in;
          *(out++) = *in;
          *(out++) = *in;
          *(out++);
          in++;
        }

    if (colorize) {
      free(frame->image);
      free(frame);
    }

    err=dc1394_capture_enqueue(camera, in_frame);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not release frame");

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
              teardown_cam(camera);

              setup_cam(d, ind, &camera);

              ind++;
              break;

            case SDLK_c: 
              colorize = !colorize;
              break;
            default: break;
          }
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }

  teardown_cam(camera);

  dc1394_free (d);

  return 0;
}
