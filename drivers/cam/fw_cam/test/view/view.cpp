///////////////////////////////////////////////////////////////////////////////
// The flea2 provides a bare-bones driver for the flea2 camera
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


#include <cstdio>
#include <cassert>
#include "fw_cam.h"
#include "SDL/SDL.h"

int main(int argc, char **argv)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error [%s]\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);
  SDL_Surface *surf = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE);
  assert(surf);
  
  FwHost fw1(0);
  FwHost fw2(1);

  FwCam cam1(&fw1, 0, 0);
  FwCam cam2(&fw1, 1, 1);
  FwCam cam3(&fw2, 0, 0);
  FwCam cam4(&fw2, 1, 1);

  uint8_t *frame1;
  uint32_t w1, h1;

  uint8_t *frame2;
  uint32_t w2, h2;

  uint8_t *frame3;
  uint32_t w3, h3;

  uint8_t *frame4;
  uint32_t w4, h4;

  int use_cam = 0;

  bool done = false;
  while (!done)
  {
    cam1.get_frame(&frame1, &w1, &h1);
    cam2.get_frame(&frame2, &w2, &h2);
    cam3.get_frame(&frame3, &w3, &h3);
    cam4.get_frame(&frame4, &w4, &h4);
    // I know this looks inefficient, but the compiler & CPU will 
    // speed this up a lot. this is what speculative execution was
    // designed for.

    if (use_cam % 4 == 0) {
      uint8_t *in = frame1, *out = (uint8_t *)surf->pixels;
      for (uint32_t row = 0; row < h1; row++)
        for (uint32_t col = 0; col < w1; col++)
          if (cam1.video_mode == FwCam::FLEA2_RGB)
          {
            *(out  ) = *(in+2);
            *(out+1) = *(in+1);
            *(out+2) = *(in  );
            in += 3;
            out += 4;
          }
          else if (cam1.video_mode == FwCam::FLEA2_MONO)
          {
            *(out++) = *in;
            *(out++) = *in;
            *(out++) = *in;
            *(out++); // skip the 4th byte
            in++;
          }
    } else if (use_cam % 4 == 1) {
      uint8_t *in = frame2, *out = (uint8_t *)surf->pixels;
      for (uint32_t row = 0; row < h2; row++)
        for (uint32_t col = 0; col < w2; col++)
          if (cam1.video_mode == FwCam::FLEA2_RGB)
          {
            *(out  ) = *(in+2);
            *(out+1) = *(in+1);
            *(out+2) = *(in  );
            in += 3;
            out += 4;
          }
          else if (cam1.video_mode == FwCam::FLEA2_MONO)
          {
            *(out++) = *in;
            *(out++) = *in;
            *(out++) = *in;
            *(out++); // skip the 4th byte
            in++;
          }
    } else if (use_cam % 4 == 2) {
      uint8_t *in = frame3, *out = (uint8_t *)surf->pixels;
      for (uint32_t row = 0; row < h3; row++)
        for (uint32_t col = 0; col < w3; col++)
          if (cam1.video_mode == FwCam::FLEA2_RGB)
          {
            *(out  ) = *(in+2);
            *(out+1) = *(in+1);
            *(out+2) = *(in  );
            in += 3;
            out += 4;
          }
          else if (cam1.video_mode == FwCam::FLEA2_MONO)
          {
            *(out++) = *in;
            *(out++) = *in;
            *(out++) = *in;
            *(out++); // skip the 4th byte
            in++;
          }
    } else if (use_cam % 4 == 3) {
      uint8_t *in = frame4, *out = (uint8_t *)surf->pixels;
      for (uint32_t row = 0; row < h4; row++)
        for (uint32_t col = 0; col < w4; col++)
          if (cam1.video_mode == FwCam::FLEA2_RGB)
          {
            *(out  ) = *(in+2);
            *(out+1) = *(in+1);
            *(out+2) = *(in  );
            in += 3;
            out += 4;
          }
          else if (cam1.video_mode == FwCam::FLEA2_MONO)
          {
            *(out++) = *in;
            *(out++) = *in;
            *(out++) = *in;
            *(out++); // skip the 4th byte
            in++;
          }
    }

    cam1.release_frame();
    cam2.release_frame();
    cam3.release_frame();
    cam4.release_frame();

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
            case SDLK_SPACE: use_cam++; break;
            default: break;
          }
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  return 0;
}

