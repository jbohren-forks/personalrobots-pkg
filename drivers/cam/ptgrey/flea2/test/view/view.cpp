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
#include "flea2/flea2.h"
#include "SDL/SDL.h"
#include "image_utils/pgm_wrapper.h"

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
  
  Flea2 flea2; // default is monochrome
  uint8_t *frame;
  uint32_t w, h;

  double shutter = 0.2;
  flea2.set_shutter(shutter);
  double gamma = 0.3;
  flea2.set_gamma(gamma);
  double gain = 0.5;
  flea2.set_gain(gain);

  bool done = false;
  while (!done)
  {
    flea2.get_frame(&frame, &w, &h);
    // I know this looks inefficient, but the compiler & CPU will 
    // speed this up a lot. this is what speculative execution was
    // designed for.
    uint8_t *in = frame, *out = (uint8_t *)surf->pixels;
    for (uint32_t row = 0; row < h; row++)
      for (uint32_t col = 0; col < w; col++)
        if (flea2.video_mode == Flea2::FLEA2_RGB)
        {
          *(out  ) = *(in+2);
          *(out+1) = *(in+1);
          *(out+2) = *(in  );
          in += 3;
          out += 4;
        }
        else if (flea2.video_mode == Flea2::FLEA2_MONO)
        {
          *(out++) = *in;
          *(out++) = *in;
          *(out++) = *in;
          *(out++); // skip the 4th byte
          in++;
        }
    flea2.release_frame();
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
            {
              static int num = 1;
              char fn[100];
              /*
              snprintf(fn, sizeof(fn), "img%06d.jpg", num++);
              const uint8_t *buf;
              uint32_t buf_size;
              flea2.get_jpeg(&buf, &buf_size);
              FILE *of = fopen(fn, "wb");
              fwrite(buf, 1, buf_size, of);
              fclose(of);
              printf("wrote %d bytes to %s\n", buf_size, fn);
              */
              snprintf(fn, sizeof(fn), "img%06d.pgm", num++);
              flea2.get_frame(&frame, &w, &h);
              PgmWrapper::write_file(string(fn), 640, 480, frame);
              flea2.release_frame();
              break;
            }
            case SDLK_EQUALS: flea2.set_shutter(shutter += 0.05); break;
            case SDLK_MINUS:  flea2.set_shutter(shutter -= 0.05); break;
            case SDLK_g: flea2.set_gamma(gamma += 0.02); break;
            case SDLK_b: flea2.set_gamma(gamma -= 0.02); break;
            case SDLK_i: flea2.set_gain(gain += 0.02); break;
            case SDLK_k: flea2.set_gain(gain -= 0.02); break;
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

