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
  
  Flea2 flea2;
  uint8_t *frame;
  uint32_t w, h;

  bool done = false;
  while (!done)
  {
    flea2.get_frame(&frame, &w, &h);
    uint8_t *in = frame, *out = (uint8_t *)surf->pixels;
    for (uint32_t row = 0; row < h; row++)
      for (uint32_t col = 0; col < w; col++)
      {
        *(out  ) = *(in+2);
        *(out+1) = *(in+1);
        *(out+2) = *(in  );
        out += 4;
        in += 3;
      }
    flea2.release_frame();
    SDL_UpdateRect(surf, 0, 0, 640, 480);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
            done = true;
          else if (event.key.keysym.sym == SDLK_SPACE)
            printf("space\n");
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  return 0;
}

