#include <unistd.h>
#include <cstdio>
#include <cassert>
#include "uvc_cam/uvc_cam.h"
#include "SDL/SDL.h"

int main(int argc, char **argv)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error: %s\n", SDL_GetError());
    return 1;
  }
  atexit(SDL_Quit);
  SDL_Surface *surf = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE);
  assert(surf);
  SDL_WM_SetCaption("hello world", "hello world");
  for (bool done = false; !done;)
  {
    usleep(1000);
    SDL_UpdateRect(surf, 0, 0, 640, 480);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
        case SDL_KEYDOWN:
          switch(event.key.keysym.sym)
          {
            case SDLK_ESCAPE: done = true; break;
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

