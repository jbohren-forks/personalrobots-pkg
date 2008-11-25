#include <cassert>
#include <cstdio>
#include "SDL/SDL.h"
#include "ros/time.h"
#include "borg.h"

using namespace borg;

int main(int, char **)
{
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    printf("couldn't init sdl: %s\n", SDL_GetError());
    exit(1);
  }
  atexit(SDL_Quit);
  SDL_Surface *surface = SDL_SetVideoMode(640, 480, 32, SDL_HWSURFACE);
  assert(surface);
  SDL_WM_SetCaption("borg", "borg");
  Borg borg(Borg::INIT_CAM);
  borg.cam->startImageStream();
  bool done = false;
  ros::Time t(ros::Time::now());
  while (!done)
  {
    uint8_t *raster = borg.cam->savePhoto();
    ros::Time t2(ros::Time::now());
    double dt = (t2 - t).to_double();
    t = t2;
    printf("fps = %.1f\n", 1.0 / dt);
    //printf("raster = %x\n", (unsigned)raster);
    if (raster)
    {
      uint8_t *in = raster, *out = (uint8_t *)surface->pixels;
      for (uint32_t row = 0; row < 640; row++)
        for (uint32_t col = 0; col < 480; col++)
        {
          *(out++) = *in;
          *(out++) = *in;
          *(out++) = *in;
          *(out++);
          in++;
        }
      delete[] raster;
    }
    SDL_UpdateRect(surface, 0, 0, 640, 480);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
        case SDL_KEYDOWN:
          switch (event.key.keysym.sym)
          {
            case SDLK_ESCAPE: done = true; break;
            default: break;
          }
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  borg.cam->stopImageStream();
  return 0;
}

