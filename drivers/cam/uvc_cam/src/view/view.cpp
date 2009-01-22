#include <unistd.h>
#include <cstdio>
#include <cassert>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include "SDL/SDL.h"

const unsigned WIDTH = 640, HEIGHT = 480;

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: view DEVICE\n");
    return 1;
  }

  uvc_cam::Cam cam(argv[1]);
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "sdl init error: %s\n", SDL_GetError());
    return 1;
  }
  atexit(SDL_Quit);
  SDL_Surface *surf = SDL_SetVideoMode(WIDTH, HEIGHT, 24, SDL_HWSURFACE);
  assert(surf);
  SDL_WM_SetCaption("hello world", "hello world");
  ros::Time t_prev(ros::Time::now());
  int count = 0;
  for (bool done = false; !done;)
  {
    unsigned char *frame = NULL;
    uint32_t bytes_used;
    int buf_idx = cam.grab(&frame, bytes_used);
    if (count++ % 30 == 0)
    {
      ros::Time t(ros::Time::now());
      ros::Duration d(t - t_prev);
      printf("%.1f fps\n", 30.0 / d.toSec());
      t_prev = t;
    }
    if (frame)
    {
      memcpy(surf->pixels, frame, WIDTH*HEIGHT*3);
      cam.release(buf_idx);
    }
    usleep(1000);
    SDL_UpdateRect(surf, 0, 0, WIDTH, HEIGHT);
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

