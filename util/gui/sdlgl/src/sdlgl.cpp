#include <unistd.h>
#include "sdlgl/sdlgl.h"

using namespace ros;

SDLGL::SDLGL()
{
}

SDLGL::~SDLGL()
{
}

void SDLGL::request_render()
{
  SDL_Event e;
  e.type = SDL_USEREVENT;
  e.user.code = RENDER_USER_EVENT;
  e.user.data1 = NULL;
  e.user.data2 = NULL;
  SDL_PushEvent(&e);
}

bool SDLGL::init_gui(int width, int height, const char *title)
{
  SDL_EnableUNICODE(SDL_ENABLE);
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    printf("video init failed: %s\n", SDL_GetError());
    return false;
  }
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  if (title)
    SDL_WM_SetCaption(title, title);
  if (!SDL_SetVideoMode(width, height, 32, SDL_OPENGL | SDL_HWSURFACE))
  {
    printf("setvideomode failed: %s\n", SDL_GetError());
    return false;
  }
  set_view_params(width, height);
  request_render();
  return true;
}
/*
void *srosSDLGL::g_main_loop(void *parent)
{
  ((srosSDLGL *)parent)->main_loop();
}
*/
void SDLGL::main_loop()
{
  bool done = false;
  while (!done)
  {
    usleep(1000);
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_USEREVENT:
          if (event.user.code == RENDER_USER_EVENT)
            render();
          break;
        case SDL_MOUSEMOTION:
          mouse_motion(event.motion.x, event.motion.y,
            event.motion.xrel, event.motion.yrel,
            event.motion.state);
          break;
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
          mouse_button(event.button.x, event.button.y,
            event.button.button, 
            (event.type == SDL_MOUSEBUTTONDOWN ? true : false));
          break;
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
            done = true;
          else
            keypress(event.key.keysym.sym, 
                     event.key.keysym.unicode,
                     event.key.keysym.mod);
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  SDL_Quit();
}

