#ifndef SDLGL_SDLGL_H
#define SDLGL_SDLGL_H

//#include <SDL/SDL.h>
#include <SDL.h>

namespace ros
{

class SDLGL
{
public:
  /** if max_frame_rate > 0, it is treated as the maximum number of frames
   *  per second to render. Requests for renderings over this limit will
   *  be silently ignored. */
  SDLGL(double max_frame_rate = -1);
  virtual ~SDLGL();
  virtual void mouse_motion(int x, int y, int dx, int dy, int buttons) { }
  virtual void mouse_button(int x, int y, int button, bool is_down) { }
  virtual void keypress(char c, uint16_t u, SDLMod mod) { }
  virtual void set_view_params(int width, int height) { }
  bool init_gui(int width, int height, const char *window_title = NULL);
  void main_loop();
  void request_render();
protected:
  static const int RENDER_USER_EVENT = 0xdeadbeef; // ha ha
  virtual void render() = 0;
  double max_frame_rate;
private:  
/*
  static void *g_main_loop(void *parent);
*/
  /** uses SDL_GetTicks() to save the last time a render was requested,
   *  in terms of milliseconds since the program was started. */
  uint32_t last_render_time;
};

}

#endif

