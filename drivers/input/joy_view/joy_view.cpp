#include <unistd.h>
#include <math.h>
#include <GL/gl.h>
#include "ros/node.h"
#include "joy/MsgJoy.h"
#include "sdlgl/sdlgl.h"

using namespace ros;

class JoyView : public node, public SDLGL
{
public:
  MsgJoy joy;

  JoyView() : node("joy_view")
  {
    joy.buttons = 0;
    joy.x1 = joy.x2 = joy.y1 = joy.y2 = 0;
    subscribe("joy", joy, &JoyView::joy_cb);
    init_gui(640, 480, "nav view");
  }
  void joy_cb()
  {
    request_render();
  }
  void render()
  {
    glClearColor(0.2,0.2,0.2,0);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    joy.lock();
    glLineWidth(5);
    glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex2f(-1,0);
      glVertex2f(-1 + joy.x1, joy.y1);

      glColor3f(0, 1, 0);
      glVertex2f(1, 0);
      glVertex2f(1 + joy.x2, joy.y2);
    glEnd();
    for (int i = 0; i < 16; i++)
    {
      if (joy.buttons & (1 << i))
        glColor3f(1, 0.8, 0.4);
      else
        glColor3f(0.5f, 0.5f, 0.5f);
      float lx = -2.0f + (float)i / 4.0f;
      float rx = -2.0f + (float)(i+1) / 4.0f;
      glBegin(GL_QUADS);
      glVertex2f(lx, 1.8);
      glVertex2f(rx, 1.8);
      glVertex2f(rx, 2.0);
      glVertex2f(lx, 2.0);
      glEnd();
    }
    joy.unlock();
    SDL_GL_SwapBuffers();
  }
  void set_view_params(int width, int height)
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-2, 2, -2, 2, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    request_render();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  JoyView view;
  view.main_loop();
  return 0;
}

