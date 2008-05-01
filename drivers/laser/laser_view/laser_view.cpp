#include <unistd.h>
#include <math.h>
#include <GL/gl.h>
#include "ros/node.h"
#include "std_msgs/MsgLaserScan.h"
#include "sdlgl/sdlgl.h"

using namespace ros;

class LaserView : public node, public SDLGL
{
public:
  MsgLaserScan laser;
  float view_scale, view_x, view_y;

  LaserView() : node("laser_view"),
    view_scale(50), view_x(0), view_y(0)
  {
    subscribe("laser", laser, &LaserView::laser_cb);
    init_gui(640, 480, "laser view");
  }
  void laser_cb()
  {
    request_render();
  }
  void render()
  {
    glClearColor(0.2,0.2,0.2,0);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glScalef(view_scale, view_scale, view_scale);
    glTranslatef(view_x, view_y, 0);

    glPushMatrix();
      laser.lock();
      glColor3f(0.2, 1.0, 0.4);
      glBegin(GL_LINES);
        for (int i = 0; i < laser.get_ranges_size(); i++)
        {
          glVertex2f(0,0);
          double ang = laser.angle_min + laser.angle_increment * i;
          double lx = laser.ranges[i] * cos(ang);
          double ly = laser.ranges[i] * sin(ang);
          glVertex2f(lx, ly);
        }
      glEnd();
      laser.unlock();
    glPopMatrix();

    SDL_GL_SwapBuffers();
  }
  void set_view_params(int width, int height)
  {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-width/2, width/2, -height/2, height/2, -1, 1);
    glMatrixMode(GL_MODELVIEW);
  }
  virtual void mouse_motion(int x, int y, int dx, int dy, int buttons)
  {
    if (buttons & SDL_BUTTON(1)) // left button: translate view
    {
      view_x += dx / view_scale;
      view_y -= dy / view_scale;
      request_render();
    }
    else if (buttons & SDL_BUTTON(3)) // right button: scale view
    {
      view_scale *= 1.0 - dy * 0.01;
      request_render();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  LaserView view;
  view.main_loop();
  return 0;
}

