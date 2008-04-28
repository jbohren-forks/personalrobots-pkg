#include <unistd.h>
#include <math.h>
#include <GL/gl.h>
#include "ros/node.h"
#include "std_msgs/MsgRobotBase2DOdom.h"
#include "sdlgl/sdlgl.h"

class NavView : public ros::node, public ros::SDLGL
{
public:
  MsgRobotBase2DOdom odom;
  float view_scale, view_x, view_y;

  NavView() : ros::node("nav_view"),
    view_scale(50), view_x(0), view_y(0)
  {
    subscribe("odom", odom, &NavView::odom_cb);
    init_gui(640, 480, "nav view");
  }
  void odom_cb()
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
      odom.lock();
      glTranslatef(odom.px, odom.py, 0);
      glRotatef(odom.pyaw * 180 / M_PI, 0, 0, 1);
      odom.unlock();
      glColor3f(0.2, 1.0, 0.4);
      glBegin(GL_LINE_LOOP);
        const float robot_rad = 0.3;
        for (float f = 0; f < (float)(2*M_PI); f += 0.5f)
          glVertex2f(robot_rad * cos(f), robot_rad * sin(f));
      glEnd();
      glBegin(GL_LINES);
        glVertex2f(0,0);
        glVertex2f(robot_rad,0);
      glEnd();
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
  NavView view;
  view.main_loop();
  return 0;
}

