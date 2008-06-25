#include <unistd.h>
#include <math.h>
#include <GL/gl.h>
#include "ros/node.h"
#include "std_msgs/LaserScan.h"
#include "sdlgl/sdlgl.h"

using namespace ros;

class LaserView : public node, public SDLGL
{
public:
  std_msgs::LaserScan laser;
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
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glScalef(view_scale, view_scale, view_scale);
    glTranslatef(view_x, view_y, 0);

    glPushMatrix();
    bool intensity_avail = false;
    double min_intensity = 1e9, max_intensity = -1e9;
    laser.lock();
    if (laser.get_intensities_size() == laser.get_ranges_size())
    {
      for (size_t i = 0; i < laser.get_intensities_size(); i++)
      {
        if (laser.intensities[i] > max_intensity)
          max_intensity = laser.intensities[i];
        if (laser.intensities[i] < min_intensity)
          min_intensity = laser.intensities[i];
      }
      if (min_intensity == max_intensity)
        min_intensity = max_intensity - 1; // whatever. this won't happen.
      intensity_avail = true;
    }
    glPointSize(4.0);

    for (size_t i = 0; i < laser.get_ranges_size(); i++)
    {
      glColor3f(0.1, 0.3, 0.1);
      glBegin(GL_LINES);
      glVertex2f(0,0);
      const double ang = laser.angle_min + laser.angle_increment * i;
      const double lx = laser.ranges[i] * cos(ang);
      const double ly = laser.ranges[i] * sin(ang);
      glVertex2f(lx, ly);
      glEnd();
      if (intensity_avail)
      {
        const float inten = (laser.intensities[i] - min_intensity) / 
                            (max_intensity - min_intensity);
        glColor3f(inten, 0, 1.0 - inten);
        glBegin(GL_POINTS);
        glVertex2f(lx, ly);
        glEnd();
      }
    }
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
  ros::fini();
  return 0;
}

