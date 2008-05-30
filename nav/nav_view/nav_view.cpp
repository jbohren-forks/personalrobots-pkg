/*
 * nav_view
 * Copyright (c) 2008, Morgan Quigley
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**

@mainpage

@htmlinclude manifest.html

@b nav_view is a GUI for 2-D navigation.  It can:
 - display a map
 - display a robot's pose
 - display a cloud of particles (e.g., from a localization system)
 - display a path (e.g., from a path planner)
 - send a goal to a planner

<hr>

@section usage Usage
@verbatim
$ nav_view [map res] [standard ROS args]
@endverbatim

@param map An image file to load as an occupancy grid map. The lower-left pixel of the map is assigned the pose (0,0,0).
@param res The resolution of the map, in meters, pixel.

@todo Remove the map and res arguments in favor map retrieval via ROSRPC.

@par Example

@verbatim
$ nav_view mymap.png 0.1 odom:=localizedpose
@endverbatim

@par GUI controls
Mouse bindings:
- drag left button: pan view
- drag right button: zoom view
- click middle button (set goal):
 - 1st click: set goal position
 - 2nd click: set goal orientation
- SHIFT + click middle button (set robot pose):
 - 1st click: set robot position
 - 2nd click: set robot orientation

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "odom"/RobotBase2DOdom : the robot's 2D pose.  Rendered as a circle with a heading line.
- @b "particlecloud"/ParticleCloud2D : a set particles from a probabilistic localization system.  Rendered is a set of small arrows.
- @b "gui_path"/Polyline2D : a path from a planner.  Rendered as a dashed line.
- @b "gui_laser"/Polyline2D : re-projected laser scan from a planner.  Rendered as a set of points.

Publishes to (name / type):
- @b "goal"/Planner2DGoal : goal for planner.  Sent on middle button click.
- @b "initialpose"/Pose2DFloat32 : pose to initialize localization system.  Sent on SHIFT + middle button click.

<hr>

@section parameters ROS parameters

- None

 **/

#include <unistd.h>
#include <assert.h>
#include <math.h>
#include <GL/gl.h>
#include <SDL/SDL_image.h>
#include "ros/node.h"
//#include "std_msgs/MsgRobotBase2DOdom.h"
#include "std_msgs/MsgParticleCloud2D.h"
#include "std_msgs/MsgPlanner2DGoal.h"
//#include "std_msgs/MsgPlanner2DState.h"
#include "std_msgs/MsgPolyline2D.h"
#include "std_msgs/MsgPose2DFloat32.h"
#include "sdlgl/sdlgl.h"

#include "rosTF/rosTF.h"
#include "rostime/clock.h"

class NavView : public ros::node, public ros::SDLGL
{
public:
  //MsgRobotBase2DOdom odom;
  MsgParticleCloud2D cloud;
  MsgPlanner2DGoal goal;
  //MsgPlanner2DState pstate;
  MsgPolyline2D pathline;
  MsgPolyline2D laserscan;
  MsgPose2DFloat32 initialpose;
  float view_scale, view_x, view_y;
  SDL_Surface* map_surface;
  GLuint map_texture;
  double map_res;
  int gwidth, gheight;
  bool setting_theta;
  double gx,gy;

  rosTFClient tf;
  ros::time::clock myClock;

  NavView() : ros::node("nav_view",false),
	      view_scale(10), view_x(0), view_y(0),
	      tf(*this,false)
  {
    advertise<MsgPlanner2DGoal>("goal");
    advertise<MsgPose2DFloat32>("initialpose");
    //subscribe("state", pstate, &NavView::pstate_cb);
    //subscribe("odom", odom, &NavView::odom_cb); //replaced with rosTF
    subscribe("particlecloud", cloud, &NavView::odom_cb);
    subscribe("gui_path", pathline, &NavView::odom_cb);
    subscribe("gui_laser", laserscan, &NavView::odom_cb);
    gwidth = 1024;
    gheight = 768;
    init_gui(gwidth, gheight, "nav view");
    glClearColor(1.0,1.0,1.0,0);
    map_surface = NULL;
    setting_theta = false;
  }

  ~NavView()
  {
    if(map_surface)
      SDL_FreeSurface(map_surface);
  }
  
  void load_map(const char* fname, double res);
  SDL_Surface* flip_vert(SDL_Surface* sfc);

  void odom_cb()
  {
    request_render();
  }

  /*
  void pstate_cb()
  {
    printf("Planner state: %d %d %d (%.3f %.3f %.3f)\n", 
           pstate.active,
           pstate.valid,
           pstate.done,
           pstate.pos.x,
           pstate.pos.y,
           pstate.pos.th);
  }
  */
  void render()
  {
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    glScalef(view_scale, view_scale, view_scale);
    glTranslatef(view_x, view_y, 0);
    
    if(map_surface)
    {
      // Draw the map
      glColor3f(1.0,1.0,1.0);
      glBindTexture( GL_TEXTURE_2D, map_texture );
      // Bind the texture to which subsequent calls refer to
      glBegin( GL_QUADS );
      //Top-left vertex (corner)
      glTexCoord2i( 0, 0 );
      glVertex3f( 0, 0, 0.0f );

      //Bottom-left vertex (corner)
      glTexCoord2i( 1, 0 );
      glVertex3f( map_res*map_surface->w, 0, 0 );

      //Bottom-right vertex (corner)
      glTexCoord2i( 1, 1 );
      glVertex3f( map_res*map_surface->w, map_res*map_surface->h, 0 );

      //Top-right vertex (corner)
      glTexCoord2i( 0, 1 );
      glVertex3f( 0, map_res*map_surface->h, 0 );
      glEnd();
    }

    const float robot_rad = 0.3;
    try
    {
      libTF::TFPose2D robotPose;
      robotPose.x = 0;
      robotPose.y = 0;
      robotPose.yaw = 0;
      robotPose.frame = FRAMEID_ROBOT;
      robotPose.time = 0;

      libTF::TFPose2D mapPose = tf.transformPose2D(FRAMEID_MAP, robotPose);

      glPushMatrix();
      glTranslatef(mapPose.x, mapPose.y, 0);
      glRotatef(mapPose.yaw * 180 / M_PI, 0, 0, 1);
      glColor3f(0.2, 1.0, 0.4);
      glBegin(GL_LINE_LOOP);
      for (float f = 0; f < (float)(2*M_PI); f += 0.5f)
        glVertex2f(robot_rad * cos(f), robot_rad * sin(f));
      glEnd();
      glBegin(GL_LINES);
      glVertex2f(0,0);
      glVertex2f(robot_rad,0);
      glEnd();
      glPopMatrix();
    }
    catch(libTF::TransformReference::LookupException& ex)
    {
    }
    
    cloud.lock();
    for(unsigned int i=0;i<cloud.get_particles_size();i++)
    {
      glPushMatrix();
      glTranslatef(cloud.particles[i].x, cloud.particles[i].y, 0);
      glRotatef(cloud.particles[i].th * 180 / M_PI, 0, 0, 1);
      glColor3f(1.0, 0.0, 0.0);
      glBegin(GL_LINE_LOOP);
      glVertex2f(0,0);
      glVertex2f(robot_rad,0);
      glVertex2f(0.75*robot_rad,-0.25*robot_rad);
      glVertex2f(0.75*robot_rad,0.25*robot_rad);
      glVertex2f(robot_rad,0);
      glEnd();
      glPopMatrix();
    }
    cloud.unlock();

    pathline.lock();
    glColor3f(pathline.color.r,pathline.color.g,pathline.color.b);
    glBegin(GL_LINES);
    for(unsigned int i=0;i<pathline.get_points_size();i++)
      glVertex2f(pathline.points[i].x,pathline.points[i].y);
    glEnd();
    pathline.unlock();

    pathline.lock();
    glColor3f(laserscan.color.r,laserscan.color.g,laserscan.color.b);
    glBegin(GL_POINTS);
    for(unsigned int i=0;i<laserscan.get_points_size();i++)
      glVertex2f(laserscan.points[i].x,laserscan.points[i].y);
    glEnd();
    pathline.unlock();

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
  virtual void mouse_button(int x, int y, int button, bool is_down) 
  { 
    if((button == 2) && !is_down) // middle mouse button
    {
      // If this is the first click, we're setting the position, next click
      // will give direction
      if(!setting_theta)
      {
        // Convert to conventional coords
        gx = (x-gwidth/2.0)/view_scale-view_x;
        gy = -((y-gheight/2.0)/view_scale+view_y);

        setting_theta = true;
      }
      else
      {
        // Convert to conventional coords
        double nx = (x-gwidth/2.0)/view_scale-view_x;
        double ny = -((y-gheight/2.0)/view_scale+view_y);

        // Compute orientation
        double ga = atan2(ny-gy,nx-gx);
        
        // Check modifier keys
        Uint8* keystate = SDL_GetKeyState(NULL);
        // If shift is down, set initial pose
        if(keystate[SDLK_LSHIFT] || keystate[SDLK_RSHIFT])
        {
          // Send out the pose
          initialpose.x = gx;
          initialpose.y = gy;
          initialpose.th = ga;
          printf("setting pose: %.3f %.3f %.3f\n",
                 initialpose.x,
                 initialpose.y,
                 initialpose.th);
          publish("initialpose", initialpose);
        }
        // No modifiers; set goal
        else 
        {
          // Send out the goal
          goal.goal.x = gx;
          goal.goal.y = gy;
          goal.goal.th = ga;
          goal.enable = 1;
          printf("setting goal: %.3f %.3f %.3f\n",
                 goal.goal.x,
                 goal.goal.y,
                 goal.goal.th);
          publish("goal", goal);
        }

        setting_theta = false;
      }
    }
  }
};

void
quit(int sig)
{
  SDL_Quit();
  ros::fini();
  exit(0);
}

int main(int argc, char **argv)
{
  char* fname=NULL;
  double res=0.0;

  signal(SIGINT, quit);
  signal(SIGPIPE, SIG_IGN);

  ros::init(argc, argv);
  
  // HACK: assume args are: image filename and resolution
  if(argc >= 3)
  {
    fname = argv[1];
    res = atof(argv[2]);
  }


  NavView view;
  if(fname)
    view.load_map(fname,res);
  view.main_loop();
  ros::fini();
  return 0;
}

void
NavView::load_map(const char* fname, double res)
{
  map_res = res;
  glEnable( GL_TEXTURE_2D ); 
  // Load the image from file
  SDL_Surface* temp;
  if(!(temp = IMG_Load(fname)))
  {
    printf("Warning: failed to load map from file \"%s\"\n",
           fname);
    return;
  }

  // Check that the image's width is a power of 2
  if((temp->w & (temp->w - 1)) != 0) {
    printf("warning: image.bmp's width is not a power of 2\n");
  }
  // Also check if the height is a power of 2
  if((temp->h & (temp->h - 1)) != 0) {
    printf("warning: image.bmp's height is not a power of 2\n");
  }


  // Flip the image vertically
  map_surface = flip_vert(temp);
  SDL_FreeSurface(temp);

  // get the number of channels in the SDL surface
  GLint nOfColors = map_surface->format->BytesPerPixel;
  GLenum texture_format;
  if(nOfColors == 4)     // contains an alpha channel
  {
    if(map_surface->format->Rmask == 0x000000ff)
      texture_format = GL_RGBA;
    else
      texture_format = GL_BGRA;
  } 
  else if(nOfColors == 3)     // no alpha channel
  {
    if(map_surface->format->Rmask == 0x000000ff)
      texture_format = GL_RGB;
    else
      texture_format = GL_BGR;
  } else {
    printf("warning: the image is not truecolor..  this will probably break\n");
    // this error should not go unhandled
    return;
  }


  // Have OpenGL generate a texture object handle for us
  glGenTextures( 1, &map_texture );

  // Bind the texture object
  glBindTexture( GL_TEXTURE_2D, map_texture );

  // Set the texture's stretching properties
  glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                   GL_LINEAR );
  glTexParameteri( GL_TEXTURE_2D,
                   GL_TEXTURE_MAG_FILTER, GL_LINEAR );

  // Edit the texture object's image data using
  //the information SDL_Surface gives us
  glTexImage2D( GL_TEXTURE_2D, 0, nOfColors,
                map_surface->w, map_surface->h, 0,
                texture_format,
                GL_UNSIGNED_BYTE,
                map_surface->pixels );
}


SDL_Surface* 
NavView::flip_vert(SDL_Surface* sfc)
{
  SDL_Surface* result = 
          SDL_CreateRGBSurface(sfc->flags, sfc->w, sfc->h,
                               sfc->format->BytesPerPixel * 8, 
                               sfc->format->Rmask, sfc->format->Gmask,
                               sfc->format->Bmask, sfc->format->Amask);
  assert(result);
  unsigned char* pixels = (unsigned char*)(sfc->pixels);
  unsigned char* rpixels = (unsigned char*)(result->pixels);
  unsigned int pitch = sfc->pitch;

  for(int line = 0; line < sfc->h; ++line) 
  {
    memcpy(rpixels+line*pitch,
           pixels+(sfc->h-line-1)*pitch,
           pitch);
  }

  return result;
}
