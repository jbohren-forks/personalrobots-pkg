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

@b nav_view is a GUI for 2-D navigation.  It can:
 - display a map
 - display a robot's pose
 - display a cloud of particles (e.g., from a localization system)
 - display a path (e.g., from a path planner)
 - send a goal to a planner

<hr>

@section usage Usage
@verbatim
$ nav_view
@endverbatim

@par Example

@verbatim
$ nav_view
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

// OpenGL
#include <GL/gl.h>
#include <SDL/SDL_image.h>

// roscpp and friends
#include "ros/node.h"
#include "rosTF/rosTF.h"

// messages and services
#include "std_msgs/ParticleCloud2D.h"
#include "std_msgs/Planner2DGoal.h"
#include "std_msgs/Polyline2D.h"
#include "std_msgs/Pose2DFloat32.h"
#include "pr2_msgs/OccDiff.h"
#include "std_srvs/StaticMap.h"
#include <pr2_srvs/TransientObstacles.h>
#include "sdlgl/sdlgl.h"

struct ObstaclePoint {
  double x;
  double y;
};

class NavView : public ros::node, public ros::SDLGL
{
public:
  std_msgs::ParticleCloud2D cloud;
  std_msgs::Planner2DGoal goal;
  std_msgs::Polyline2D pathline;
  std_msgs::Polyline2D local_path;
  std_msgs::Polyline2D robot_footprint;
  std_msgs::Polyline2D laserscan;
  std_msgs::Pose2DFloat32 initialpose;
  std_msgs::Polyline2D inflatedObstacles;
  std_msgs::Polyline2D rawObstacles;
  pr2_msgs::OccDiff occ_diff_;
  float view_scale, view_x, view_y;
  SDL_Surface* map_surface;
  GLuint map_texture;
  double map_res;
  int map_width, map_height;
  int gwidth, gheight;
  bool setting_theta;
  double gx,gy;
  int n_val_;
  std::list<ObstaclePoint> nav_view_points_;
  bool full_transient_init_;

  // Lock for access to class members in callbacks
  ros::thread::mutex lock;

  rosTFClient tf;

  NavView(int n_val) : ros::node("nav_view",ros::node::DONT_HANDLE_SIGINT),
                       view_scale(10), view_x(0), view_y(0), n_val_(n_val),
                       tf(*this,false)
  {
    param("max_frame_rate", max_frame_rate, 5.0);
    advertise<std_msgs::Planner2DGoal>("goal");
    advertise<std_msgs::Pose2DFloat32>("initialpose");
    subscribe("particlecloud", cloud, &NavView::generic_cb);
    subscribe("gui_path", pathline, &NavView::generic_cb);
    subscribe("local_path", local_path, &NavView::generic_cb);
    subscribe("robot_footprint", robot_footprint, &NavView::generic_cb);
    subscribe("inflated_obstacles", inflatedObstacles, &NavView::generic_cb);
    subscribe("raw_obstacles", rawObstacles, &NavView::generic_cb);

    if(n_val_ == 0) {
      subscribe("gui_laser", laserscan, &NavView::generic_cb);
    } else {
      subscribe("transient_obstacles_diff", occ_diff_, &NavView::occDiffCallback);
    }
    gwidth = 640;
    gheight = 480;
    init_gui(gwidth, gheight, "nav view");
    glClearColor(1.0,1.0,1.0,0);
    map_surface = NULL;
    map_texture = 0;
    setting_theta = false;
  }

  ~NavView()
  {
    if(map_surface)
      SDL_FreeSurface(map_surface);
  }

  bool load_map();
  bool load_map_from_image(const char*, double);
  bool load_full_transient();

  void occDiffCallback();

  SDL_Surface* flip_vert(SDL_Surface* sfc);

  void generic_cb()
  {
    request_render();
  }

  void render();

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

  virtual void mouse_button(int x, int y, int button, bool is_down);
};

void
quit(int sig)
{
  SDL_Quit();
  ros::fini();
  exit(0);
}

int
main(int argc, char **argv)
{
  signal(SIGINT, quit);
  signal(SIGPIPE, SIG_IGN);

  ros::init(argc, argv);

  int n_val = 0;
  if(argc > 1)
  {
    if(strcmp(argv[1],"transient") == 0)
    {
      n_val = 1;
    }
  }

  NavView view(n_val);
  if(!view.load_map())
  //if(!view.load_map_from_image(argv[1],atof(argv[2])))
    puts("WARNING: failed to load map");
  if(n_val == 1) {
    view.load_full_transient();
  }
  view.main_loop();
  ros::fini();
  return 0;
}

void
NavView::mouse_button(int x, int y, int button, bool is_down)
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

void
NavView::render()
{
  lock.lock();

  glClear(GL_COLOR_BUFFER_BIT);
  glLoadIdentity();
  glScalef(view_scale, view_scale, view_scale);
  glTranslatef(view_x, view_y, 0);

  if(map_texture)
  {
    glEnable(GL_TEXTURE_2D);
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
    glVertex3f( map_res*map_width, 0, 0 );

    //Bottom-right vertex (corner)
    glTexCoord2i( 1, 1 );
    glVertex3f( map_res*map_width, map_res*map_height, 0 );

    //Top-right vertex (corner)
    glTexCoord2i( 0, 1 );
    glVertex3f( 0, map_res*map_height, 0 );
    glEnd();
  }
  // don't texture-map the vector graphics
  glDisable(GL_TEXTURE_2D);

  const float robot_rad = 0.3;
  try
  {
    libTF::TFPose2D robotPose;
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.yaw = 0;
    robotPose.frame = "base";
    robotPose.time = 0;

    libTF::TFPose2D mapPose = tf.transformPose2D("map", robotPose);

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
  catch(...)
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

  local_path.lock();
  glColor3f(local_path.color.r,local_path.color.g,local_path.color.b);
  glBegin(GL_LINES);
  for(unsigned int i=0;i<local_path.get_points_size();i++)
    glVertex2f(local_path.points[i].x,local_path.points[i].y);
  glEnd();
  local_path.unlock();

  robot_footprint.lock();
  glColor3f(robot_footprint.color.r,robot_footprint.color.g,robot_footprint.color.b);
  glBegin(GL_LINE_LOOP);
  for(unsigned int i=0;i<robot_footprint.get_points_size();i++)
    glVertex2f(robot_footprint.points[i].x,robot_footprint.points[i].y);
  glEnd();
  robot_footprint.unlock();  inflatedObstacles.lock();

  // Rendering obstacle data from a map is done by first rendering the inflated obstacles and then over-writing them
  // with the raw obstacles which should be on the interior
  glColor3f(inflatedObstacles.color.r,inflatedObstacles.color.g,inflatedObstacles.color.b);
  glBegin(GL_POINTS);
  for(unsigned int i=0;i<inflatedObstacles.get_points_size();i++)
    glVertex2f(inflatedObstacles.points[i].x,inflatedObstacles.points[i].y);
  glEnd();
  inflatedObstacles.unlock();


  rawObstacles.lock();
  glColor3f(rawObstacles.color.r,rawObstacles.color.g,rawObstacles.color.b);
  glBegin(GL_POINTS);
  for(unsigned int i=0;i<rawObstacles.get_points_size();i++)
    glVertex2f(rawObstacles.points[i].x,rawObstacles.points[i].y);
  glEnd();
  rawObstacles.unlock();


  if(n_val_ == 0) {
    laserscan.lock();
    glColor3f(laserscan.color.r,laserscan.color.g,laserscan.color.b);
    glBegin(GL_POINTS);
    for(unsigned int i=0;i<laserscan.get_points_size();i++)
      glVertex2f(laserscan.points[i].x,laserscan.points[i].y);
    glEnd();
    laserscan.unlock();
  } else {
    glColor3f(0,0,255);
    glBegin(GL_POINTS);
    for(std::list<ObstaclePoint>::iterator it = nav_view_points_.begin();
        it != nav_view_points_.end();
        it++)
    {
      glVertex2f((*it).x, (*it).y);
    }
    glEnd();
  }

  SDL_GL_SwapBuffers();

  lock.unlock();
}

bool NavView::load_full_transient() {
  pr2_srvs::TransientObstacles::request req;
  pr2_srvs::TransientObstacles::response res;

  puts("Calling full transient obstacle service");
  while(!ros::service::call("transient_obstacles_full", req, res)) {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  puts("Got full transient obstacles");

  for(size_t i = 0; i < res.obs.get_points_size(); i++)
  {
    ObstaclePoint nvp;
    nvp.x = res.obs.points[i].x;
    nvp.y = res.obs.points[i].y;
    nav_view_points_.push_back(nvp);
  }
  full_transient_init_ = true;
  return true;
}

void NavView::occDiffCallback() {

  if(!full_transient_init_) {
    printf("No full transient init.\n");
    return;
  }

  //first take out points
  for(size_t i = 0; i < occ_diff_.get_unocc_points_size(); i++)
  {
    bool found = false;
    for(std::list<ObstaclePoint>::iterator it = nav_view_points_.begin();
        it != nav_view_points_.end();
        it++)
    {
      if((*it).x == occ_diff_.unocc_points[i].x &&
         (*it).y == occ_diff_.unocc_points[i].y)
      {
        nav_view_points_.erase(it);
        found = true;
        //std::cout << "Point " << occ_diff_.unocc_points[i].x << " " <<  occ_diff_.unocc_points[i].y << " was found in nav points.\n";
        break;
      }
    }
    if(!found) {
      std::cout << "Point " << occ_diff_.unocc_points[i].x << " " <<  occ_diff_.unocc_points[i].y << " not found in nav points.\n";
    }
  }

  //adding new points
  for(size_t i = 0; i < occ_diff_.get_occ_points_size(); i++)
  {
    ObstaclePoint nvp;
    nvp.x = occ_diff_.occ_points[i].x;
    nvp.y = occ_diff_.occ_points[i].y;
    nav_view_points_.push_back(nvp);
  }
}

bool
NavView::load_map()
{
  std_srvs::StaticMap::request  req;
  std_srvs::StaticMap::response resp;
  puts("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    puts("request failed; trying again...");
    usleep(1000000);
  }
  puts("success");
  printf("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.width,
         resp.map.height,
         resp.map.resolution);

  map_res = resp.map.resolution;

  // Pad dimensions to power of 2
  map_width = (int)pow(2,ceil(log2(resp.map.width)));
  map_height = (int)pow(2,ceil(log2(resp.map.height)));

  printf("Padded dimensions to %d X %d\n", map_width, map_height);

  // Expand it to be RGB data
  unsigned char* pixels = new unsigned char[map_width * map_height * 3];
  assert(pixels);
  memset(pixels,255,map_width*map_height*3);
  for(unsigned int j=0;j<resp.map.height;j++)
  {
    for(unsigned int i=0;i<resp.map.width;i++)
    {
      unsigned char val;
      if(resp.map.data[j*resp.map.width+i] == 100)
        val = 0;
      else if(resp.map.data[j*resp.map.width+i] == 0)
        val = 255;
      else
        val = 127;

      int pidx = 3*(j*map_width + i);
      pixels[pidx+0] = val;
      pixels[pidx+1] = val;
      pixels[pidx+2] = val;
    }
  }

  glEnable( GL_TEXTURE_2D );

  // Have OpenGL generate a texture object handle for us
  glGenTextures(1, &map_texture);

  // Bind the texture object
  glBindTexture(GL_TEXTURE_2D, map_texture);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // Set the texture's stretching properties
  // explicitly show the granularity of the underlying grid-map
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // Edit the texture object's image data
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
               map_width, map_height, 0,
               GL_RGB, GL_UNSIGNED_BYTE,
               pixels);
  return(true);
}

bool
NavView::load_map_from_image(const char* fname, double res)
{
  map_res = res;
  glEnable( GL_TEXTURE_2D );
  // Load the image from file
  SDL_Surface* temp;
  if(!(temp = IMG_Load(fname)))
  {
    printf("Warning: failed to load map from file \"%s\"\n",
           fname);
    return(false);
  }

  map_width = temp->w;
  map_height = temp->h;

  // Check that the image's width is a power of 2
  if((temp->w & (temp->w - 1)) != 0) {
    printf("warning: image width is not a power of 2\n");
  }
  // Also check if the height is a power of 2
  if((temp->h & (temp->h - 1)) != 0) {
    printf("warning: image height is not a power of 2\n");
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
    return(false);
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
  return(true);
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
