#include <unistd.h>
#include <assert.h>
#include <math.h>
#include <GL/gl.h>
#include <SDL/SDL_image.h>
#include "ros/node.h"
#include "std_msgs/MsgRobotBase2DOdom.h"
#include "std_msgs/MsgParticleCloud2D.h"
#include "std_msgs/MsgPlanner2DGoal.h"
#include "sdlgl/sdlgl.h"

class NavView : public ros::node, public ros::SDLGL
{
public:
  MsgRobotBase2DOdom odom;
  MsgParticleCloud2D cloud;
  MsgPlanner2DGoal goal;
  float view_scale, view_x, view_y;
  SDL_Surface* map_surface;
  GLuint map_texture;
  double map_res;
  int gwidth, gheight;

  NavView() : ros::node("nav_view",false),
    view_scale(10), view_x(0), view_y(0)
  {
    advertise<MsgPlanner2DGoal>("goal");
    subscribe("odom", odom, &NavView::odom_cb);
    subscribe("particlecloud", cloud, &NavView::odom_cb);
    gwidth = 1024;
    gheight = 768;
    init_gui(gwidth, gheight, "nav view");
    glClearColor(1.0,1.0,1.0,0);
    map_surface = NULL;
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

    glPushMatrix();
      odom.lock();
      glTranslatef(odom.pos.x, odom.pos.y, 0);
      glRotatef(odom.pos.th * 180 / M_PI, 0, 0, 1);
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
    if((button == 2) && !is_down) // middle mouse button; set goal
    {
      // Convert to conventional coords
      double gx = (x-gwidth/2.0)/view_scale-view_x;
      double gy = -((y-gheight/2.0)/view_scale+view_y);
      
      // Send out the goal
      goal.goal.x = gx;
      goal.goal.y = gy;
      goal.goal.th = 0.0;
      goal.enable = 1;
      publish("goal", goal);
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
