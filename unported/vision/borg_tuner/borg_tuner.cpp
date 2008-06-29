#include <cstdio>
#include <SDL.h>
#include <GL/gl.h>
#include "cloud_viewer/cloud_viewer.h"
#include "mqmat.h"
#include "mqdh.h"
#include "mqunits.h"

//#define DEBUG

using namespace std;
using namespace mqmath;

double ray[3], laser_norm[3], laser_origin[3], cam_point[3], world_point[3];
const double cam_tilt = 30.0 * units::DEGREES(); //30



//void intrinsics(double u, double v)
//{
////	focal length: (393.756   393.798)
////principal point:  (315.432  247.361)
//
//  //double fx = 949.415, fy = 870.535;
//  //double u0 = 340.7  , v0 = 278.8;
//	double fx = 393.756, fy = 393.798;
//	double u0 = 315.432 , v0 = 247.361;
//  // express the pixel ray in the base kinematic frame
//  ray[0] = 1; 
//  ray[1] = -(v - v0) / fy;
//  ray[2] = (u - u0) / fx;
//
//  double ray_norm = sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]);
//  ray[0] /= ray_norm;
//  ray[1] /= ray_norm;
//  ray[2] /= ray_norm;
//}

void intrinsics(double xp, double yp)
{

	//394.493   0   318.619
	//0  394.327  247.207
	//0 0 1
	//first two radial distortion coefficients:
	//-0.344023   0.127415
	//and the first two tangential distortion coefficients:
	//-0.000815   -0.000350
	double fx = 392.953, fy = 392.757;
	double x0 = 316.125, y0 = 248.496; //318.619 , y0 = 247.207;

	// express the pixel ray in the base kinematic frame
	//ray[0] = 1; 
	//ray[1] = -(y - y0) / fy;
	//ray[2] = (x - x0) / fx;
	//double zd =	1;
	double yd = (yp - y0) / fy;
	double xd = (xp - x0) / fx;
/*

	double r2 = pow(xd,2) + pow(yd,2);

	double radialDistortion =  1 + k1 *	r2 + k2 * pow(r2,2) + k5 * pow(r2,2) * r2;

	double tangentialX = 2 * k3 * xd * yd + k4 * ( r2  + 2 * pow(xd,2));
	double tangentialY = k3 * (r2 + 2 * pow(yd,2)) + 2 * k4 * xd * yd;

	double x =  (xd - tangentialX) / radialDistortion;
	double y =  (yd - tangentialY) / radialDistortion;
 */ 
	ray[0] = 1;
	ray[1] = -yd;
	ray[2] = xd;
	

	double ray_norm = sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]);
	ray[0] /= ray_norm;
	ray[1] /= ray_norm;
	ray[2] /= ray_norm;

}

void extrinsics(const mqmat<4,4> &T)
{
  laser_norm[0] = T.at(0, 0);
  laser_norm[1] = T.at(1, 0);
  laser_norm[2] = T.at(2, 0);
  laser_origin[0] = T.at(0, 3);
  laser_origin[1] = T.at(1, 3);
  laser_origin[2] = T.at(2, 3);
}

void intersect()
{
  double u = (laser_norm[0] * laser_origin[0] +
              laser_norm[1] * laser_origin[1] +
              laser_norm[2] * laser_origin[2]) /
             (laser_norm[0] * ray[0] +
              laser_norm[1] * ray[1] +
              laser_norm[2] * ray[2]);
  cam_point[0] = u * ray[0];
  cam_point[1] = u * ray[1];
  cam_point[2] = u * ray[2];
}

void camera_to_world()
{ 
  // express these in the standard graphics frame:
  // x = right
  // y = down
  // z = into the world
  world_point[0] = cam_point[2]; 
  world_point[1] = -cam_point[1]; //cos(-cam_tilt) * cam_point[1] - sin(-cam_tilt) * cam_point[2]; 
  world_point[2] = cam_point[0]; //sin(-cam_tilt) * cam_point[1] + cos(-cam_tilt) * cam_point[2];
  // un-rotate the camera

  double tmp[3];
  tmp[1] = cos(-cam_tilt) * world_point[1] - sin(-cam_tilt) * world_point[2];
  tmp[2] = sin(-cam_tilt) * world_point[1] + cos(-cam_tilt) * world_point[2];
  world_point[1] = tmp[1];
  world_point[2] = tmp[2];

}
/*  
double laser_encoder_offset = 152 * units::DEGREES();
double baseline   = 6.25 * units::INCHES(); //9.25
double stage_tilt = 2 * units::DEGREES();//15
double stage_back = -2.5  * units::INCHES();
double stage_up   = 5.5  * units::INCHES();
double laser_rotation = -2.0 * units::DEGREES();
*/
double laser_encoder_offset = 2.5929;
double baseline   = 0.16875;
double stage_tilt = -0.07509;
double stage_back = -0.163;
double stage_up   = 0.0897;
double laser_rotation = -0.0849;

char *laserfile = NULL;
CloudViewer *cv = NULL;

class laser_point
{
public:
  double laser_ang, row, col, r, g, b;
};
vector<laser_point> laser_points;

void generate_cloud()
{
  double a2 = stage_up * sin(-stage_tilt) + stage_back * cos(-stage_tilt);
  double d2 = stage_up * cos(-stage_tilt) - stage_back * sin(-stage_tilt);
  mqdh T0(0, 0, 0, 0);
  mqdh T1(0, 0, baseline, stage_tilt);
  mqdh T3(M_PI/2, 0, 0, laser_rotation); //-2 * units::DEGREES()); //M_PI in linux
  glPointSize(2);
  if (laser_points.size() == 0)
  {
    FILE *f = fopen(laserfile, "r");
    if (!f)
    {
      printf("couldn't open [%s]\n", laserfile);
      return;
    }
    int line = 0;
    while (!feof(f))
    {
      line++;
      double laser_ang, row, col, r, g, b;
      if (6 != fscanf(f, "%lf %lf %lf %lf %lf %lf\n", &laser_ang, &row, &col,
                      &r, &g, &b))
      {
        printf("error in parse\n");
        break;
      }
      if ((line % 1) != 0)
        continue;
      laser_point pt;
      pt.laser_ang = laser_ang;
      pt.row = row;
      pt.col = col;
      pt.r = r;
      pt.g = g;
      pt.b = b;
      laser_points.push_back(pt);
    }
    fclose(f);
  }
  cv->clear_cloud();
  for (size_t i = 0; i < laser_points.size(); i++)
  {
    laser_point pt = laser_points[i];
    double stage = laser_encoder_offset - pt.laser_ang * units::DEGREES();
    mqdh T2(-M_PI/2, a2, d2, stage); //M_PI in linux
    mqmat<4,4> T(T0 * T1 * T2 * T3);
    extrinsics(T);
    intrinsics(pt.col, pt.row);
    intersect();
    camera_to_world();
    cv->add_point(world_point[0], world_point[1], world_point[2], 255*pt.r, 255*pt.g, 255*pt.b);
  }
}

int main(int argc, char **argv)
{
  printf("LOWER YOUR SHIELDS\n\n");
  if (argc < 2)
  {
    printf("usage: borg_tuner LASERFILE\n");
    return 1;
  }
  cv = new CloudViewer();
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "video init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   24);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  const int w = 1280, h = 960;
  if (SDL_SetVideoMode(w, h, 32, SDL_OPENGL | SDL_HWSURFACE) == 0)
  {
    fprintf(stderr, "setvideomode failed: %s\n", SDL_GetError());
    return 2;
  }
  cv->set_look_tgt(0, 0.5, 1.0);
  cv->render();
  SDL_GL_SwapBuffers();

  cv->set_opengl_params(w,h);

  laserfile = argv[1];
  generate_cloud();

  bool done = false;
  while(!done)
  {
    usleep(1000);
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
      switch(event.type)
      {
        case SDL_MOUSEMOTION:
          cv->mouse_motion(event.motion.x, event.motion.y, event.motion.xrel, event.motion.yrel);
          cv->render();
          SDL_GL_SwapBuffers();
          break;
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
        {
          int button = -1;
          switch(event.button.button)
          {
            case SDL_BUTTON_LEFT: button = 0; break;
            case SDL_BUTTON_MIDDLE: button = 1; break;
            case SDL_BUTTON_RIGHT: button = 2; break;
          }
          cv->mouse_button(event.button.x, event.button.y,
            button, (event.type == SDL_MOUSEBUTTONDOWN ? true : false));
          break;
        }
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
          {
            done = true;
            break;
          }
          else if (event.key.keysym.sym == SDLK_b)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              baseline += 0.01;
            else
              baseline -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_t)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              stage_tilt += 0.01;
            else
              stage_tilt -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_c)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              stage_back += 0.01;
            else
              stage_back -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_u)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              stage_up += 0.01;
            else
              stage_up -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_l)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              laser_encoder_offset += 0.01;
            else
              laser_encoder_offset -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_r)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              laser_rotation += 0.01;
            else
              laser_rotation -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_SPACE)
            cv->write_file("cloud.txt");
          else
            cv->keypress(event.key.keysym.sym);
          generate_cloud();
          cv->render();
          SDL_GL_SwapBuffers();
          printf("keys: r = laser rotation, l = encoder offset, b = baseline, t = tilt\n");
          printf("      c = stage back, u = stage up\n");
          printf("baseline = %f\n", baseline);
          printf("stage_tilt = %f\n", stage_tilt);
          printf("stage_back = %f\n", stage_back);
          printf("stage_up = %f\n", stage_up);
          printf("laser encoder offset = %f\n", laser_encoder_offset);
          printf("laser rotatoin = %f\n", laser_rotation);
          printf("\n");
          break;
        case SDL_QUIT:
          done = true;
          break;
      }
    }
  }
  delete cv;
  SDL_Quit();

  return 0;
}

