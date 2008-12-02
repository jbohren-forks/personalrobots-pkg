#include <cstdio>
#include <vector>
#include <math.h>
#include <SDL.h>
#include <GL/gl.h>
#include "cloud_viewer/cloud_viewer.h"
#include "matrix_utils.h"

#define PI 3.14159265358979323846

using namespace std;

vector<vector<double> > Rmotorzero2laser;
vector<double> Tmotorzero2laser;
vector<vector<double> > Rcam2laser;
vector<double> Tcam2laser;

// stair 2 initial values for extrinsic parameters
// measured angle when laser is pointing straight ahead
double laser_encoder_offset = 85.0 * PI/180.0;
// tilt of camera about x-axis (of laser frame)
double laser_tilt_offset = -5.0 * PI/180.0;
// displacement of camera frame from laser in laser frame coord (in m)
double dx = 6.1 * 2.54/100.;
double dy = 4.2 * 2.54/100.;
double dz = 4.0 * 2.54/100.;
// angle to account for bracket holding laser not being exactly 90 degrees
double laser_rotation = 0;//-0.0849;

char *laserfile = NULL;
CloudViewer *cv = NULL;

class laser_point
{
public:
  double laser_ang, row, col, r, g, b;
};
vector<laser_point> laser_points;


void update_cam2laser_transformation()
{	
  Tcam2laser.clear();
  Rcam2laser.clear();
	// tranformation from camera frame to laser frame.
	Tcam2laser.push_back(dx);
	Tcam2laser.push_back(dy);
	Tcam2laser.push_back(dz);
	Rcam2laser = x_axis_rotation(laser_tilt_offset);

	//cout << "Rcam2laser: " << endl;
	//cout << Rcam2laser[0][0] << " " << Rcam2laser[0][1] << " " << Rcam2laser[0][2] << endl;
	//cout << Rcam2laser[1][0] << " " << Rcam2laser[1][1] << " " << Rcam2laser[1][2] << endl;
	//cout << Rcam2laser[2][0] << " " << Rcam2laser[2][1] << " " << Rcam2laser[2][2] << endl;
	//cout << "Tcam2laser: " << endl;
	//cout << Tcam2laser[0] << " " << Tcam2laser[1] << " " << Tcam2laser[2] << endl;
	
	return;
}

void update_laserang_transformation(double laser_angle) 
{
  Tmotorzero2laser.clear();
  Rmotorzero2laser.clear();
	// tranformation from motor zero position to laser frame
	vector<double> v(3,0);
	Tmotorzero2laser = v;
	Rmotorzero2laser = Rmultiply(y_axis_rotation(laser_encoder_offset - laser_angle),
																z_axis_rotation(laser_rotation));
	return;
}

vector<double> intrinsics(double xp, double yp)
{
	vector<double> cam_ray;

  // stair 2 intrinsics
  // measured by Quoc and Aaron:
  //double fx = 554.69171, fy = 555.16712;
  //double x0 = 321.04175, y0 = 260.40547;
  // measured by Morgan:
  double fx = 535.985, fy = 535.071;
  double x0 = 335.106, y0 = 260.255;
	
// express the pixel ray in the camera frame
	double yc = (yp - y0) / fy;
	double xc = (xp - x0) / fx;
 
	cam_ray.push_back(xc);
	cam_ray.push_back(yc);
	cam_ray.push_back(1);

	double norm = sqrt(cam_ray[0]*cam_ray[0] + cam_ray[1]*cam_ray[1] + 
								cam_ray[2]*cam_ray[2]);
	
	for (size_t i=0; i<cam_ray.size(); i++) {
		cam_ray[i] /= norm;
	}
	
	return cam_ray;
}

void generate_cloud() 
{
	// origin of camera frame in laser frame coord
	vector<double> p0 = Tcam2laser;
	vector<double> V;
	vector<double> unit_ray;
	vector<double> N_laser_plane (3,0);
  N_laser_plane[0] = 1;
	vector<double> N;
	double t;
	vector<double> P (3,0);

  cv->clear_cloud();

	for (size_t i = 0; i < laser_points.size(); i++) {	

    laser_point pt = laser_points[i];
	
		// get unit vector along the ray from the camera origin through the pixel
		unit_ray = intrinsics(pt.col, pt.row);
		// transform unit vector along the camera ray into laser frame
		V = apply_rotation(unit_ray, Rcam2laser);

		// rotate the laser plane normal from the moving motor frame to the stationary laser frame
		update_laserang_transformation(pt.laser_ang*PI/180);
		N = apply_rotation(N_laser_plane, Rmotorzero2laser);
		t = -(dot_product(p0,N)/dot_product(V,N));
		for (size_t j = 0; j<V.size(); j++) {
			P[j] = p0[j] + t*V[j];
		}

    float rowcol[2];
    rowcol[0] = pt.row;
    rowcol[1] = pt.col;
    cv->add_point(P[0], P[1], P[2], 255*pt.r, 255*pt.g, 255*pt.b,
                  rowcol, 2);
	}
	return;
}

void read_laserfile()
{
  laser_points.clear();
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

  return;
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

	update_cam2laser_transformation();

	read_laserfile();

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
          else if (event.key.keysym.sym == SDLK_x)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              dx += 0.01;
            else
              dx -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_t)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              laser_tilt_offset += 0.01;
            else
              laser_tilt_offset -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_z)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              dz += 0.01;
            else
              dz -= 0.01;
          }
          else if (event.key.keysym.sym == SDLK_y)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              dy += 0.01;
            else
              dy -= 0.01;
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

          update_cam2laser_transformation();
          read_laserfile();
          generate_cloud();
          cv->render();
          SDL_GL_SwapBuffers();
          printf("keys: x = x-distance between laser and camera\n");
          printf("      y = y-distance between laser and camera\n");
          printf("      z = z-distance between laser and camera\n");
          printf("      l = encoder offset, t = tilt, r = laser rotation\n");
          printf("x = %f\n", dx);
          printf("y = %f\n", dy);
          printf("z = %f\n", dz);
          printf("laser encoder offset = %f\n", laser_encoder_offset);
          printf("laser tilt = %f\n", laser_tilt_offset);
          printf("laser rotation = %f\n", laser_rotation);
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
