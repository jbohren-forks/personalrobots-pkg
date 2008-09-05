#include <cstdio>
#include <vector>
#include <cmath>
#include <SDL.h>
#include <GL/gl.h>
#include "cloud_viewer/cloud_viewer.h"
#include "matrix_utils.h"

#define PI 3.14159265358979323846

using namespace std;

//vector<vector<double> > Rmotorzero2laser;
//vector<double> Tmotorzero2laser;
//vector<vector<double> > Rcam2laser;
//vector<double> Tcam2laser;
//vector< <double> > T_cam_to_stage;
vector< vector<double> > R_laser_to_stage;
vector<double> T_stage_to_cam;

// stair 2 initial values for extrinsic parameters
// measured angle when laser is pointing straight ahead
double laser_encoder_offset = 1.52; //1.5252; //1.4912; //85.0 * PI/180.0;
// tilt of camera about x-axis (of laser frame)
double laser_tilt = 0; //-0.0786; //1.4912; //-5.0 * PI/180.0;
// displacement of camera frame from laser in laser frame coord (in m)
double dx = -0.1465;//1825; //1945; 
double dy = 0; //0.1067; //4.2 * 2.54/100.;
double dz = 0;//-0.16;//138; //0.114; //0.1742; //4.0 * 2.54/100.;
// angle to account for bracket holding laser not being exactly 90 degrees
double laser_rotation = 0;//-0.0067; //0;//-0.0849;

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
  /*
  Tcam2laser.clear();
  Rcam2laser.clear();
	// tranformation from camera frame to laser frame.
	Tcam2laser.push_back(dx);
	Tcam2laser.push_back(dy);
	Tcam2laser.push_back(dz);
	Rcam2laser = x_axis_rotation(laser_tilt_offset);
  */
  T_stage_to_cam.clear();
  T_stage_to_cam.push_back(dx);
  T_stage_to_cam.push_back(dy);
  T_stage_to_cam.push_back(dz);

	//cout << "Rcam2laser: " << endl;
	//cout << Rcam2laser[0][0] << " " << Rcam2laser[0][1] << " " << Rcam2laser[0][2] << endl;
	//cout << Rcam2laser[1][0] << " " << Rcam2laser[1][1] << " " << Rcam2laser[1][2] << endl;
	//cout << Rcam2laser[2][0] << " " << Rcam2laser[2][1] << " " << Rcam2laser[2][2] << endl;
	//cout << "Tcam2laser: " << endl;
	//cout << Tcam2laser[0] << " " << Tcam2laser[1] << " " << Tcam2laser[2] << endl;
}

void update_laserang_transformation(double laser_angle) 
{
  //Tmotorzero2laser.clear();
  //Rmotorzero2laser.clear();
	// tranformation from motor zero position to laser frame
	//vector<double> v(3,0);
	//Tmotorzero2laser = v;
	//Rmotorzero2laser = Rmultiply(y_axis_rotation(laser_encoder_offset - laser_angle),
	//															z_axis_rotation(laser_rotation));
  double lang = laser_encoder_offset - laser_angle;
  //lang *= 1.1;
  R_laser_to_stage = y_axis_rotation(lang);//laser_encoder_offset - laser_angle);
}

vector<double> intrinsics(double xp, double yp)
{
	vector<double> cam_ray;

  // stair 2 intrinsics
  // measured by Quoc and Aaron:
  //double fx = 554.69171, fy = 555.16712;
  //double x0 = 321.04175, y0 = 260.40547;
  // measured by Morgan:
  double fx = 580, fy = 580; //533.43634, fy = 532.25193;
  double x0 = 320, y0 = 240; //329.769073, y0 = 265.015808; //335.106, y0 = 260.255;
	
// express the pixel ray in the camera frame
	double xc = (xp - x0) / fx;
	double yc = (yp - y0) / fy;
 
	cam_ray.push_back(xc);
	cam_ray.push_back(yc);
	cam_ray.push_back(1);

	double norm = sqrt(cam_ray[0]*cam_ray[0] + cam_ray[1]*cam_ray[1] + 
								cam_ray[2]*cam_ray[2]);
	
	for (size_t i=0; i<cam_ray.size(); i++)
		cam_ray[i] /= norm;
	
	return cam_ray;
}

double sample_x = 0, sample_y = 0, sample_z = 0;
double sample_ang = 0;
double sample_ray_x = 0, sample_ray_y = 0, sample_ray_z = 0;

void generate_cloud() 
{
	vector<double> p0 = T_stage_to_cam; //Tcam2laser;
	vector<double> unit_ray, N, P(3,0);
	vector<double> N_laser_plane (3,0);
  N_laser_plane[0] = 1;
  // rotate the laser about its pointing axis
  N_laser_plane = apply_rotation(N_laser_plane, 
                                 z_axis_rotation(laser_rotation));
  cv->clear_cloud();
	for (size_t i = 0; i < laser_points.size(); i++)
  {
    laser_point pt = laser_points[i];
		// get unit vector along the ray from the camera origin through the pixel
		unit_ray = intrinsics(pt.col, pt.row);
		// transform unit vector along the camera ray into laser frame
		// rotate the laser plane normal from the moving motor frame to the stationary laser frame
		update_laserang_transformation(pt.laser_ang * PI/180);
		N = apply_rotation(N_laser_plane, R_laser_to_stage); //Rmotorzero2laser);
    // now rotate by the x-axis to account for the stage's axis of rotation
    // not being parallel ot the camera's y-axis (since the camera's principal
    // point is not dead-center on the imager)
    N = apply_rotation(N, x_axis_rotation(laser_tilt));
		double t = dot_product(p0,N) / dot_product(unit_ray, N);
		for (size_t j = 0; j < 3; j++)
			P[j] = t*unit_ray[j];
    // finally, rotate the whole frame up 30 degrees so it's level again
    const double wrot = 30 * M_PI / 180.0;
    const double wx = P[0];
    const double wy =  cos(wrot) * P[1] + sin(wrot) * P[2];
    const double wz = -sin(wrot) * P[1] + cos(wrot) * P[2];
    if (i == 90000)
    {
      sample_x = wx;
      sample_y = wy;
      sample_z = wz;
      sample_ang = pt.laser_ang * M_PI/180;
      sample_ray_x = unit_ray[0];
      sample_ray_y = unit_ray[1];
      sample_ray_z = unit_ray[2];
      printf("unit_ray = %f, %f, %f\n", unit_ray[0], unit_ray[1], unit_ray[2]);
      printf("laser ang = %f\n", pt.laser_ang * PI/180);
      printf("laser norm = %f, %f, %f\n", N[0], N[1], N[2]);
    }

    float rowcol[2];
    rowcol[0] = pt.row;
    rowcol[1] = pt.col;
    cv->add_point(wx, wy, wz, 255*pt.r, 255*pt.g, 255*pt.b, rowcol, 2);
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

void draw_plane()
{
  /*
  glColor3f(0,1,0);
  glBegin(GL_POINTS);
  glEnd();
  */
  const float tx = dx;
  const float ty = dy;
  const float tz = dz;
  const float a = laser_encoder_offset - sample_ang;
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(1, 0, 0, 0.2f);
  vector<double> top_laser(3, 0), bot_laser(3, 0);
  top_laser[1] = -5.0;
  bot_laser[1] =  5.0;
  top_laser[2] = 10.0;
  bot_laser[2] = 10.0;
  top_laser = apply_rotation(top_laser, z_axis_rotation(laser_rotation));
  bot_laser = apply_rotation(bot_laser, z_axis_rotation(laser_rotation));
  top_laser = apply_rotation(top_laser, y_axis_rotation(a));
  bot_laser = apply_rotation(bot_laser, y_axis_rotation(a));
  glBegin(GL_TRIANGLES);
    const float tri_len = 10.0f;
    const float tri_height = 5.0f;
    glVertex3f(tx, ty, tz);
    glVertex3f(tx + top_laser[0], ty + top_laser[1], tz + top_laser[2]);
    glVertex3f(tx + bot_laser[0], ty + bot_laser[1], tz + bot_laser[2]);
    //glVertex3f(tx, ty,     tz);
    //glVertex3f(tx + -tri_len * sin(a), ty + tri_height, tz + tri_len * cos(a));
    //glVertex3f(tx + -tri_len * sin(a), ty - tri_height, tz + tri_len * cos(a));
  glEnd();
  glDisable(GL_BLEND);

  glColor3f(0,1,0);
  glPointSize(5.0);
  glBegin(GL_POINTS);
    glVertex3f(sample_x, sample_y, sample_z);
  glEnd();
  glPointSize(1.0);


  const float line_len = 10.0f;
  const double wrot = 30 * M_PI / 180.0;
  float sy = sample_ray_y *  cos(wrot) + sample_ray_z * sin(wrot);
  float sz = sample_ray_y * -sin(wrot) + sample_ray_z * cos(wrot);
  glColor3f(1, 1, 0);
  glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(line_len * sample_ray_x,
               line_len * sy,
               line_len * sz);
  glEnd();


  /*
double dx = 0.1465;//1825; //1945; //6.4 * 2.54 / 100; //0.1463; //6.1 * 2.54/100.;
double dy = 0; //0.1067; //4.2 * 2.54/100.;
double dz = 0.16;//138; //0.114; //0.1742; //4.0 * 2.54/100.;
*/
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
  cv->set_postrender_cb(draw_plane);
  if (SDL_Init(SDL_INIT_VIDEO) < 0)
  {
    fprintf(stderr, "video init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,   24);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  const int w = 800, h = 600;
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
              dx += 0.002;
            else
              dx -= 0.002;
          }
          else if (event.key.keysym.sym == SDLK_t)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              laser_tilt += 0.01;//02;
            else
              laser_tilt -= 0.01;//02;
          }
          else if (event.key.keysym.sym == SDLK_z)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              dz += 0.01;//02;
            else
              dz -= 0.01;//02;
          }
          else if (event.key.keysym.sym == SDLK_y)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              dy += 0.01;//02;
            else
              dy -= 0.01;//02;
          }
          else if (event.key.keysym.sym == SDLK_l)
          {
            if (event.key.keysym.mod & KMOD_LSHIFT)
              laser_encoder_offset += 0.002;
            else
              laser_encoder_offset -= 0.002;
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
          {
            cv->keypress(event.key.keysym.sym);
            cv->render();
            SDL_GL_SwapBuffers();
            break;
          }

          update_cam2laser_transformation();
          //read_laserfile();
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
          printf("laser tilt = %f\n", laser_tilt);
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
