#include <cstdio>
#include "mqmat.h"
#include "mqdh.h"
#include "mqunits.h"

//#define DEBUG

using namespace std;
using namespace mqmath;

double ray[3], laser_norm[3], laser_origin[3], cam_point[3], world_point[3];
const double cam_tilt = 30.0 * units::DEGREES();

void intrinsics(double u, double v)
{
  double fx = 949.415, fy = 870.535;
  double u0 = 340.7  , v0 = 278.8;
  // express the pixel ray in the base kinematic frame
  ray[0] = 1; //(u - u0) / fx;
  ray[1] = -(v - v0) / fy;
  ray[2] = (u - u0) / fx;

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

int main(int argc, char **argv)
{
  printf("LOWER YOUR SHIELDS\n\n");

  const double laser_encoder_offset = 150 * units::DEGREES();
  const double baseline   = 9.25 * units::INCHES();
  const double stage_tilt = 15 * units::DEGREES();
  const double stage_back = 0.5  * units::INCHES();
  const double stage_up   = 7.5  * units::INCHES();
  const double a2 = stage_up * sin(-stage_tilt) + stage_back * cos(-stage_tilt);
  const double d2 = stage_up * cos(-stage_tilt) - stage_back * sin(-stage_tilt);

//  mqdh T0(0, 0, 0, cam_tilt);
  mqdh T0(0, 0, 0, 0);
  mqdh T1(0, 0, baseline, stage_tilt);
  mqdh T3(M_PI/2, 0, 0, 0 * units::DEGREES());

  const char *laserfile = "laserData.txt";
  FILE *f = fopen(laserfile, "r");
  if (!f)
  {
    printf("couldn't open [%s]\n", laserfile);
    return 1;
  }
  FILE *out = fopen("points.txt", "w");
  int line = 0;
  while (!feof(f))
  {
    line++;
    double laser_ang, row, col;
    if (3 != fscanf(f, "%lf %lf %lf\n", &laser_ang, &row, &col))
    {
      printf("error in parse\n");
      break;
    }
#ifdef DEBUG
    if (line > 4)
      break;
#endif
    double stage = laser_encoder_offset - laser_ang * units::DEGREES();
    mqdh T2(-M_PI/2, a2, d2, stage);
    mqmat<4,4> T(T0 * T1 * T2 * T3);
    extrinsics(T);
    intrinsics(col, row);
    intersect();
    camera_to_world();
#ifdef DEBUG
    cout << T << endl;
    printf("laser angle:  %f\n", stage * 180 / M_PI);
    printf("image point:  %f %f\n", row, col);
    printf("pixel ray:    %f %f %f\n", ray[0], ray[1], ray[2]);
    printf("laser origin: %f %f %f\n", laser_origin[0], laser_origin[1], laser_origin[2]);
    printf("laser norm:   %f %f %f\n", laser_norm[0], laser_norm[1], laser_norm[2]);
    printf("world:        %f %f %f\n", world_point[0], world_point[1], world_point[2]);
    printf("\n");
#endif

    fprintf(out, "%f %f %f\n", world_point[0], world_point[1], world_point[2]);

  }
  fclose(f);
  fclose(out);



  return 0;
}

