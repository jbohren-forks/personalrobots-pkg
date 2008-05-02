#include <cstdio>
#include <cmath>

double ext[4][4];
double ray[3];
double laser_norm[3], laser[3];
double cam_point[3], world_point[3]; // this is the output
    
const double tilt = 30*M_PI/180;



void extrinsics(double th1, double th2, double th3, double beta,
                double L1, double L2, double L3)
{
  ext[0][0] = -cos(beta)*cos(th2)*sin(th3) - sin(th2)*cos(th3);
  ext[0][1] = -cos(beta)*cos(th2)*cos(th3) + sin(th2)*sin(th3);
  ext[0][2] =  sin(beta)*cos(th2);
  ext[0][3] =  L1;

  ext[1][0] = -sin(th1)*(cos(th2)*cos(th3) - cos(beta)*sin(th2)*sin(th3)) -
               sin(beta)*cos(th1)*sin(th3);
  ext[1][1] = -sin(th1)*(-cos(th2)*sin(th3) - cos(beta)*sin(th2)*cos(th3)) -
               sin(beta)*cos(th1)*cos(th3);
  ext[1][2] = -sin(beta)*sin(th1)*sin(th2) - cos(beta)*cos(th1);
  ext[1][3] = -sin(th1)*L2 - cos(th1)*L3;

  ext[2][0] =  cos(th1)*(cos(th2)*cos(th3) - cos(beta)*sin(th2)*sin(th3)) -
               sin(beta)*sin(th1)*sin(th3);
  ext[2][1] =  cos(th1)*(-cos(th2)*sin(th3) - cos(beta)*sin(th2)*cos(th3)) -
               sin(beta)*sin(th1)*cos(th3);
  ext[2][2] =  sin(beta)*cos(th1)*sin(th2) - cos(beta)*sin(th1);
  ext[2][3] =  cos(th1)*L2 - sin(th1)*L3;

  ext[3][0] = 0;
  ext[3][1] = 0;
  ext[3][2] = 0;
  ext[3][3] = 1;

  laser_norm[0] = ext[0][0];
  laser_norm[1] = ext[1][0];
  laser_norm[2] = ext[2][0];

  laser[0] = ext[0][3];
  laser[1] = ext[1][3];
  laser[2] = ext[2][3];
}

void intrinsics(double u, double v)
{
  double fx = 949.415, fy = 870.535;
  double u0 = 340.7, v0 = 278.8;
  ray[0] = (u - u0) / fx;
  ray[1] = (v - v0) / fy;
  ray[2] = 1;
  double ray_norm = sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
  ray[0] /= ray_norm;
  ray[1] /= ray_norm;
  ray[2] /= ray_norm;
}

void intersect()
{
  double u = (laser_norm[0] * laser[0] + 
              laser_norm[1] * laser[1] + 
              laser_norm[2] * laser[2]) /
             (laser_norm[0] * ray[0] +
              laser_norm[1] * ray[1] +
              laser_norm[2] * ray[2]);
  cam_point[0] = u * ray[0];
  cam_point[1] = u * ray[1];
  cam_point[2] = u * ray[2];
}

void camera_to_world()
{
  // convert from camera frame to world frame
  world_point[0] = cam_point[0];
  world_point[1] = cos(-tilt) * cam_point[1] - sin(-tilt) * cam_point[2];
  world_point[2] = sin(-tilt) * cam_point[1] + cos(-tilt) * cam_point[2];
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    printf("usage: ./borg LASERFILE\n");
    return 1;
  }
  const char *laserfile = argv[1];

  FILE *f = fopen(laserfile,"r");
  if (!f)
  {
    printf("couldn't open [%s]\n", laserfile);
    return 1;
  }
  FILE *out = fopen("points.txt","w");
  int line = 0;
  const double encoder_offset = 240;
  while (!feof(f))
  {
    line++;
    double laser_ang, row, col;
    if (3 != fscanf(f, "%lf %lf %lf\n", &laser_ang, &row, &col))
    {
      printf("error parsing line %d\n", line);
      break;
    }
    if ((line % 1) != 0)
      continue;
    extrinsics(tilt, (encoder_offset - laser_ang)*M_PI/180,
               0, 135*M_PI/180,
               0.48, 0.02, 0.22);
    intrinsics(col, row);
    intersect();
    camera_to_world();
    fprintf(out, "%f %f %f\n", 
      world_point[0], 
      world_point[1], 
      world_point[2]);
  }

  fclose(f);
  fclose(out);

  return 0;
}
