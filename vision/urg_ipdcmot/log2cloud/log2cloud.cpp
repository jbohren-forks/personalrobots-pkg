#include <vector>
#include <cmath>
#include <cassert>
#include <cstdio>
using namespace std;

double laser_ang, laser_ang_time;
FILE *laser_file, *motor_file;
double laser_time, min_ang, max_ang;
int num_meas;
double *ranges = NULL, *intensities = NULL;

class point
{
public:
  double x, y, z, inten;
};

vector<point> pts;

bool get_next_motor_ang()
{
  if (feof(motor_file))
  {
    printf("end of motor log\n");
    return false;
  }
  if (2 != fscanf(motor_file, "%lf %lf\n", &laser_ang_time, &laser_ang))
  {
    printf("couldn't parse motor log\n");
    return false;
  }
  return true;
}

bool get_next_laser_scan()
{
  if (4 != fscanf(laser_file, "%lf %lf %lf %d ", &laser_time, &min_ang,
                  &max_ang, &num_meas))
  {
    printf("couldn't parse laser log header\n");
    return false;
  }
  if (!ranges)
  {
    printf("allocating space for %d ranges\n", num_meas);
    ranges = new double[num_meas];
    intensities = new double[num_meas];
  }
  for (int i = 0; i < num_meas; i++)
  {
    if (2 != fscanf(laser_file, "%lf %lf ", &ranges[i], &intensities[i]))
    {
      printf("couldn't parse laser entry\n");
      return false;
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  if (argc <= 2)
  {
    printf("usage: log2cloud LASERFILE MOTORFILE\n");
    return 1;
  }
  laser_file = fopen(argv[1], "r");
  if (!laser_file)
  {
    printf("couldn't open laser file [%s]\n", argv[1]);
    return 2;
  }
  motor_file = fopen(argv[2], "r");
  if (!motor_file)
  {
    printf("couldn't open motor file [%s]\n", argv[2]);
    return 3;
  }
  get_next_motor_ang();
  printf("starting motor time: %f angle: %f\n", laser_ang_time, laser_ang);
  int count = 0;
  bool ok = true;
  while (ok && get_next_laser_scan())
  {
    while (ok && laser_ang_time < laser_time)
      ok &= get_next_motor_ang();
    count++;
    for (int i = 0; i < num_meas; i++)
    {
      double r = ranges[i];
      double th = min_ang + i * (max_ang - min_ang) / (num_meas - 1);
      double psi = (M_PI / 180) * (165 - laser_ang);
      double x_laser = r * cos(th);
      double z_laser = r * sin(th);
      double x_stage = x_laser * cos(psi);
      double y_stage = x_laser * sin(psi);
      double z_stage = z_laser;
      double x_world = x_stage;
      const double stage_tilt = -30 * M_PI / 180.0;
      double y_world = cos(stage_tilt) * y_stage - sin(stage_tilt) * z_stage;
      double z_world = sin(stage_tilt) * y_stage + cos(stage_tilt) * z_stage;
      point p;
      p.x = x_world;
      p.y = -z_world;
      p.z = y_world;
      p.inten = intensities[i];
      pts.push_back(p);
    }
  }
  printf("read %d scans\n", count);
  double min_inten = 1e9, max_inten = -1e9;
  /*
  for (size_t i = 0; i < pts.size(); i++)
  {
    if (pts[i].inten < min_inten)
      min_inten = pts[i].inten;
    if (pts[i].inten > max_inten)
      max_inten = pts[i].inten;
  }
  printf("intensity range: [%f, %f]\n", min_inten, max_inten);
  */
  FILE *out = fopen("out.txt", "w");
  assert(out);
  max_inten = 4000;
  min_inten = 2000;
  for (size_t i = 0; i < pts.size(); i++)
  {
    pts[i].inten = (pts[i].inten - min_inten) / (max_inten - min_inten);
    if (pts[i].inten > 1.0)
      pts[i].inten = 1.0;
    else if (pts[i].inten < 0)
      pts[i].inten = 0.0;
    fprintf(out, "%f %f %f %f %f %f\n", pts[i].x, pts[i].y, pts[i].z,
            pts[i].inten, pts[i].inten, pts[i].inten);
  }
  fclose(out);
  fclose(motor_file);
  fclose(laser_file);
  return 0;
}

