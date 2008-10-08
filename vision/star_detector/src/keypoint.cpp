#include "star_detector/keypoint.h"
#include <cstdio>

void WriteKeypoints(std::string file_name, std::vector<Keypoint> const& pts)
{
  FILE* file = fopen(file_name.c_str(), "w");
  if (file) {
    fprintf(file, "# %d points\n", pts.size());
    
    typedef std::vector<Keypoint>::const_iterator iter;
    for (iter i = pts.begin(); i != pts.end(); ++i) {
      fprintf(file, "%d %d %f %f\n", i->x, i->y, i->scale, i->response);
    }
    fclose(file);
  }
}

std::vector<Keypoint> ReadKeypoints(std::string file_name)
{
  std::vector<Keypoint> pts;
  FILE* file = fopen(file_name.c_str(), "r");
  if (file) {
    int num_pts;
    fscanf(file, "# %d points\n", &num_pts);
    pts.resize(num_pts);
    for (int i = 0; i < num_pts; ++i) {
      fscanf(file, "%d %d %f %f\n", &pts[i].x, &pts[i].y, &pts[i].scale, &pts[i].response);
    }
  }
  
  return pts;
}
