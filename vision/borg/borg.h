#ifndef BORG_H
#define BORG_H

#include "ros/types.h"
#include "cam.h"
#include "stage.h"
#include <opencv/cv.h>
#include <vector>

namespace borg
{

class Borg
{
public:
  const static uint32_t INIT_CAM   = 0x1;
  const static uint32_t INIT_STAGE = 0x2;

  Borg(uint32_t opts);
  ~Borg();
  
  Cam *cam;
  Stage *stage;
  bool scan();
private:
  int fps;
  double left, right;
  int scan_duty, return_duty;
  double fx, fy, x0, y0, k1, k2, k3, k4;
  bool calib_set(const char *setting, double value);
  CvMat *intrinsics, *distortion;
  class Centroid
  {
  public:
    float col;
    int row, val, noisy;
    Centroid(float _col, int _row, int _val)
    : col(_col), row(_row), val(_val), noisy(0) { }
  };
  class Image
  {
  public:
    uint8_t *raster;
    double   angle;
    std::vector<Centroid> centroids;
    Image(uint8_t *_raster, double _angle)
    : raster(_raster), angle(_angle) { }
  };
  void extract_from_images(std::vector<Image *> &images);
};

}

#endif

