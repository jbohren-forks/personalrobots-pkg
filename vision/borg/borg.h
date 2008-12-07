#ifndef BORG_H
#define BORG_H

#include "ros/types.h"
#include "cam.h"
#include "stage.h"
#include <opencv/cv.h>
#include <vector>
#include <list>

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
    double   t, angle;
    std::vector<Centroid> centroids;
    Image(uint8_t *_raster, double _t, double _angle)
    : raster(_raster), t(_t), angle(_angle) { }
    Image(const char *filename);
  };
  
  void extract(std::list<Image *> &images);
  void print_extraction(std::list<Image *> &images);
private:
  int fps;
  double left, right;
  int scan_duty, return_duty;
  double fx, fy, x0, y0, k1, k2, k3, k4;
  bool calib_set(const char *setting, double value);
  CvMat *intrinsics, *distortion, *map_x, *map_y;
  uint32_t image_queue_size;
  int laser_thresh;
};

}

#endif

