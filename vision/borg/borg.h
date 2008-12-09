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

extern bool g_silent;

class Borg
{
public:
  const static uint32_t INIT_CAM    = 0x1;
  const static uint32_t INIT_STAGE  = 0x2;
  const static uint32_t INIT_SILENT = 0x4;

  Borg(uint32_t opts);
  ~Borg();
  
  Cam *cam;
  Stage *stage;
 
  class Centroid
  {
  public:
    float col;
    int row, val, noisy;
    std::vector<int> cluster;
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

  class SensedPoint
  {
  public:
    float angle, col;
    int row;
    uint8_t r, g, b;
    SensedPoint(float _angle, float _col, int _row,
                uint8_t _r, uint8_t _g, uint8_t _b)
    : angle(_angle), col(_col), row(_row), r(_r), g(_g), b(_b) { }
  };

  class ProjectedPoint
  {
  public:
    float angle, col;
    int row;
    float x, y, z;
    uint8_t r, g, b;
    ProjectedPoint(float _x, float _y, float _z, int _row, float _col,
                   uint8_t _r, uint8_t _g, uint8_t _b)
    : angle(0), col(_col), row(_row), x(_x), y(_y), z(_z),
      r(_r), g(_g), b(_b) { }
  };
  
  void scan(std::list<Image *> &images);
  void saveScan(std::list<Image *> &images, const char *prefix);
  void extract(std::list<Image *> &images, bool show_gui = true);
  void sensedPoints(std::list<Image *> &images, std::vector<SensedPoint> &pts);
  void printExtraction(std::list<Image *> &images);
  void project(const std::vector<SensedPoint> &sensed,
               std::vector<ProjectedPoint> &projected);
  void loadExtractionFile(const char *fn, std::vector<SensedPoint> &extraction);
  void calibrate(const double size, const uint32_t x, const uint32_t y, 
                 const std::list<std::string> &filename_prefixes);
  const double &get_tilt() { return tilt; }
private:
  class CheckerCorner
  {
  public:
    double row, col;
    double x, y, z;
    CheckerCorner(double _row, double _col) : row(_row), col(_col) { }
  };
  class CalibrationScene
  {
  public:
    double sq_size; // length of a side of a checker on the board
    std::vector<CheckerCorner> corners;
    std::vector<SensedPoint> points;
    std::vector<ProjectedPoint> proj;
    CalibrationScene(double _size) : sq_size(_size) { }
    void projectCorners();
    double objective();
    void writeFile(const char *filename);
  };
  double calibration_objective(std::vector<CalibrationScene *> &scenes);
  void project(std::vector<CalibrationScene *> &scenes);

  int fps;
  double left, right;
  int scan_duty, return_duty;
  double fx, fy, x0, y0, k1, k2, k3, k4;
  bool calib_set(const char *setting, double value);
  CvMat *intrinsics, *distortion, *map_x, *map_y;
  uint32_t image_queue_size;
  int laser_thresh;
  double tx, ty, tz, enc_offset, laser_rot;
  double max_stripe_width;
  double tilt; // tilt of the entire assembly (it's usually aimed down a bit)
  double laser_roll;
};

}

#endif

