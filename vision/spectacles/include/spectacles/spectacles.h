#ifndef SPECTACLES_SPECTACLES_H
#define SPECTACLES_SPECTACLES_H

#include "opencv/cv.h"
#include <string>

using namespace std;

namespace spectacles
{

class labelrect
{
public:
  string name;
  double x, y, w, h;
  labelrect(string _name, double _x, double _y, double _w, double _h)
  : name(_name), x(_x), y(_y), w(_w), h(_h) { }
  labelrect() { }
};

class labeled_image
{
public:
  vector<labelrect> labels;
  IplImage *image;
};

}

#endif

