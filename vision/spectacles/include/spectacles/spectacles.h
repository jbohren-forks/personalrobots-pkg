#ifndef SPECTACLES_SPECTACLES_H
#define SPECTACLES_SPECTACLES_H

#include "opencv/cv.h"
#include <string>
#include <vector>

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
  IplImage *create_annotated_image();
};

class labeled_imageset
{
public:
  labeled_imageset();
  ~labeled_imageset();
  vector<labeled_image *> images;
  bool load(const string &labelfile, const vector<string> &imagefiles);
  bool load(const string &labelfile, int argc, char **argv, int start = 0);
};

}

#endif

