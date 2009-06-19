#include <iostream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>
#include <string>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <math.h>

class CvImageDescriptor {
 public:
  string name_;
  unsigned int result_size_;
  
  bool compute(IplImage* img, int r, int c, NEWMAT::Matrix** result, bool debug);
   void display(const NEWMAT::Matrix& result);
};

class Patch : public CvImageDescriptor {
 public:
  //! Length of the sides of the patch, before scaling.
  int raw_size_;
  //! Scaling to apply to the raw patch before computing the feature.
  float scale_;
  //! Length of the sides of the patch, after scaling.  (This is not set by the user, and will always be odd.)
  int size_;
  //! "intensity", "color", "edge"
  string type_;
  
  Patch(int raw_size, string type, float scale = 1.0);
  bool compute(IplImage* img, int r, int c, NEWMAT::Matrix* result, bool debug);
};


