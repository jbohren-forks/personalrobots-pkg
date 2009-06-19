#include <iostream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>
#include <string>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <math.h>

class ImageDescriptor {
 public:
  string name_;
  unsigned int result_size_;
  
  virtual bool compute(IplImage* img, int r, int c, NEWMAT::Matrix* result, bool debug);
  virtual void display(const NEWMAT::Matrix& result);
  virtual void clearPointCache() {}
  virtual void clearImageCache() {}
  virtual ~ImageDescriptor() {}
  void commonDebug(IplImage* img, int row, int col);
};

class Patch : public ImageDescriptor {
 public:
  //! Length of the sides of the patch, before scaling.
  int raw_size_;
  //! Scaling to apply to the raw patch before computing the feature.
  float scale_;
  //! Length of the sides of the patch, after scaling.  (This is not set by the user, and will always be odd.)
  int size_;
  //! "intensity", "edge"
  string type_;
  //! If true, the resulting feature vector has its mean set to 0 and variance set to 1.  This might give some lighting invariance for intensity patches, for example.
  bool whiten_;
  //! Threshold 1 for canny edge detector.
  int thresh1_;
  //! Threshold 2 for canny edge detector.
  int thresh2_;
  //! The final patch, made available for other descriptors to re-use computation.
  IplImage* final_patch_;

  Patch(int raw_size, string type, float scale, bool whiten, int thresh1=150, int thresh2=100);
  bool compute(IplImage* img, int r, int c, NEWMAT::Matrix* result, bool debug);
  void display(const NEWMAT::Matrix& result) {}
  void clearPointCache();
  void clearImageCache();
  ~Patch() {}
  
};

class PatchStatistic : public ImageDescriptor {
 public:
  //! "variance"
  string type_;
  //! Pointer to Patch object which will contain the final_patch_ to compute the statistic on.
  Patch* patch_;

  PatchStatistic(string type, Patch* patch);
  bool compute(IplImage* img, int r, int c, NEWMAT::Matrix* result, bool debug);
  void display(const NEWMAT::Matrix& result);
  void clearPointCache() {}
  void clearImageCache() {}
  ~PatchStatistic() {}

};

vector<ImageDescriptor*> setupImageDescriptors();
void whiten(NEWMAT::Matrix* m);

