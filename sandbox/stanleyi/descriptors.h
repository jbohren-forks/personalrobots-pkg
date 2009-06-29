#include <iostream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.hpp"
#include <string>
//#include <newmat10/newmat.h>
//#include <newmat10/newmatio.h>
#include <Eigen/Core>
#include <math.h>
#include <list>
#include <vector>

class Histogram {
public:
  std::vector<float> bins_;
  Histogram(int nBins, float min, float max);
  int nInsertions_;
  bool insert(float val);
  void normalize();
  void print();
  void printGraph();
  void printBoundaries();
  void clear();

private:
  std::vector<float> boundaries_;
  int nBins_;
  float min_;
  float max_;
  float bin_size_;
};

class ImageDescriptor {
 public:
  std::string name_;
  unsigned int result_size_;
  IplImage* img_;
  int row_;
  int col_;

  virtual bool compute(Eigen::MatrixXf** result, bool debug) = 0;
  virtual void display(const Eigen::MatrixXf& result) = 0;
  virtual void clearPointCache() = 0;
  virtual void clearImageCache() = 0;
  //! Show the input image and a red + at the point at which the descriptor is being computed.
  void commonDebug();
  void setImage(IplImage* img);
  void setPoint(int row, int col);
};

class Patch : public ImageDescriptor {
 public:
  //! Length of the sides of the patch, before scaling.
  int raw_size_;
  //! Scaling to apply to the raw patch before computing the feature.
  float scale_;
  //! The final patch, made available for other descriptors to re-use computation.
  IplImage* final_patch_;
  //! Length of the sides of the patch, after scaling.
  int size_;
  //! The scaled color image patch.
  IplImage* scaled_patch_;


  Patch(int raw_size, float scale);
  //! Common patch constructor computation.
  bool preCompute(bool debug);
  //virtual void display(const Eigen::MatrixXf& result) {}
  void clearPointCache();
  //virtual void clearImageCache();
  ~Patch() {}  
};

class IntensityPatch : public Patch {
 public:
  //! If true, the resulting feature vector has its mean set to 0 and variance set to 1.  This might give some lighting invariance for intensity patches, for example.
  bool whiten_;

  IntensityPatch(int raw_size, float scale, bool whiten);
  bool compute(Eigen::MatrixXf** result, bool debug);
  void display(const Eigen::MatrixXf& result) {}
  void clearImageCache() {}
};

class PatchStatistic : public ImageDescriptor {
 public:
  //! "variance"
  std::string type_;
  //! Pointer to Patch object which will contain the final_patch_ to compute the statistic on.
  Patch* patch_;

  PatchStatistic(std::string type, Patch* patch);
  bool compute(Eigen::MatrixXf** result, bool debug);
  void display(const Eigen::MatrixXf& result);
  void clearPointCache() {}
  void clearImageCache() {}
};

class SuperpixelStatistic : public ImageDescriptor {
 public:
  //! (*index_)[i] returns the vector of CvPoints for segment i of the image.
  std::vector< std::vector<CvPoint> > *index_;
  int seed_spacing_;
  //! Scaling factor to apply to the image when computing the segmentation.
  float scale_;
  //! Pointer to an object from which the segmentation can be gotten.  
  SuperpixelStatistic* seg_provider_;
  IplImage* seg_;

  SuperpixelStatistic(int seed_spacing, float scale, SuperpixelStatistic* provider);
  //! Computes superpixels and puts into seg_, and computes the superpixel to pixel index.  Is called automatically, if necessary, by the compute(.) function.
  void segment(bool debug);
};

class SuperpixelColorHistogram : public SuperpixelStatistic {
 public:
  IplImage* hsv_;
  IplImage* hue_;
  IplImage* sat_;
  IplImage* val_;
  int nBins_;
  std::string type_;
  SuperpixelColorHistogram* hsv_provider_;
  //! histograms_[s] corresponds to the histogram for segment s of the segmentation. (s=0 is always left NULL).
  std::vector<Histogram*> histograms_;
  bool hists_reserved_;

  SuperpixelColorHistogram(int seed_spacing, float scale, int nBins, std::string type, SuperpixelStatistic* seg_provider=NULL, SuperpixelColorHistogram* hsv_provider_=NULL);
  bool compute(Eigen::MatrixXf** result, bool debug);
  void display(const Eigen::MatrixXf& result) {}
  void clearPointCache() {}
  void clearImageCache();
};

std::vector<ImageDescriptor*> setupImageDescriptors();
void whiten(Eigen::MatrixXf* m);


/*
class EdgePatch : public Patch {
 public:
  //! Threshold 1 for canny edge detector.
  int thresh1_;
  //! Threshold 2 for canny edge detector.
  int thresh2_;

  EdgePatch(int raw_size, float scale, int thresh1=150, int thresh2=100);
}


class Hog : public ImageDescriptor {
 public:
  //! Should be pointed to an intensity patch object.
  Patch* patch_;
  //! The opencv class.
  HOGDescriptor cvHog;

  Hog(Patch* patch);
  bool compute(Eigen::MatrixXf** result, bool debug);
  void display(const Eigen::MatrixXf& result) {}
  void clearPointCache() {}
  void clearImageCache() {}
  ~Hog() {}

 private:
}


*/
