#include <iostream>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cvaux.hpp"
#include <string>
#include <Eigen/Core>
#include <math.h>
#include <list>
#include <vector>
#include "ros/console.h"
#include "ros/assert.h"


typedef cv::Vector< cv::Vector<float> > vvf;

  
/***************************************************************************
***********  Misc. useful classes.
****************************************************************************/

class Keypoint
{
 public:    
 Keypoint() : pt(0,0), size(-1), angle(-1), response(0) {}
  
 Keypoint(cv::Point2f _pt, float _size=-1, float _angle=-1, float _response=0)
   : pt(_pt), size(_size), angle(_angle), response(_response) {}

 Keypoint(float x, float y, float _size=-1, float _angle=-1, float _response=0)
   : pt(x, y), size(_size), angle(_angle), response(_response) {}

  cv::Point2f pt;
  float size;
  float angle;
  float response;
};

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

  
/***************************************************************************
***********  ImageDescriptor classes.
****************************************************************************/


class ImageDescriptor {
 public:
  //! Name of descriptor.  Should be unique for any parameter setting.
  std::string name_;
  //! Length of feature vector.
  unsigned int result_size_;
  IplImage* img_;
  //! Deprecated.
  int row_;
  //! Deprecated.
  int col_;
  //! Visualize feature computation.
  bool debug_;

  void setDebug(bool debug);
  //! Deprecated.
  virtual bool compute(Eigen::MatrixXf** result);
  //! Vectorized feature computation call.
  virtual void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results) {};

  //! Clean up any data specific to computation at a point.
  virtual void clearPointCache() {}
  //! Clean up any data specific to computation on a particular image.
  virtual void clearImageCache() {}
  //! Show the input image and a red + at the point at which the descriptor is being computed.
  void commonDebug();
  //! Deprecated.
  virtual void setImage(IplImage* img);
  //! Deprecated.
  virtual void setPoint(int row, int col);
  virtual ~ImageDescriptor() {};
  ImageDescriptor();
};

class HogWrapper : public ImageDescriptor {
 public:
  cv::HOGDescriptor hog_;
  
  HogWrapper();
  HogWrapper(cv::Size winSize, cv::Size blockSize, cv::Size blockStride, cv::Size cellSize,
	     int nbins, int derivAperture=1, double winSigma=-1,
	     int histogramNormType=0, double L2HysThreshold=0.2, bool gammaCorrection=false); //0=L2Hys

  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);
  void clearImageCache() {};
  void clearPointCache() {};
};

class IntegralImageDescriptor : public ImageDescriptor {
 public:
  IplImage* ii_;
  IplImage* ii_tilt_;
  IplImage* gray_;
  IntegralImageDescriptor* ii_provider_;
  //! Which image - r, g, b, gray, etc.
  IplImage* channel_;

  IntegralImageDescriptor(IntegralImageDescriptor* ii_provider);
  ~IntegralImageDescriptor();
  void integrate();
  virtual void clearImageCache();
  virtual void clearPointCache() = 0;
  bool integrateRect(float* result, int row_offset, int col_offset, int half_height, int half_width, float* area = NULL);

};

class IntegralImageTexture : public IntegralImageDescriptor {
 public:
  int scale_;

  IntegralImageTexture(int scale = 1, IntegralImageDescriptor* ii_provider = NULL);
  //bool compute(Eigen::MatrixXf** result);
  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);
  void compute(IplImage* img, const Keypoint& point, cv::Vector<float>& result);
  void clearPointCache() {}
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
  bool preCompute();

  void clearPointCache();
  //virtual void clearImageCache();
  ~Patch() {}  
};

class IntensityPatch : public Patch {
 public:
  //! If true, the resulting feature vector has its mean set to 0 and variance set to 1.  This might give some lighting invariance for intensity patches, for example.
  bool whiten_;

  IntensityPatch(int raw_size, float scale, bool whiten);
  bool compute(Eigen::MatrixXf** result);
  void clearImageCache() {}
};

class PatchStatistic : public ImageDescriptor {
 public:
  //! "variance"
  std::string type_;
  //! Pointer to Patch object which will contain the final_patch_ to compute the statistic on.
  Patch* patch_;

  PatchStatistic(std::string type, Patch* patch);
  bool compute(Eigen::MatrixXf** result);

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
  void segment();
  IplImage* createSegmentMask(long label, CvRect* rect);
};

class SuperpixelColorHistogram : public SuperpixelStatistic {
 public:
  IplImage* hsv_;
  IplImage* hue_;
  IplImage* sat_;
  IplImage* val_;
  int nBins_;
  //! Not used right now.
  std::string type_;
  SuperpixelColorHistogram* hsv_provider_;
  //! histograms_[s] corresponds to the histogram for segment s of the segmentation. s=0 is always left NULL.  (segment numbering starts at 1).
  //std::vector<Histogram*> histograms_;
  std::vector<CvHistogram*> histograms_cv_;
  bool hists_reserved_;
  float max_val_;
  IplImage* channel_;

  SuperpixelColorHistogram(int seed_spacing, float scale, int nBins, std::string type, SuperpixelStatistic* seg_provider=NULL, SuperpixelColorHistogram* hsv_provider_=NULL);
  //bool compute(Eigen::MatrixXf** result);
  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);
  void compute(IplImage* img, const Keypoint& point, cv::Vector<float>& result);

  void clearPointCache() {}
  void clearImageCache();
  ~SuperpixelColorHistogram();
  void computeHistogram(long label);
  void computeHistogramCV(long label); 
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

*/
