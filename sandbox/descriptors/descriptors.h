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
#include <chamfer_matching/chamfer_matching.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

typedef cv::Vector< cv::Vector<float> > vvf;

  
/***************************************************************************
***********  Misc. useful classes.
****************************************************************************/

/**
 * Temporary class representing image keypoint.  This should be in opencv soon.
 */
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


/**
 * 1D histogram class.  Deprecated.  
 */
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
***********  ImageDescriptor
****************************************************************************/

/**
 * Abstract base class for all descriptors.
 */
class ImageDescriptor {
 public:
  //! Whether to visualize feature computation.
  bool debug_;

  ImageDescriptor();
  virtual ~ImageDescriptor() {};
  std::string getName();
  //! Returns result_size_.
  unsigned int getSize();
  //! Vectorized feature computation call.
  virtual void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results) {};

 protected:
  //! Name of the descriptor.  Should be unique for any parameter setting.
  std::string name_;
  //! Length of the vector that results from computing the feature at a point.
  unsigned int result_size_;
  //! The image that we are computing descriptors on.
  IplImage* img_;

  //! Clean up any data specific to computation at a point.
  virtual void clearPointCache() {}
  //! Clean up any data specific to computation on a particular image.
  virtual void clearImageCache() {}
  //! Show the input image and a red + at the point at which the descriptor is being computed.
  void commonDebug(int row, int col);
  void commonDebug(Keypoint kp, IplImage* vis = NULL);
  //! Sets the img_ pointer and clears the image cache.
  virtual void setImage(IplImage* img);
};


/***************************************************************************
***********  SURF (Not yet implemented.)
****************************************************************************/

/* class SurfWrapper : public ImageDescriptor { */
/*  public: */
/*   bool extended_; */

/*   SurfWrapper(bool extended); */
/*   ~SurfWrapper(); */
/*   virtual void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results) {}; */
/* } */

/***************************************************************************
***********  Hog
****************************************************************************/

/**
 * Histogram of oriented gradients.  Wraps the opencv descriptor.
 */
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


/***************************************************************************
***********  Integral Image-based descriptors.
****************************************************************************/

/**
 * Abstract base class for descriptors that use integral images.
 */
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
  bool integrateRect(float* result, int row_offset, int col_offset, int half_height, int half_width, const Keypoint& kp, float* area = NULL);
  bool integrateRect(float* result, const Keypoint& kp, const CvRect& rect);
};

/**
 * Haar descriptor like those from Viola-Jones.
 */
class HaarDescriptor : public IntegralImageDescriptor {
 public:
  cv::Vector<CvRect> rects_;
  //! e.g. weights_[i] == -1 if the sum of values in rects_[i] should be subtracted.
  cv::Vector<int> weights_;

  HaarDescriptor(cv::Vector<CvRect> rects, cv::Vector<int> weights, IntegralImageDescriptor* ii_provider = NULL);
  //! Scale of the window is determined by Keypoint.
  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);
};

vector<ImageDescriptor*> setupDefaultHaarDescriptors();

/**
 * Experimental texture descriptor based on integral images.  TODO: Add more textures, make scale be determined by keypoint.
 */
class IntegralImageTexture : public IntegralImageDescriptor {
 public:
  int scale_;

  IntegralImageTexture(int scale = 1, IntegralImageDescriptor* ii_provider = NULL);
  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);
  void compute(IplImage* img, const Keypoint& point, cv::Vector<float>& result);
};


/***************************************************************************
***********  Contour Fragments
****************************************************************************/

/**
 * Class to load, save, and extract contour fragments. 
 */
class ContourFragmentManager {
 public:
  ContourFragmentManager(int num_templates_per_label = 10, bool debug = false, int min_area = 30, 
			   float min_density = 0.01, int min_side = 5, int min_edge_pix = 10, int min_edge_pix_besides_line = 5);
  ~ContourFragmentManager();
  void learnContours(std::vector<IplImage*> imgs, std::vector<IplImage*> masks);
  void saveContours(string dir);
  void loadContours(string dir);
  
  std::vector<IplImage*> contours_;
  

 private:
  int num_templates_per_label_;
  bool debug_;
  int min_area_;
  float min_density_;
  int min_side_;
  int min_edge_pix_;
  //! Min number of edge pixels AFTER removing the primary line from the template.
  int min_edge_pix_besides_line_; 

  
  bool contourTest(IplImage* cf);
};

/**
 * Descriptor based on chamfer matching of contour fragments.  Under construction.
 */
class ContourFragmentDescriptor : public ImageDescriptor {
 public:
  ContourFragmentDescriptor* chamfer_provider_;
  
  //! Loads contours from a dir.
  ContourFragmentDescriptor(int cf_id, string dir);
  //! Uses another ContourFragmentDescriptor object to get its data.  
  ContourFragmentDescriptor(int cf_id, ContourFragmentDescriptor* chamfer_provider);
  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);

 private:
  int cf_id_;
  ContourFragmentManager cfc_;
  ChamferMatching* chamfer_;
  ChamferMatch* matches_;
};



/***************************************************************************
***********  Superpixel Statistics
****************************************************************************/

/**
 * Abstract base class for all descriptors based on superpixels.
 */
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
  IplImage* createSegmentMask(int label, CvRect* rect);
};

class SuperpixelColorHistogram : public SuperpixelStatistic {
 public:
  std::vector<CvHistogram*> histograms_cv_;
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
  float max_val_;
  IplImage* channel_;
  bool hists_reserved_;

  SuperpixelColorHistogram(int seed_spacing, float scale, int nBins, std::string type, SuperpixelStatistic* seg_provider=NULL, SuperpixelColorHistogram* hsv_provider_=NULL);
  //bool compute(Eigen::MatrixXf** result);
  void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results);
  void compute(IplImage* img, const Keypoint& point, cv::Vector<float>& result);

  void clearPointCache() {}
  void clearImageCache();
  ~SuperpixelColorHistogram();
  void computeHistogram(int label);
  void computeHistogramCV(int label); 
};

std::vector<ImageDescriptor*> setupImageDescriptors();
void whiten(Eigen::MatrixXf* m);
int getdir (string dir, vector<string> &files);


/***************************************************************************
***********  Patch-based.  
****************************************************************************/
//Not converted to standard yet.  Only will be if wanted.

/* class Patch : public ImageDescriptor { */
/*  public: */
/*   //! Length of the sides of the patch, before scaling. */
/*   int raw_size_; */
/*   //! Scaling to apply to the raw patch before computing the feature. */
/*   float scale_; */
/*   //! The final patch, made available for other descriptors to re-use computation. */
/*   IplImage* final_patch_; */
/*   //! Length of the sides of the patch, after scaling. */
/*   int size_; */
/*   //! The scaled color image patch. */
/*   IplImage* scaled_patch_; */


/*   Patch(int raw_size, float scale); */
/*   //! Common patch constructor computation. */
/*   bool preCompute(); */

/*   void clearPointCache(); */
/*   ~Patch() {}   */
/* }; */

/* class IntensityPatch : public Patch { */
/*  public: */
/*   //! If true, the resulting feature vector has its mean set to 0 and variance set to 1.  This might give some lighting invariance for intensity patches, for example. */
/*   bool whiten_; */

/*   IntensityPatch(int raw_size, float scale, bool whiten); */
/*   bool compute(Eigen::MatrixXf** result); */
/*   void compute(IplImage* img, const cv::Vector<Keypoint>& points, vvf& results); */
/*   void clearImageCache() {} */
/* }; */

/* class PatchStatistic : public ImageDescriptor { */
/*  public: */
/*   //! "variance" */
/*   std::string type_; */
/*   //! Pointer to Patch object which will contain the final_patch_ to compute the statistic on. */
/*   Patch* patch_; */

/*   PatchStatistic(std::string type, Patch* patch); */
/*   bool compute(Eigen::MatrixXf** result); */

/*   void clearPointCache() {} */
/*   void clearImageCache() {} */
/* }; */
