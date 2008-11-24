#ifndef FEATURES_STAR_DETECTOR_H
#define FEATURES_STAR_DETECTOR_H

#include "star_detector/keypoint.h"
#include "star_detector/integral.h"
#include "star_detector/nonmax_suppress.h"
#include "optimized_width.h"
#include <cv.h>
#include <vector>
#include <cmath>

/*!
  This class offers a simple interface for keypoint detection.

  The detector searches for center-surround extrema, much like the
  the human low-level vision system. It uses a bilevel center-surround
  filter, approximating a circular shape by combining a box filter with
  another box filter rotated 45 degrees. This filter offers reasonable
  rotational invariance and can be calculated very efficiently using
  integral images. The filter is applied in several sizes (7 by default)
  to determine a keypoint's scale.

  The detector uses non-maximal suppression over the scale and spatial
  dimensions to pick out keypoints. Each keypoint must have a response
  with magnitude exceeding a user-specified threshold. Keypoints which
  are poorly localized along a line are also discarded using a line
  suppression check similar to SIFT's. Note that smaller line response
  thresholds are more stringent. Reasonable defaults for the thresholds
  are provided.

  The default response threshold (30) is most appropriate for outdoor
  scenes. For indoor scenes, a smaller threshold (~10) may be needed.
 */
class StarDetector
{
public:
  static const int DEFAULT_SCALES = 7;
  static const float DEFAULT_RESPONSE_THRESHOLD = 30;
  static const float DEFAULT_LINE_THRESHOLD_PROJECTED = 10;
  static const float DEFAULT_LINE_THRESHOLD_BINARIZED = 8;
  
  //! Constructor. The dimensions of the source images that will be used
  //! with the detector are required to pre-allocate memory.
  StarDetector(CvSize size, int n = DEFAULT_SCALES,
               float response_threshold = DEFAULT_RESPONSE_THRESHOLD,
               float line_threshold_projected = DEFAULT_LINE_THRESHOLD_PROJECTED,
               float line_threshold_binarized = DEFAULT_LINE_THRESHOLD_BINARIZED);

  ~StarDetector();
  
  //! Detects keypoints and outputs them through inserter. Returns the
  //! number of keypoints detected.
  template< typename OutputIterator >
  int DetectPoints(IplImage* source, OutputIterator inserter);

  CvSize imageSize() const;
  //! Reallocates internal images
  void setImageSize(CvSize size);

  //! Number of scales used
  inline int scales() const { return m_n; }
  void setScales(int n);
  
  float responseThreshold() const;
  void setResponseThreshold(float threshold);
  
  float projectedThreshold() const;
  void setProjectedThreshold(float threshold);
  
  float binarizedThreshold() const;
  void setBinarizedThreshold(float threshold);

  //! StarDetector will not find keypoints within 'border' pixels of the edge.
  inline int border() const { return m_border; }

private:
  //! Scale/spatial dimensions
  int m_n, m_W, m_H;
  //! Normal integral image
  IplImage* m_upright;
  //! Tilted (by 45 degrees) integral image
  IplImage* m_tilted;
  //! Tilted integral image with a rounded two-pixel corner
  IplImage* m_flat;
  //! Projected response image
  IplImage* m_projected;
  //! Scales of points in projected response image
  IplImage* m_scales;
  //! Filter size at each scale
  int* m_filter_sizes;
  //! Non-maximal suppression functor
  NonmaxSuppressWxH<5, 5, float, LineSuppressHybrid> m_nonmax;
  //! Border size for non-max suppression
  int m_border;

  struct FilterParams
  {
    int inner_r;
    int outer_r;
    int inner_offset;
    int outer_offset;
    float inner_normalizer;
    float outer_normalizer;
  };
  FilterParams* m_filter_params;
  
  static const float SCALE_RATIO = M_SQRT2;

  //! Release memory for internal images.
  void releaseImages();
  
  //! Calculate sum of all pixel values in the "star" shape.
  int StarAreaSum(CvPoint center, int radius, int offset);
  
  //! Count the number of pixels in the "star" shape. Pixels in the intersection
  //! are counted twice.
  int StarPixels(int radius, int offset);
  
  //! Calculate the filter responses over the range of desired scales.
  void FilterResponses(); // fallback pure C++ implementation
  void FilterResponsesGen3();
  void FilterResponsesGen4();
  void FilterResponsesGen5();
  void FilterResponsesGen6();
  void FilterResponsesGen7();
  void FilterResponsesGen8();
  void FilterResponsesGen9();
  void FilterResponsesGen10();
  void FilterResponsesGen11();
  void FilterResponsesGen12();
  
  //! Return extrema which satisfy the strength and line response thresholds
  template< typename OutputIterator >
  int FindExtrema(OutputIterator inserter);
};


template< typename OutputIterator >
int StarDetector::DetectPoints(IplImage* source, OutputIterator inserter)
{
  assert(source && source->depth == (int)IPL_DEPTH_8U);

  cvIntegral(source, m_upright, NULL, NULL);
  TiltedIntegral(source, m_tilted, m_flat);

  // If possible, run one of the optimized versions
  //if (false) {
  if ((m_W < OPTIMIZED_WIDTH) && (3 <= m_n) && (m_n <= 12)) {
    switch (m_n) {
      case 3: FilterResponsesGen3(); break;
      case 4: FilterResponsesGen4(); break;
      case 5: FilterResponsesGen5(); break;
      case 6: FilterResponsesGen6(); break;
      case 7: FilterResponsesGen7(); break;
      case 8: FilterResponsesGen7(); break;
      case 9: FilterResponsesGen9(); break;
      case 10: FilterResponsesGen10(); break;
      case 11: FilterResponsesGen11(); break;
      case 12: FilterResponsesGen12(); break;
    }
  } else {
    FilterResponses();
  }

  return FindExtrema(inserter);
}

inline CvSize StarDetector::imageSize() const
{
  return cvSize(m_W, m_H);
}

inline float StarDetector::responseThreshold() const
{
  return m_nonmax.responseThreshold();
}

inline void StarDetector::setResponseThreshold(float threshold)
{
  m_nonmax.setResponseThreshold(threshold);
}

inline float StarDetector::projectedThreshold() const
{
  return m_nonmax.thresholdFunction().projectedThreshold();
}

inline void StarDetector::setProjectedThreshold(float threshold)
{
  m_nonmax.thresholdFunction().setProjectedThreshold(threshold);
}

inline float StarDetector::binarizedThreshold() const
{
  return m_nonmax.thresholdFunction().binarizedThreshold();
}

inline void StarDetector::setBinarizedThreshold(float threshold)
{
  m_nonmax.thresholdFunction().setBinarizedThreshold(threshold);
}

template< typename OutputIterator >
inline int StarDetector::FindExtrema(OutputIterator inserter)
{
  return m_nonmax(m_projected, m_scales, m_n, inserter, m_border);
}

#endif
