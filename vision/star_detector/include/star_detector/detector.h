#ifndef FEATURES_STAR_DETECTOR_H
#define FEATURES_STAR_DETECTOR_H

#include "star_detector/keypoint.h"
#include "star_detector/integral.h"
#include "star_detector/nonmax_suppress.h"
//#include "star_detector/timer.h" // DEBUG
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
  StarDetector(CvSize dims, int n = DEFAULT_SCALES,
               float response_threshold = DEFAULT_RESPONSE_THRESHOLD,
               float line_threshold_projected = DEFAULT_LINE_THRESHOLD_PROJECTED,
               float line_threshold_binarized = DEFAULT_LINE_THRESHOLD_BINARIZED);

  ~StarDetector();
  
  //! Returns a vector of keypoints in the source image.
  template< typename OutputIterator >
  int DetectPoints(IplImage* source, OutputIterator inserter);

  //! Fill in scales based on indices and interpolation
  template< typename ForwardIterator >
  void InterpolateScales(ForwardIterator first, ForwardIterator last);

private:
  float m_response_threshold;
  //! Scale/spatial dimensions
  int m_n, m_W, m_H;
  //! Normal integral image
  IplImage* m_upright;
  //! Tilted (by 45 degrees) integral image
  IplImage* m_tilted;
  //! Tilted integral image with a rounded two-pixel corner
  IplImage* m_flat;
  //! Array of filter response images over different scales
  // TODO: remove once FilterResponses is replaced
  IplImage** m_responses;
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
  //! Scale interpolation flag (TODO: currently useless)
  bool m_interpolate;
  
  static const float SCALE_RATIO = M_SQRT2;
  
  //! Calculate sum of all pixel values in the "star" shape.
  int StarAreaSum(CvPoint center, int radius, int offset);
  
  //! Count the number of pixels in the "star" shape. Pixels in the intersection
  //! are counted twice.
  int StarPixels(int radius, int offset);
  
  //! Calculate the center-surround filter response for some scale.
  void BilevelFilter(IplImage* dst, int scale);
  
    //! Calculate the filter responses over the range of desired scales.
  void FilterResponses();
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

  // cvIntegral needs a destination 1 pixel larger than source,
  // while we use a larger width for all summed areas.
  // So run the function then paste the scratch into m_upright.

  IplImage *scratch = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);
  cvIntegral(source, scratch, NULL, NULL);
  cvSetImageROI(scratch, cvRect(0, 0, m_W+1,m_H+1));
  cvSetImageROI(m_upright, cvRect(0, 0, m_W+1,m_H+1));
  cvCopy(scratch, m_upright);
  cvReleaseImage(&scratch);
  cvResetImageROI(m_upright);

  TiltedIntegral(source, m_tilted, m_flat);

  // If possible, run one of the optimized versions

  if ((3 <= m_n) && (m_n <= 12)) {
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

#if 0
  // DEBUG
  for (int i = 0; i < m_n; i++) {
    char filename[10];
    sprintf(filename, "out%d.pgm", i);
    SaveResponseImage(filename, m_responses[i]);
  }
#endif

  return FindExtrema(inserter);
}

template< typename OutputIterator >
inline int StarDetector::FindExtrema(OutputIterator inserter)
{
  //Timer t("Nonmax suppression");
  return m_nonmax(m_projected, m_scales, m_n, inserter, m_border);
}

// TODO: this is useless when we discard the full set of response images, but we
//       could do the interpolation when creating the projected image.
template< typename ForwardIterator >
void StarDetector::InterpolateScales(ForwardIterator first, ForwardIterator last)
{
  for (ForwardIterator i = first; i != last; ++i) {
    // Do quadratic interpolation using finite differences
    float r1 = CV_IMAGE_ELEM(m_responses[i->s - 2], float, i->y, i->x);
    float r2 = CV_IMAGE_ELEM(m_responses[i->s - 1], float, i->y, i->x);
    float r3 = CV_IMAGE_ELEM(m_responses[i->s], float, i->y, i->x);
    float s_hat = 0.5*(r1 - r3) / (r1 - 2*r2 + r3);
    
    i->scale = pow(SCALE_RATIO, s_hat + i->s);
  }
}

#endif
