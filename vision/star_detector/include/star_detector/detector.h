#ifndef FEATURES_STAR_DETECTOR_H
#define FEATURES_STAR_DETECTOR_H

#include "star_detector/keypoint.h"
#include "star_detector/integral.h"
#include "star_detector/nonmax_suppress.h"
#include <cv.h>
#include <highgui.h>
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
  static const float DEFAULT_THRESHOLD = 30;
  static const float DEFAULT_LINE_THRESHOLD = 10;
  
  //! Constructor. The dimensions of the source images that will be used
  //! with the detector are required to pre-allocate memory.
  StarDetector(CvSize dims, int n = 7, float threshold = DEFAULT_THRESHOLD,
               float line_threshold = DEFAULT_LINE_THRESHOLD);

  ~StarDetector();
  
  //! Returns a vector of keypoints in the source image.
  template< typename OutputIterator >
  int DetectPoints(IplImage* source, OutputIterator inserter);

  //! Fill in scales based in indices and interpolation
  template< typename ForwardIterator >
  void InterpolateScales(ForwardIterator first, ForwardIterator last);

private:
  //! Scale/spatial dimensions
  int m_n, m_W, m_H;
  //! Normal integral image
  IplImage* m_upright;
  //! Tilted (by 45 degrees) integral image
  IplImage* m_tilted;
  //! Tilted integral image with a rounded two-pixel corner
  IplImage* m_flat;
  //! Array of filter response images over different scales
  IplImage** m_responses;
  //! Response magnitude threshold
  float m_threshold;
  //! Line response threshold
  float m_line_threshold;
  //! Filter size at each scale
  int* m_filter_sizes;
  //! Non-maximal suppression functor
  NonmaxSuppressProject<float, LineSuppressCombined> m_nonmax;
  //! Border size for non-max suppression
  int m_border;
  //! Scale interpolation flag
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

    FilterResponses();
#if 0
    for (int i = 0; i < m_n; i++) {
        char filename[10];
        sprintf(filename, "out%d.pgm", i);
        SaveResponseImage(filename, m_responses[i]);
    }
#endif

    return FindExtrema(inserter);
}

template< typename OutputIterator >
int StarDetector::FindExtrema(OutputIterator inserter)
{
    int num_points = m_nonmax(m_responses, m_n, inserter, m_border);
    //num_points += m_nonmin(m_responses, m_n, inserter, m_border);

    return num_points;
}

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
