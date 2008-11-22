#ifndef FEATURES_NON_MAXIMA_SUPPRESSION_H
#define FEATURES_NON_MAXIMA_SUPPRESSION_H

#include "star_detector/keypoint.h"
#include "star_detector/threshold.h"
#include <cv.h>
#include <vector>
#include <iterator>
#include <functional>
#include <boost/static_assert.hpp>

/*!
  Functor to perform non-maximal suppression on an image of projected
  responses over a scale space.

  This version considers a point maximal if it is an extremum over a WxH
  spatial neighborhood, i.e. it finds both maxima and minima in one pass.
  It uses some template parameters to achieve a useful degree of generality
  without sacrificing performance. By default it finds extrema on float
  images, but this behavior can be adapted.

  NonmaxSuppressWxH uses a tiled algorithm which has complexity independent
  of W and H. See http://www.vision.ee.ethz.ch/~aneubeck/.

  W, H: the width and height of the suppression window. Both must be odd.
  
  T: The pixel type of the images.
  
  PostThreshold: Optionally, an additional thresholding functor can be
    specified. The functor may be expensive, as it will only be applied to
    a small number of extrema which already exceed the strength threshold.

  Compare: The comparison functor used to compare response values.
    By default NonmaxSuppressWxH will use operator <=.
 */
template< int W, int H, typename T = float,
          typename PostThreshold = NullThreshold,
          typename Compare = std::less_equal<T> >
class NonmaxSuppressWxH
{
  BOOST_STATIC_ASSERT( W % 2 == 1 && H % 2 == 1 );
  
public:
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppressWxH(T response_thresh = T(), PostThreshold post_thresh = PostThreshold(),
                    Compare compare = Compare())
    : m_response_thresh(response_thresh), m_post_thresh(post_thresh),
      m_compare(compare)
  {}

  inline T responseThreshold() const { return m_response_thresh; }
  inline void setResponseThreshold(T threshold) { m_response_thresh = threshold; }
  inline PostThreshold& thresholdFunction() { return m_post_thresh; }
  inline const PostThreshold& thresholdFunction() const { return m_post_thresh; }
  
  //! Outputs (through inserter) all maxima found and returns how many were found.
  template< typename OutputIterator >
  int operator() (IplImage* projected, IplImage* scales, int n,
                  OutputIterator inserter, int border) {
    int num_pts = 0;
    int max_x = 0, max_y = 0, min_x = 0, min_y = 0;

    // NOTE: We use a constant, conservative border here, even though we may miss some
    // small features near the edges of the image.
    int end_y = projected->height - border - 1;
    int end_x = projected->width - border - 1;
    for (int y = border; y <= end_y; y += Y_OFFSET + 1) {
      for (int x = border; x <= end_x; x += X_OFFSET + 1) {
        T max_response = 0, min_response = 0;

        // Find candidate maximum and minimum in current (W/2)x(H/2) tile
        int tile_end_y = std::min(y + Y_OFFSET, end_y);
        int tile_end_x = std::min(x + X_OFFSET, end_x);
        for (int next_y = y; next_y <= tile_end_y; ++next_y) {
          for (int next_x = x; next_x <= tile_end_x; ++next_x) {
            T response = CV_IMAGE_ELEM(projected, T, next_y, next_x);
            if (m_compare(max_response, response)) {
              max_x = next_x;
              max_y = next_y;
              max_response = response;
            }
            else if (m_compare(response, min_response)) {
              min_x = next_x;
              min_y = next_y;
              min_response = response;
            }
          }
        }
        uchar max_scale = CV_IMAGE_ELEM(scales, uchar, max_y, max_x);
        uchar min_scale = CV_IMAGE_ELEM(scales, uchar, min_y, min_x);

        // Discard maximum if an end scale
        if (max_scale == 1 || max_scale == n)
          goto max_failed;

        // Threshold on response strength
        if (m_compare(max_response, m_response_thresh))
          goto max_failed;
        
        // Check if candidate point is maximal over full WxH neighborhood
        for (int yy = max_y - Y_OFFSET; yy <= max_y + Y_OFFSET; ++yy) {
          for (int xx = max_x - X_OFFSET; xx <= max_x + X_OFFSET; ++xx) {
            if (yy == max_y && xx == max_x) continue;
            T response = CV_IMAGE_ELEM(projected, T, yy, xx);
            if (m_compare(max_response, response))
              goto max_failed;
          }
        }

        // Final thresholding step (e.g. line suppression)
        if ( m_post_thresh(max_x, max_y, max_scale) )
          goto max_failed;

        *inserter = Keypoint(max_x, max_y, max_scale, max_response);
        ++num_pts;

      max_failed:

        // Discard minimum if an end scale
        if (min_scale == 1 || min_scale == n)
          continue;

        // Threshold on response strength
        if (m_compare(-m_response_thresh, min_response))
          continue;
        
        // Check if candidate point is minimal over full WxH neighborhood
        for (int yy = min_y - Y_OFFSET; yy <= min_y + Y_OFFSET; ++yy) {
          for (int xx = min_x - X_OFFSET; xx <= min_x + X_OFFSET; ++xx) {
            if (yy == min_y && xx == min_x) continue;
            T response = CV_IMAGE_ELEM(projected, T, yy, xx);
            if (m_compare(response, min_response))
              goto min_failed;
          }
        }

        // Final thresholding step (e.g. line suppression)
        if ( m_post_thresh(min_x, min_y, min_scale) )
          continue;

        *inserter = Keypoint(min_x, min_y, min_scale, min_response);
        ++num_pts;

      min_failed: /* null statement */ ;
      }
    }
    
    return num_pts;
  }

private:
  static const int X_OFFSET = W / 2;
  static const int Y_OFFSET = H / 2;
  
  T m_response_thresh;
  PostThreshold m_post_thresh;
  Compare m_compare;
};

#endif
