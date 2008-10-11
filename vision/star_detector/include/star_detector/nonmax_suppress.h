#ifndef FEATURES_NON_MAXIMA_SUPPRESSION_H
#define FEATURES_NON_MAXIMA_SUPPRESSION_H

#include "star_detector/keypoint.h"
#include "star_detector/threshold.h"
#include <cv.h>
#include <vector>
#include <iterator>
#include <functional>
#include <boost/static_assert.hpp>
#include <highgui.h> // DEBUG

// TODO: remove obsolete implementations

// DEBUG
inline void SaveResponseImage(char* filename, IplImage* img)
{
  int W = img->width, H = img->height;
  IplImage *gray = cvCreateImage(cvSize(W, H), IPL_DEPTH_8U, 1);
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      float v = CV_IMAGE_ELEM(img, float, y, x);
      int b = (int)(255.0 * (0.5 + 0.5 * (v / 256.0)));
      CV_IMAGE_ELEM(gray, unsigned char, y, x) = b;
    }
  }
  cvSaveImage(filename, img);
  cvReleaseImage(&gray);
}

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
struct NonmaxSuppressWxH
{
  BOOST_STATIC_ASSERT( W % 2 == 1 && H % 2 == 1 );
  static const int X_OFFSET = W / 2;
  static const int Y_OFFSET = H / 2;
  
  T m_response_thresh;
  PostThreshold m_post_thresh;
  Compare m_compare;
  
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppressWxH(T response_thresh = T(), PostThreshold post_thresh = PostThreshold(),
                    Compare compare = Compare())
    : m_response_thresh(response_thresh), m_post_thresh(post_thresh),
      m_compare(compare)
  {}
  
  //! Appends all maxima found to pts and returns how many were found.
  template< typename OutputIterator >
  int operator() (IplImage* projected, IplImage* scales, int n,
                  OutputIterator inserter, int border) {
    int num_pts = 0;
    int max_x = 0, max_y = 0, min_x = 0, min_y = 0;

    // NOTE: We use a constant, conservative border here, even though we may miss some
    // small features near the edges of the image.
    for (int y = border; y < projected->height - border; y += Y_OFFSET + 1) {
      for (int x = border; x < projected->width - border; x += X_OFFSET + 1) {
        T max_response = 0, min_response = 0;

        // Find candidate maximum and minimum in current (W/2)x(H/2) tile
        for (int next_y = y; next_y <= y + Y_OFFSET; ++next_y) {
          for (int next_x = x; next_x <= x + X_OFFSET; ++next_x) {
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

        *inserter = Keypoint(max_x, max_y, (float)max_scale,
                             max_response, max_scale);
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

        *inserter = Keypoint(min_x, min_y, (float)min_scale,
                             min_response, min_scale);
        ++num_pts;

      min_failed: /* null statement */ ;
      }
    }
    
    return num_pts;
  }
};

/*!
  This version uses a 3x3 spatial neighborhood and finds both maxima and
  minima in one pass. It uses a straightforward (non-tiled) implementation,
  which seems efficient enough for the 3x3 case.
*/
template< typename T = float, typename PostThreshold = NullThreshold,
          typename Compare = std::less_equal<T> >
struct NonmaxSuppress3x3
{
  T m_response_thresh;
  PostThreshold m_post_thresh;
  Compare m_compare;
  
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppress3x3(T response_thresh = T(), PostThreshold post_thresh = PostThreshold(),
                    Compare compare = Compare())
    : m_response_thresh(response_thresh), m_post_thresh(post_thresh),
      m_compare(compare)
  {}
  
  //! Appends all maxima found to pts and returns how many were found.
  template< typename OutputIterator >
  int operator() (IplImage* projected, IplImage* scales, int n,
                  OutputIterator inserter, int border) {
    int W = projected->width, H = projected->height;
    int num_pts = 0;

    // NOTE: We use a constant, conservative border here, even though we may miss some
    // small features near the edges of the image.
    for (int y = border; y < H - border; ++y) {
      for (int x = border; x < W - border; ++x) {
        T response = CV_IMAGE_ELEM(projected, T, y, x);
        uchar scale = CV_IMAGE_ELEM(scales, uchar, y, x);

        // Discard if an end scale
        if (scale == 1 || scale == n)
          continue;

        // Check if a maximum/minimum depending on the sign of the response
        if (m_compare(T(0), response)) {
          // Reject immediately if not strong enough
          if (m_compare(response, m_response_thresh) ||
              // Compare orthogonal points looking for quick rejection
              m_compare(response, CV_IMAGE_ELEM(projected, T, y, x-1)) ||
              m_compare(response, CV_IMAGE_ELEM(projected, T, y, x+1)) ||
              m_compare(response, CV_IMAGE_ELEM(projected, T, y-1, x)) ||
              m_compare(response, CV_IMAGE_ELEM(projected, T, y+1, x)) ||
              // Corners of 3x3 spatial neighborhood
              m_compare(response, CV_IMAGE_ELEM(projected, T, y-1, x-1)) ||
              m_compare(response, CV_IMAGE_ELEM(projected, T, y-1, x+1)) ||
              m_compare(response, CV_IMAGE_ELEM(projected, T, y+1, x-1)) ||
              m_compare(response, CV_IMAGE_ELEM(projected, T, y+1, x+1))
            )
            continue;
        } else {
          // Reject immediately if not strong enough
          if (m_compare(-m_response_thresh, response) ||
              // Compare orthogonal points looking for quick rejection
              m_compare(CV_IMAGE_ELEM(projected, T, y, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(projected, T, y, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(projected, T, y-1, x), response) ||
              m_compare(CV_IMAGE_ELEM(projected, T, y+1, x), response) ||
              // Corners of 3x3 spatial neighborhood
              m_compare(CV_IMAGE_ELEM(projected, T, y-1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(projected, T, y-1, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(projected, T, y+1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(projected, T, y+1, x+1), response)
            )
            continue;
        }

        // Final thresholding step (e.g. line suppression)
        if ( m_post_thresh(x, y, scale) )
          continue;

        *inserter = Keypoint(x, y, (float)scale, response, scale);
        ++num_pts;
      }
    }
    
    return num_pts;
  }
};

/*!
  Functor to perform non-maximal suppression over a scale space.

  At each pixel coordinate, this version first looks for the scale which
  maximizes response. It then checks a 3x3x3 neighborhood around
  the point to determine whether it is a maximum. This seems to give
  better repeatability than the simple 3x3x3 version below.

  By default it finds maxima. Minima can be found by using
  std::less_equal<T> as the comparison functor.
 */
template< typename T = float, typename PostThreshold = NullThreshold,
          typename Compare = std::greater_equal<T> >
struct NonmaxSuppress3x3xN
{
  T m_response_thresh;
  PostThreshold m_post_thresh;
  Compare m_compare;
  
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppress3x3xN(T response_thresh = T(), PostThreshold post_thresh = PostThreshold(),
                      Compare compare = Compare())
    : m_response_thresh(response_thresh), m_post_thresh(post_thresh),
      m_compare(compare)
  {}
  
  //! Appends all maxima found to pts and returns how many were found.
  template< typename OutputIterator >
  int operator() (IplImage** responses, int n, OutputIterator inserter, int border) {
    int W = responses[0]->width, H = responses[0]->height;
    int num_pts = 0;
    
    // NOTE: We use a constant, conservative border here, even though we may miss some
    // small features near the edges of the image.
    for (int y = border; y < H - border; ++y) {
      for (int x = border; x < W - border; ++x) {
        // Determine scale which maximizes response
        int scale = 1;
        T response = CV_IMAGE_ELEM(responses[0], T, y, x);
        for (int s = 1; s < n; ++s) {
          T next_response = CV_IMAGE_ELEM(responses[s], T, y, x);
          if (m_compare(next_response, response)) {
            response = next_response;
            scale = s + 1;
          }
        }
        
        // Discard if first or last scale is maximal
        if (scale == 1 || scale == n) continue;
        
        IplImage *curr = responses[scale - 1];
        IplImage *prev = responses[scale - 2];
        IplImage *next = responses[scale];
        // NOTE: searching only the 3x3 spatial neighborhood seems to allow
        // too many redundant keypoints at wedge-shaped features. And the
        // 3x3x3 version is actually faster when using line suppression.
        // Reject immediately if not strong enough
        if (m_compare(m_response_thresh, response) ||
            // Compare orthogonal points looking for quick rejection
            m_compare(CV_IMAGE_ELEM(curr, T, y, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(curr, T, y, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(curr, T, y-1, x), response) ||
            m_compare(CV_IMAGE_ELEM(curr, T, y+1, x), response) ||
            // Corners of 3x3 spatial neighborhood
            m_compare(CV_IMAGE_ELEM(curr, T, y-1, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(curr, T, y-1, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(curr, T, y+1, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(curr, T, y+1, x+1), response) ||
            // Remaining comparisons from 3x3x3 neighborhood
            m_compare(CV_IMAGE_ELEM(prev, T, y, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y-1, x), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y+1, x), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y-1, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y-1, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y+1, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(prev, T, y+1, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y-1, x), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y+1, x), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y-1, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y-1, x+1), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y+1, x-1), response) ||
            m_compare(CV_IMAGE_ELEM(next, T, y+1, x+1), response) ||
            m_post_thresh(x, y, scale)
          )
          continue;
        
        *inserter = Keypoint(x, y, (float)scale, response, scale);
        ++num_pts;
      }
    }
    
    return num_pts;
  }
};

/*!
  Functor to perform non-maximal suppression over a scale space.

  This version considers a point maximal if it is the maximum over a 3x3x3
  scale/spatial neighborhood. By default it finds maxima. Minima can be found
  by using std::less_equal<T> as the comparison functor.
 */
template< typename T = float, typename PostThreshold = NullThreshold,
          typename Compare = std::greater_equal<T> >
struct NonmaxSuppress3x3x3
{
  T m_response_thresh;
  PostThreshold m_post_thresh;
  Compare m_compare;
  
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppress3x3x3(T response_thresh = T(), PostThreshold post_thresh = PostThreshold(),
                      Compare compare = Compare())
    : m_response_thresh(response_thresh), m_post_thresh(post_thresh),
      m_compare(compare)
  {}

  //! Appends all maxima found to pts and returns how many were found.
  int operator() (IplImage** responses, int n, std::vector<Keypoint> &pts) {
    int W = responses[0]->width, H = responses[0]->height;
    int num_pts = 0;
    
    for (int s = 2; s < n; ++s) {
      // TODO: this offset is no longer correct, need array of filter sizes
      int offset = 3*(s+1);
      IplImage *prev = responses[s-2];
      IplImage *curr = responses[s-1];
      IplImage *next = responses[s];
      for (int y = offset; y < H - offset; ++y) {
        for (int x = offset; x < W - offset; ++x) {
          T response = CV_IMAGE_ELEM(curr, T, y, x);
          // Compare orthogonal points first to try to get a quick rejection
          if (m_compare(m_response_thresh, response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y-1, x), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y+1, x), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y, x), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y, x), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y-1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y-1, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y+1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(curr, T, y+1, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y-1, x), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y+1, x), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y-1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y-1, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y+1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(prev, T, y+1, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y-1, x), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y+1, x), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y-1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y-1, x+1), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y+1, x-1), response) ||
              m_compare(CV_IMAGE_ELEM(next, T, y+1, x+1), response) ||
              m_post_thresh(x, y, s)
            )
            continue;
          pts.push_back(Keypoint(x, y, (float)s, response, s));
          ++num_pts;
        }
      }
    }
    
    return num_pts;
  }
};

#endif
