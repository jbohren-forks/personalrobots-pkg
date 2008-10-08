#ifndef FEATURES_NON_MAXIMA_SUPPRESSION_H
#define FEATURES_NON_MAXIMA_SUPPRESSION_H

#include "star_detector/keypoint.h"
#include "star_detector/threshold.h"
#include <cv.h>
#include <vector>
#include <iterator>
#include <functional>
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

// Uses the projected response image
template< typename T = float, typename PostThreshold = NullThreshold,
          typename Compare = std::greater_equal<T> >
struct NonmaxSuppressProject
{
  T m_response_thresh;
  PostThreshold m_post_thresh;
  Compare m_compare;
  
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppressProject(T response_thresh = T(), PostThreshold post_thresh = PostThreshold(),
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
        if (m_compare(response, T(0))) {
          // Reject immediately if not strong enough
          if (m_compare(m_response_thresh, response) ||
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
        } else {
          // Reject immediately if not strong enough
          if (m_compare(response, -m_response_thresh) ||
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

// TODO: this is a simple implementation, may be optimizable
//       see http://www.vision.ee.ethz.ch/~aneubeck/
//       though probably not worth it for 3x3 case
// TODO: possible to speed up by combining maxima and minima finding?

/*!
  Functor to perform non-maximal suppression over a scale space.

  At each pixel coordinate, this version first looks for the scale which
  maximizes response. It then checks a 3x3x3 neighborhood around
  the point to determine whether it is a maximum. This seems to give
  better repeatability than the simple 3x3x3 version below.
  
  NonmaxSuppress3x3xN uses some template parameters to
  achieve a useful degree of generality without sacrificing performance.
  By default it finds maxima on float images, but this behavior can be
  adapted.

  T: the pixel type of the images

  PostThreshold: Optionally, an additional thresholding functor can be
    specified. The functor may be expensive, as it will only be applied to
    a small number of extrema which already exceed the strength threshold.

  Compare: the comparison functor used to compare response values.
    By default std::greater_equal<T> is used, which finds maxima.
    Instead std::less_equal<T> will find minima.
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
  scale/spatial neighborhood. It uses some template parameters to
  achieve a useful degree of generality without sacrificing performance.
  By default it finds maxima on float images, but this behavior can be
  adapted.

  T: the pixel type of the images

  PostThreshold: Optionally, an additional thresholding functor can be
    specified. The functor may be expensive, as it will only be applied to
    a small number of extrema which already exceed the strength threshold.

  Compare: the comparison functor used to compare response values.
    By default std::greater_equal<T> is used, which finds maxima.
    Instead std::less_equal<T> will find minima.
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
