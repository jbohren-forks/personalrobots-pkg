#ifndef FEATURES_NON_MAXIMA_SUPPRESSION_H
#define FEATURES_NON_MAXIMA_SUPPRESSION_H

#include "keypoint.h"
#include "threshold.h"
#include <cv.h>
#include <vector>
#include <functional>

// TODO: this is a simple implementation, may be optimizable
//       see http://www.vision.ee.ethz.ch/~aneubeck/
// TODO: possible to speed up by combining maxima and minima finding?

/*!
  Functor to perform non-maximal suppression over a scale space.

  At each pixel coordinate, this version first looks for the scale which
  maximizes response. It then checks a 3x3x3 neighborhood around
  the point to determine whether it is a maximum.
  
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
    int operator() (IplImage** responses, int n, std::vector<Keypoint> &pts) {
        int W = responses[0]->width, H = responses[0]->height;
        int num_pts = 0;

        int offset = 3*n;
        for (int y = offset; y < H - offset; ++y) {
            for (int x = offset; x < W - offset; ++x) {
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
                if (m_compare(m_response_thresh, response) ||
                    m_compare(CV_IMAGE_ELEM(curr, T, y, x-1), response) ||
                    m_compare(CV_IMAGE_ELEM(curr, T, y, x+1), response) ||
                    m_compare(CV_IMAGE_ELEM(curr, T, y-1, x), response) ||
                    m_compare(CV_IMAGE_ELEM(curr, T, y+1, x), response) ||
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
                    m_post_thresh(x, y, scale)
                    )
                    continue;
                pts.push_back(Keypoint(x, y, scale, response));
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
                    pts.push_back(Keypoint(x, y, s, response));
                    ++num_pts;
                }
            }
        }
        
        return num_pts;
    }
};

#endif
