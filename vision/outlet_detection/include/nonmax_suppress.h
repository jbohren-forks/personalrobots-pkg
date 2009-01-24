#ifndef OUTLET_DETECTION_NONMAX_SUPPRESS_H
#define OUTLET_DETECTION_NONMAX_SUPPRESS_H

#include <cv.h>
#include <algorithm>
#include <functional>
#include <boost/static_assert.hpp>

template< int W, int H, typename T = float,
          typename Compare = std::less_equal<T> >
class NonmaxSuppressWxH
{
  BOOST_STATIC_ASSERT( W % 2 == 1 && H % 2 == 1 );
  
public:
  //! Constructor taking the thresholds and comparison function to be used.
  NonmaxSuppressWxH(T response_thresh = T(), Compare compare = Compare())
    : m_response_thresh(response_thresh), m_compare(compare)
  {}

  inline T responseThreshold() const { return m_response_thresh; }
  inline void setResponseThreshold(T threshold) { m_response_thresh = threshold; }
  
  //! Outputs (through inserter) all maxima found and returns how many were found.
  template< typename OutputIterator >
  int operator() (IplImage* image, OutputIterator inserter, int border) {
    int num_pts = 0;
    int max_x = 0, max_y = 0;

    // NOTE: We use a constant, conservative border here, even though we may miss some
    // small features near the edges of the image.
    int end_y = image->height - border - 1;
    int end_x = image->width - border - 1;
    for (int y = border; y <= end_y; y += Y_OFFSET + 1) {
      for (int x = border; x <= end_x; x += X_OFFSET + 1) {
        T max_response = 255;

        // Find candidate maximum in current (W/2)x(H/2) tile
        int tile_end_y = std::min(y + Y_OFFSET, end_y);
        int tile_end_x = std::min(x + X_OFFSET, end_x);
        for (int next_y = y; next_y <= tile_end_y; ++next_y) {
          for (int next_x = x; next_x <= tile_end_x; ++next_x) {
            T response = CV_IMAGE_ELEM(image, T, next_y, next_x);
            if (m_compare(max_response, response)) {
              max_x = next_x;
              max_y = next_y;
              max_response = response;
            }
          }
        }

        // Threshold on response strength
        if (m_compare(max_response, m_response_thresh))
          goto max_failed;
        
        // Check if candidate point is maximal over full WxH neighborhood
        for (int yy = max_y - Y_OFFSET; yy <= max_y + Y_OFFSET; ++yy) {
          for (int xx = max_x - X_OFFSET; xx <= max_x + X_OFFSET; ++xx) {
            if (yy == max_y && xx == max_x) continue;
            T response = CV_IMAGE_ELEM(image, T, yy, xx);
            if (m_compare(max_response, response))
              goto max_failed;
          }
        }

        *inserter = cvPoint(max_x, max_y);
        ++num_pts;

      max_failed: /* null statement */ ;
      }
    }
    
    return num_pts;
  }

private:
  static const int X_OFFSET = W / 2;
  static const int Y_OFFSET = H / 2;
  
  T m_response_thresh;
  Compare m_compare;
};

#endif
