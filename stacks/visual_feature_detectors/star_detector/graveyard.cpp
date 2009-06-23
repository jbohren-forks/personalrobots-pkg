/* detector.h - scale interpolation */

//! Fill in scales based on indices and interpolation
  template< typename ForwardIterator >
  void InterpolateScales(ForwardIterator first, ForwardIterator last);

  //! Scale interpolation flag (TODO: currently useless)
  bool m_interpolate;

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

/* threshold.h - old line suppression functors */

/*!
  A post-threshold functor which rejects keypoints that are poorly
  localized along a line. The line response is taken to be the ratio
  of the two principal curvatures in a patch around the point, which
  we approximate using the second-moment matrix (the Harris measure).
  This is very similar to Lowe's Hessian-based approach.
 */
struct LineSuppress
{
  IplImage** m_responses;
  int* m_scale_sizes;
  float m_threshold;
  
  LineSuppress(IplImage** responses, int* sizes, float threshold)
    : m_responses(responses), m_scale_sizes(sizes), m_threshold(threshold)
  {}
  
  bool operator() (int pt_x, int pt_y, int pt_s) {
    IplImage* L = m_responses[pt_s - 1];
    int offset = 2*m_scale_sizes[pt_s - 1]; // use the outer box as the window
    float sum_Lx_2 = 0;
    float sum_Ly_2 = 0;
    float sum_LxLy = 0;
    
    // Average over a window surrounding the point
    for (int y = pt_y - offset; y <= pt_y + offset; ++y) {
      for (int x = pt_x - offset; x <= pt_x + offset; ++x) {
        // Compute derivatives by the central finite difference.
        float Lx = CV_IMAGE_ELEM(L, float, y, x-1) - CV_IMAGE_ELEM(L, float, y, x+1);
        float Ly = CV_IMAGE_ELEM(L, float, y-1, x) - CV_IMAGE_ELEM(L, float, y+1, x);
        
        sum_Lx_2 += Lx*Lx;
        sum_Ly_2 += Ly*Ly;
        sum_LxLy += Lx*Ly;
      }
    }
    
    float trace = sum_Lx_2 + sum_Ly_2;
    float det = sum_Lx_2*sum_Ly_2 - sum_LxLy*sum_LxLy;
    
    return trace*trace/det >= m_threshold;
  }
};

// Same as LineSuppress, but only samples 81 gradients regardless of scale.
struct LineSuppressSubsample
{
  IplImage** m_responses;
  int* m_scale_sizes;
  float m_threshold;
  
  LineSuppressSubsample(IplImage** responses, int* sizes, float threshold)
    : m_responses(responses), m_scale_sizes(sizes), m_threshold(threshold)
  {}
  
  bool operator() (int pt_x, int pt_y, int pt_s) {
    // Use 81 samples from a 9x9 grid over a window approximately the size of the
    // outer box. For scale 2 this is dense sampling; larger scales are subsampled.
    IplImage* L = m_responses[pt_s - 1];
    int step = m_scale_sizes[pt_s - 1] / 2;
    int offset = 4 * step;
    float sum_Lx_2 = 0;
    float sum_Ly_2 = 0;
    float sum_LxLy = 0;
    
    // Average over a window surrounding the point
    for (int y = pt_y - offset; y <= pt_y + offset; y += step) {
      for (int x = pt_x - offset; x <= pt_x + offset; x += step) {
        // Compute derivatives by the central finite difference.
        float Lx = CV_IMAGE_ELEM(L, float, y, x-1) - CV_IMAGE_ELEM(L, float, y, x+1);
        float Ly = CV_IMAGE_ELEM(L, float, y-1, x) - CV_IMAGE_ELEM(L, float, y+1, x);
        
        sum_Lx_2 += Lx*Lx;
        sum_Ly_2 += Ly*Ly;
        sum_LxLy += Lx*Ly;
      }
    }
    
    float trace = sum_Lx_2 + sum_Ly_2;
    float det = sum_Lx_2*sum_Ly_2 - sum_LxLy*sum_LxLy;
    
    return trace*trace/det >= m_threshold;
  }
};

/* nonmax_suppress.h - debug and obsolete code */

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
