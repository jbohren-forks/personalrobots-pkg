#include "star_detector/detector.h"

StarDetector::StarDetector(CvSize size, int n, float response_threshold,
                           float line_threshold_projected,
                           float line_threshold_binarized)
  : m_n(n), m_W(size.width), m_H(size.height),
    // Pre-allocate all the memory we need
    m_tilted( cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1) ),
    m_flat( cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1) ),
    m_responses(new IplImage*[n]),
    m_projected( cvCreateImage(size, IPL_DEPTH_32F, 1) ),
    m_scales( cvCreateImage(size, IPL_DEPTH_8U, 1) ),
    m_filter_sizes(new int[n]),
    m_nonmax(response_threshold,
             LineSuppressHybrid(m_projected, m_scales, m_filter_sizes,
                                line_threshold_projected,
                                line_threshold_binarized)),
    m_interpolate(true)
{
  int sumwidth = 1777;
  m_upright = cvCreateImage(cvSize(sumwidth,m_H+1), IPL_DEPTH_32S, 1);
  m_tilted  = cvCreateImage(cvSize(sumwidth,m_H+1), IPL_DEPTH_32S, 1);
  m_flat    = cvCreateImage(cvSize(sumwidth,m_H+1), IPL_DEPTH_32S, 1);

  m_response_threshold = response_threshold;

  for (int i = 0; i < n; ++i) {
    m_responses[i] = cvCreateImage(size, IPL_DEPTH_32F, 1);
    cvZero(m_responses[i]);
  }
  cvSet(m_scales, cvScalar(1));

  // Filter sizes increase geometrically, rounded to nearest integer
  m_filter_sizes[0] = 1;
  float cur_size = 1;
  int scale = 1;
  while (scale < n) {
    cur_size *= SCALE_RATIO;
    int rounded_size = (int)(cur_size + 0.5);
    if (rounded_size == m_filter_sizes[scale - 1])
      continue;
    m_filter_sizes[scale++] = rounded_size;
  }

  // Set border to size of maximum offset
  m_border = m_filter_sizes[m_n - 1] * 3;
}

StarDetector::~StarDetector()
{
  cvReleaseImage(&m_upright);
  cvReleaseImage(&m_tilted);
  cvReleaseImage(&m_flat);
  cvReleaseImage(&m_projected);
  cvReleaseImage(&m_scales);

  for (int i = 0; i < m_n; ++i) {
    cvReleaseImage(&m_responses[i]);
  }
  delete[] m_responses;
  delete[] m_filter_sizes;
}

inline
int StarDetector::StarAreaSum(CvPoint center, int radius, int offset)
{
  int upright_area = UprightAreaSum(m_upright, cvPoint(center.x - radius, center.y - radius),
                                    cvPoint(center.x + radius, center.y + radius));
  int tilt_area = TiltedAreaSum(m_tilted, m_flat, center, offset);
  
  return upright_area + tilt_area;
}

// TODO: change this to LUT?
inline
int StarDetector::StarPixels(int radius, int offset)
{
  int upright_pixels = (2*radius + 1)*(2*radius + 1);
  int tilt_pixels = offset*offset + (offset + 1)*(offset + 1);
  
  return upright_pixels + tilt_pixels;
}

// TODO: use fixed point instead of floating point
void StarDetector::BilevelFilter(IplImage* dst, int scale)
{
  int inner_r = m_filter_sizes[scale - 1];
  int outer_r = 2*inner_r;
  int inner_offset = inner_r + inner_r / 2;
  int outer_offset = outer_r + outer_r / 2;
  int inner_pix = StarPixels(inner_r, inner_offset);
  int outer_pix = StarPixels(outer_r, outer_offset) - inner_pix;
  
  for (int y = outer_offset; y < m_H - outer_offset; ++y) {
    for (int x = outer_offset; x < m_W - outer_offset; ++x) {
      int inner_area = StarAreaSum(cvPoint(x, y), inner_r, inner_offset);
      int outer_area = StarAreaSum(cvPoint(x, y), outer_r, outer_offset);
      outer_area -= inner_area;
      // TODO: next line is expensive
      CV_IMAGE_ELEM(dst, float, y, x) = (float)inner_area/inner_pix -
        (float)outer_area/outer_pix;
    }
  }
}

/*inline*/ void StarDetector::FilterResponses()
{
  for (int scale = 1; scale <= m_n; ++scale) {
    BilevelFilter(m_responses[scale-1], scale);
  }

  // Project responses into single maximal image
  for (int y = 0; y < m_H; ++y) {
    for (int x = 0; x < m_W; ++x) {
      uchar scale = 1;
      float max_response = CV_IMAGE_ELEM(m_responses[0], float, y, x);
      float abs_max_response = std::abs(max_response);
      for (int s = 1; s < m_n; ++s) {
        float response = CV_IMAGE_ELEM(m_responses[s], float, y, x);
        float abs_response = std::abs(response);
        if (abs_response > abs_max_response) {
          max_response = response;
          abs_max_response = abs_response;
          scale = s + 1;
        }
      }
      CV_IMAGE_ELEM(m_projected, float, y, x) = max_response;
      CV_IMAGE_ELEM(m_scales, uchar, y, x) = scale;
    }
  }

  // DEBUG
  //cvSaveImage("scales.pgm", scales);
  //SaveResponseImage("projected.pgm", projected);
}

#include "generated.i"
