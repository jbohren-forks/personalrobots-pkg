#include "star_detector/detector.h"
#include "star_detector/optimized_width.h"

StarDetector::StarDetector(CvSize size, int n, float response_threshold,
                           float line_threshold_projected,
                           float line_threshold_binarized)
  : m_upright(NULL),
    m_tilted(NULL),
    m_flat(NULL),
    m_projected(NULL),
    m_scales(NULL),
    m_filter_sizes(NULL),
    m_nonmax(response_threshold,
             LineSuppressHybrid(line_threshold_projected,
                                line_threshold_binarized)),
    m_filter_params(NULL)
{
  // Pre-allocate all the memory we need
  setScales(n);
  setImageSize(size);
}

void StarDetector::releaseImages()
{
  cvReleaseImage(&m_upright);
  cvReleaseImage(&m_tilted);
  cvReleaseImage(&m_flat);
  cvReleaseImage(&m_projected);
  cvReleaseImage(&m_scales);
}

StarDetector::~StarDetector()
{
  releaseImages();
  delete[] m_filter_sizes;
  delete[] m_filter_params;
}

void StarDetector::setScales(int n)
{
  delete[] m_filter_sizes;
  m_n = n;
  m_filter_sizes = new int[m_n];
  
  // Filter sizes increase geometrically, rounded to nearest integer
  m_filter_sizes[0] = 1;
  float cur_size = 1;
  int scale = 1;
  while (scale < m_n) {
    cur_size *= SCALE_RATIO;
    int rounded_size = (int)(cur_size + 0.5);
    if (rounded_size == m_filter_sizes[scale - 1])
      continue;
    m_filter_sizes[scale++] = rounded_size;
  }

  m_nonmax.thresholdFunction().setScaleSizes(m_filter_sizes);

  // Set border to size of maximum offset
  m_border = m_filter_sizes[m_n - 1] * 3;
}

void StarDetector::setImageSize(CvSize size)
{
  releaseImages();
  
  m_W = size.width;
  m_H = size.height;

  // SIMD optimized code requires integral images to have fixed width.
  int sumwidth = std::max(OPTIMIZED_WIDTH, m_W+1);
  m_upright = cvCreateImage(cvSize(sumwidth,m_H+1), IPL_DEPTH_32S, 1);
  m_tilted  = cvCreateImage(cvSize(sumwidth,m_H+1), IPL_DEPTH_32S, 1);
  m_flat    = cvCreateImage(cvSize(sumwidth,m_H+1), IPL_DEPTH_32S, 1);

  m_projected = cvCreateImage(size, IPL_DEPTH_32F, 1);
  m_scales = cvCreateImage(size, IPL_DEPTH_8U, 1);
  
  cvSet(m_scales, cvScalar(1));
  cvZero(m_projected);

  m_nonmax.thresholdFunction().setProjectedImage(m_projected);
  m_nonmax.thresholdFunction().setScaleImage(m_scales);
}

inline
int StarDetector::StarAreaSum(CvPoint center, int radius, int offset)
{
  int upright_area = UprightAreaSum(m_upright, cvPoint(center.x - radius, center.y - radius),
                                    cvPoint(center.x + radius, center.y + radius));
  int tilt_area = TiltedAreaSum(m_tilted, m_flat, center, offset);
  
  return upright_area + tilt_area;
}

inline
int StarDetector::StarPixels(int radius, int offset)
{
  int upright_pixels = (2*radius + 1)*(2*radius + 1);
  int tilt_pixels = offset*offset + (offset + 1)*(offset + 1);
  
  return upright_pixels + tilt_pixels;
}

void StarDetector::FilterResponses()
{
  if (!m_filter_params) {
    // Cache constants associated with each scale
    m_filter_params = new FilterParams[m_n];
    for (int s = 0; s < m_n; ++s) {
      FilterParams &fp = m_filter_params[s];
      fp.inner_r = m_filter_sizes[s];
      fp.outer_r = 2*fp.inner_r;
      fp.inner_offset = fp.inner_r + fp.inner_r / 2;
      fp.outer_offset = fp.outer_r + fp.outer_r / 2;
      int inner_pix = StarPixels(fp.inner_r, fp.inner_offset);
      int outer_pix = StarPixels(fp.outer_r, fp.outer_offset) - inner_pix;
      fp.inner_normalizer = 1.0f / inner_pix;
      fp.outer_normalizer = 1.0f / outer_pix;
    }
  }
  
  // Calculate responses over all scales and project into single maximal image
  for (int y = m_border; y < m_H - m_border; ++y) {
    for (int x = m_border; x < m_W - m_border; ++x) {
      uchar scale = 0;
      float max_response = 0.0f;
      float abs_max_response = 0.0f;
      for (int s = 0; s < m_n; ++s) {
        FilterParams &fp = m_filter_params[s];
        int inner_area = StarAreaSum(cvPoint(x, y), fp.inner_r, fp.inner_offset);
        int outer_area = StarAreaSum(cvPoint(x, y), fp.outer_r, fp.outer_offset);
        outer_area -= inner_area;
        float response = inner_area*fp.inner_normalizer - outer_area*fp.outer_normalizer;
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
}

#include "generated.i"
