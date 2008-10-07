#ifndef FEATURES_THRESHOLD_H
#define FEATURES_THRESHOLD_H

#include <cv.h>
#include <cvwimage.h>

//! A noop post-threshold functor which rejects no keypoints.
struct NullThreshold
{
    inline bool operator() (int x, int y, int s) {
        return false;
    }
};

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
  float m_threshold;
  int* m_scale_sizes;
  
  LineSuppress(IplImage** responses, float threshold, int* sizes)
    : m_responses(responses), m_threshold(threshold), m_scale_sizes(sizes)
  {}
  
  bool operator() (int pt_x, int pt_y, int pt_s, float &line_response) {
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
    
    //return trace*trace/det >= m_threshold;
    line_response = trace*trace/det;
    return line_response >= m_threshold;
  }
};

struct LineSuppressSubsample
{
  IplImage** m_responses;
  float m_threshold;
  int* m_scale_sizes;
  
  LineSuppressSubsample(IplImage** responses, float threshold, int* sizes)
    : m_responses(responses), m_threshold(threshold), m_scale_sizes(sizes)
  {}
  
  bool operator() (int pt_x, int pt_y, int pt_s, float &line_response) {
    IplImage* L = m_responses[pt_s - 1];
    int offset = 2*m_scale_sizes[pt_s - 1]; // use the outer box as the window
    int step = pt_s;
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
    
    //return trace*trace/det >= m_threshold;
    line_response = trace*trace/det;
    return line_response >= m_threshold;
  }
};

struct LineSuppressProjected
{
  float m_threshold;
  int* m_scale_sizes;
  
  LineSuppressProjected(float threshold, int* sizes)
    : m_threshold(threshold), m_scale_sizes(sizes)
  {}

  bool operator() (int pt_x, int pt_y, int pt_s, IplImage* projected, float &line_response) {
    // Use 81 samples from a 9x9 grid over a window approximately the size of the
    // outer box. For scale 2 this is dense sampling; larger scales are subsampled.
    int step = m_scale_sizes[pt_s - 1] / 2;
    int offset = 4 * step;
    float sum_Lx_2 = 0;
    float sum_Ly_2 = 0;
    float sum_LxLy = 0;
    
    // Average over a window surrounding the point
    for (int y = pt_y - offset; y <= pt_y + offset; y += step) {
      for (int x = pt_x - offset; x <= pt_x + offset; x += step) {
        // Compute derivatives by the central finite difference.
        float Lx = CV_IMAGE_ELEM(projected, float, y, x-1) - CV_IMAGE_ELEM(projected, float, y, x+1);
        float Ly = CV_IMAGE_ELEM(projected, float, y-1, x) - CV_IMAGE_ELEM(projected, float, y+1, x);
        
        sum_Lx_2 += Lx*Lx;
        sum_Ly_2 += Ly*Ly;
        sum_LxLy += Lx*Ly;
      }
    }
    
    float trace = sum_Lx_2 + sum_Ly_2;
    float det = sum_Lx_2*sum_Ly_2 - sum_LxLy*sum_LxLy;
    
    //return trace*trace/det >= m_threshold;
    line_response = trace*trace/det;
    return line_response >= m_threshold;
  }
};

struct LineSuppressBinarized
{
  float m_threshold;
  int* m_scale_sizes;
  
  LineSuppressBinarized(float threshold, int* sizes)
    : m_threshold(threshold), m_scale_sizes(sizes)
  {}

  bool operator() (int pt_x, int pt_y, int pt_s, IplImage* scales, float &line_response) {
    // Subsample as in LineSuppressProjected
    int step = m_scale_sizes[pt_s - 1] / 2;
    int offset = 4 * step;
    int sum_Lx_2 = 0;
    int sum_Ly_2 = 0;
    int sum_LxLy = 0;
    
    // Average over a window surrounding the point
    for (int y = pt_y - offset; y <= pt_y + offset; y += step) {
      for (int x = pt_x - offset; x <= pt_x + offset; x += step) {
        // Compute derivatives by the central finite difference.
        int Lx = (CV_IMAGE_ELEM(scales, uchar, y, x-1) == pt_s) -
          (CV_IMAGE_ELEM(scales, uchar, y, x+1) == pt_s);
        int Ly = (CV_IMAGE_ELEM(scales, uchar, y-1, x) == pt_s) -
          (CV_IMAGE_ELEM(scales, uchar, y+1, x) == pt_s);

        // use abs instead for first two sums? Lx = -1, 0 or 1.
        sum_Lx_2 += Lx*Lx;
        sum_Ly_2 += Ly*Ly;
        sum_LxLy += Lx*Ly;
      }
    }
    
    float trace = sum_Lx_2 + sum_Ly_2;
    float det = sum_Lx_2*sum_Ly_2 - sum_LxLy*sum_LxLy;
    
    //return trace*trace/det >= m_threshold;
    line_response = trace*trace/det;
    return line_response >= m_threshold;
  }
};

struct LineSuppressCombined
{
  LineSuppressProjected m_lsp;
  LineSuppressBinarized m_lsb;

  LineSuppressCombined(float projected_threshold, float binarized_threshold, int* sizes)
    : m_lsp(projected_threshold, sizes), m_lsb(binarized_threshold, sizes)
  {}

  bool operator() (int pt_x, int pt_y, int pt_s, IplImage* projected, IplImage* scales,
                   float &line_response) {
    bool lsp_result = m_lsp(pt_x, pt_y, pt_s, projected, line_response);
    bool lsb_result = m_lsb(pt_x, pt_y, pt_s, scales, line_response);
    return lsp_result || lsb_result;
  }
};

#endif
