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

// Same as LineSuppressSubsample, but uses the projected response image rather
// than the actual response image at the appropriate scale.
struct LineSuppressProjected
{
  IplImage* m_projected;
  int* m_scale_sizes;
  float m_threshold;
  
  LineSuppressProjected(IplImage* projected, int* sizes, float threshold)
    : m_projected(projected), m_scale_sizes(sizes), m_threshold(threshold)
  {}

  bool operator() (int pt_x, int pt_y, int pt_s) {
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
        float Lx = CV_IMAGE_ELEM(m_projected, float, y, x-1) - CV_IMAGE_ELEM(m_projected, float, y, x+1);
        float Ly = CV_IMAGE_ELEM(m_projected, float, y-1, x) - CV_IMAGE_ELEM(m_projected, float, y+1, x);
        
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

// Performs (subsampled) line suppression on a binarized patch from the
// image of projected scale values. Each pixel is given value 1 if
// its scale matches that of the keypoint, 0 otherwise. This cleans up
// some poorly localized points that slip through LineSuppressProjected
// due to variations in projected scale around the point.
struct LineSuppressBinarized
{
  IplImage* m_scales;
  int* m_scale_sizes;
  float m_threshold;
  
  LineSuppressBinarized(IplImage* scales, int* sizes, float threshold)
    : m_scales(scales), m_scale_sizes(sizes), m_threshold(threshold)
  {}

  bool operator() (int pt_x, int pt_y, int pt_s) {
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
        int Lx = (CV_IMAGE_ELEM(m_scales, uchar, y, x-1) == pt_s) -
          (CV_IMAGE_ELEM(m_scales, uchar, y, x+1) == pt_s);
        int Ly = (CV_IMAGE_ELEM(m_scales, uchar, y-1, x) == pt_s) -
          (CV_IMAGE_ELEM(m_scales, uchar, y+1, x) == pt_s);

        // use abs instead for first two sums? Lx = -1, 0 or 1.
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

// Applies both LineSuppressProjected and LineSuppressBinarized.
// TODO: Possibly should merge the implementations for only one pass. Or
// if kept separate, do the faster one first.
struct LineSuppressHybrid
{
  LineSuppressProjected m_lsp;
  LineSuppressBinarized m_lsb;

  LineSuppressHybrid(IplImage* projected, IplImage* scales, int* sizes,
                     float projected_threshold, float binarized_threshold)
    : m_lsp(projected, sizes, projected_threshold),
      m_lsb(scales, sizes, binarized_threshold)
  {}

  inline bool operator() (int pt_x, int pt_y, int pt_s) {
    return m_lsp(pt_x, pt_y, pt_s) || m_lsb(pt_x, pt_y, pt_s);
  }
};

#endif
