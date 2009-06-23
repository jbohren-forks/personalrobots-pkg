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

  LineSuppressProjected uses two approximations for efficiency:
    1. Subsampling - 81 samples are used independent of the size of the window.
    2. Uses the 'projected' response image rather than the exact response image
       at the keypoint's scale.
 */
class LineSuppressProjected
{
public:
  LineSuppressProjected(float threshold, IplImage* projected = NULL,
                        int* sizes = NULL)
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

  inline IplImage* image() const { return m_projected; }
  inline void setImage(IplImage* projected) { m_projected = projected; }
  
  inline int* scaleSizes() const { return m_scale_sizes; }
  inline void setScaleSizes(int* sizes) { m_scale_sizes = sizes; }
  
  inline float threshold() const { return m_threshold; }
  inline void setThreshold(float threshold) { m_threshold = threshold; }

private:
  IplImage* m_projected;
  int* m_scale_sizes;
  float m_threshold;
};

/*!
  Performs (subsampled) line suppression on a binarized patch from the
  image of projected scale values. Each pixel is given value 1 if
  its scale matches that of the keypoint, 0 otherwise. This cleans up
  some poorly localized points that slip through LineSuppressProjected
  due to variations in projected scale around the point.
*/
struct LineSuppressBinarized
{
public:
  LineSuppressBinarized(float threshold, IplImage* scales = NULL,
                        int* sizes = NULL)
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

        // Could use abs instead for first two sums, but doesn't seem to be quicker
        sum_Lx_2 += Lx*Lx;
        sum_Ly_2 += Ly*Ly;
        sum_LxLy += Lx*Ly;
      }
    }
    
    float trace = sum_Lx_2 + sum_Ly_2;
    float det = sum_Lx_2*sum_Ly_2 - sum_LxLy*sum_LxLy;
    
    return trace*trace/det >= m_threshold;
  }

  inline IplImage* image() const { return m_scales; }
  inline void setImage(IplImage* scales) { m_scales = scales; }
  
  inline int* scaleSizes() const { return m_scale_sizes; }
  inline void setScaleSizes(int* sizes) { m_scale_sizes = sizes; }
  
  inline float threshold() const { return m_threshold; }
  inline void setThreshold(float threshold) { m_threshold = threshold; }

private:
  IplImage* m_scales;
  int* m_scale_sizes;
  float m_threshold;
};

//! Applies both LineSuppressProjected and LineSuppressBinarized.
class LineSuppressHybrid
{
public:
  LineSuppressHybrid(float projected_threshold, float binarized_threshold,
                     IplImage* projected = NULL, IplImage* scales = NULL,
                     int* sizes = NULL)
    : m_lsp(projected_threshold, projected, sizes),
      m_lsb(binarized_threshold, scales, sizes)
  {}

  inline bool operator() (int pt_x, int pt_y, int pt_s) {
    return m_lsp(pt_x, pt_y, pt_s) || m_lsb(pt_x, pt_y, pt_s);
  }

  inline IplImage* projectedImage() const { return m_lsp.image(); }
  inline void setProjectedImage(IplImage* projected) { m_lsp.setImage(projected); }
  
  inline IplImage* scaleImage() const { return m_lsb.image(); }
  inline void setScaleImage(IplImage* scales) { m_lsb.setImage(scales); }
  
  inline int* scaleSizes() const { return m_lsp.scaleSizes(); }
  inline void setScaleSizes(int* sizes) {
    m_lsp.setScaleSizes(sizes);
    m_lsb.setScaleSizes(sizes);
  }
  
  inline float projectedThreshold() const { return m_lsp.threshold(); }
  inline void setProjectedThreshold(float threshold) { m_lsp.setThreshold(threshold); }
  
  inline float binarizedThreshold() const { return m_lsb.threshold(); }
  inline void setBinarizedThreshold(float threshold) { m_lsb.setThreshold(threshold); }

private:
  LineSuppressProjected m_lsp;
  LineSuppressBinarized m_lsb;
};

#endif
