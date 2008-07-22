#ifndef FEATURES_THRESHOLD_H
#define FEATURES_THRESHOLD_H

#include <cv.h>

//! A noop post-threshold functor which rejects no keypoints.
struct NullThreshold
{
    inline bool operator() (int x, int y, int scale) {
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

    LineSuppress(IplImage** responses, float threshold)
        : m_responses(responses), m_threshold(threshold)
    {}

    bool operator() (int pt_x, int pt_y, int scale) {
        IplImage* L = m_responses[scale - 1];
        int offset = 2*scale; // use the outer box as the window
        float sum_Lx_2 = 0;
        float sum_Ly_2 = 0;
        float sum_LxLy = 0;
        
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

#endif
