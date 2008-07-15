#include "censure.h"
#include "nonmax_suppress.h"
#include <stdio.h>
#include <iostream>

CensureDetector::CensureDetector(CvSize size, int n, float threshold, float line_threshold)
    : m_n(n), m_W(size.width), m_H(size.height), m_threshold(threshold),
      m_line_threshold(line_threshold)
{
    m_upright = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);
    m_tilted  = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);
    m_flat    = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);

    m_responses = new IplImage*[n];
    for (int i = 0; i < n; ++i) {
        m_responses[i] = cvCreateImage(size, IPL_DEPTH_32F, 1);
        cvZero(m_responses[i]);
    }
}

CensureDetector::~CensureDetector()
{
    cvReleaseImage(&m_upright);
    cvReleaseImage(&m_tilted);
    cvReleaseImage(&m_flat);

    for (int i = 0; i < m_n; ++i) {
        cvReleaseImage(&m_responses[i]);
    }
    delete[] m_responses;
}

std::vector<Keypoint> CensureDetector::DetectPoints(IplImage* source)
{
    assert(source && source->depth == (int)IPL_DEPTH_8U);

    cvIntegral(source, m_upright, NULL, NULL);
    TiltedIntegral(source, m_tilted, m_flat);

    FilterResponses();
    
    return FindExtrema();
}

inline
int CensureDetector::OctagonalAreaSum(CvPoint center, int radius, int offset)
{
    int upright_area = UprightAreaSum(m_upright, cvPoint(center.x - radius, center.y - radius),
                                      cvPoint(center.x + radius, center.y + radius));
    int tilt_area = TiltedAreaSum(m_tilted, m_flat, center, offset);

    return upright_area + tilt_area;
}

// TODO: change this to LUT?
inline
int CensureDetector::OctagonPixels(int radius, int offset)
{
    int upright_pixels = (2*radius + 1)*(2*radius + 1);
    int tilt_pixels = offset*offset + (offset + 1)*(offset + 1);

    return upright_pixels + tilt_pixels;
}

// TODO: use fixed point instead of floating point
void CensureDetector::BilevelFilter(IplImage* dst, int scale)
{
    int inner_r = scale, outer_r = 2*scale;
    int inner_offset = inner_r + inner_r / 2;
    int outer_offset = outer_r + outer_r / 2;
    int inner_pix = OctagonPixels(inner_r, inner_offset);
    int outer_pix = OctagonPixels(outer_r, outer_offset) - inner_pix;

    for (int y = outer_offset; y < m_H - outer_offset; ++y) {
        for (int x = outer_offset; x < m_W - outer_offset; ++x) {
            int inner_area = OctagonalAreaSum(cvPoint(x, y), inner_r, inner_offset);
            int outer_area = OctagonalAreaSum(cvPoint(x, y), outer_r, outer_offset);
            outer_area -= inner_area;
            CV_IMAGE_ELEM(dst, float, y, x) = (float)inner_area/inner_pix -
                (float)outer_area/outer_pix;
        }
    }
}

inline void CensureDetector::FilterResponses()
{
    for (int scale = 1; scale <= m_n; ++scale) {
        BilevelFilter(m_responses[scale-1], scale);
    }
}

std::vector<Keypoint> CensureDetector::FindExtrema()
{
    std::vector<Keypoint> keypoints;
    nonmax_suppress<float, std::greater<float> >(m_responses, m_n, keypoints);
    nonmax_suppress<float, std::less<float> >(m_responses, m_n, keypoints);

    // TODO: temporary spot, for debugging
    typedef std::vector<Keypoint>::iterator iter;
    for (iter i = keypoints.begin(); i != keypoints.end(); ++i) {
        LineSuppress(*i, m_line_threshold);
    }
    
    return keypoints;
}

bool CensureDetector::LineSuppress(Keypoint & pt, float threshold)
{
    IplImage* L = m_responses[pt.scale - 1];
    int offset = 2*pt.scale; // use the outer box as the window
    float sum_Lx_2 = 0;
    float sum_Ly_2 = 0;
    float sum_LxLy = 0;
    
    for (int y = pt.y - offset; y <= pt.y + offset; ++y) {
        for (int x = pt.x - offset; x <= pt.x + offset; ++x) {
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

    pt.line_response = trace*trace/det;
    return pt.line_response < threshold;
    //return trace*trace/det;
}
