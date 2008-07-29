#include "detector.h"
#include "nonmax_suppress.h"
#include <cstdio>
#include <iostream>

WgDetector::WgDetector(CvSize size, int n, float threshold, float line_threshold)
    : m_n(n), m_W(size.width), m_H(size.height),
      m_responses(new IplImage*[n]),
      m_threshold(threshold),
      m_line_threshold(line_threshold),
      m_nonmax(threshold, LineSuppress(m_responses, line_threshold)),
      m_nonmin(-threshold, LineSuppress(m_responses, line_threshold)),
      m_filter_sizes(new int[n])
{
    m_upright = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);
    m_tilted  = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);
    m_flat    = cvCreateImage(cvSize(m_W+1,m_H+1), IPL_DEPTH_32S, 1);

    for (int i = 0; i < n; ++i) {
        m_responses[i] = cvCreateImage(size, IPL_DEPTH_32F, 1);
        cvZero(m_responses[i]);
    }

    m_filter_sizes[0] = 1;
    float cur_size = 1;
    int scale = 1;
    while (scale < n) {
        cur_size *= SCALE_MULTIPLIER;
        int rounded_size = (int)(cur_size + 0.5);
        if (rounded_size == m_filter_sizes[scale - 1])
            continue;
        m_filter_sizes[scale++] = rounded_size;
    }
}

WgDetector::~WgDetector()
{
    cvReleaseImage(&m_upright);
    cvReleaseImage(&m_tilted);
    cvReleaseImage(&m_flat);

    for (int i = 0; i < m_n; ++i) {
        cvReleaseImage(&m_responses[i]);
    }
    delete[] m_responses;
    delete[] m_filter_sizes;
}

std::vector<Keypoint> WgDetector::DetectPoints(IplImage* source)
{
    assert(source && source->depth == (int)IPL_DEPTH_8U);

    cvIntegral(source, m_upright, NULL, NULL);
    TiltedIntegral(source, m_tilted, m_flat);

    FilterResponses();

    return FindExtrema();
}

inline
int WgDetector::StarAreaSum(CvPoint center, int radius, int offset)
{
    int upright_area = UprightAreaSum(m_upright, cvPoint(center.x - radius, center.y - radius),
                                      cvPoint(center.x + radius, center.y + radius));
    int tilt_area = TiltedAreaSum(m_tilted, m_flat, center, offset);

    return upright_area + tilt_area;
}

// TODO: change this to LUT?
inline
int WgDetector::StarPixels(int radius, int offset)
{
    int upright_pixels = (2*radius + 1)*(2*radius + 1);
    int tilt_pixels = offset*offset + (offset + 1)*(offset + 1);

    return upright_pixels + tilt_pixels;
}

// TODO: use fixed point instead of floating point
void WgDetector::BilevelFilter(IplImage* dst, int scale)
{
    //int inner_r = scale, outer_r = 2*scale;
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
            // TODO: next line is a bottleneck
            CV_IMAGE_ELEM(dst, float, y, x) = (float)inner_area/inner_pix -
                (float)outer_area/outer_pix;
        }
    }
}

inline void WgDetector::FilterResponses()
{
    for (int scale = 1; scale <= m_n; ++scale) {
        BilevelFilter(m_responses[scale-1], scale);
    }
}

std::vector<Keypoint> WgDetector::FindExtrema()
{
    std::vector<Keypoint> keypoints;

    m_nonmax(m_responses, m_n, keypoints);
    m_nonmin(m_responses, m_n, keypoints);

    // TODO: make this cleaner
    typedef std::vector<Keypoint>::iterator iter;
    for (iter i = keypoints.begin(); i != keypoints.end(); ++i) {
        int index = i->scale;
        i->scale = m_filter_sizes[index - 1];
    }
    
    return keypoints;
}
