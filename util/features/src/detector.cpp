#include "detector.h"
#include "nonmax_suppress.h"
#include <cstdio>
#include <iostream>

StarDetector::StarDetector(CvSize size, int n, float threshold, float line_threshold)
    : m_n(n), m_W(size.width), m_H(size.height),
      m_responses(new IplImage*[n]),
      m_threshold(threshold),
      m_line_threshold(line_threshold),
      m_filter_sizes(new int[n]),
      m_nonmax(threshold, LineSuppress(m_responses, line_threshold, m_filter_sizes)),
      m_nonmin(-threshold, LineSuppress(m_responses, line_threshold, m_filter_sizes)),
      m_interpolate(true)
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
        cur_size *= SCALE_RATIO;
        int rounded_size = (int)(cur_size + 0.5);
        if (rounded_size == m_filter_sizes[scale - 1])
            continue;
        m_filter_sizes[scale++] = rounded_size;
    }
}

StarDetector::~StarDetector()
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

std::vector<Keypoint> StarDetector::DetectPoints(IplImage* source)
{
    assert(source && source->depth == (int)IPL_DEPTH_8U);

    cvIntegral(source, m_upright, NULL, NULL);
    TiltedIntegral(source, m_tilted, m_flat);

    FilterResponses();

    return FindExtrema();
}

void StarDetector::interpolate(bool value)
{
    m_interpolate = value;
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

void StarDetector::BilevelFilter(IplImage* dst, int scale)
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

// This version uses outer_r ~= 1.4*inner_r
void StarDetector::FilterResponses3()
{
    int inner_r = m_filter_sizes[0];
    int inner_offset = inner_r + inner_r / 2;
    int inner_pix = StarPixels(inner_r, inner_offset);
    
    // Inner area for first scale
    // TODO: could use outer offset
    for (int y = inner_offset; y < m_H - inner_offset; ++y) {
        for (int x = inner_offset; x < m_W - inner_offset; ++x) {
            int inner_area = StarAreaSum(cvPoint(x, y), inner_r, inner_offset);
            CV_IMAGE_ELEM(m_responses[0], float, y, x) = inner_area;
        }
    }
    
    // TODO: replace CV_IMAGE_ELEMs
    int s;
    for (s = 0; s < m_n - 1; ++s) {
        int outer_r = m_filter_sizes[s + 1];
        int outer_offset = outer_r + outer_r / 2;
        int outer_pix = StarPixels(outer_r, outer_offset);
        
        for (int y = outer_offset; y < m_H - outer_offset; ++y) {
            for (int x = outer_offset; x < m_W - outer_offset; ++x) {
                int outer_area = StarAreaSum(cvPoint(x, y), outer_r, outer_offset);
                
                // Finish calculating response at current scale
                float &response = CV_IMAGE_ELEM(m_responses[s], float, y, x);
                response = response/inner_pix - (float)outer_area/outer_pix;
                
                // Save outer area as inner area for the next scale
                CV_IMAGE_ELEM(m_responses[s + 1], float, y, x) = outer_area;
            }
        }
        
        inner_r = outer_r;
        inner_pix = outer_pix;
    }
    
    // Outer area for last scale
    // TODO: can this all be compressed a little?
    int outer_r = m_filter_sizes[s];
    int outer_offset = outer_r + outer_r / 2;
    int outer_pix = StarPixels(outer_r, outer_offset);
    
    for (int y = outer_offset; y < m_H - outer_offset; ++y) {
        for (int x = outer_offset; x < m_W - outer_offset; ++x) {
            int outer_area = StarAreaSum(cvPoint(x, y), outer_r, outer_offset);
            float &response = CV_IMAGE_ELEM(m_responses[s], float, y, x);
            response = response/inner_pix - (float)outer_area/outer_pix;
        }
    }
}

// TODO: use fixed point instead of floating point
void StarDetector::FilterResponses2()
{
    for (int parity = 0; parity < 2; ++parity) {
        int inner_r = m_filter_sizes[parity];
        int inner_offset = inner_r + inner_r / 2;
        int inner_pix = StarPixels(inner_r, inner_offset);

        // Inner area for first scale
        // TODO: could use outer offset
        for (int y = inner_offset; y < m_H - inner_offset; ++y) {
            for (int x = inner_offset; x < m_W - inner_offset; ++x) {
                int inner_area = StarAreaSum(cvPoint(x, y), inner_r, inner_offset);
                CV_IMAGE_ELEM(m_responses[parity], float, y, x) = inner_area;
            }
        }

        // TODO: replace CV_IMAGE_ELEMs
        int s;
        for (s = parity; s < m_n - 2; s += 2) {
            int outer_r = m_filter_sizes[s + 2];
            int outer_offset = outer_r + outer_r / 2;
            int outer_pix = StarPixels(outer_r, outer_offset);

            for (int y = outer_offset; y < m_H - outer_offset; ++y) {
                for (int x = outer_offset; x < m_W - outer_offset; ++x) {
                    int outer_area = StarAreaSum(cvPoint(x, y), outer_r, outer_offset);
                    
                    // Finish calculating response at current scale
                    float &response = CV_IMAGE_ELEM(m_responses[s], float, y, x);
                    response = response/inner_pix - (float)outer_area/outer_pix;

                    // Save outer area as inner area for the next scale
                    CV_IMAGE_ELEM(m_responses[s + 2], float, y, x) = outer_area;
                }
            }

            inner_r = outer_r;
            inner_pix = outer_pix;
        }

        // Outer area for last scale
        // TODO: can this all be compressed a little?
        int outer_r = m_filter_sizes[s];
        int outer_offset = outer_r + outer_r / 2;
        int outer_pix = StarPixels(outer_r, outer_offset);

        for (int y = outer_offset; y < m_H - outer_offset; ++y) {
            for (int x = outer_offset; x < m_W - outer_offset; ++x) {
                int outer_area = StarAreaSum(cvPoint(x, y), outer_r, outer_offset);
                float &response = CV_IMAGE_ELEM(m_responses[s], float, y, x);
                response = response/inner_pix - (float)outer_area/outer_pix;
            }
        }
    }
}

inline void StarDetector::FilterResponses()
{
    for (int scale = 1; scale <= m_n; ++scale) {
        BilevelFilter(m_responses[scale-1], scale);
    }
}

std::vector<Keypoint> StarDetector::FindExtrema()
{
    std::vector<Keypoint> keypoints;

    m_nonmax(m_responses, m_n, keypoints);
    m_nonmin(m_responses, m_n, keypoints);

    if (m_interpolate) InterpolateScales(keypoints);
    
    return keypoints;
}

void StarDetector::InterpolateScales(std::vector<Keypoint> &keypoints)
{
    typedef std::vector<Keypoint>::iterator iter;
    for (iter i = keypoints.begin(); i != keypoints.end(); ++i) {
        // Do quadratic interpolation using finite differences
        float r1 = CV_IMAGE_ELEM(m_responses[i->s - 2], float, i->y, i->x);
        float r2 = CV_IMAGE_ELEM(m_responses[i->s - 1], float, i->y, i->x);
        float r3 = CV_IMAGE_ELEM(m_responses[i->s], float, i->y, i->x);
        float s_hat = 0.5*(r1 - r3) / (r1 - 2*r2 + r3);

        i->scale = pow(SCALE_RATIO, s_hat + i->s);
    }
}
