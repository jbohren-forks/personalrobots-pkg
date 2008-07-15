#ifndef CENSURE_H
#define CENSURE_H

#include "integral.h"
#include "keypoint.h"
#include <cv.h>
#include <vector>

class CensureDetector
{
public:
    static const float DEFAULT_THRESHOLD = 30;
    static const float DEFAULT_LINE_THRESHOLD = 10;
    
    CensureDetector(CvSize dims, int n = 7, float threshold = DEFAULT_THRESHOLD,
                    float line_threshold = DEFAULT_LINE_THRESHOLD);

    ~CensureDetector();
    
    std::vector<Keypoint> DetectPoints(IplImage* source);

    //Integral?

//private:
    int m_n, m_W, m_H;
    IplImage* m_upright;
    IplImage* m_tilted;
    IplImage* m_flat;
    IplImage** m_responses;
    // Intermediate octagonal sum images?
    float m_threshold;
    float m_line_threshold;

    int OctagonalAreaSum(CvPoint center, int radius, int offset);

    int OctagonPixels(int radius, int offset);

    void BilevelFilter(IplImage* dst, int scale);
    
    void FilterResponses();

    std::vector<Keypoint> FindExtrema();

    bool LineSuppress(Keypoint & pt, float threshold);
};

#endif
