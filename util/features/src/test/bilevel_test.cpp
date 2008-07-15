#include "censure.h"
#include "cv_wrapper.h"
#include <stdio.h>
#include <iostream>

// The response over a smooth ramp should be all zeros.
int main()
{
    const int dim = 13;
    Image<uchar> range( cvCreateImage(cvSize(dim,dim), IPL_DEPTH_8U, 1) );

    for (int i = 0; i < dim*dim; ++i) {
        range(i/dim, i%dim) = i;
        //range(i/dim, i%dim) = 5;
    }

    CensureDetector detector(cvSize(dim,dim), 1);
    detector.DetectPoints(range);
    Image<float> response(detector.m_responses[0], false);

    std::cout << range;
    std::cout << response;
    
    return 0;
}
