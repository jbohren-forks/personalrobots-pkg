#ifndef FEATURES_INTEGRAL_H
#define FEATURES_INTEGRAL_H

#include <cv.h>

//! Generates the tilted and "flat-tilted" integral images using dynamic programming.
void TiltedIntegral(IplImage* source, IplImage* tilted, IplImage* flat_tilted);

//! Calculates the sum of pixels within a rectangle using the integral image.
inline int UprightAreaSum(IplImage* upright, CvPoint top_left, CvPoint bottom_right)
{
    assert(upright && upright->depth == (int)IPL_DEPTH_32S);
    
    // upright is W+1 X H+1, so we adjust coordinates accordingly.
    return CV_IMAGE_ELEM(upright, int, bottom_right.y + 1, bottom_right.x + 1) -
        CV_IMAGE_ELEM(upright, int, top_left.y, bottom_right.x + 1) -
        CV_IMAGE_ELEM(upright, int, bottom_right.y + 1, top_left.x) +
        CV_IMAGE_ELEM(upright, int, top_left.y, top_left.x);
}

//! Calculates the sum of pixels within a tilted square using the tilted integral images.
//! We need the "flat-tilted" integral image due to a discretization artifact; using the
//! tilted integral image alone, a tilted square has two "flattened" two-pixel corners
//! which degrade its rotational invariance.
inline int TiltedAreaSum(IplImage* tilted, IplImage* flat_tilted, CvPoint center, int offset)
{
    assert(tilted && tilted->depth == (int) IPL_DEPTH_32S);
    assert(flat_tilted && flat_tilted->depth == (int) IPL_DEPTH_32S);

    // Integral images are W+1 X H+1, so we adjust coordinates accordingly.
    return CV_IMAGE_ELEM(tilted, int, center.y + offset + 1, center.x + 1) -
        CV_IMAGE_ELEM(flat_tilted, int, center.y, center.x - offset) -
        CV_IMAGE_ELEM(flat_tilted, int, center.y, center.x + offset + 1) +
        CV_IMAGE_ELEM(tilted, int, center.y - offset, center.x + 1);

    return 0;
}

#endif
