#ifndef NON_MAXIMA_SUPPRESSION_H
#define NON_MAXIMA_SUPPRESSION_H

#include "keypoint.h"
#include <cv.h>
#include <vector>
#include <functional>

// TODO: this is a simple implementation, may be optimizable
//       see http://www.vision.ee.ethz.ch/~aneubeck/
// TODO: possible to speed up by combining maxima and minima finding? check CLRS
// TODO: thresholding should probably be done here as first cmp for speed
// TODO: integrate line suppression here?
// TODO: probably cleaner as a functor
template< typename T, typename Compare >
int nonmax_suppress(IplImage** responses, int n, std::vector<Keypoint> &pts)
                    //Compare compare = std::less<T>())
{
    int W = responses[0]->width, H = responses[0]->height;
    int num_pts = 0;
    Compare compare;
    
    for (int s = 2; s < n; ++s) {
        int offset = 3*(s+1);
        IplImage *prev = responses[s-2];
        IplImage *curr = responses[s-1];
        IplImage *next = responses[s];
        for (int y = offset; y < H - offset; ++y) {
            for (int x = offset; x < W - offset; ++x) {
                T response = CV_IMAGE_ELEM(curr, T, y, x);
                // Compare orthogonal points first to try to get a quick rejection
                if ( compare(response, CV_IMAGE_ELEM(curr, T, y, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y-1, x)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y+1, x)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y, x)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y, x)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y-1, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y-1, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y+1, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(curr, T, y+1, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y-1, x)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y+1, x)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y-1, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y-1, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y+1, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(prev, T, y+1, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y-1, x)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y+1, x)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y-1, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y-1, x+1)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y+1, x-1)) ||
                     compare(response, CV_IMAGE_ELEM(next, T, y+1, x+1))
                    )
                    continue;
                pts.push_back(Keypoint(x, y, s, response));
                ++num_pts;
            }
        }
    }

    return num_pts;
}

#endif
