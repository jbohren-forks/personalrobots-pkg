#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <stdlib.h>

struct Keypoint
{
    int x;
    int y;
    int scale;
    float response;
    float line_response; // TODO: this is for debugging/visualization, remove eventually
    // TODO: sign on dark or bright
    //descriptor?

    Keypoint(int x, int y, int scale, float response)
        : x(x), y(y), scale(scale), response(response)
    {};

    // This allows sorting a list of keypoints into descending order
    // of response magnitude using std::sort.
    inline bool operator< (Keypoint const& other) const {
        return fabs(response) > fabs(other.response);
    }
};



#endif
