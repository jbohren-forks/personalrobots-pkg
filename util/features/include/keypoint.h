#ifndef FEATURES_KEYPOINT_H
#define FEATURES_KEYPOINT_H

#include <cmath>
#include <vector>
#include <string>

/*!
  A struct representing a keypoint. The scale and pixel coordinates are
  assumed to be integers.
 */
struct Keypoint
{
    int x;
    int y;
    int scale;
    float response;
    // TODO: sign on dark or bright

    Keypoint()
        : x(0), y(0), scale(0), response(0)
    {}

    Keypoint(int x, int y, int scale, float response)
        : x(x), y(y), scale(scale), response(response)
    {};

    //! Allow sorting a list of keypoints into descending order
    //! of response magnitude using std::sort.
    inline bool operator< (Keypoint const& other) const {
        return fabs(response) > fabs(other.response);
    }
};

//! Writes a set of keypoints to a file.
void WriteKeypoints(std::string file_name, std::vector<Keypoint> const& pts);

//! Reads a set of keypoints from a file.
std::vector<Keypoint> ReadKeypoints(std::string file_name);

#endif
