#ifndef FEATURES_TEST_KEYPOINT_UTILS_H
#define FEATURES_TEST_KEYPOINT_UTILS_H

#include "star_detector/keypoint.h"
#include <cmath>
#include <algorithm>

//! Keypoint which stores x, y, scale as floats so we can deal with
//! other detectors (SIFT, SURF, etc.).
struct KeypointFl
{
  float x;
  float y;
  float scale;
  float response;

  KeypointFl()
    : x(0), y(0), scale(0), response(0)
  {}

  KeypointFl(float x, float y, float scale, float response)
    : x(x), y(y), scale(scale), response(response)
  {};

  //! This allows sorting a list of keypoints into descending order
  //! of response magnitude using std::sort.
  inline bool operator< (KeypointFl const& other) const {
    return fabs(response) > fabs(other.response);
  }
};

void WriteKeypointsFl(std::string file_name, std::vector<KeypointFl> const& pts);

std::vector<KeypointFl> ReadKeypointsFl(std::string file_name);

// Returns number of correspondences.
template< typename KeyptT >
int CountCorrespondences(std::vector<KeyptT> const& keys1,
                         std::vector<KeyptT> const& keys2, float r)
{
    typedef typename std::vector<KeyptT>::const_iterator iter;

    int correspondences = 0;
    
    for (iter i = keys1.begin(); i != keys1.end(); ++i) {
        for (iter j = keys2.begin(); j != keys2.end(); ++j) {
            if (fabs(i->x - j->x) + fabs(i->y - j->y) + fabs(i->scale - j->scale) <= r) {
                ++correspondences;
                break;
            }
        }
    }

    return correspondences;
}

//! Calculate area of the circular segment defined by a chord with
//! distance d from the center of a circle with radius r.
inline float CircularSegmentArea(float r, float d)
{
    return r*r*acos(d/r) - d*sqrt(r*r - d*d);
}

float OverlapError(float r1, float r2, float d);

// Returns number of correspondences using overlap test.
template< typename KeyptT >
int OverlapCorrespondences(std::vector<KeyptT> const& keys1,
                           std::vector<KeyptT> const& keys2,
                           float threshold = 0.4,
                           float scale_factor = 10,
                           bool normalize_size = true)
{
    typedef typename std::vector<KeyptT>::const_iterator iter;

    int correspondences = 0;

    for (iter i = keys1.begin(); i != keys1.end(); ++i) {
        float r1, scale_multiplier;
        if (normalize_size) {
            r1 = scale_factor;
            scale_multiplier = scale_factor / i->scale;
        } else {
            r1 = scale_factor * i->scale;
            scale_multiplier = scale_factor;
        }

        for (iter j = keys2.begin(); j != keys2.end(); ++j) {
            float r2 = scale_multiplier * j->scale;
            float d = hypot(i->x - j->x, i->y - j->y);
            if (OverlapError(r1, r2, d) <= threshold) {
                ++correspondences;
                break;
            }
        }
    }

    return correspondences;
}

struct ScaleLowerBound
{
    float lb;

    ScaleLowerBound(float lb) : lb(lb) {}

    template< typename KeyptT >
    inline bool operator() (KeyptT const& pt) {
        return pt.scale < lb;
    }
};

struct ScaleUpperBound
{
    float ub;

    ScaleUpperBound(float ub) : ub(ub) {}

    template< typename KeyptT >
    inline bool operator() (KeyptT const& pt) {
        return pt.scale > ub;
    }
};

// Functor to reject points that exceed the line response threshold.
/*
struct LineThreshold
{
    float line_threshold;
    
    LineThreshold(float thresh = 10)
        : line_threshold(thresh)
    {};
    
    inline bool operator() (Keypoint const& pt) {
        return pt.line_response >= line_threshold;
    }
};
*/
#endif
