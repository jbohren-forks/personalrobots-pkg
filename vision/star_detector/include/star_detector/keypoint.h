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
  int s;
  // scale may differ from s if scale interpolation is used
  float scale;
  // sign of response indicates dark/bright feature
  float response;
  // TODO: re-remove this
  float line_response;

  Keypoint()
    : x(0), y(0), s(0), scale(0), response(0), line_response(0)
  {}

  Keypoint(int x, int y, float scale, float response, int s = 0, float line_response = 0)
    : x(x), y(y), s(s), scale(scale), response(response), line_response(line_response)
  {};

  //! Allow sorting a list of keypoints into descending order
  //! of response magnitude using std::sort.
  inline bool operator< (Keypoint const& other) const {
    return fabs(response) > fabs(other.response);
  }
};

//! Keep only the best N keypoints.
inline void KeepBestPoints(std::vector<Keypoint> &pts, int N)
{
  if ((int)pts.size() > N) {
    std::vector<Keypoint>::iterator nth = pts.begin() + N;
    std::nth_element(pts.begin(), nth, pts.end());
    pts.erase(nth, pts.end());
  }
}

//! Predicate for partitioning a set of keypoints into dark and bright.
struct DarkOrBright
{
  inline bool operator() (Keypoint const& pt) {
    return pt.response > 0;
  }
};

//! Writes a set of keypoints to a file.
void WriteKeypoints(std::string file_name, std::vector<Keypoint> const& pts);

//! Reads a set of keypoints from a file.
std::vector<Keypoint> ReadKeypoints(std::string file_name);

#endif
