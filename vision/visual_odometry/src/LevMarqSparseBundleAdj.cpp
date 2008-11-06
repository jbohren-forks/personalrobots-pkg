/*
 * LevMarqSparseBundleAdj.cpp
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#include "LevMarqSparseBundleAdj.h"
#include "PointTracks.h"
#include "boost/foreach.hpp"

namespace cv {
namespace willow {

LevMarqSparseBundleAdj::LevMarqSparseBundleAdj(const CvMat *disparityTo3D, const CvMat *threeDToDisparity,
    int numMaxIter) :
      Parent(disparityTo3D, threeDToDisparity, numMaxIter)
      {
  // TODO: set up partitioned LevMarq
  /// - set up the partitioned Levenberg-Marquardt
}

LevMarqSparseBundleAdj::~LevMarqSparseBundleAdj() {
}

bool LevMarqSparseBundleAdj::optimize(
    deque<PoseEstFrameEntry *> windowOfFrames,
    PointTracks& tracks
) {
  /// Loop thru each track p
  BOOST_FOREACH( PointTrack& p, tracks.mTracks) {
    /// - Compute the part of JtJ w.r.t to p
    double Hpp[Dim*Dim]; // the part of JtJ w.r.t. track p (or point p)
    double bp[Dim];      // the part of bP  w.r.t. track p
    /// - Compute derivatives.
    /// -- Loop through all frames of this track

    /// Augment diagonal of Hpp.

    /// Invert Hpp. Note that Hpp is symmetric. And we do not need
    /// Hpp anymore.

    /// - Outer product of track. Loop thru each free frame (camera)
    /// of this track

  }

  ///
  return true;
}

}

}
