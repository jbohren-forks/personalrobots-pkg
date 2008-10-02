/*
 * LevMarqBundleAdj.cpp
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#include "LevMarqBundleAdj.h"
#include "PathRecon.h"
#include "VisOdomBundleAdj.h"
#include "boost/foreach.hpp"

namespace cv {
namespace willow {

LevMarqBundleAdj::LevMarqBundleAdj(const CvMat *disparityTo3D, const CvMat *threeDToDisparity,
    int numErrors, int numMaxIter) :
      Parent(disparityTo3D, threeDToDisparity, numErrors, numMaxIter)
      {
  // TODO: set up partitioned LevMarq
  /// - set up the partitioned Levenberg-Marquardt
}

LevMarqBundleAdj::~LevMarqBundleAdj() {
}

bool LevMarqBundleAdj::optimize(
    deque<PoseEstFrameEntry *> windowOfFrames,
    VisOdomBundleAdj::Tracks& tracks
) {
  /// Loop thru each track p
  BOOST_FOREACH( VisOdomBundleAdj::Track& p, tracks.mTracks) {
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
