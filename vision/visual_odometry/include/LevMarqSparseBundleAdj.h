/*
 * LevMarqSparseBundleAdj.h
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#ifndef LEVMARQBUNDLEADJ_H_
#define LEVMARQBUNDLEADJ_H_

#include "LevMarqTransformDispSpace.h"
#include "PointTracks.h"
// #include "LevMarqPartitioned.h"

class Foo {};

namespace cv { namespace willow {

class Foo2 {};

/// A special Levenberg-Marquardt for bundle adjustment in visual odometry
class LevMarqSparseBundleAdj: public LevMarqTransformDispSpace {
public:
  typedef LevMarqTransformDispSpace Parent;
  LevMarqSparseBundleAdj(
      /// transformation matrix from disparity space to Cartesian space.
      const CvMat *disparityTo3D,
      /// transformation matrix from Cartesian space to disparity space.
      const CvMat *threeDToDisparity,
      /// number of max iterations in optimization.
      int numMaxInter = defNumMaxIter);
  virtual ~LevMarqSparseBundleAdj();
  /// Bundle-adjustment of a set of frames and tracks of points.
  /// The parameters are used as input as well as output.
  bool optimize(
      /// The window of frames. For a free frame, the transformation matrix
      /// is used as initial value in entry and output in exit.
      deque<PoseEstFrameEntry *> windowOfFrames,
      /// The tracks of points. The global coordinates for each track are
      /// used as initial value in entry and output in exit.
      PointTracks& tracks
  );
protected:
  static const unsigned int Dim = 3;
  // LevMarqPartitioned levMarqPartitioned;
};

}

}

#endif /* LEVMARQBUNDLEADJ_H_ */
