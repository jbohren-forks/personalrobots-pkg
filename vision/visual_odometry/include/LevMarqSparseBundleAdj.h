/*
 * LevMarqSparseBundleAdj.h
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#ifndef LEVMARQBUNDLEADJ_H_
#define LEVMARQBUNDLEADJ_H_

#include "LevMarqTransformDispSpace.h"
#include "LevMarqPartitioned.h"

#include "PathRecon.h"
#include "VOSparseBundleAdj.h"

namespace cv { namespace willow {

/// A special Levenberg-Marquardt for bundle adjustment in visual odometry
class LevMarqSparseBundleAdj: public LevMarqTransformDispSpace {
public:
  typedef LevMarqTransformDispSpace Parent;
  LevMarqSparseBundleAdj(const CvMat *disparityTo3D, const CvMat *threeDToDisparity,
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
      VOSparseBundleAdj::Tracks& traks
  );
protected:
  static const unsigned int Dim = 3;
  LevMarqPartitioned levMarqPartitioned;
};

}

}

#endif /* LEVMARQBUNDLEADJ_H_ */
