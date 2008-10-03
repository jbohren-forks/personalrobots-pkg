/*
 * LevMarqBundleAdj.h
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#ifndef LEVMARQBUNDLEADJ_H_
#define LEVMARQBUNDLEADJ_H_

#include "LevMarqTransformDispSpace.h"
#include "LevMarqPartitioned.h"

#include "PathRecon.h"
#include "VisOdomBundleAdj.h"

namespace cv { namespace willow {

/// A special Levenberg-Marquardt for bundle adjustment in visual odometry
class LevMarqBundleAdj: public LevMarqTransformDispSpace {
public:
  typedef LevMarqTransformDispSpace Parent;
  LevMarqBundleAdj(const CvMat *disparityTo3D, const CvMat *threeDToDisparity, int numErrors,
      int numMaxInter = defNumMaxIter);
  virtual ~LevMarqBundleAdj();
  /// Bundle-adjustment of a set of frames and tracks of points.
  /// The parameters are used as input as well as output.
  bool optimize(
      /// The window of frames. For a free frame, the transformation matrix
      /// is used as initial value in entry and output in exit.
      deque<PoseEstFrameEntry *> windowOfFrames,
      /// The tracks of points. The global coordinates for each track are
      /// used as initial value in entry and output in exit.
      VisOdomBundleAdj::Tracks& traks
  );
protected:
  static const unsigned int Dim = 3;
  LevMarqPartitioned levMarqPartitioned;
};

}

}

#endif /* LEVMARQBUNDLEADJ_H_ */
