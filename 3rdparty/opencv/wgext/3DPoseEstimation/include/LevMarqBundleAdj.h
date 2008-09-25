/*
 * LevMarqBundleAdj.h
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#ifndef LEVMARQBUNDLEADJ_H_
#define LEVMARQBUNDLEADJ_H_

#include "LevMarqTransformDispSpace.h"

namespace cv {namespace willow {

/// A special Levenberg Marquardt for bundle adjustment in visual odometry
class LevMarqBundleAdj: public LevMarqTransformDispSpace {
public:
  typedef LevMarqTransformDispSpace Parent;
  LevMarqBundleAdj(const CvMat *disparityTo3D, const CvMat *threeDToDisparity, int numErrors,
      int numMaxInter = defNumMaxIter);
  virtual ~LevMarqBundleAdj();
};

}

}

#endif /* LEVMARQBUNDLEADJ_H_ */
