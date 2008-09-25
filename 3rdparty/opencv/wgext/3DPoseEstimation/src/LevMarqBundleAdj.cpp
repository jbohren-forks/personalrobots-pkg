/*
 * LevMarqBundleAdj.cpp
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#include "LevMarqBundleAdj.h"

namespace cv {

namespace willow {

LevMarqBundleAdj::LevMarqBundleAdj(const CvMat *disparityTo3D, const CvMat *threeDToDisparity,
    int numErrors, int numMaxIter) :
      Parent(disparityTo3D, threeDToDisparity, numErrors, numMaxIter)
      {
  // TODO Auto-generated constructor stub

}

LevMarqBundleAdj::~LevMarqBundleAdj() {
  // TODO Auto-generated destructor stub
}

}

}
