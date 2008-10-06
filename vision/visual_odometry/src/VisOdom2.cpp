/*
 * VisOdom2.cpp
 *
 *  Created on: Oct 1, 2008
 *      Author: jdchen
 */

#include "VisOdom.h"
#include "CvTestTimer.h"

// Please note that because the timing code is executed is called lots of lots of times
// they themselves have taken substantial timing as well
#define CHECKTIMING 1

#if CHECKTIMING == 0
#define TIMERSTART(x)
#define TIMEREND(x)
#define TIMERSTART2(x)
#define TIMEREND2(x)
#else
#define TIMERSTART(x)  CvTestTimerStart(x)
#define TIMEREND(x)    CvTestTimerEnd(x)
#define TIMERSTART2(x) CvTestTimerStart2(x)
#define TIMEREND2(x)   CvTestTimerEnd2(x)
#endif


namespace cv { namespace willow {

bool trackOneFrame(CamTracker* tracker, FrameSeq& frameSeq) {
  return true;
}
}
}
