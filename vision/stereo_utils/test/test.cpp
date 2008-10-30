#include "CvTest3DPoseEstimate.h"

#include <iostream>
#include <vector>
#include <queue>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
// the google wrapper
#include <opencv/cvwimage.h>

#include <opencv/cxcore.hpp>

#include <CvStereoCamModel.h>

using namespace cv::willow;

using namespace std;


#define DEBUG 1
#define DISPLAY 1

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


void testDeltaStereo() {

  CvStereoCamModel camModel;
  double du = 3;
  double disp = 5;
  double z = 1500;

  double dx = camModel.getDeltaX(du, disp);

  double du0 = camModel.getDeltaU(dx, z);

  double xyzs[30];
  double uvds[30];
  CvMat xyzs_ = cvMat(10, 3, CV_64FC1, xyzs);
  CvMat uvds_ = cvMat(10, 3, CV_64FC1, uvds);

  for (int i=0; i<10; i++) {
    xyzs[i*3]   = 110*i+4;
    xyzs[i*3+1] = 1500.;
    xyzs[i*3+2] = z;

    if (i>0) {
      double du0 = camModel.getDeltaU(xyzs[i*3] - xyzs[(i-1)*3], z);
      printf( "delta u0 %d = %f\n", i, du0);
    }
  }

  camModel.cartToDisp(xyzs_, uvds_);

  for (int i=1; i<10; i++) {
    printf("delta u1 %d = %f\n", i, uvds[i*3] - uvds[(i-1)*3]);
  }

  for (int i=0; i<10; i++) {
    xyzs[i*3] = 1500.;
    xyzs[i*3+1]   = 110*i+4;
    xyzs[i*3+2] = z;

    if (i>0) {
      double dv0 = camModel.getDeltaV(xyzs[i*3+1] - xyzs[(i-1)*3+1], z);
      printf( "delta v0 %d = %f\n", i, dv0);
    }
  }

  camModel.cartToDisp(xyzs_, uvds_);

  for (int i=1; i<10; i++) {
    printf("delta v1 %d = %f\n", i, uvds[i*3+1] - uvds[(i-1)*3+1]);
  }

  for (int i=0; i<10; i++) {
    xyzs[i*3] = 1500.;
    xyzs[i*3+1]   = 110*i+4;
    xyzs[i*3+2] = 150*i+3;

    double d = camModel.getDisparity(xyzs[i*3+2]);
    printf("disparity 1 %d = %f\n", i, d);
  }

  camModel.cartToDisp(xyzs_, uvds_);

  for (int i=0; i<10; i++) {
    printf("disparity 2 %d = %f\n", i, uvds[i*3+2]);
  }

}

int main() {
  testDeltaStereo();
}
