/*
 * CvVisOdemBundleAdj.cpp
 *
 *  Created on: Sep 17, 2008
 *      Author: jdchen
 */

#include "CvVisOdomBundleAdj.h"
#include "CvTestTimer.h"

CvVisOdomBundleAdj::CvVisOdomBundleAdj(const CvSize& imageSize):
  CvPathRecon(imageSize),
  mSlideWindowSize(DefaultSlideWindowSize),
  mNumFrozenWindows(DefaultNumFrozenWindows)
{
  // TODO Auto-generated constructor stub

}

CvVisOdomBundleAdj::~CvVisOdomBundleAdj() {
  // TODO Auto-generated destructor stub
}

bool CvVisOdomBundleAdj::recon(const string & dirname, const string & leftFileFmt, const string & rightFileFmt, int start, int end, int step)
{
  bool status = false;
  setInputVideoParams(dirname, leftFileFmt, rightFileFmt, start, end, step);

  int maxDisp = (int)(mPoseEstimator.getD(400));// the closest point we care is at least 1000 mm away
  cout << "Max disparity is: " << maxDisp << endl;
  mErrMeas.setCameraParams((const CvStereoCamParams& )(mPoseEstimator));

  // current frame
  PoseEstFrameEntry*& currFrame = mCurrentFrame;
  delete currFrame;
  currFrame = NULL;
  // last good frame as candidate for next key frame
  delete mLastGoodFrame;
  mLastGoodFrame = NULL;

  // next frame. Needed when backtracking happens
  PoseEstFrameEntry* nextFrame = mNextFrame;
  nextFrame = NULL;

  for (int i=mStartFrameIndex;
  i<mEndFrameIndex && mStop == false;
  i+= (nextFrame==NULL)?mFrameStep:0
  ) {
    bool newKeyFrame = reconOneFrame(i);
    if (newKeyFrame == false) {
      continue;
    }

    // slide the window by getting rid of old windows
    while(mActiveKeyFrames.size() > (unsigned int)this->mSlideWindowSize) {
      PoseEstFrameEntry* frame = mActiveKeyFrames.front();
      mActiveKeyFrames.pop_front();
      delete frame;
    }
    int numWinSize = mActiveKeyFrames.size();
    assert(numWinSize<=this->mSlideWindowSize);
    int numFixedFrames = std::min(this->mNumFrozenWindows, numWinSize-mNumFrozenWindows);
    numFixedFrames = std::max(1, numFixedFrames);
#ifdef DEBUG
    cout << "window size: "<< numWinSize << " # fixed: " << numFixedFrames << endl;
#endif
  }
  int numKeyFrames = mEndFrameIndex - mStartFrameIndex - mNumFramesSkipped;
  double scale = 1. / (double)((mEndFrameIndex - mStartFrameIndex));
  double kfScale = 1. / (double)(numKeyFrames);
  fprintf(stdout, "Num of frames skipped:    %d\n", mNumFramesSkipped);
  fprintf(stdout, "Total distance covered:   %05.2f mm\n",mPathLength);
  fprintf(stdout, "Total/Average keypoints:           %d,   %05.2f\n", mTotalKeypoints, (double)(mTotalKeypoints) * scale);
  fprintf(stdout, "Total/Average trackable pairs:     %d,   %05.2f\n", mTotalTrackablePairs, (double)(mTotalTrackablePairs) * scale);
  fprintf(stdout, "Total/Average inliers:             %d,   %05.2f\n", mTotalInliers, (double)(mTotalInliers) * scale);
  fprintf(stdout, "Total/Average keypoints:           %d,   %05.2f\n", mPoseEstimator.mNumKeyPointsWithNoDisparity, (double)(mPoseEstimator.mNumKeyPointsWithNoDisparity) * kfScale);
  fprintf(stdout, "In Key Frames:\n");
  fprintf(stdout, "Total/Average keypoints:           %d,   %05.2f\n", mTotalKeypointsInKeyFrames, (double)(mTotalKeypointsInKeyFrames) * kfScale);
  fprintf(stdout, "Total/Average trackable pairs:     %d,   %05.2f\n", mTotalTrackablePairsInKeyFrames, (double)(mTotalTrackablePairsInKeyFrames) * kfScale);
  fprintf(stdout, "Total/Average inliers:             %d,   %05.2f\n", mTotalInliersInKeyFrames, (double)(mTotalInliersInKeyFrames) *kfScale);

  saveFramePoses(string("Output/indoor1/"));

  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = mNumFrames/mFrameStep;
  timer.printStat();

  return status;
}


