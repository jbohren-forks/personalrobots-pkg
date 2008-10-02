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
  int& frameIndex = frameSeq.mCurrentFrameIndex;
  bool insertNewKeyFrame = false;
  TIMERSTART(Total);
  PoseEstFrameEntry *& currFrame = frameSeq.mCurrentFrame;

  if (frameSeq.mNextFrame.get()){
    // do not need to load new images nor compute the key points again
    currFrame = frameSeq.mNextFrame.release();
  } else {
    // load and process next stereo pair of images.
//    loadAndProcessStereoFrame(frameIndex, currFrame);
    currFrame = new PoseEstFrameEntry(frameIndex);
    bool status =
      loadStereoFrame(tracker, frameIndex, currFrame->mImage, currFrame->mDispMap);

    if (status) {
      const WImage1_b*   image = currFrame->mImage;
      const WImage1_16s* dispImage = currFrame->mDispMap;
      status = goodFeaturesToTrack(tracker, *image, dispImage, currFrame->mKeypoints);
    } else {
      std::cerr << "failed on loading frame: "<<frameIndex<<std::endl;
    }

//    if (mVisualizer) mVisualizer->drawDispMap(*currFrame);
  }

  if (currFrame->mFrameIndex == frameSeq.mStartFrameIndex) {
    // First frame ever, do not need to do anything more.
    setLastKeyFrame(tracker, currFrame);
    insertNewKeyFrame = true;
    currFrame = NULL;
    return insertNewKeyFrame;
  }
  //
  // match the good feature points between this iteration and last key frame
  //
  vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;
  if (currFrame->mTrackableIndexPairs == NULL) {
    currFrame->mTrackableIndexPairs = new vector<pair<int, int> >();
  } else {
    currFrame->mTrackableIndexPairs->clear();
  }
  matchKeypoints(tracker, &trackablePairs, currFrame->mTrackableIndexPairs);
  assert(currFrame->mTrackableIndexPairs->size() == trackablePairs.size());
#ifdef DEBUG
  cout << "Num of trackable pairs for pose estimate: "<<trackablePairs.size() << endl;
#endif

  // if applicable, pass a reference of the current frame for visualization
#if 0
  if (mVisualizer) {
    mVisualizer->drawKeypoints(*getLastKeyFrame(), *currFrame, trackablePairs);
  }
#endif

  if (currFrame->mNumTrackablePairs< defMinNumTrackablePairs) {
#ifdef DEBUG
    cout << "Too few trackable pairs" <<endl;
#endif
  } else {

    //  pose estimation given the feature point pairs
    //  note we do not do Levenberg-Marquardt here, as we are not sure if
    //  this is key frame yet.
    TIMERSTART2(PoseEstimateRANSAC);
    PoseEstimator* pe = getPoseEstimator(tracker);
    PoseEstFrameEntry* lastKeyFrame = getLastKeyFrame(tracker);
    currFrame->mNumInliers =
        poseEstimate(pe, *currFrame->mKeypoints, *lastKeyFrame->mKeypoints,
            *currFrame->mTrackableIndexPairs,
            currFrame->mRot, currFrame->mShift, false);
    TIMEREND2(PoseEstimateRANSAC);

//    mStat.mHistoInliers.push_back(currFrame->mNumInliers);

    currFrame->mInliers0 = NULL;
    currFrame->mInliers1 = NULL;
    fetchInliers(tracker, currFrame->mInliers1, currFrame->mInliers0);
    currFrame->mInlierIndices = fetchInliers(tracker);
    currFrame->mLastKeyFrameIndex = getLastKeyFrame(tracker)->mFrameIndex;
#ifdef DEBUG
    cout << "num of inliers: "<< currFrame->mNumInliers <<endl;
#endif
    assert(getLastKeyFrame(tracker));
//    if (mVisualizer) mVisualizer->drawTracking(*getLastKeyFrame(), *currFrame);

    // Decide if we need select a key frame by now
    KeyFramingDecision kfd =
      keyFrameEval(tracker, frameIndex, currFrame->mKeypoints->size(),
          currFrame->mNumInliers,
          currFrame->mRot, currFrame->mShift);

    // key frame action
    insertNewKeyFrame = keyFrameAction(tracker, kfd, frameSeq);
  }
  TIMEREND(Total);
  return insertNewKeyFrame;
}
}
}
