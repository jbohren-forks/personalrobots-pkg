/*
 * CvVisOdemBundleAdj.cpp
 *
 *  Created on: Sep 17, 2008
 *      Author: jdchen
 */

#include "CvVisOdomBundleAdj.h"
#include "CvMatUtils.h"
#include "CvTestTimer.h"

#define DISPLAY 1

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
  mStat.mErrMeas.setCameraParams((const CvStereoCamParams& )(mPoseEstimator));

#if DISPLAY
  // Optionally, set up the visualizer
  mVisualizer = new CvVisOdomBundleAdj::Visualizer(mPoseEstimator);
#endif

  // current frame
  PoseEstFrameEntry*& currFrame = mCurrentFrame;
  delete currFrame;
  currFrame = NULL;
  // last good frame as candidate for next key frame
  delete mLastGoodFrame;
  mLastGoodFrame = NULL;

  for (setStartFrame(); notDoneWithIteration(); setNextFrame()){
    bool newKeyFrame = reconOneFrame();
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

    // construct the tracks
    status = updateTracks(mActiveKeyFrames, mTracks);

    if (mVisualizer) {
      mVisualizer->draw(*mCurrentFrame, *getLastKeyFrame());
      mVisualizer->show();
      mVisualizer->save();
    }
  }

  saveFramePoses(mOutputDir);

  mStat.mNumKeyPointsWithNoDisparity = mPoseEstimator.mNumKeyPointsWithNoDisparity;
  mStat.mPathLength = mPathLength;
  mStat.print();

  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters    = mNumFrames/mFrameStep;
  timer.printStat();

  return status;
}

bool CvVisOdomBundleAdj::updateTracks(deque<PoseEstFrameEntry*> & frames, Tracks & tracks)
{
  bool status = true;
  int lastFrame = tracks.mCurrentFrameIndex;
  int firstFrameInWin = frames.front()->mFrameIndex;
  BOOST_FOREACH( PoseEstFrameEntry* frame, frames) {
    assert(frame != NULL);
    if (frame->mFrameIndex <= lastFrame) {
      continue;
    }
    // loop thru all the inlier pairs
    for (int i=0; i<frame->mNumInliers; i++) {
      // a pair of inliers will end up being either an extension to
      // existing track, or the start of a new track.

      // loop thru all the existing tracks to
      // - extending old tracks,
      // - adding new tracks, and
      // - remove old tracks that will not be used anymore
      if (extendTrack(tracks, *frame, i) == false) {
        // no track extended. Add the new pair as a new track
        addTrack(tracks, *frame, i);
      }
    }
  }
  return status;
}

bool CvVisOdomBundleAdj::extendTrack(Tracks& tracks, PoseEstFrameEntry& frame,
    int inlierIndex){
  bool status = false;
  int inlier = frame.mInlierIndices[inlierIndex];
  std::pair<int, int>& p = frame.mTrackableIndexPairs->at(inlier);
  BOOST_FOREACH( Track& track, tracks.mTracks ) {
    if (track.mLastFrame != frame.mLastKeyFrameIndex) {
      // The last frame of this track is not the same as
      // the last frame of this inlier pair. Skip it.
      continue;
    }
    TrackObserv& observ = track.mObservs.back();
    if (observ.mKeypointIndex != p.first) {
      // keypoint index does not match
      // skip to next track
      continue;
    } else {
      // found a matching track. Extend it.
      CvPoint3D64f coord = CvMatUtils::rowToPoint(*frame.mInliers1, inlierIndex);
      TrackObserv obsv(frame.mFrameIndex, coord, p.second);
      track.mObservs.push_back(obsv);
      status = true;
      break;
    }
  }
  return status;
}
bool CvVisOdomBundleAdj::addTrack(Tracks& tracks, PoseEstFrameEntry& frame,
    int inlierIndex){
  bool status = false;
  int inlier = frame.mInlierIndices[inlierIndex];
  std::pair<int, int>& p = frame.mTrackableIndexPairs->at(inlier);
  CvPoint3D64f dispCoord0 = CvMatUtils::rowToPoint(*frame.mInliers0, inlierIndex);
  CvPoint3D64f dispCoord1 = CvMatUtils::rowToPoint(*frame.mInliers1, inlierIndex);
  TrackObserv obsv0(frame.mLastKeyFrameIndex, dispCoord0, p.first);
  TrackObserv obsv1(frame.mFrameIndex, dispCoord1, p.second);
  // initial estimate of the position of the 3d point in Cartesian coord.
  CvMat disp1;
  cvGetRow(frame.mInliers1, &disp1, inlierIndex);
  CvPoint3D64f cartCoord1; //< Estimated global Cartesian coordinate.
  CvMat _cartCoord1 = cvMat(1, 3, CV_64FC1, &cartCoord1);
  dispToGlobal(disp1, frame.mGlobalTransform, _cartCoord1);
  Track newtrack(obsv0, obsv1, cartCoord1, frame.mFrameIndex);
  tracks.mTracks.push_back(newtrack);

  return status;
}

void CvVisOdomBundleAdj::Visualizer::draw(
    const PoseEstFrameEntry& frame,
    const PoseEstFrameEntry& lastFrame) {
  Parent::draw(frame, lastFrame);
}

