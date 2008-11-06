/*
 * VOSparseBundleAdj.cpp
 *
 *  Created on: Sep 17, 2008
 *      Author: jdchen
 */

#include "VOSparseBundleAdj.h"
#include "CvMatUtils.h"
#include "CvTestTimer.h"
#include "LevMarqSparseBundleAdj.h"

using namespace cv::willow;

// boost
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
using namespace boost::accumulators;

// opencv
#include <opencv/cv.h>

#include <queue>

#define DISPLAY 1

VOSparseBundleAdj::VOSparseBundleAdj(const CvSize& imageSize):
  PathRecon(imageSize),
  mSlideWindowSize(DefaultSlideWindowSize),
  mNumFrozenWindows(1),
  mNumIteration(DefaultNumIteration),
  mTrackId(0)
{

}

VOSparseBundleAdj::~VOSparseBundleAdj() {
  // TODO Auto-generated destructor stub
}

void VOSparseBundleAdj::updateSlideWindow() {
  while(mActiveKeyFrames.size() > (unsigned int)this->mSlideWindowSize) {
    PoseEstFrameEntry* frame = mActiveKeyFrames.front();
    mActiveKeyFrames.pop_front();
    delete frame;
  }
  // Since slide down the window, we shall purge the tracks as well
  if (mVisualizer) {
    ((SBAVisualizer*)mVisualizer)->slideWindowFront =
      mActiveKeyFrames.front()->mFrameIndex;
  }
  // loop thru all tracks and get purge the old entries
  purgeTracks(mActiveKeyFrames.front()->mFrameIndex);
#if DEBUG==1
  cout << "Current slice window: ["<<mActiveKeyFrames.front()->mFrameIndex<<","
  <<mActiveKeyFrames.back()->mFrameIndex << "]"<<endl;
#endif

  int numWinSize = mActiveKeyFrames.size();
  assert(numWinSize<=this->mSlideWindowSize);
  int numFixedFrames = std::min(this->mNumFrozenWindows, numWinSize-mNumFrozenWindows);
  numFixedFrames = std::max(1, numFixedFrames);
#if DEBUG==1
  cout << "window size: "<< numWinSize << " # fixed: " << numFixedFrames << endl;
#endif

}

bool VOSparseBundleAdj::track(queue<StereoFrame>& inputImageQueue) {
  bool status = false;

  while (inputImageQueue.size()>0) {
    bool newKeyFrame = trackOneFrame(inputImageQueue, mFrameSeq);
    if (newKeyFrame == true) {
      // update the sliding window
      // slide the window by getting rid of old windows
      updateSlideWindow();

      // update the tracks
      status = updateTracks(mActiveKeyFrames, mTracks);



      cout << mActiveKeyFrames.back()->mFrameIndex << endl;
      updateStat2();

      cout << mActiveKeyFrames.back()->mFrameIndex << endl;

      if (mVisualizer) {
        PoseEstFrameEntry* pee = mActiveKeyFrames.back();
        PoseEstFrameEntry& peef = *pee;
        cout << pee->mFrameIndex << endl;
        SBAVisualizer* vis = (SBAVisualizer*)mVisualizer;
        vis->drawTrack(peef);
      }
    }
    visualize();

  }
  return status;
}

void VOSparseBundleAdj::purgeTracks(int oldtestFrameIndex){
  mTracks.purge(oldtestFrameIndex);
}

bool VOSparseBundleAdj::updateTracks(deque<PoseEstFrameEntry*> & frames,
    PointTracks & tracks)
{
  bool status = true;
  int lastFrame = tracks.mCurrentFrameIndex;

  // loop thru all the frames in the current sliding window
  BOOST_FOREACH( PoseEstFrameEntry* frame, frames) {
    assert(frame != NULL);
    // skip over the frames that are not newer than the last frame
    // the tracks have been updated against.
    if (frame->mFrameIndex <= lastFrame) {
      continue;
    }
    // loop thru all the inlier pairs of this frame
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
  if (frames.size()>0)
    tracks.mCurrentFrameIndex = frames.back()->mFrameIndex;

  return status;
}

bool VOSparseBundleAdj::extendTrack(PointTracks& tracks, PoseEstFrameEntry& frame,
    int inlierIndex){
  bool status = false;
  int inlier = frame.mInlierIndices[inlierIndex];
  std::pair<int, int>& p = frame.mTrackableIndexPairs->at(inlier);
  BOOST_FOREACH( PointTrack& track, tracks.mTracks ) {
    if (// The last frame of this track is not the same as
        // the last frame of this inlier pair. Skip it.
        track.lastFrameIndex() == frame.mLastKeyFrameIndex ) {
      PointTrackObserv& observ = track.back();
      if (// keypoint index needs to match skip to next track
          observ.mKeypointIndex == p.second) {
        // found a matching track. Extend it.
        CvPoint3D64f coord1 = CvMatUtils::rowToPoint(*frame.mInliers1, inlierIndex);
        CvPoint3D64f coord0 = CvMatUtils::rowToPoint(*frame.mInliers0, inlierIndex);
        PointTrackObserv obsv(frame.mFrameIndex, coord1, p.first);

        track.extend(obsv);
        status = true;
#if DEBUG==1
        printf("extend track %3d => f=%3d, pi=%3d:[%3d, %3d]->f=%3d, pi=%3d:[%3d, %3d]\n",
            track.mId,
            frame.mLastKeyFrameIndex, p.second,
            (int)(coord0.x+.5), (int)(coord0.y+.5),
            frame.mFrameIndex,        p.first,
            (int)(coord1.x+.5), (int)(coord1.y+.5));
#endif
        break;
      }
    }
  }
  return status;
}
bool VOSparseBundleAdj::addTrack(PointTracks& tracks, PoseEstFrameEntry& frame,
    int inlierIndex){
  bool status = false;
  int inlier = frame.mInlierIndices[inlierIndex];
  std::pair<int, int>& p = frame.mTrackableIndexPairs->at(inlier);
  CvPoint3D64f dispCoord0 = CvMatUtils::rowToPoint(*frame.mInliers0, inlierIndex);
  CvPoint3D64f dispCoord1 = CvMatUtils::rowToPoint(*frame.mInliers1, inlierIndex);
  PointTrackObserv obsv0(frame.mLastKeyFrameIndex, dispCoord0, p.second);
  PointTrackObserv obsv1(frame.mFrameIndex,        dispCoord1, p.first);
  // initial estimate of the position of the 3d point in Cartesian coord.
  CvMat disp1;
  cvGetRow(frame.mInliers1, &disp1, inlierIndex);
  CvPoint3D64f cartCoord1; //< Estimated global Cartesian coordinate.
  CvMat _cartCoord1 = cvMat(1, 3, CV_64FC1, &cartCoord1);
  dispToGlobal(disp1, frame.mGlobalTransform, _cartCoord1);
  PointTrack newtrack(obsv0, obsv1, cartCoord1, frame.mFrameIndex, mTrackId++);
  tracks.mTracks.push_back(newtrack);

  int trackId = mTrackId-1;
#if DEBUG==1
  printf("add new track %3d => %3d, %3d:[%8.2f, %8.2f, %8.2f]\n", trackId, inlierIndex, inlier,
      cartCoord1.x, cartCoord1.y, cartCoord1.z);
  printf("f=%3d, pi=%3d: [%3d, %3d], f=%3d, pi=%3d: [%3d, %3d]\n",
      frame.mLastKeyFrameIndex, p.second,  (int)(dispCoord0.x+.5), (int)(dispCoord0.y+.5),
      frame.mFrameIndex,        p.first,   (int)(dispCoord1.x+.5), (int)(dispCoord1.y+.5));
#endif
  return status;
}

void SBAVisualizer::drawTrackingCanvas(
    const PoseEstFrameEntry& lastFrame,
    const PoseEstFrameEntry& frame
){
  Parent::drawTrackingCanvas(lastFrame, frame);

  drawTrack(frame);
}

void SBAVisualizer::drawTrack(const PoseEstFrameEntry& frame){
  drawTrackTrajectories(frame);
  drawTrackEstimatedLocations(frame);
}

void SBAVisualizer::drawTrackTrajectories(const PoseEstFrameEntry& frame) {
  // draw all the tracks on canvasTracking
  BOOST_FOREACH( const PointTrack& track, this->tracks.mTracks ){
    const PointTrackObserv& lastObsv = track.back();
    const CvScalar colorFixedFrame = CvMatUtils::blue;
    CvScalar colorFreeFrame;

    // drawing it green if the last observation is over the current frame
    // drawing it yellow otherwise.
    if (lastObsv.mFrameIndex < frame.mFrameIndex) {
      colorFreeFrame = CvMatUtils::yellow;
    } else {
      colorFreeFrame  = CvMatUtils::green;
    }

    int thickness = 1;
    int i=0;
    deque<PointTrackObserv>::const_iterator iObsv = track.begin();
    CvPoint pt0 = CvStereoCamModel::dispToLeftCam(iObsv->mDispCoord);
#if DEBUG==1
    printf("track %3d, len=%3d\n", track.mId, track.size());
    printf("%3d: [%3d, %3d]\n", i++, pt0.x, pt0.y);
#endif
    CvScalar color;
    if (iObsv->mFrameIndex < slideWindowFront) {
      color = colorFixedFrame;
    } else {
      color = colorFreeFrame;
    }
    for (iObsv++; iObsv != track.end(); iObsv++) {
      CvPoint pt1 = CvStereoCamModel::dispToLeftCam(iObsv->mDispCoord);
      cvLine(canvasTracking.Ipl(), pt0, pt1, color, thickness, CV_AA);
      // setting up for next iteration
      pt0 = pt1;
      if (iObsv->mFrameIndex < slideWindowFront) {
        color = colorFixedFrame;
      } else {
        color = colorFreeFrame;
      }
#if DEBUG==1
      printf("%3d: [%3d, %3d]\n", i++, pt0.x, pt0.y);
#endif
    }
  }
}

void SBAVisualizer::drawTrackEstimatedLocations(const PoseEstFrameEntry& frame) {
  return;
  // now draw all the observation of the same 3d points on to a copy of the
  int imgWidth  = frame.mImage->Width();
  int imgHeight = frame.mImage->Height();
  // make sure the image buffers is allocated to the right sizes
  canvasTracking.Allocate(imgWidth, imgHeight);

  assert(frame.mImage);
  cvCvtColor(frame.mImage->Ipl(), canvasTracking.Ipl(),  CV_GRAY2RGB);
  // draw all the tracks on canvasTracking

  // compute the transformation from the global frame (same as first frame)
  // to this frame.
  vector<FramePose>::const_reverse_iterator ifp = framePoses.rbegin();

  while (ifp != framePoses.rend()) {
    if (ifp->mIndex == frame.mFrameIndex) {
      // found it
      break;
    }
    ifp++;
  }
  assert(ifp != framePoses.rend());
  const CvPoint3D64f& rodCurrentToGlobal   = ifp->mRod;
  const CvPoint3D64f& shiftCurrentToGlobal = ifp->mShift;
  CvPoint3D64f rodGlobalToCurrent =  cvPoint3D64f(-rodCurrentToGlobal.x,
      -rodCurrentToGlobal.y, -rodCurrentToGlobal.z);
  CvPoint3D64f shifGlobaltoCurrent = cvPoint3D64f(-shiftCurrentToGlobal.x,
      -shiftCurrentToGlobal.y, -shiftCurrentToGlobal.z);

  BOOST_FOREACH( const PointTrack& track, this->tracks.mTracks ){
    BOOST_FOREACH( const PointTrackObserv& obsv, track ) {

    }
  }
}


void VOSparseBundleAdj::Stat2::print() {
  // The accumulator set which will calculate the properties for us:
  accumulator_set< int, stats<tag::min, tag::mean, tag::max> > acc;
  accumulator_set< int, stats<tag::min, tag::mean, tag::max> > acc2;
  accumulator_set< int, stats<tag::min, tag::mean, tag::max> > acc3;

  // Use std::for_each to accumulate the statistical properties:
  acc = std::for_each( numTracks.begin(), numTracks.end(), acc );
  printf("min numTracks = %d, ", extract::min( acc ));
  printf("max numTracks = %d, ", extract::max( acc ));
  printf("avg numTracks = %f, ", extract::mean( acc ));
  printf("\n");

  acc2 = std::for_each( minTrackLens.begin(), minTrackLens.end(), acc2 );
  printf("min mintracklen = %d, ", extract::min( acc2 ));
  printf("max mintracklen = %d, ", extract::max( acc2 ));
  printf("avg mintracklen = %f, ", extract::mean( acc2 ));
  printf("\n");
  acc3 = std::for_each( maxTrackLens.begin(), maxTrackLens.end(), acc3 );
  printf("min maxtracklen = %d, ", extract::min( acc3 ));
  printf("max maxtracklen = %d, ", extract::max( acc3 ));
  printf("avg maxtracklen = %f, ", extract::mean( acc3 ));
  printf("\n");
  accumulator_set< double, stats<tag::min, tag::mean, tag::max> > acc1;
  acc1 = std::for_each( avgTrackLens.begin(), avgTrackLens.end(), acc1 );
  printf("min avgTracklen = %f, ", extract::min( acc1 ));
  printf("max avgTracklen = %f, ", extract::max( acc1 ));
  printf("mean avgTracklen = %f, ", extract::mean( acc1 ));
  printf("\n");

  printf("Histogram of track lengths  [len, count, avg count]:\n");
  int len = 0;
  int numKeyFrames = minTrackLens.size();
  BOOST_FOREACH( const int count, trackLenHisto) {
    printf("%2d  %5d, %7.2f\n", len, count, (double)count/(double)numKeyFrames);
    len++;
  }
}

void VOSparseBundleAdj::updateStat2() {
  int numTracks, maxLen, minLen;
  double avgLen;
  vector<int> lenHisto;
  mTracks.stats(&numTracks, &maxLen, &minLen, &avgLen, &lenHisto);
  if (numTracks>0) {
    mStat2.numTracks.push_back(numTracks);
    mStat2.maxTrackLens.push_back(maxLen);
    mStat2.minTrackLens.push_back(minLen);
    mStat2.avgTrackLens.push_back(avgLen);

    if (lenHisto.size()>mStat2.trackLenHisto.size()) {
      mStat2.trackLenHisto.resize(lenHisto.size());
    }
    int len=0;
    BOOST_FOREACH( const int count, lenHisto ) {
      mStat2.trackLenHisto.at(len) += count;
      len++;
    }
  }
}

