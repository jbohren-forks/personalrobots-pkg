/*
 * PointTracks.h
 *
 *  Created on: Nov 5, 2008
 *      Author: jdchen
 */

#ifndef POINTTRACKS_H_
#define POINTTRACKS_H_

#include <opencv/cxcore.h>
#include <deque>
#include <vector>
using namespace std;

class PointTrackObserv {
public:
  PointTrackObserv(const int fi, const CvPoint3D64f& coord, const int keypointIndex):
    mFrameIndex(fi), mDispCoord(coord), mKeypointIndex(keypointIndex){}
  /// the index of the image frame;
  int           mFrameIndex;
  /// disparity coordinates of the point in this frame
  CvPoint3D64f  mDispCoord;
  /// the index of this point in the keypoint list of the frame
  int           mKeypointIndex;
};
/// A sequence of observations of a 3D feature over a sequence of
/// video images.
class PointTrack: public deque<PointTrackObserv> {
public:
  typedef deque<PointTrackObserv> Parent;
  PointTrack(const PointTrackObserv& obsv0, const PointTrackObserv& obsv1,
      const CvPoint3D64f& coord, int frameIndex, int trackId):
        mCoordinates(coord), mId(trackId)
        {
    push_back(obsv0);
    push_back(obsv1);
        }
  inline void extend(const PointTrackObserv& obsv){
    push_back(obsv);
  }
  /// purge all observation that is older than oldestFrameIndex
  void purge(int oldestFrameIndex) {
    while (front().mFrameIndex < oldestFrameIndex) {
      pop_front();
    }
  }
  inline size_type size() const { return Parent::size();}
  /// Index of the frame within the slide window with the lowest index in which
  /// this track is detected.
  inline int firstFrameIndex() const { return front().mFrameIndex;}
  /// Index of the frame within the slide window with the highest index in which
  /// this track is detected.
  inline int lastFrameIndex()  const { return back().mFrameIndex; }
  void print() const;

  /// estimated 3D Cartesian coordinates.
  CvPoint3D64f     mCoordinates;
  /// for debugging analysis
  int              mId;
protected:
};

  /// Book keeping of point tracks.
class PointTracks {
public:
  PointTracks():mCurrentFrameIndex(0){}
  PointTracks(PointTrack& track, int frameIndex):mCurrentFrameIndex(frameIndex){
    mTracks.push_back(track);
  }
  /// purge the tracks for tracks and track observations
  /// that are older than oldestFrameIndex
  void purge(int oldestFrameIndex);
  void print() const;
  /// collection stats of the tracks
  void stats(int *numTracks, int *maxLen, int* minLen, double *avgLen,
      /// histogram of the length of the tracks
      vector<int>* lenHisto) const;
  /// a container for all the tracks
  deque<PointTrack> mTracks;
  /// The index of the last frame that tracks have been
  /// constructed against.
  int mCurrentFrameIndex;
};

#endif /* POINTTRACKS_H_ */
