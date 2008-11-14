/*
 * PointTracks.h
 *
 *  Created on: Nov 5, 2008
 *      Author: jdchen
 */

#ifndef POINTTRACKS_H_
#define POINTTRACKS_H_

#include <opencv/cxcore.h>
#include <list>
#include <deque>
#include <vector>
using namespace std;

class PointTrackObserv {
public:
  PointTrackObserv(const int fi, const CvPoint3D64f& coord, const int keypointIndex):
    frame_index_(fi),
    disp_coord_(coord), disp_coord_est_(coord),
    keypoint_index_(keypointIndex),
    local_frame_index_(-1),
    mat_Hpc_(cvMat(3, 6, CV_64FC1, Hpc_)),
    mat_Tcp_(cvMat(6, 3, CV_64FC1, Tcp_))
    {}
  /// the index of the image frame;
  int           frame_index_;
  /// observed disparity coordinates of the point in this frame
  CvPoint3D64f  disp_coord_;
  /// re-projected disparity coordinate of the point from global estimation,
  /// initially set the same as disp_coord_
  CvPoint3D64f  disp_coord_est_;
  /// the index of this point in the keypoint list of the frame
  int           keypoint_index_;

  /// "local" index of the frame in each the free window or the fixed window.
  int           local_frame_index_;
  /// Hessian between point parameter vector p and camera parameter vector c
  /// usually approximated as /f$ J_p^T J_c /f$
  double        Hpc_[3*6];
  CvMat         mat_Hpc_;
  double        Tcp_[6*3];
  CvMat         mat_Tcp_;
};
/// A sequence of observations of a 3D feature over a sequence of
/// video images.
class PointTrack: public deque<PointTrackObserv> {
public:
  typedef deque<PointTrackObserv> Parent;
  PointTrack(const PointTrackObserv& obsv0, const PointTrackObserv& obsv1,
      const CvPoint3D64f& coord, int frameIndex, int trackId):
        coordinates_(coord), id_(trackId),
        mat_tp_(cvMat(3, 1, CV_64FC1, tp_)),
        mat_dp_(cvMat(3, 1, CV_64FC1, dp_))  {
      push_back(obsv0);
      push_back(obsv1);
  }
  inline void extend(const PointTrackObserv& obsv){
    push_back(obsv);
  }
  /// purge all observation that is older than oldestFrameIndex
  void purge(int oldestFrameIndex) {
    while (front().frame_index_ < oldestFrameIndex) {
      pop_front();
    }
  }
  inline size_type size() const { return Parent::size();}
  /// Index of the frame with the lowest index in which
  /// this track is detected.
  inline int firstFrameIndex() const { return front().frame_index_;}
  /// Index of the frame with the highest index in which
  /// this track is detected.
  inline int lastFrameIndex()  const { return back().frame_index_; }
  void print() const;

  /// estimated 3D Cartesian coordinates.
  CvPoint3D64f     coordinates_;
  /// for debugging analysis
  int              id_;

  /// a buffer to store \f$ H_{pp}^{-1} b_{p} \f$
  double        tp_[3];
  CvMat         mat_tp_;
  /// updates on the point
  double        dp_[3];
  CvMat         mat_dp_;
  /// estimated 3D Cartesian coordinates of previous iteration in optimization.
  CvPoint3D64f  prev_coordinates_;
};

/// Book keeping of point tracks.
class PointTracks {
public:
  PointTracks():current_frame_index_(0){}
  PointTracks(PointTrack& track, int frameIndex):
    current_frame_index_(frameIndex){
    tracks_.push_back(track);
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
  list<PointTrack> tracks_;
  /// The index of the last frame that tracks have been
  /// constructed against.
  int current_frame_index_;
  /// The index of the oldest frame (lowest index) of all tracks.
  int oldest_frame_index_in_tracks_;
};

#endif /* POINTTRACKS_H_ */
