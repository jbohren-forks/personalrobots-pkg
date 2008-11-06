/*
 * VOSparseBundleAdj.h
 *
 *  Created on: Sep 17, 2008
 *      Author: jdchen
 */

#ifndef VOSPARSEBUNDLEADJ_H_
#define VOSPARSEBUNDLEADJ_H_

#include "PathRecon.h"
#include "PointTracks.h"
#include "LevMarqSparseBundleAdj.h"
using namespace cv::willow;

namespace cv {
namespace willow {


/**
 * (Under construction) Visual Odometry by sliding window bundle adjustment.
 * The input are a sequence of stereo video images.
 */
class VOSparseBundleAdj: public PathRecon {
public:
  /// Record the observation of a tracked point w.r.t.  a frame
  typedef PathRecon Parent;
  VOSparseBundleAdj(
      /// Image size. Use for buffer allocation.
      const CvSize& imageSize);
  virtual ~VOSparseBundleAdj();

  /**
   * Given a sequence of stereo video, reconstruct the path of the
   * camera.
   */
  virtual bool track(queue<StereoFrame>& inputImageQueue);

  /// Slide down the end mark of the sliding window. Update the
  /// beginning end of applicable. Plus book keeping of the tracks
  /// and others.
  void updateSlideWindow();

  bool updateTracks(
      deque<PoseEstFrameEntry*>& frames,
      PointTracks& tracks);
  void purgeTracks(int frameIndex);

  /// Default size of the sliding window
  static const int DefaultSlideWindowSize  = 10;
  /// Default number of iteration
  static const int DefaultNumIteration = 20;

    /// Statistics for bundle adjustment
  class Stat2 {
  public:
    void print();
    vector<int> numTracks;
    vector<int> maxTrackLens;
    vector<int> minTrackLens;
    vector<double> avgTrackLens;
    vector<int> trackLenHisto;
  };
  Stat2 mStat2;
  void updateStat2();

  /// returns a reference to the tracks. No ownership passed.
  const PointTracks& getTracks() const {
    return mTracks;
  }

protected:
  /// If matched, extend an existing old track.
  /// @return true if a track is matched and extended. False otherwise.
  bool extendTrack(
      /// Reference to a collection of tracks.
      PointTracks& tracks,
      /// The new frame, with new trackable pairs.
      PoseEstFrameEntry& frame,
      /// the index of the inlier,
      int inlierIndex
  );
  bool addTrack(
      /// Reference to a collection of tracks.
      PointTracks& tracks,
      /// The new frame, with new trackable pairs.
      PoseEstFrameEntry& frame,
      /// the index of the inlier,
      int inlierIndex
  );
  PointTracks mTracks;
  /// size of the sliding window
  int mSlideWindowSize;
  /// number of frozen cameras in bundle adjustment.  Frozen (or fixed)
  /// frame (cameras) are those fall out of the sliding window, but still share
  /// tracks with frames(cameras) inside the sliding window.
  /// At the beginning when there are not enough frames (cameras) for a full
  /// slide window, we always keep the first frame frozen.
  int mNumFrozenWindows;

  /// number of iteration for bundle adjustment.
  int mNumIteration;

  /// unique id of the tracks
  int mTrackId;

  Foo foo_;
  Foo2 foo2_;

//  LevMarqSparseBundleAdj* levmarq_sba_;
};

/// Visualizing the visual odometry process of bundle adjustment.
class SBAVisualizer: public F2FVisualizer {
  public:
    typedef F2FVisualizer Parent;
    SBAVisualizer(PoseEstimateDisp& poseEstimator,
        const vector<FramePose>& framePoses,
        const PointTracks& trcks):
      Parent(poseEstimator), framePoses(framePoses), tracks(trcks){}
    virtual ~SBAVisualizer(){}
    /// Draw keypoints, tracks and disparity map on canvases for visualization
    virtual void drawTrackingCanvas(
        const PoseEstFrameEntry& lastFrame,
        const PoseEstFrameEntry& frame);

    /// Draw the tracks. A track is drawn as a polyline connecting the observations
    /// of the same point in a sequence of frames. A line is drawn in yellow
    /// if one of the end point is outside of the slide window. Red otherwise.
    virtual void drawTrack(const PoseEstFrameEntry& frame);
    /// Draw on the tracking canvas trajectories of the points of key frames.
    /// The part of a trajectory that is on the fixed frames are drawn in blue.
    /// For the part that is in the free frames,
    /// a trajectory, or track, is green if the last observation of the track is
    /// on the current frame. Yellow otherwise.
    virtual void drawTrackTrajectories(const PoseEstFrameEntry& frame);
    virtual void drawTrackEstimatedLocations(const PoseEstFrameEntry& frame);


    /// a reference to the estimated pose of the frames
    const vector<FramePose>& framePoses;
    /// a reference to the tracks.
    const PointTracks& tracks;
    int   slideWindowFront;
};


}
}
#endif /* CVVISODOMBUNDLEADJ_H_ */
