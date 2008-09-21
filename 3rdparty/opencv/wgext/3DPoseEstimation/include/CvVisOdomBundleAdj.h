/*
 * CvVisOdemBundleAdj.h
 *
 *  Created on: Sep 17, 2008
 *      Author: jdchen
 */

#ifndef CVVISODOMBUNDLEADJ_H_
#define CVVISODOMBUNDLEADJ_H_

#include "CvPathRecon.h"

/**
 * (Under construction) Visual Odometry by sliding window bundle adjustment.
 * The input are a sequence of stereo video images.
 */
class CvVisOdomBundleAdj: public CvPathRecon {
public:
  /// Record the observation of a tracked point w.r.t.  a frame
  class TrackObserv {
  public:
    TrackObserv(const int fi, const CvPoint3D64f& coord, const int keypointIndex):
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
  class Track {
  public:
    Track(const TrackObserv& obsv0, const TrackObserv& obsv1,
        const CvPoint3D64f& coord, int frameIndex):
      mCoordinates(coord), mFirstFrame(obsv0.mFrameIndex), mLastFrame(obsv1.mFrameIndex)
    {
      mObservs.push_back(obsv0);
      mObservs.push_back(obsv1);
    }
    deque<TrackObserv>  mObservs;
    /// estimated 3D Cartesian coordinates.
    CvPoint3D64f     mCoordinates;
    /// Index of the frame with the lowest index in which
    /// this track is detected.
    int mFirstFrame;
    /// Index of the frame with the highest index in which
    /// this track is detected.
    int mLastFrame;
  };
  /// A class to keep track of the tracks
  class Tracks {
  public:
    Tracks():mCurrentFrameIndex(0){}
    Tracks(Track& track, int frameIndex):mCurrentFrameIndex(frameIndex){
      mTracks.push_back(track);
    }
    /// a container for all the tracks
    deque<Track> mTracks;
    /// The index of the last frame that tracks have been
    /// constructed against.
    int mCurrentFrameIndex;
  };
  CvVisOdomBundleAdj(
      /// Image size. Use for buffer allocation.
      const CvSize& imageSize);
  virtual ~CvVisOdomBundleAdj();

  /**
   * Given a sequence of stereo video, reconstruct the path of the
   * camera.
   * dirname  - directory of where the video sequence is stored
   * leftFileFmt  - format for generating the filename of an image from the
   *                left camera. e.g. left-%04d.ppm, for filenames like
   *                left-0500.ppm, etc.
   * rightFileFmt - format for generating the filename of an image from the
   *                right camera. Same convention as leftFileFmt.
   * start        - index of the first frame to be processed
   * end          - index of the first frame not to be process
   *              - namely, process the frame at most to frame number end-1
   * step         - step size of the increase of the index from one frame
   *                to next one.
   */
  bool recon(const string& dirname, const string& leftFileFmt,
      const string& rightFileFmt, int start, int end, int step);

  bool updateTracks(
      deque<PoseEstFrameEntry*>& frames,
      Tracks& tracks);

  /// Default size of the sliding window
  static const int DefaultSlideWindowSize  = 10;
  /// Default max number of frozen windows,
  static const int DefaultNumFrozenWindows = 3;

  class Visualizer: public CvPathRecon::Visualizer {
  public:
    typedef CvPathRecon::Visualizer Parent;
    Visualizer(Cv3DPoseEstimateDisp& poseEstimator):
      Parent(poseEstimator){}
    virtual void draw(
        const PoseEstFrameEntry& frame,
        const PoseEstFrameEntry& lastFrame);
  };
protected:
  /// If matched, extend an existing old track.
  /// @return true if a track is matched and extended. False otherwise.
  bool extendTrack(
      /// Reference to a collection of tracks.
      Tracks& tracks,
      /// The new frame, with new trackable pairs.
      PoseEstFrameEntry& frame,
      /// the index of the inlier,
      int inlierIndex
  );
  bool addTrack(
      /// Reference to a collection of tracks.
      Tracks& tracks,
      /// The new frame, with new trackable pairs.
      PoseEstFrameEntry& frame,
      /// the index of the inlier,
      int inlierIndex
  );
  Tracks mTracks;
  /// size of the sliding window
  int mSlideWindowSize;
  /// number of windows shall be fixed in bundle adjustment
  int mNumFrozenWindows;

};

#endif /* CVVISODOMBUNDLEADJ_H_ */
