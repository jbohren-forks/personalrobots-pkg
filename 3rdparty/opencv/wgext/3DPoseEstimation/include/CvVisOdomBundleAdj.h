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
  class TrackObservs {
  public:
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
    deque<TrackObservs>  mObservs;
    /// 3D Cartesian coordinates to be estimated.
    CvPoint3D64f     mCoordinates;
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

  /// Default size of the sliding window
  static const int DefaultSlideWindowSize  = 10;
  /// Default max number of frozen windows,
  static const int DefaultNumFrozenWindows = 3;
protected:
  /// size of the sliding window
  int mSlideWindowSize;
  /// number of windows shall be fixed in bundle adjustment
  int mNumFrozenWindows;
};

#endif /* CVVISODOMBUNDLEADJ_H_ */
