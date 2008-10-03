/*
 * VisOdom.h
 *
 *  Created on: Oct 1, 2008
 *      Author: jdchen
 */

#ifndef VISODOM_H_
#define VISODOM_H_

#include <string>
#include <vector>
#include <iostream>

#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>

using namespace std;

namespace cv { namespace willow {

/// Default value of disparity unit (in pixels) in disparity map
static const double DefDisparityUnitInPixels = 16.;
/// The minimum number needed to do tracking between two cams
static const int defMinNumTrackablePairs = 10;

class KeypointDescriptor {
public:
  virtual ~KeypointDescriptor(){}
  virtual double compare(KeypointDescriptor& kpd);
};

/// Key point extracted from (stereo) images.
/// - For one single 2D image, x, y are interpreted as u and v, the image location
/// of the feature point.
/// - For stereo pair, the class members x, y, z are interpreted as
/// u, v, d, where u and v are the image coordinate on the left image and d
/// is the disparity.
/// - For spin image, or other representations that may want to specify a key point
/// in space, x,y,z can be interpreted as the Cartesian coordinates.
class Keypoint: public CvPoint3D64f
{
public:
  Keypoint(double _x, double _y, double _z, double response, double scale, KeypointDescriptor* descriptor):
    r(response), s(scale), desc(descriptor){
    x = _x; y=_y; z=_z;
  }
  ~Keypoint() {
    delete desc;
  }
  double r; //< the response of the keypoint
  double s; //< scale of the keypoint
  KeypointDescriptor* desc;
};


/// Disparity coordinates of the key points
/// x, y, z in class CvPoint3D64f is used to represent x, y, d (or u, v, d)
typedef vector<CvPoint3D64f> Keypoints;
//typedef vector<Keypoint> Keypoints;


/// Information of pose estimation for one frame.
class PoseEstFrameEntry  {
public:
  /**
   * The object takes ownership of the images, keypoints and inliers
   */
  PoseEstFrameEntry(
      /// left camera image
      WImageBuffer1_b* image,
      /// disparity map
      WImageBuffer1_16s * dispMap,
      /// key points
      Keypoints * keypoints,
      /// rotation matrix
      CvMat & rot,
      /// translation matrix
      CvMat & shift,
      /// number of trackable pairs
      int numTrackablePair,
      /// number of inliers
      int numInliers,
      /// index of this frame in the video sequence
      int frameIndex,
      /// image buffer for visualization
      WImageBuffer3_b *imageC3a,
      /// indices of the inliers in the trackable pairs
      int   *inlierIndices,
      /// index pairs of the trackable pairs w.r.t the keypoint list
      vector<pair<int, int> >* trackableIndexPairs,
      /// inliers from previous key frame
      CvMat *inliers0,
      /// inliers from this frame
      CvMat *inliers1);
  PoseEstFrameEntry(int frameIndex):
    mImage(NULL), mDispMap(NULL), mKeypoints(NULL),
    mRot(cvMat(3, 3, CV_64FC1, _mRot)),
    mShift(cvMat(3, 1, CV_64FC1, _mShift)),
    mGlobalTransform(cvMat(4, 4, CV_64FC1, _mTransform)),
    mNumTrackablePairs(0),
    mNumInliers(0),
    mFrameIndex(frameIndex),
    mImageC3a(NULL),
    mTrackableIndexPairs(NULL),
    mInlierIndices(NULL),
    mInliers0(NULL), mInliers1(NULL){}

  ~PoseEstFrameEntry();

  WImageBuffer1_b* mImage;
  WImageBuffer1_16s* mDispMap;
  Keypoints* mKeypoints;
  /// Estimated rotation matrix from this frame to last key frame
  CvMat mRot;
  /// Estimated translation matrix from this frame to last key frame.
  CvMat mShift;
  /// Estimated global transformation matrix of this frame to the
  /// reference frame (this first frame)
  CvMat mGlobalTransform;
  /// Number of trackable pairs
  int mNumTrackablePairs;
  /// Number of inliers
  int mNumInliers;
  /// index of this frame in the video sequence
  int mFrameIndex;
  /// Index of last key frame, with which the transformation is estimated
  /// Use mostly for sanity checking
  int mLastKeyFrameIndex;
  WImageBuffer3_b *mImageC3a;
  /// index pairs of the trackable pairs w.r.t the keypoint list
  vector<pair<int, int> >* mTrackableIndexPairs;
  /// indices of the inliers in the trackable pairs
  int* mInlierIndices;
  /// inliers from keypoint0
  CvMat *mInliers0;
  /// inliers from keypoint1
  CvMat *mInliers1;
protected:

  void clear();
  // buffers
  double _mRot[9];
  double _mShift[3];
  double _mTransform[16];
};


/// transformation of each frame.
class FramePose   {
public:
  FramePose()
  :mIndex(-1), mRod(cvPoint3D64f(0., 0., 0.)), mShift(cvPoint3D64f(0., 0., 0.))
  {}

  FramePose(int i)
  :mIndex(i), mRod(cvPoint3D64f(0., 0., 0.)), mShift(cvPoint3D64f(0., 0., 0.))
  {}

  FramePose(int i, const CvMat & rod, const CvMat & shift)
  :mIndex(i), mRod(cvPoint3D64f(0., 0., 0.)), mShift(cvPoint3D64f(0., 0., 0.))
  {
    cvCopy(&rod, &mRod);
    cvCopy(&shift, &mShift);
  }

  /// Index of the frame
  int mIndex;
  /// Rodrigues of the rotation
  CvPoint3D64f mRod;
  /// translation matrix
  CvPoint3D64f mShift;
};

class CamTracker;

typedef enum {
  Pairwise,
  BundleAdjust
} CamTrackerType;

/// construct a camera tracker object
CamTracker* getCamTracker(
    /// type of the tracker
    const CamTrackerType type,
    /// size of the image the tracker expects to track on
    CvSize& imgSize,
    /// focal length in x
    double Fx,
    /// focal length in y
    double Fy,
    /// baseline
    double Tx,
    /// optical center of left cam, x coordinate
    double Clx,
    /// optical center of right cam, x coorindate
    double Crx,
    /// optical center, y cooridinate
    double Cy);

/// Tracks a sequence of camera frames.
bool trackCameras(
    /// The tracker
    const CamTracker* tracker,
    /// The directory of the video files
    const string & dirname,
    /// Format of the filename of the left images, e.g. "left-%04d"
    const string & leftFileFmt,
    /// Format of the filename of the right images, e.g. "right-%04d"
    const string & rightFileFmt,
    /// Starting index of the image sequence
    int start,
    /// Ending index (exclusive) of the image sequence
    int end,
    /// increment to add from the index of one image pair to next one
    int step,
    vector<FramePose>*& framePoses);

/// set up the tracker for loading a sequence of stereo camera files for
/// tracking.
void setInputVideoParams(
    /// The tracker
    const CamTracker* tracker,
    /// The directory of the video files
    const string & dirname,
    /// Format of the filename of the left images, e.g. "left-%04d"
    const string & leftFileFmt,
    /// Format of the filename of the right images, e.g. "right-%04d"
    const string & rightFileFmt,
    /// Starting index of the image sequence
    int start,
    /// Ending index (exclusive) of the image sequence
    int end,
    /// increment to add from the index of one image pair to next one
    int step
);

/// Load a stereo image
bool loadStereoFrame(
    /// pointer to the tracker object
    CamTracker* camTracker,
    /// frame index to be loaded
    int frameIndex,
    /// (Output) the left image loaded
    WImageBuffer1_b* & leftImage,
    /// (Output) disparity map.
    WImageBuffer1_16s* & dispMap);

/// print the stats of the last tracking
void printStat(const CamTracker*tracker);

bool goodFeaturesToTrack(
    /// input image
    const WImage1_b& img,
    /// disparity map of the input image
    const WImage1_16s* dispMap,
    /// Interpretation of a unit value in dispMap, in pixels.
    double disparityUnitInPixels,
    /// (OUTPUT) key point detected
    Keypoints& keypoints,
    /// Buffer needed for Harris corner detection
    CvMat* eigImg,
    /// Another buffer needed for Harris Corner detection.
    CvMat* tempImg
);

/// detects 3D keypoints from a stereo image, given as an image and its disparity mag.
bool goodFeaturesToTrack(
    CamTracker* tracker,
    /// input image
    const WImage1_b& img,
    /// disparity map of the input image
    const WImage1_16s* dispMap,
    /// (OUTPUT) key point detected
    Keypoints*& keypoints
);


/**
 * matching method to find a best match of a key point in a neighborhood
 */
typedef enum {
  CrossCorrelation,   //< best location by cross correlation
  KeyPointCrossCorrelation, //< best key point cross correlation
  CalonderDescriptor  //< best key point by Calonder descriptor
} MatchMethod;

/**
  *  Match up two list of key points and output a list of matching pairs.
  *  @return true if the status of execution is normal.
  */
bool matchKeypoints(
    /// type of the key point matcher
    MatchMethod matcherType,
    /// input image 0
    WImage1_b& img0,
    /// input image 1
    WImage1_b& img1,
    /// disparity map of input image 0
    WImage1_16s& dispMap0,
    /// disparity map of input image 1
    WImage1_16s& dispMap1,
    /// Detected key points in image 0
    Keypoints& keyPoints0,
    /// Detected Key points in image 1
    Keypoints& keyPoints1,
    /// (Output) pairs of corresponding 3d locations for possibly the same
    /// 3d features. Used for pose estimation.
    /// Set it to NULL if not interested.
    vector<pair<CvPoint3D64f, CvPoint3D64f> >* matchPairs,
    /// (Output) pairs of indices, to the input keypoints, of the corresponding
    /// 3d locations for possibly the same 3d features. Used for pose estimation.
    /// Set it to NULL if not interested.
    vector<pair<int, int> >* matchIndexPairs
);

/// Find matching keypoints in between the current frame and the last key frame
/// of the tracker.
void matchKeypoints(
    /// pointer to the tracker
    CamTracker* tracker,
    /// (Output) coordinates of trackable pairs
    vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
    /// (Output and Optional) indices of the trackable pairs into
    /// their corresponding key points.
    vector<pair<int, int> >*  trackableIndexPairs = NULL);


/// Forward declaration of a class for pose estimation.
class PoseEstimator;

/// Construct a pose estimator.
PoseEstimator* getPoseEstimator(
    /// The size of the image this pose estimator is expected to see
    CvSize& imgSize,
    /// focal length in x
    double Fx,
    /// focal length in y
    double Fy,
    /// baseline length
    double Tx,
    /// x coordinate of the optical center of the left cam
    double Clx,
    /// x coordinate of the optical center of the right cam
    double Crx,
    /// y coordinate optical center
    double Cy);

/// get a pointer to the pose estimator used by this tracker
PoseEstimator* getPoseEstimator(CamTracker* tracker);

/// Method to estimate transformation based pairs of 3D points, in
/// disparity coordinates
/// @return number of inliers
int poseEstimate(
    /// pointer to pose estimation object
    PoseEstimator* poseEstimator,
    /// key point list 0
    Keypoints& keypoints0,
    /// key point list 1
    Keypoints& keypoints1,
    /// index pairs of matching key points
    vector<pair<int, int> >& matchIndexPairs,
    /// (Output) rotation matrix
    CvMat& rot,
    /// (Output) translation vector
    CvMat& shift,
    /// If true, Levenberg-Marquardt is used at the end for smoothing
    bool smoothed);

/// estimate the transformation with Levenberg-Marquardt.
void estimateWithLevMarq(
    /// pointer to the pose estimation object.
    PoseEstimator* poseEstimator,
    /// inlier list 0
    const CvMat& points0inlier,
    /// inlier list 1
    const CvMat& points1inlier,
    /// transformation matrix from Cartesian coordinates to disparity coordinates
    const CvMat& CartToDisp,
    /// transformation matrix from disparity coordinates to Cartesian coordinates.
    const CvMat& DispToCart,
    CvMat& rot, CvMat& trans);

/// a data structure for tracking image sequence
class FrameSeq {
public:
  FrameSeq():
    mNumFrames(-1),
    mStartFrameIndex(0),
    mEndFrameIndex(0),
    mFrameStep(1),
    mStop(false),
    mLastGoodFrame(NULL),
    mCurrentFrame(NULL),
    mNextFrame(NULL){}

  int mNumFrames;
  int mStartFrameIndex;
  int mEndFrameIndex;
  int mFrameStep;
  bool mStop;

  /// Last good frame
  PoseEstFrameEntry *mLastGoodFrame;
  /// Current frame
  PoseEstFrameEntry *mCurrentFrame;
  /// Next frame. Used in back tracking to hold the current frame
  auto_ptr<PoseEstFrameEntry> mNextFrame;
  void backTrack();
  void keepCurrentAsGoodFrame(){
    assert(mCurrentFrame != NULL);
    releasePoseEstFrameEntry( &mLastGoodFrame);
    mLastGoodFrame = mCurrentFrame;
    mCurrentFrame = NULL;
  }
  void reset() {
    releasePoseEstFrameEntry( &mCurrentFrame);
    releasePoseEstFrameEntry( &mLastGoodFrame);
  }
  int mCurrentFrameIndex;
  inline void setNextFrame() {
    mCurrentFrameIndex += (mNextFrame.get()==NULL)?mFrameStep:0;
  }
  bool notDoneWithIteration() {
    return mCurrentFrameIndex < mEndFrameIndex && mStop == false;
  }
  inline void setStartFrame() {mCurrentFrameIndex = mStartFrameIndex;}
  void releasePoseEstFrameEntry( PoseEstFrameEntry** frameEntry );
};

/// Get a reference of  frame sequence management object
FrameSeq& getFrameSeq(const CamTracker* tracker);

/// Use the tracker to track one single frame.
/// @return true if this frame is used as a key frame.
bool trackOneFrame(CamTracker* tracker, FrameSeq& frameSeq);

typedef enum {
  /// This frame shall be skipped.
  KeyFrameSkip      = 0x0,
  /// This frame shall be kept. Maybe used as key frame later.
  KeyFrameKeep      = 0x1,
  /// Use this frame as key frame right now.
  KeyFrameUse       = 0x2,
  /// Backtrack to last kept frame and use it as key frame. This frame shall
  /// still be used moving forward.
  KeyFrameBackTrack = 0x3
} KeyFramingDecision;

/// Evaluate the stat from the current frame and the tracker and decide
/// appropriate action to the frame.
/// @return recommended action for this frame.
KeyFramingDecision keyFrameEval(
    CamTracker* camTracker,
    /// the index of this frame
    int frameIndex,
    /// number of key points detected in this frame.
    int numKeypoints,
    /// number of inliers detected in this frame with respect to last key frame.
    int numInliers,
    /// estimated rotation matrix.
    const CvMat & rot,
    /// estimated translation matrix.
    const CvMat & shift);

/// perform the recommended action on the current frame
bool keyFrameAction(
    /// Tracker
    CamTracker* tracker,
    /// recommended key frame action
    KeyFramingDecision kfd,
    /// the frame sequence management module
    FrameSeq& frameSeq);

/// set the keyFrame as the last key frame
void setLastKeyFrame(
    /// The tracker
    CamTracker* tracker,
    /// the frame to be set as last key frame
    PoseEstFrameEntry* keyFrame);
/// get the pointer to the last key frame.
/// @return a pointer to the last key frame
PoseEstFrameEntry* getLastKeyFrame(
    /// tracker
    CamTracker* tracker);

/// fetch the inliers of the current frame
bool fetchInliers(
    CamTracker* tracker,
    /// inlier list 0, from last key frame
    CvMat *& inliers0,
    /// inlier list 1 from current key frame
    CvMat *& inliers1);
/// fetch the list of index of inlier, indexed into the matchIndexPairs
/// in cv::willow::poseEstimate
/// @return the index of the inlier pairs
int *fetchInliers(CamTracker* tracker);
/// delete all key frame except the last from the sliding window.
/// (the sliding window is a place holder for bundle adjustment)
void deleteAllButLastKeyFrame(CamTracker* tracker);

/// return a pointer to the list of frame poses
vector<FramePose>* getFramePoses(CamTracker* tracker);

}
}

#endif /* VISODOM_H_ */
