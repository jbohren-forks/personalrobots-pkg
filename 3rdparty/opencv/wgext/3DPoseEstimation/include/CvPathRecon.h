/*
 * CvPathRecon.h
 *
 *  Created on: Sep 2, 2008
 *      Author: jdchen
 */

#ifndef CVPATHRECON_H_
#define CVPATHRECON_H_

#include <vector>
#include <deque>
using namespace std;

#include <opencv/cxtypes.h>

// star detector
#include <star_detector/include/keypoint.h>

#include <Cv3DPoseEstimateStereo.h>
#include "CvPoseEstErrMeasDisp.h"

namespace cv {
namespace willow {
/**
 * Visual Odometry by pose estimation of consecutive pairs of
 * key frames.
 * The input are a sequence of stereo video images.
 */
class PathRecon {
public:
  class PoseEstFrameEntry
  {
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
        vector<Keypoint> * keypoints,
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
    vector<Keypoint>* mKeypoints;
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
    double _mRot[9];
    double _mShift[3];
    double _mTransform[16];
  };
  class FramePose
  {
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
  PathRecon(const CvSize & imageSize);
  virtual ~PathRecon();
  void _init();
  typedef enum { KeyFrameSkip = 0x0, KeyFrameKeep = 0x1, KeyFrameUse = 0x2, KeyFrameBackTrack = 0x3} KeyFramingDecision;
  KeyFramingDecision keyFrameEval(int frameIndex, vector<pair<CvPoint3D64f,CvPoint3D64f> > & trackablePairs,
      vector<Keypoint> & keyPoints, int numInliers, const CvMat *inliers0, const CvMat *inliers1, const CvMat & rot, const CvMat & shift);
  bool appendTransform(const CvMat & rot, const CvMat & shift);
  /// Convert 3d points from local disparity coordinates to
  /// global coordinates, according to the estimated transformation of
  /// current frame
  void dispToGlobal(const CvMat& uvds, CvMat& xyzs);
  void dispToGlobal(const CvMat& uvds, const CvMat& transform, CvMat& xyzs);
  bool saveKeyPoints(const CvMat & keypoints, const string & filename);
  bool storeTransform(const CvMat & rot, const CvMat & shift, int frameIndex);
  bool saveFramePoses(const string & dirname);
  void measureErr(const CvMat *inliers0, const CvMat *inliers1);
  bool getInliers(CvMat *& inliers0, CvMat *& inliers1)
  {
    if(mReversed == true){
      return mPoseEstimator.getInliers(inliers1, inliers0);
    }else{
      return mPoseEstimator.getInliers(inliers0, inliers1);
    }
  }
  const int *getInliers() {
    return mPoseEstimator.getInliers();
  }
  bool fetchInliers(CvMat *& inliers0, CvMat *& inliers1)
  {
    if(mReversed == true){
      return mPoseEstimator.fetchInliers(inliers1, inliers0);
    }else{
      return mPoseEstimator.fetchInliers(inliers0, inliers1);
    }
  }
  int *fetchInliers() {
    return mPoseEstimator.fetchInliers();
  }

  bool recon(const string & dirname, const string & leftFileFmt, const string & rightFileFmt, int start, int end, int step);
  void loadStereoImagePair(int & frameIndex, WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage);
  static void loadStereoImagePair(string & dirname, string & leftimagefmt, string & rightimagefmt, int & frameIndex, WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage);
  /// Load a pair of stereo image, compute disparity map and extract key points.
  bool loadAndProcessStereoFrame(
      /// The frame index
      int frameIndex,
      /// (Output) the left image loaded.
      WImageBuffer1_b* & leftImage,
      /// (Output) disparity map
      WImageBuffer1_16s* & dispMap,
      /// (Output) Key points w.r.t left image. Old contents in keypoints
      /// will be cleared.
      vector<Keypoint>*& keypoints);
  bool loadAndProcessStereoFrame(
      /// The frame index
      int frameIndex,
      /// (Output) a new frame with mImage, mDispMap and mKeypoints
      /// populated
      PoseEstFrameEntry* & frame);
  void setInputVideoParams(
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
      int step);
  /// A routine to visualize the keypoints, tracks, disparity images
  /// etc. By default, it shows on the screen and save to disk.
  virtual void visualize();
  static const int defMaxDisparity = 15;
  static const int defMinNumInliersForGoodFrame = 10;
  static const int defMinNumInliers = 50;
  static const double defMinInlierRatio = .3;
  static const double defMaxAngleAlpha = 15.;
  static const double defMaxAngleBeta = 15.;
  static const double defMaxAngleGamma = 15.;
  static const double defMaxShift = 300.;
  static const int defMinNumTrackablePairs = 10;

  Cv3DPoseEstimateStereo mPoseEstimator;
  /// global transformation matrix up to the last key frame
  CvMat mTransform;
  vector<FramePose> mFramePoses;
  bool mReversed;
  /// Last good frame
  PoseEstFrameEntry *mLastGoodFrame;
  /// Current frame
  PoseEstFrameEntry *mCurrentFrame;
  /// Next frame. Used in back tracking to hold the current frame
  auto_ptr<PoseEstFrameEntry> mNextFrame;
  deque<PoseEstFrameEntry *> mActiveKeyFrames;
  PoseEstFrameEntry* getLastKeyFrame() {
    if (mActiveKeyFrames.size() == 0)
      return NULL;
    return mActiveKeyFrames.back();
  }
  void setLastKeyFrame(PoseEstFrameEntry* keyFrame) {
    assert(keyFrame);
    // make a copy of the current transformation in the record
    // this key frame.
    cvCopy(&mTransform, &keyFrame->mGlobalTransform);
    // enter this key frame into the queue of active key frames
    mActiveKeyFrames.push_back(keyFrame);
  }
  int mNumFrames;
  int mStartFrameIndex;
  int mEndFrameIndex;
  int mFrameStep;
  int mMinNumInliersForGoodFrame;
  int mMinNumInliers;
  int mMinInlierRatio;
  int mMaxAngleAlpha;
  int mMaxAngleBeta;
  int mMaxAngleGamma;
  int mMaxShift;
  double mPathLength;
  /// Statistics of the visual odometry process
  class Stat {
  public:
    Stat();
    void print();

    int mNumKeyPointsWithNoDisparity;
    double mPathLength;
    // histograms
    vector<int> mHistoInliers;
    vector<int> mHistoTrackablePairs;
    vector<int> mHistoKeypoints;
    vector<pair<int, int> > mHistoKeyFrameInliers;
    vector<pair<int, int> > mHistoKeyFrameTrackablePairs;
    vector<pair<int, int> > mHistoKeyFrameKeypoints;

    /// an "independent" error measurement object, for measurement of
    /// of the errors in transformation estimation. Used mostly
    /// for debugging / analysis purposes.
    CvPoseEstErrMeasDisp mErrMeas;
  };
  Stat   mStat; //< Statistics of the visual odometry process

  class Visualizer {
  public:
    Visualizer(Cv3DPoseEstimateDisp& poseEstimator);
    virtual ~Visualizer(){};
    virtual void drawKeypoints(
        const PoseEstFrameEntry& lastFrame,
        const PoseEstFrameEntry& currentFrame,
        const vector<pair<CvPoint3D64f, CvPoint3D64f> >& pointPairsInDisp
    );
    virtual void drawDispMap(const PoseEstFrameEntry& frame);
    virtual void drawTracking(
        const PoseEstFrameEntry& lastFrame,
        const PoseEstFrameEntry& frame
    );
    void show();
    void save();
    void reset();
    string poseEstWinName;
    string leftCamWinName;
    string lastTrackedLeftCam;
    string dispWindowName;
    string outputDirname;
    char dispMapFilename[PATH_MAX];
    char poseEstFilename[PATH_MAX];
    char leftCamWithMarks[PATH_MAX];
    /// a reference to the pose estimator, for projection transformations
    const Cv3DPoseEstimateDisp& poseEstimator;

    WImageBuffer3_b   canvasKeypoint;
    WImageBuffer3_b   canvasTracking;
    WImageBuffer3_b   canvasDispMap;

protected:
    void drawDisparityMap(WImageBuffer1_16s& dispMap);
    bool canvasKeypointRedrawn;
    bool canvasTrackingRedrawn;
    bool canvasDispMapRedrawn;
  };
  Visualizer* mVisualizer;

protected:
  void keepCurrentAsGoodFrame();
  void backTrack();
  void updateTrajectory();
  void getTrackablePairs(
      /// (Output) coordinates of trackable pairs
      vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
      /// (Output and Optional) indices of the trackable pairs into
      /// their corresponding key points.
      vector<pair<int, int> >*  trackableIndexPairs = NULL);
  bool reconOneFrame();
  inline void setStartFrame() {mCurrentFrameIndex = mStartFrameIndex;}
  inline bool notDoneWithIteration() {
    return mCurrentFrameIndex < mEndFrameIndex && mStop == false;
  }
  inline void setNextFrame() {
    mCurrentFrameIndex += (mNextFrame.get()==NULL)?mFrameStep:0;
  }
  int mCurrentFrameIndex;

  // Input Output stuff
  /// input directory name
  string mDirname;
  /// Format of the filename of the left images, e.g. "left-%04d"
  string mLeftImageFilenameFmt;
  /// Format of the filename of the right images, e.g. "right-%04d"
  string mRightImageFilenameFmt;
  /// Output directory name
  string mOutputDir;

  // buffers
  double _transform[16];
  double _rt[16];
  CvMat mRT;
  double _tempMat[16];
  CvMat _mTempMat;
#if 0 // TODO: delete them
  double _rot[9], _shift[3];
#endif

  bool mStop;
};
} // willow
} // cv

#endif /* CVPATHRECON_H_ */
