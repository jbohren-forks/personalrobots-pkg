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
#include <boost/unordered_map.hpp>
using namespace std;
#include <limits.h>

#include <opencv/cxtypes.h>

#include <Cv3DPoseEstimateStereo.h>
#include "CvPoseEstErrMeasDisp.h"

#include "VisOdom.h"

namespace cv {
namespace willow {

class VOVisualizer {
public:
  virtual ~VOVisualizer(){}
  virtual void drawKeypoints(
    const PoseEstFrameEntry& lastFrame,
    const PoseEstFrameEntry& currentFrame,
    const vector<pair<CvPoint3D64f, CvPoint3D64f> >& pointPairsInDisp
  ) = 0;
  virtual void drawDispMap(const PoseEstFrameEntry& frame) = 0;
  virtual void drawTrackingCanvas(
      const PoseEstFrameEntry& lastFrame,
      const PoseEstFrameEntry& frame
  ) = 0;

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
  const PoseEstimateDisp& poseEstimator;

  WImageBuffer3_b   canvasKeypoint;
  WImageBuffer3_b   canvasTracking;
  WImageBuffer3_b   canvasDispMap;

protected:
  VOVisualizer(PoseEstimateDisp& poseEstimator);

  void drawDisparityMap(WImageBuffer1_16s& dispMap);
  bool canvasKeypointRedrawn;
  bool canvasTrackingRedrawn;
  bool canvasDispMapRedrawn;

};

/// Visualizing the process of visual odometry process with frame to frame
/// pose estimation.
class F2FVisualizer: public VOVisualizer {
  public:
    F2FVisualizer(PoseEstimateDisp& poseEstimator);
    virtual ~F2FVisualizer(){};
    virtual void drawKeypoints(
        const PoseEstFrameEntry& lastFrame,
        const PoseEstFrameEntry& currentFrame,
        const vector<pair<CvPoint3D64f, CvPoint3D64f> >& pointPairsInDisp
    );
    virtual void drawDispMap(const PoseEstFrameEntry& frame);
    virtual void drawTrackingCanvas(
        const PoseEstFrameEntry& lastFrame,
        const PoseEstFrameEntry& frame
    );
};

/**
 * Visual Odometry by pose estimation of consecutive pairs of
 * key frames.
 * The input are a sequence of stereo video images.
 */
class PathRecon: public CamTracker {
public:

  PathRecon(const CvSize & imageSize);
  virtual ~PathRecon();
  void _init();

  KeyFramingDecision keyFrameEval(
      int frameIndex,
      int numKeypoints,
      int numInliers,
      const CvMat & rot,
      const CvMat & shift);
  bool keyFrameAction(KeyFramingDecision kfd, FrameSeq& frameSeq);
  bool appendTransform(const CvMat & rot, const CvMat & shift);
  /// Convert 3d points from local disparity coordinates to
  /// global coordinates, according to the estimated transformation of
  /// current frame
  void dispToGlobal(const CvMat& uvds, CvMat& xyzs);
  void dispToGlobal(const CvMat& uvds, const CvMat& transform, CvMat& xyzs);
  bool saveKeyPoints(const CvMat & keypoints, const string & filename);
  bool storeTransform(const CvMat & rot, const CvMat & shift, int frameIndex);
  void measureErr(const CvMat *inliers0, const CvMat *inliers1);
  bool getInliers(CvMat *& inliers0, CvMat *& inliers1)
  {
    return mPoseEstimator.getInliers(inliers0, inliers1);
  }
  const int *getInliers() {
    return mPoseEstimator.getInliers();
  }
  bool fetchInliers(CvMat *& inliers0, CvMat *& inliers1)
  {
    return mPoseEstimator.fetchInliers(inliers0, inliers1);
  }
  int *fetchInliers() {
    return mPoseEstimator.fetchInliers();
  }

  /// Process all the stereo images in the queue.
  virtual bool track(
      /// queue of input stereo images
      queue<StereoFrame>& inputImages
  );

  /// compute the feature point to tracking
  virtual bool goodFeaturesToTrack(
      /// the left image
      const WImage1_b& leftImage,
      /// the disparity map
      const WImage1_16s* dispMap,
      /// compute feature points.
      /// (Output) Key points w.r.t left image. Old contents in keypoints
      /// will be cleared.
      Keypoints*& keypoints
  );

  virtual vector<FramePose*>* getFramePoses();

  /// \brief setting the camera parameters
  virtual void setCameraParams(double Fx, double Fy, double Tx,
      double Clx, double Crx, double Cy, double dispUnitScale);

  /// A routine to visualize the keypoints, tracks, disparity images
  /// etc. By default, it shows on the screen and save to disk.
  virtual void visualize();
  virtual void printStat();
  static const int defMaxDisparity = 15;
  static const int defMinNumInliersForGoodFrame = 10;
  static const int defMinNumInliers = 50;
  static const double defMinInlierRatio = .0;
  static const double defMaxAngleAlpha = 15.;
  static const double defMaxAngleBeta = 15.;
  static const double defMaxAngleGamma = 15.;
  static const double defMaxShift = 300.;

  PoseEstimateStereo mPoseEstimator;
  /// global transformation matrix up to the last key frame, in Cartesian space.
  CvMat mTransform;
  vector<FramePose *> mFramePoses;
  boost::unordered_map<int, FramePose*> map_index_to_FramePose_;

  FrameSeq mFrameSeq;
  virtual FrameSeq& getFrameSeq() {return mFrameSeq;}
  virtual PoseEstimator* getPoseEstimator() {return &mPoseEstimator;}
  virtual bool trackOneFrame(queue<StereoFrame>& inputImageQueue, FrameSeq& frameSeq);
  /// the sliding window if you will.
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
    cvCopy(&mTransform, &keyFrame->transf_local_to_global_);
    // enter this key frame into the queue of active key frames
    mActiveKeyFrames.push_back(keyFrame);
  }
  /// Reduce the key frame window size to winSize.
  virtual void reduceWindowSize(unsigned int winSize) {
    while(mActiveKeyFrames.size()>winSize) {
      PoseEstFrameEntry* frame = mActiveKeyFrames.front();
      mActiveKeyFrames.pop_front();
      delete frame;
    }
  }
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
    FramePose* mFinalPose;
  };
  Stat   mStat; //< Statistics of the visual odometry process

  VOVisualizer* mVisualizer;

  void matchKeypoints(
      /// (Output) coordinates of trackable pairs
      vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
      /// (Output and Optional) indices of the trackable pairs into
      /// their corresponding key points.
      vector<pair<int, int> >*  trackableIndexPairs = NULL);

protected:
  void updateTrajectory();

  // Input Output stuff

  // buffers
  double _transform[16];
  double _rt[16];
  CvMat mRT;
  double _tempMat[16];
  CvMat _mTempMat;
};
} // willow
} // cv

#endif /* CVPATHRECON_H_ */
