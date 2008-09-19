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

/**
 * Visual Odometry by pose estimation of consecutive pairs of
 * key frames.
 * The input are a sequence of stereo video images.
 */
class CvPathRecon {
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
          /// inliers from previous key frame
          CvMat *inliers0,
          /// inliers from this frame
          CvMat *inliers1);
      PoseEstFrameEntry(int frameIndex):
        mImage(NULL), mDispMap(NULL), mKeypoints(NULL),
        mRot(cvMat(3, 3, CV_64FC1, _mRot)),
        mShift(cvMat(3, 1, CV_64FC1, _mShift)),
        mNumTrackablePairs(0),
        mNumInliers(0),
        mFrameIndex(frameIndex),
        mImageC3a(NULL),
        mInliers0(NULL), mInliers1(NULL){}

      ~PoseEstFrameEntry();

      WImageBuffer1_b* mImage;
      WImageBuffer1_16s* mDispMap;
      vector<Keypoint>* mKeypoints;
      CvMat mRot;
      CvMat mShift;
      int mNumTrackablePairs;
      int mNumInliers;
      int mFrameIndex;
      WImageBuffer3_b *mImageC3a;
      CvMat *mInliers0;
      CvMat *mInliers1;
    protected:

      void clear();
      double _mRot[9];
      double _mShift[3];
    };
    class FramePose
    {
    public:
        FramePose()
        :mIndex(-1), mRod(cvPoint3D64f(0., 0., 0.)), mShift(cvPoint3D64f(0., 0., 0.))
        {
        }

        FramePose(int i)
        :mIndex(i), mRod(cvPoint3D64f(0., 0., 0.)), mShift(cvPoint3D64f(0., 0., 0.))
        {
        }

        FramePose(int i, const CvMat & rod, const CvMat & shift)
        :mIndex(i), mRod(cvPoint3D64f(0., 0., 0.)), mShift(cvPoint3D64f(0., 0., 0.))
        {
            cvCopy(&rod, &mRod);
            cvCopy(&shift, &mShift);
        }

        int mIndex;
        CvPoint3D64f mRod;
        CvPoint3D64f mShift;
    };
    CvPathRecon(const CvSize & imageSize);
    virtual ~CvPathRecon();
    void _init();
    typedef enum { KeyFrameSkip = 0x0, KeyFrameKeep = 0x1, KeyFrameUse = 0x2, KeyFrameBackTrack = 0x3} KeyFramingDecision;
    KeyFramingDecision keyFrameEval(int frameIndex, vector<pair<CvPoint3D64f,CvPoint3D64f> > & trackablePairs,
        vector<Keypoint> & keyPoints, int numInliers, const CvMat *inliers0, const CvMat *inliers1, const CvMat & rot, const CvMat & shift);
    bool appendTransform(const CvMat & rot, const CvMat & shift);
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
    bool fetchInliers(CvMat *& inliers0, CvMat *& inliers1)
    {
        if(mReversed == true){
            return mPoseEstimator.fetchInliers(inliers1, inliers0);
        }else{
            return mPoseEstimator.fetchInliers(inliers0, inliers1);
        }
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
    static const int defMaxDisparity = 15;
    static const int defMinNumInliersForGoodFrame = 10;
    static const int defMinNumInliers = 50;
    static const double defMinInlierRatio = .3;
    static const double defMaxAngleAlpha = 15.;
    static const double defMaxAngleBeta = 15.;
    static const double defMaxAngleGamma = 15.;
    static const double defMaxShift = 300.;
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
    PoseEstFrameEntry *mNextFrame;
    deque<PoseEstFrameEntry *> mActiveKeyFrames;
    PoseEstFrameEntry* getLastKeyFrame() {
      if (mActiveKeyFrames.size() == 0)
        return NULL;
      return mActiveKeyFrames.back();
    }
    void setLastKeyFrame(PoseEstFrameEntry* keyFrame) {
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
    int mNumFramesSkipped;
    int mTotalInliers;
    int mTotalTrackablePairs;
    int mTotalKeypoints;
    int mTotalInliersInKeyFrames;
    int mTotalTrackablePairsInKeyFrames;
    int mTotalKeypointsInKeyFrames;
    /// an "independent" error measurement object, for measurement of
    /// of the errors in transformation estimation. Used mostly
    /// for debugging / analysis purposes.
    CvPoseEstErrMeasDisp mErrMeas;
    string mDirname;
    string mLeftImageFilenameFmt;
    string mRightImageFilenameFmt;
protected:
  void keepCurrentAsGoodFrame();
  void backTrack();
  void updateTrajectory();
  void getTrackablePairs(
      vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs);
  bool reconOneFrame(int frameIndex);
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

#endif /* CVPATHRECON_H_ */
