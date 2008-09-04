/*
 * CvPathRecon.h
 *
 *  Created on: Sep 2, 2008
 *      Author: jdchen
 */

#ifndef CVPATHRECON_H_
#define CVPATHRECON_H_

#include <vector>
using namespace std;

#include <opencv/cxtypes.h>

#include <keypoint.h>

#include <Cv3DPoseEstimateStereo.h>
#include "CvPoseEstErrMeasDisp.h"

/**
 * Reconstruction of trajectory from stereo pairs of videos
 */
class CvPathRecon {
public:
	class PoseEstFrameEntry {
	public:
		PoseEstFrameEntry(WImageBuffer1_b& image, WImageBuffer1_16s& dispMap,
				vector<Keypoint>& keypoints, CvMat& rot, CvMat& shift,
				int numTrackablePair,
				int numInliers, int frameIndex,
				WImageBuffer3_b& imageC3a, CvMat* inliers0, CvMat* inliers1){
			mRot   = cvMat(3, 3, CV_64FC1, _mRot);
			mShift = cvMat(3, 1, CV_64FC1, _mShift);
			mImage.CloneFrom(image);
			mDispMap.CloneFrom(dispMap);
			mKeypoints  = keypoints;
			cvCopy(&rot,   &mRot);
			cvCopy(&shift, &mShift);
			mNumTrackablePairs = numTrackablePair;
			mNumInliers = numInliers;
			mFrameIndex = frameIndex;

			mImageC3a.CloneFrom(imageC3a);
			mInliers0 = cvCloneMat(inliers0);
			mInliers1 = cvCloneMat(inliers1);
		}
		WImageBuffer1_b   mImage;
		WImageBuffer1_16s mDispMap;
		vector<Keypoint>  mKeypoints;

		CvMat mRot;
		CvMat mShift;
		int   mNumTrackablePairs;
		int   mNumInliers;
		int   mFrameIndex;
		// display and debugging stuff
		WImageBuffer3_b  mImageC3a;
		CvMat* mInliers0;
		CvMat* mInliers1;
	protected:
		double _mRot[9];
		double _mShift[3];
	};
	/**
	 * a record of the pose of a frame
	 */
	class FramePose {
	public:
		FramePose(): mIndex(-1),mRod(cvPoint3D64f(0.,0.,0.)), mShift(cvPoint3D64f(0.,0.,0.)){}
		FramePose(int i): mIndex(i),mRod(cvPoint3D64f(0.,0.,0.)), mShift(cvPoint3D64f(0.,0.,0.)){}
		FramePose(int i, const CvMat& rod, const CvMat& shift):
			mIndex(i), mRod(cvPoint3D64f(0.,0.,0.)), mShift(cvPoint3D64f(0.,0.,0.))
		{
			cvCopy(&rod,   &mRod);
			cvCopy(&shift, &mShift);
		}
		int   mIndex;
		CvPoint3D64f mRod;		// rotation from start frame
		CvPoint3D64f mShift;    // shift from the start frame
	};
	CvPathRecon(const CvSize& imageSize);
	virtual ~CvPathRecon();
	void _init();

    typedef enum {
    	KeyFrameSkip      = 0x0,
    	KeyFrameKeep      = 0x1,
    	KeyFrameUse       = 0x2,
    	KeyFrameBackTrack = 0x3
    } KeyFramingDecision;
    KeyFramingDecision keyFrameEval(int frameIndex, vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
    		vector<Keypoint>& keyPoints,
			int numInliers, const CvMat* inliers0, const CvMat* inliers1,
			const CvMat& rot, const CvMat& shift);

    /**
     * Keeping track of the trajectory
     */
    bool appendTransform(const CvMat& rot, const CvMat& shift);
    /**
     * save the keypoints in a file for visualization or debugging
     */
    bool saveKeyPoints(const CvMat& keypoints, const string& filename);
    /**
     *  stores rotation matrix and shift matrix w.r.t to start frame
     */
    bool storeTransform(const CvMat& rot, const CvMat& shift, int frameIndex);

    bool saveFramePoses(const string& dirname);

    void measureErr(const CvMat* inliers0, const CvMat* inliers1);

    bool getInliers(CvMat*& inliers0, CvMat*& inliers1) {
    	if (mReversed == true) {
    		return mPoseEstimator.getInliers(inliers1, inliers0);
    	} else {
    		return mPoseEstimator.getInliers(inliers0, inliers1);
    	}
    }

    void keepGoodFrame(WImageBuffer1_b & image, WImageBuffer1_16s & dispMap,
    		vector<Keypoint>& keyPoints, CvMat& rot, CvMat& shift,
    		int numTrackablePairs, int numInliers, int frameIndex,
    		WImageBuffer3_b& imageC3a, CvMat* inliers0, CvMat* inliers1);


    static const int    defMaxDisparity  = 15;
    static const int    defMinNumInliersForGoodFrame = 10;
    static const int    defMinNumInliers = 50;
    static const double defMinInlierRatio  = .3;  // ratio between # inliers and # keypoints

    static const double defMaxAngleAlpha = 15.; // degree (0, 180)
    static const double defMaxAngleBeta  = 15.; // degree (0, 180)
    static const double defMaxAngleGamma = 15.; // degree (0, 180)
    static const double defMaxShift      = 300.; // mm

    Cv3DPoseEstimateStereo mPoseEstimator;

    CvMat  mTransform;
    CvMat  mRot;
    CvMat  mShift;
    vector<FramePose> mFramePoses;
    bool   mReversed; 		// true iff pose estimation is from current frame to previous frame

    bool   mLastGoodFrameAvailable;     // true iff a good frame is available for back tracking
    PoseEstFrameEntry* mLastGoodFrame;  // last good frame for back tracking to

    int    mNumFrames;
    int    mStartFrameIndex;
    int    mEndFrameIndex;  // exclusive
    int    mFrameStep;      // increase of frame index from one to another

    int mMinNumInliersForGoodFrame;
    int mMinNumInliers;
    int mMinInlierRatio;
    int mMaxAngleAlpha;
    int mMaxAngleBeta;
    int mMaxAngleGamma;
    int mMaxShift;

    vector<Keypoint>  mKeyPointsLast;  // keypoints of last key frame
    vector<Keypoint>  mKeyPointsCurr;  // keypoints of current frame - may not be key frame

    WImageBuffer1_b   mLastKeyFrameImage;
    WImageBuffer1_16s mLastKeyFrameDispMap;

    double mPathLength; // path length so far
    int    mNumFramesSkipped;
    int	   mTotalInliers; // total number of inliers in all frames
    int    mTotalTrackablePairs; // total number of trackable pairs in all frames
    int    mTotalKeypoints;      // total number of keypoints in all frames
    int    mTotalInliersInKeyFrames;        // total number of inliers in all key frames
    int    mTotalTrackablePairsInKeyFrames; // total number of trackable pairs in all key frames
    int    mTotalKeypointsInKeyFrames;      // total number of keypoints in all key frames

    // error meas
    CvPoseEstErrMeasDisp mErrMeas;
protected:

    double _transform[16];  // data portion of mTransform
	double _rt[16];
	CvMat mRT;
	double _tempMat[16];
	CvMat  _mTempMat;
	// buffer for rotation matrix and shift matrix
	double _rot[9], _shift[3];
};

#endif /* CVPATHRECON_H_ */
