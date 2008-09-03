/*
 * CvPathRecon.cpp
 *
 *  Created on: Sep 2, 2008
 *      Author: jdchen
 */

#include <iostream>
using namespace std;

#include "CvPathRecon.h"
#include "../include/CvMatUtils.h"
#include "../include/Cv3DPoseEstimate.h"

CvPathRecon::CvPathRecon(const CvSize& imageSize):
	mPoseEstimator(imageSize.width, imageSize.height),
	mTransform(cvMat(4, 4, CV_64FC1, _transform)),
	mRot(cvMat(3, 3, CV_64FC1, _rot)),
	mShift(cvMat(3, 1, CV_64FC1, _shift)),
	mReversed(true),
	mLastGoodFrameAvailable(false),
	mLastGoodFrame(NULL),
	mNumFrames(-1),
	mStartFrameIndex(0),
	mEndFrameIndex(0),
	mFrameStep(1),
	mNumFramesSkipped(0),
	mLastKeyFrameImage(imageSize.width, imageSize.height),
	mLastKeyFrameDispMap(imageSize.width, imageSize.height),
	mPathLength(0.),
	mRT(cvMat(4,4, CV_64FC1, _rt)),
	_mTempMat(cvMat(4,4,CV_64FC1, _tempMat))
{
	_init();
}

CvPathRecon::~CvPathRecon() {
	// TODO Auto-generated destructor stub
}

void CvPathRecon::_init() {
	mMinNumInliersForGoodFrame  = defMinNumInliersForGoodFrame;
	mMinNumInliers  = defMinNumInliers;
	mMinInlierRatio = defMinInlierRatio;
	mMaxAngleAlpha  = defMaxAngleAlpha;
	mMaxAngleBeta   = defMaxAngleBeta;
	mMaxAngleGamma  = defMaxAngleGamma;
	mMaxShift       = defMaxShift;

	cvSetIdentity(&mTransform);
}

CvPathRecon::KeyFramingDecision
CvPathRecon::keyFrameEval(
		int frameIndex,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
		vector<Keypoint>& keyPoints,
		int numInliers,
		const CvMat* inliers0, const CvMat* inliers1,
		const CvMat& rot, const CvMat& shift
		) {
	KeyFramingDecision kfd = KeyFrameKeep;
	bool keyFrameNeeded = false;

	//
	// check if the frame is good enough for checking
	//
	if (numInliers < defMinNumInliersForGoodFrame) {
		// Too few inliers to ensure good tracking
		keyFrameNeeded = true;
	} else if (cvNorm(&shift) > mMaxShift) {
		// the shift is large enough that a key frame is needed
		keyFrameNeeded = true;
	} else if (numInliers < mMinNumInliers) {
		keyFrameNeeded = true;
	} else if ((double)numInliers/(double)keyPoints.size() < mMinInlierRatio) {
		keyFrameNeeded = true;
	} else {
		CvPoint3D64f eulerAngles;
		CvMatUtils::eulerAngle(rot, eulerAngles);
		if (fabs(eulerAngles.x) > mMaxAngleAlpha ||
			fabs(eulerAngles.y) > mMaxAngleBeta  ||
			fabs(eulerAngles.z) > mMaxAngleGamma ){
			// at least one of the angle is large enough that a key frame is needed
			keyFrameNeeded = true;
		}
	}

	if (keyFrameNeeded == true) {
		if (mLastGoodFrameAvailable == false) {
			// nothing to back track to. What can I do except use it :(
			kfd = KeyFrameUse;
		} else {
			kfd = KeyFrameBackTrack;
		}
	} else {
		kfd = KeyFrameKeep;
	}

	// special logic for first frame and last frame
	if (mNumFrames > 0 && frameIndex == mNumFrames - mStartFrameIndex - 1) {
		// last frame, just use it for now.
		kfd = KeyFrameUse;
	}

	return kfd;
}

/**
 * Keep track of the trajectory
 * store the current transformation from starting point to current position
 * in transform as transform = transform * rt
 */
bool CvPathRecon::appendTransform(const CvMat& rot, const CvMat& shift){
	Cv3DPoseEstimate::constructTransform(rot, shift, mRT);
	cvCopy(&mTransform, &_mTempMat);
	if (mReversed == true) {
		cvMatMul(&_mTempMat, &mRT, &mTransform);
	} else {
		cvMatMul(&mRT, &_mTempMat, &mTransform);
	}
	return true;
}

/**
 * Store the current key frame to key frame transformation with w.r.t to starting frame
 */
bool CvPathRecon::storeTransform(const CvMat& rot, const CvMat& shift, int frameIndex){
	CvMat rotGlobal;
	CvMat shiftGlobal;
	cvGetSubRect(&mTransform, &rotGlobal,   cvRect(0, 0, 3, 3));
	cvGetSubRect(&mTransform, &shiftGlobal, cvRect(3, 0, 1, 3));
	FramePose* fp = new FramePose(frameIndex);
	CvMat rodGlobal2   = cvMat(1,3, CV_64FC1, &(fp->mRod));
	CvMat shiftGlobal2 = cvMat(1,3, CV_64FC1, &(fp->mShift));
	// make a copy
	if (mReversed == true) {
		cvRodrigues2(&rotGlobal, &rodGlobal2);
		cvTranspose(&shiftGlobal,  &shiftGlobal2);
	} else {
		cvRodrigues2(&rotGlobal, &rodGlobal2);
		cvTranspose(&shiftGlobal,  &shiftGlobal2);
		cvScale(&shiftGlobal2, &shiftGlobal2, -1.0);
	}
	mFramePoses.push_back(*fp);

	// update the path length as well
	mPathLength += cvNorm((const CvMat *)&shift);
	return true;
}

bool CvPathRecon::saveKeyPoints(const CvMat& keypoints, const string& filename){
	bool status = true;
	if (mReversed == true) {
		double _inliers1xyz[3*keypoints.rows];
		CvMat inliers1xyz = cvMat(keypoints.rows, 3, CV_64FC1, _inliers1xyz);
		mPoseEstimator.reprojection(&keypoints, &inliers1xyz);
		double _inliers1t[3*keypoints.rows];
		CvMat inliers1t = cvMat(keypoints.rows, 1, CV_64FC3, _inliers1t);
		CvMat inliers1Reshaped;
		CvMat transform3x4;
		cvReshape(&inliers1xyz, &inliers1Reshaped, 3, 0);
		cvGetRows(&mTransform, &transform3x4, 0, 3);
		cvTransform(&inliers1Reshaped, &inliers1t, &transform3x4);
		cvSave(filename.c_str(), &inliers1t,
				"inliers1", "inliers of current image in start frame");
	} else {
		cerr << __PRETTY_FUNCTION__<< "Not implemented yet for reversed == false"<<endl;
		status = false;
	}
	return status;
}

bool CvPathRecon::saveFramePoses(const string& dirname) {
	bool status = true;
	// TODO: for now, turn poses into a CvMat of numOfKeyFrames x 7 (index, rod[3], shift[3])
	double _poses[mFramePoses.size()*7];
	CvMat  framePoses = cvMat(mFramePoses.size(), 7, CV_64FC1, _poses);
	int i=0;
	for (vector<FramePose>::const_iterator iter= mFramePoses.begin(); iter!=mFramePoses.end(); iter++,i++) {
		_poses[i*7 + 0] = iter->mIndex;
		_poses[i*7 + 1] = iter->mRod.x;
		_poses[i*7 + 2] = iter->mRod.y;
		_poses[i*7 + 3] = iter->mRod.z;
		_poses[i*7 + 4] = iter->mShift.x;
		_poses[i*7 + 5] = iter->mShift.y;
		_poses[i*7 + 6] = iter->mShift.z;
	}
	string framePosesFilename("framePoses.xml");
	cvSave((dirname+framePosesFilename).c_str(), &framePoses, "index-rod3-shift3", "indices, rodrigues and shifts w.r.t. starting frame");
	return status;
}

void CvPathRecon::measureErr(const CvMat* inliers0, const CvMat* inliers1){
	mErrMeas.setTransform(mRot, mShift);
	if (mReversed == true) {
		// rot and shift transform inliers1 to inliers0
		mErrMeas.measure(*inliers1, *inliers0);
	} else {
		// rot and shift transform inliers0 to inliers1
		mErrMeas.measure(*inliers0, *inliers1);
	}
}
