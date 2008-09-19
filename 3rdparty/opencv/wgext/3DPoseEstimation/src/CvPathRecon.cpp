/*
 * CvPathRecon.cpp
 *
 *  Created on: Sep 2, 2008
 *      Author: jdchen
 */

#include <iostream>
using namespace std;

// opencv
#include <opencv/highgui.h>

#include "CvPathRecon.h"
#include "../include/CvMatUtils.h"
#include "../include/Cv3DPoseEstimate.h"

// timing
#include "CvTestTimer.h"

//#define DISPLAY

//#undef DEBUG
#ifndef DEBUG
#define DEBUG  // to print debug message in release build
#endif

// Please note that because the timing code is executed is called lots of lots of times
// they themselves have taken substantial timing as well
#define CHECKTIMING 1

#if CHECKTIMING == 0
#define TIMERSTART(x)
#define TIMEREND(x)
#define TIMERSTART2(x)
#define TIMEREND2(x)
#else
#define TIMERSTART(x)  CvTestTimerStart(x)
#define TIMEREND(x)    CvTestTimerEnd(x)
#define TIMERSTART2(x) CvTestTimerStart2(x)
#define TIMEREND2(x)   CvTestTimerEnd2(x)
#endif

CvPathRecon::CvPathRecon(const CvSize& imageSize):
	mPoseEstimator(imageSize.width, imageSize.height),
	mTransform(cvMat(4, 4, CV_64FC1, _transform)),
	mReversed(true),
	mLastGoodFrame(NULL),
	mCurrentFrame(NULL),
  mNextFrame(NULL),
	mNumFrames(-1),
	mStartFrameIndex(0),
	mEndFrameIndex(0),
	mFrameStep(1),
	mPathLength(0.),
  mNumFramesSkipped(0),
	mTotalInliers(0),
	mTotalTrackablePairs(0),
	mTotalKeypoints(0),
	mTotalInliersInKeyFrames(0),
	mTotalTrackablePairsInKeyFrames(0),
	mTotalKeypointsInKeyFrames(0),
	mDirname(string("Data/indoor1")),
  mLeftImageFilenameFmt(mDirname.append("/left-%04.ppm")),
  mRightImageFilenameFmt(mDirname.append("/right-%04.ppm")),
  mRT(cvMat(4,4, CV_64FC1, _rt)),
  _mTempMat(cvMat(4,4,CV_64FC1, _tempMat)),
  mStop(false)
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
		if (mLastGoodFrame == NULL) {
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
  assert(mCurrentFrame);
	mErrMeas.setTransform(mCurrentFrame->mRot, mCurrentFrame->mShift);
	if (mReversed == true) {
		// rot and shift transform inliers1 to inliers0
		mErrMeas.measure(*inliers1, *inliers0);
	} else {
		// rot and shift transform inliers0 to inliers1
		mErrMeas.measure(*inliers0, *inliers1);
	}
}

void CvPathRecon::keepCurrentAsGoodFrame(){
  assert(mCurrentFrame != NULL);
  delete mLastGoodFrame;
  mLastGoodFrame = mCurrentFrame;
  mCurrentFrame = NULL;
  mNumFramesSkipped++;
}

void CvPathRecon::loadStereoImagePair(int & frameIndex,
    WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage)
{
  loadStereoImagePair(mDirname, mLeftImageFilenameFmt,
      mRightImageFilenameFmt, frameIndex, leftImage, rightImage);
}

void CvPathRecon::loadStereoImagePair(string& dirname, string& leftimagefmt,
    string& rightimagefmt, int & frameIndex,
    WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage)
{
  char leftfilename[PATH_MAX];
  char rightfilename[PATH_MAX];
  sprintf(leftfilename, leftimagefmt.c_str(), frameIndex);
  sprintf(rightfilename, rightimagefmt.c_str(), frameIndex);
#ifdef DEBUG
  cout << "loading " << leftfilename << " and " << rightfilename << endl;
#endif
  IplImage* leftimg  = cvLoadImage(leftfilename,  CV_LOAD_IMAGE_GRAYSCALE);
  leftImage.SetIpl(leftimg);
  IplImage* rightimg = cvLoadImage(rightfilename, CV_LOAD_IMAGE_GRAYSCALE);
  rightImage.SetIpl(rightimg);
}

bool CvPathRecon::loadAndProcessStereoFrame(int frameIndex, PoseEstFrameEntry* & frame){
  frame = new PoseEstFrameEntry(frameIndex);

  return loadAndProcessStereoFrame(frameIndex, frame->mImage, frame->mDispMap, frame->mKeypoints);
}

bool CvPathRecon::loadAndProcessStereoFrame(int frameIndex,
    WImageBuffer1_b* & leftImage, WImageBuffer1_16s* & dispMap,
    vector<Keypoint>* & keypoints)
{
  bool status = true;
  WImageBuffer1_b rightImage;
  leftImage = new WImageBuffer1_b();
  CvSize& imgSize = mPoseEstimator.getSize();
  dispMap = new WImageBuffer1_16s(imgSize.width, imgSize.height);
  keypoints = new vector<Keypoint>();
  // load stereo image pair
  TIMERSTART2(LoadImage);
  loadStereoImagePair(frameIndex, *leftImage, rightImage);
  TIMEREND2(LoadImage);
  // compute disparity map
  TIMERSTART2(DisparityMap);
  mPoseEstimator.getDisparityMap(*leftImage, rightImage, *dispMap);
  TIMEREND2(DisparityMap);
  TIMERSTART2(FeaturePoint);

  keypoints->clear();
  status = mPoseEstimator.goodFeaturesToTrack(*leftImage, dispMap, *keypoints);
  TIMEREND2(FeaturePoint);
  mTotalKeypoints += keypoints->size();
#ifdef DEBUG
  cout << "Found " << keypoints->size() << " good features in left  image" << endl;
#endif

#ifdef DISPLAY
  cvCvtColor(leftImage.Ipl(),  leftimgC3,  CV_GRAY2RGB);
  leftimgC3a = leftImageC3a.Ipl();
  cvCvtColor(leftImage.Ipl(),  leftimgC3a, CV_GRAY2RGB);

  showDisparityMap(dispMap, dispWindowName, outputDirname, frameIndex, maxDisp);
  drawKeypoints(leftImageC3, keyPointsLast, keyPointsCurr);
#endif
  return status;
}

void CvPathRecon::setInputVideoParams(const string& dirname, const string& leftFileFmt,
    const string& rightFileFmt, int start, int end, int step)
{
    mDirname = dirname;
    mLeftImageFilenameFmt = dirname;
    mLeftImageFilenameFmt += leftFileFmt;
    mRightImageFilenameFmt = dirname;
    mRightImageFilenameFmt += rightFileFmt;
    mStartFrameIndex = start;
    mNumFrames = end - start;
    mEndFrameIndex = end;
    mFrameStep = step;
}

void CvPathRecon::backTrack() {
  assert(mNextFrame == NULL);
  assert(mLastGoodFrame != NULL);
  assert(mCurrentFrame != NULL);
#ifdef DEBUG
  cerr << "Going back to last good frame  from frame "<<mCurrentFrame->mFrameIndex<<endl;
  cerr << "Last good frame is "<<mLastGoodFrame->mFrameIndex << endl;
#endif
  mNumFramesSkipped--;
  mNextFrame     = mCurrentFrame;
  mCurrentFrame  = mLastGoodFrame;
  mLastGoodFrame = NULL;
}

void CvPathRecon::updateTrajectory() {
  // keep track of the trajectory
  appendTransform(mCurrentFrame->mRot, mCurrentFrame->mShift);
  mTotalInliersInKeyFrames        += mCurrentFrame->mNumInliers;
  mTotalKeypointsInKeyFrames      += mCurrentFrame->mKeypoints->size();
  mTotalTrackablePairsInKeyFrames += mCurrentFrame->mNumTrackablePairs;

  // save the inliers into a file
  char inliersFilename[256];
  sprintf(inliersFilename, "Output/indoor1/inliers1_%04d.xml", mCurrentFrame->mFrameIndex);
  saveKeyPoints(*mCurrentFrame->mInliers1, string(inliersFilename));

  // stores rotation mat and shift vector in rods and shifts
  storeTransform(mCurrentFrame->mRot, mCurrentFrame->mShift, mCurrentFrame->mFrameIndex - mStartFrameIndex);

#ifdef DISPLAY
  CvMatUtils::drawMatchingPairs(*inliers0, *inliers1, mCurrentFrame->mImageC3a,
      rot, shift,
      (Cv3DPoseEstimateDisp&)mPoseEstimator, reversed);
#endif

#ifdef DEBUG
  // measure the errors
  measureErr(mCurrentFrame->mInliers0, mCurrentFrame->mInliers1);
#endif

  // getting ready for next iteration
  setLastKeyFrame(mCurrentFrame);
  mCurrentFrame = NULL;
}

void CvPathRecon::getTrackablePairs(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs) {
  TIMERSTART2(TrackablePair);
  PoseEstFrameEntry* lastKeyFrame = getLastKeyFrame();
  assert(lastKeyFrame != NULL);
  mPoseEstimator.getTrackablePairs(
      *lastKeyFrame->mImage,     *mCurrentFrame->mImage,
      *lastKeyFrame->mDispMap,   *mCurrentFrame->mDispMap,
      *lastKeyFrame->mKeypoints, *mCurrentFrame->mKeypoints,
      trackablePairs);
  mTotalTrackablePairs += trackablePairs.size();
  mCurrentFrame->mNumTrackablePairs = trackablePairs.size();
  TIMEREND2(TrackablePair);
}

/// reconstruction w.r.t one additional frame
bool CvPathRecon::reconOneFrame(int frameIndex) {
    bool insertNewKeyFrame = false;
    TIMERSTART(Total);
    bool & reversed = mReversed;
    PoseEstFrameEntry *& currFrame = mCurrentFrame;

    if (mNextFrame){
      // do not need to load new images nor compute the key points again
      currFrame = mNextFrame;
      mNextFrame = NULL;
    } else {
      // load and process next stereo pair of images.
      loadAndProcessStereoFrame(frameIndex, currFrame);
    }

    if (currFrame->mFrameIndex == mStartFrameIndex) {
      // First frame ever, do not need to do anything more.
      setLastKeyFrame(currFrame);
      insertNewKeyFrame = true;
      currFrame = NULL;
      return insertNewKeyFrame;
    }
    //
    // match the good feature points between this iteration and last key frame
    //
    vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;
    getTrackablePairs(trackablePairs);
#ifdef DEBUG
    cout << "Num of trackable pairs for pose estimate: "<<trackablePairs.size() <<endl;
#endif

    if (currFrame->mNumTrackablePairs<10) {
#ifdef DEBUG
      cout << "Too few trackable pairs" <<endl;
#endif
#ifdef DISPLAY
      sprintf(info, "%04d, #TrackablePair: %d, Too few to track",
          frameIndex, currFrame->mNumTrackablePairs);
#endif
    } else {
#ifdef DISPLAY
      // Draw all the trackable pairs
      this->drawTrackablePairs(leftImageC3, trackablePairs);
#endif

      //  pose estimation given the feature point pairs
      TIMERSTART2(PoseEstimate);
      currFrame->mNumInliers =
        mPoseEstimator.estimate(trackablePairs,
            currFrame->mRot, currFrame->mShift, reversed);
      TIMEREND2(PoseEstimate);

      mTotalInliers += currFrame->mNumInliers;

      currFrame->mInliers0 = NULL;
      currFrame->mInliers1 = NULL;
      fetchInliers(currFrame->mInliers0, currFrame->mInliers1);
#ifdef DEBUG
      cout << "num of inliers: "<< currFrame->mNumInliers <<endl;
#endif

      // Decide if we need select a key frame by now
      CvPathRecon::KeyFramingDecision kfd =
        keyFrameEval(frameIndex, trackablePairs, *currFrame->mKeypoints, currFrame->mNumInliers,
            currFrame->mInliers0, currFrame->mInliers1,
            currFrame->mRot, currFrame->mShift);

      switch (kfd) {
      case CvPathRecon::KeyFrameSkip:  {
        // skip this frame
        mNumFramesSkipped++;
        break;
      }
      case CvPathRecon::KeyFrameBackTrack:   {
        // go back to the last good frame
        backTrack();
        updateTrajectory();
        insertNewKeyFrame = true;
        // next we are supposed to try nextFrame with currFrame
        break;
      }
      case CvPathRecon::KeyFrameKeep:   {
        // keep current frame as last good frame.
        keepCurrentAsGoodFrame();
        break;
      }
      case CvPathRecon::KeyFrameUse:  {
        // use currFrame as key frame
        updateTrajectory();
        insertNewKeyFrame = true;
        break;
      }
      default:
        break;
      }

#ifdef DISPLAY
      CvPoint3D64f euler;
      CvMatUtils::eulerAngle(rot, euler);
      sprintf(info, "%04d, KyPt %d, TrckPir %d, Inlrs %d, eulr=(%4.2f,%4.2f,%4.2f), d=%4.1f",
          frameIndex, keyPointsCurr.size(), currFrame->mNumTrackablePairs, numInliers, euler.x, euler.y, euler.z, cvNorm((const CvMat *)&shift));
#endif
    }
#ifdef DISPLAY
    cvPutText(leftimgC3a, info, org, &font, CvMatUtils::yellow);
    cvShowImage(poseEstWinName.c_str(), leftimgC3a);
    cvShowImage(leftCamWinName.c_str(), leftimgC3);
#endif
    // save the marked images
#ifdef DISPLAY
    sprintf(leftCamWithMarks, "%s/leftCamWithMarks-%04d.png", outputDirname.c_str(), frameIndex);
    sprintf(poseEstFilename, "%s/poseEst-%04d.png", outputDirname.c_str(), frameIndex);
    cvSaveImage(leftCamWithMarks,  leftimgC3);
    cvSaveImage(poseEstFilename,   leftimgC3a);

    // wait for a while for opencv to draw stuff on screen
    cvWaitKey(25);  //  milliseconds
    //    cvWaitKey(0);  //  wait indefinitely
#endif
    TIMEREND(Total);
    return insertNewKeyFrame;
}

bool CvPathRecon::recon(const string & dirname, const string & leftFileFmt,
    const string & rightFileFmt, int start, int end, int step)
{
  bool status = false;
  setInputVideoParams(dirname, leftFileFmt, rightFileFmt, start, end, step);

  int maxDisp = (int)(mPoseEstimator.getD(400));// the closest point we care is at least 1000 mm away
  cout << "Max disparity is: " << maxDisp << endl;
  mErrMeas.setCameraParams((const CvStereoCamParams& )(mPoseEstimator));

  // current frame
  PoseEstFrameEntry*& currFrame = mCurrentFrame;
  delete currFrame;
  currFrame = NULL;
  // last good frame as candidate for next key frame
  delete mLastGoodFrame;
  mLastGoodFrame = NULL;

  // next frame. Needed when backtracking happens
  PoseEstFrameEntry* nextFrame = mNextFrame;
  nextFrame = NULL;

  for (int i=mStartFrameIndex;
  i<mEndFrameIndex && mStop == false;
  i+= (nextFrame==NULL)?mFrameStep:0
  ) {
    bool newKeyFrame = reconOneFrame(i);
    if (newKeyFrame == false) {
      continue;
    }

    // only keep the last frame in the queue
    while(mActiveKeyFrames.size()>1) {
      PoseEstFrameEntry* frame = mActiveKeyFrames.front();
      mActiveKeyFrames.pop_front();
      delete frame;
    }
  }
  int numKeyFrames = mEndFrameIndex - mStartFrameIndex - mNumFramesSkipped;
  double scale = 1. / (double)((mEndFrameIndex - mStartFrameIndex));
  double kfScale = 1. / (double)(numKeyFrames);
  fprintf(stdout, "Num of frames skipped:    %d\n", mNumFramesSkipped);
  fprintf(stdout, "Total distance covered:   %05.2f mm\n",mPathLength);
  fprintf(stdout, "Total/Average keypoints:           %d,   %05.2f\n", mTotalKeypoints, (double)(mTotalKeypoints) * scale);
  fprintf(stdout, "Total/Average trackable pairs:     %d,   %05.2f\n", mTotalTrackablePairs, (double)(mTotalTrackablePairs) * scale);
  fprintf(stdout, "Total/Average inliers:             %d,   %05.2f\n", mTotalInliers, (double)(mTotalInliers) * scale);
  fprintf(stdout, "Total/Average keypoints w/o disp:  %d,   %05.2f\n", mPoseEstimator.mNumKeyPointsWithNoDisparity,
      (double)(mPoseEstimator.mNumKeyPointsWithNoDisparity) * kfScale);
  fprintf(stdout, "In Key Frames:\n");
  fprintf(stdout, "Total/Average keypoints:           %d,   %05.2f\n", mTotalKeypointsInKeyFrames, (double)(mTotalKeypointsInKeyFrames) * kfScale);
  fprintf(stdout, "Total/Average trackable pairs:     %d,   %05.2f\n", mTotalTrackablePairsInKeyFrames, (double)(mTotalTrackablePairsInKeyFrames) * kfScale);
  fprintf(stdout, "Total/Average inliers:             %d,   %05.2f\n", mTotalInliersInKeyFrames, (double)(mTotalInliersInKeyFrames) *kfScale);

  saveFramePoses(string("Output/indoor1/"));

  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = mNumFrames/mFrameStep;
  timer.printStat();

  return status;
}

// See CvPathRecon.h for documentation
CvPathRecon::PoseEstFrameEntry::PoseEstFrameEntry(WImageBuffer1_b* image,
    WImageBuffer1_16s* dispMap,
    vector<Keypoint>* keypoints, CvMat& rot, CvMat& shift,
    int numTrackablePair,
    int numInliers, int frameIndex,
    WImageBuffer3_b* imageC3a, CvMat* inliers0, CvMat* inliers1){
  mRot   = cvMat(3, 3, CV_64FC1, _mRot);
  mShift = cvMat(3, 1, CV_64FC1, _mShift);
  mImage = image;
  mDispMap = dispMap;
  mKeypoints  = keypoints;
  cvCopy(&rot,   &mRot);
  cvCopy(&shift, &mShift);
  mNumTrackablePairs = numTrackablePair;
  mNumInliers = numInliers;
  mFrameIndex = frameIndex;

  mImageC3a = imageC3a;
  mInliers0 = inliers0;
  mInliers1 = inliers1;
}

void CvPathRecon::PoseEstFrameEntry::clear() {
  if (mInliers0) cvReleaseMat(&mInliers0);
  if (mInliers1) cvReleaseMat(&mInliers1);
  delete this->mKeypoints;
  mKeypoints = NULL;
  delete this->mImage;
  mImage = NULL;
  delete this->mDispMap;
  mDispMap = NULL;
  delete mImageC3a;
  mImageC3a = NULL;
}

CvPathRecon::PoseEstFrameEntry::~PoseEstFrameEntry(){
  clear();
}
