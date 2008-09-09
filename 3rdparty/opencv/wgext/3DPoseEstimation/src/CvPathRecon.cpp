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

//#define DISPLAY

#define DEBUG
#define TIMING

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
	mTotalInliers(0),
	mTotalTrackablePairs(0),
	mTotalKeypoints(0),
	mTotalInliersInKeyFrames(0),
	mTotalTrackablePairsInKeyFrames(0),
	mTotalKeypointsInKeyFrames(0),
	mRT(cvMat(4,4, CV_64FC1, _rt)),
	_mTempMat(cvMat(4,4,CV_64FC1, _tempMat)),
	mStop(false),
	mDirname(string("Data/indoor1")),
  mLeftImageFilenameFmt(mDirname.append("/left-%04.ppm")),
  mRightImageFilenameFmt(mDirname.append("/right-%04.ppm"))
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

void CvPathRecon::keepGoodFrame(WImageBuffer1_b& image,
		WImageBuffer1_16s & dispMap, vector<Keypoint>& keyPoints, CvMat& rot, CvMat& shift,
		int numTrackablePairs, int numInliers, int frameIndex, const WImageBuffer3_b* imageC3a,
		CvMat* inliers0, CvMat* inliers1)
{
    mNumFramesSkipped++;
    delete mLastGoodFrame;
    mLastGoodFrame = new CvPathRecon::PoseEstFrameEntry(image, dispMap, keyPoints,
    		rot, shift, numTrackablePairs, numInliers, frameIndex, imageC3a, inliers0, inliers1);
    mLastGoodFrameAvailable = true;
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


bool CvPathRecon::recon(const string & dirname, const string & leftFileFmt,
    const string & rightFileFmt, int start, int end, int step)
{
  bool status = false;

  mDirname = dirname;
  mLeftImageFilenameFmt = dirname;
  mLeftImageFilenameFmt += leftFileFmt;
  mRightImageFilenameFmt = dirname;
  mRightImageFilenameFmt += rightFileFmt;

  double ransacInlierthreshold = 2.0;
  int numRansacIterations = 200;

  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm
  mPoseEstimator.setInlierErrorThreshold(ransacInlierthreshold);
  mPoseEstimator.setNumRansacIterations(numRansacIterations);

  vector<Keypoint>& keyPointsLast = mKeyPointsLast;

  WImageBuffer1_b&   lastLeftImage = mLastKeyFrameImage;
  WImageBuffer1_16s& lastDispMap   = mLastKeyFrameDispMap;

  int maxDisp = (int)mPoseEstimator.getD(400); // the closest point we care is at least 1000 mm away
  cout << "Max disparity is: "<< maxDisp<<endl;

  mErrMeas.setCameraParams((const CvStereoCamParams&)mPoseEstimator);

  mStartFrameIndex = start;
  mNumFrames       = end - start;
  mEndFrameIndex   = end;
  mFrameStep       = step;

  // current transformation w.r.t. to the current frame

  bool& reversed = mReversed;

  CvMat& rot   = mRot;
  CvMat& shift = mShift;
  CvSize& imgSize = mPoseEstimator.getSize();

  int fBackTracked = false;

  WImageBuffer1_16s dispMap(imgSize.width, imgSize.height);
  WImageBuffer1_b leftImage;
  WImageBuffer1_b rightImage;
#ifdef DISPLAY
  WImageBuffer3_b leftImageC3(imgSize.width, imgSize.height);
  WImageBuffer3_b leftImageC3a(imgSize.width, imgSize.height);
  IplImage *leftimgC3a = leftImageC3a.Ipl();
  IplImage *leftimgC3  = leftImageC3.Ipl();
#endif
  vector<Keypoint>& keyPointsCurr = mKeyPointsCurr;

  char inliersFilename[256];

  for (int i=mStartFrameIndex;
    i<mEndFrameIndex && mStop == false;
    i+= (fBackTracked==false)?mFrameStep:0
  ) {
    int frameIndex=i;
//    fBackTracked = false;

    if (fBackTracked==true){
      // do not need to load the image or compute the key points again
#ifdef DISPLAY
      leftimgC3a = leftImageC3a.Ipl();
#endif
      fBackTracked = false;
    } else {
      loadStereoImagePair(frameIndex, leftImage, rightImage);
      mPoseEstimator.getDisparityMap(leftImage, rightImage, dispMap);
      keyPointsCurr = mPoseEstimator.goodFeaturesToTrack(leftImage, &dispMap);
      mTotalKeypoints += keyPointsCurr.size();
#ifdef DEBUG
      cout << "Found " << keyPointsCurr.size() << " good features in left  image" << endl;
#endif

#ifdef DISPLAY
      cvCvtColor(leftImage.Ipl(),  leftimgC3,  CV_GRAY2RGB);
      leftimgC3a = leftImageC3a.Ipl();
      cvCvtColor(leftImage.Ipl(),  leftimgC3a, CV_GRAY2RGB);

      showDisparityMap(dispMap, dispWindowName, outputDirname, frameIndex, maxDisp);
      drawKeypoints(leftImageC3, keyPointsLast, keyPointsCurr);
#endif
    }
    //
    // match the good feature points between this iteration and last
    //
    vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;

    mPoseEstimator.getTrackablePairs(lastLeftImage, leftImage,
        lastDispMap, dispMap, keyPointsLast, keyPointsCurr, trackablePairs);
    mTotalTrackablePairs += trackablePairs.size();

#ifdef DEBUG
    cout << "Num of trackable pairs for pose estimate: "<<trackablePairs.size() <<endl;
#endif

    int numTrackablePairs = trackablePairs.size();

    if (numTrackablePairs<10) {
#ifdef DEBUG
      cout << "Too few trackable pairs" <<endl;
#endif
#ifdef DISPLAY
      sprintf(info, "%04d, #TrackablePair: %d, Too few to track",
          i, numTrackablePairs);
#endif
    } else {
#ifdef DISPLAY
      // Draw all the trackable pairs
      this->drawTrackablePairs(leftImageC3, trackablePairs);
#endif
      //  pose estimation given the feature point pairs
      int numInliers =
        mPoseEstimator.estimate(trackablePairs, rot, shift, reversed);

      mTotalInliers += numInliers;

      CvMat *inliers0 = NULL;
      CvMat *inliers1 = NULL;
      getInliers(inliers0, inliers1);
#ifdef DEBUG
      cout << "num of inliers: "<< numInliers <<endl;
#endif

      CvPathRecon::KeyFramingDecision kfd =
        keyFrameEval(i, trackablePairs, keyPointsCurr, numInliers, inliers0, inliers1, rot, shift);

      switch (kfd) {
      case CvPathRecon::KeyFrameSkip:  {
        // skip this frame
        mNumFramesSkipped++;
        break;
      }
      case CvPathRecon::KeyFrameBackTrack:   {
        // go back to the last good frame
        fBackTracked = true;
#ifdef DEBUG
        cerr << "Going back to last good frame  from frame "<<i<<endl;
#endif
        assert(mLastGoodFrame != NULL);

        cerr << "Last good frame is "<<mLastGoodFrame->mFrameIndex << endl;
        mNumFramesSkipped--;
        // show the inlier
        inliers0 = mLastGoodFrame->mInliers0;
        inliers1 = mLastGoodFrame->mInliers1;
        cvCopy(&mLastGoodFrame->mRot, &rot);
        cvCopy(&mLastGoodFrame->mShift, &shift);
        frameIndex = mLastGoodFrame->mFrameIndex;
        numTrackablePairs = mLastGoodFrame->mNumTrackablePairs;
        numInliers = mLastGoodFrame->mNumInliers;
#ifdef DISPLAY
        leftimgC3a = mLastGoodFrame->mImageC3a.Ipl();
#endif

        // keep track of the trajectory
        appendTransform(rot, shift);
        mTotalInliersInKeyFrames += numInliers;
        mTotalKeypointsInKeyFrames += mLastGoodFrame->mKeypoints.size();
        mTotalTrackablePairsInKeyFrames += mLastGoodFrame->mNumTrackablePairs;

        // save the inliers into a file
        sprintf(inliersFilename, "Output/indoor1/inliers1_%04d.xml", frameIndex);
        saveKeyPoints(*inliers1, string(inliersFilename));

        // stores rotation mat and shift vector in rods and shifts
        storeTransform(rot, shift, frameIndex - mStartFrameIndex);


        CvMatUtils::drawMatchingPairs(*inliers0, *inliers1, mLastGoodFrame->mImageC3a,
            rot, shift,
            (Cv3DPoseEstimateDisp&)mPoseEstimator, reversed);

        // measure the errors
        measureErr(inliers0, inliers1);

#ifdef DISPLAY
        cvShowImage(lastTrackedLeftCam.c_str(), lastLeftImage.Ipl());
#endif

        // getting ready for next key frame
        // TODO: shall replace the next 3 lines with more efficient implementation
        keyPointsLast = mLastGoodFrame->mKeypoints;
        lastDispMap.CloneFrom(mLastGoodFrame->mDispMap);
        lastLeftImage.CloneFrom(mLastGoodFrame->mImage);

        // next we are supposed to try current image, frame i again with
        // last good frame
        mLastGoodFrameAvailable = false;

        break;
      }
      case CvPathRecon::KeyFrameKeep:   {
        int numTrackablePairs = trackablePairs.size();
#ifdef DISPLAY
        keepGoodFrame(leftImage, dispMap, keyPointsCurr, rot, shift,
            numTrackablePairs, numInliers, frameIndex, &leftImageC3a, inliers0, inliers1);
#else
        keepGoodFrame(leftImage, dispMap, keyPointsCurr, rot, shift,
                    numTrackablePairs, numInliers, frameIndex, NULL, inliers0, inliers1);
#endif
        break;
      }
      case CvPathRecon::KeyFrameUse:  {
        // show the inliers
        frameIndex = i;

        // keep track of the trajectory
        appendTransform(rot, shift);
        mTotalInliersInKeyFrames   += numInliers;
        mTotalKeypointsInKeyFrames += keyPointsCurr.size();
        mTotalTrackablePairsInKeyFrames += trackablePairs.size();

#ifdef DISPLAY
        // save the inliers into a file
        sprintf(inliersFilename, "Output/indoor1/inliers1_%04d.xml", frameIndex);
        saveKeyPoints(*inliers1, string(inliersFilename));
#endif

        // stores rotation mat and shift vector in rods and shifts
        storeTransform(rot, shift, frameIndex - mStartFrameIndex);

#ifdef DISPLAY
        CvMatUtils::drawMatchingPairs(*inliers0, *inliers1, leftImageC3a,
            rot, shift, (Cv3DPoseEstimateDisp&)mPoseEstimator, reversed);
#endif

#ifdef DEBUG
        // measure the errors
        measureErr(inliers0, inliers1);
#endif
        // getting ready for next key frame
        keyPointsLast = keyPointsCurr;
        lastDispMap.CloneFrom(dispMap);
#ifdef DISPLAY
        cvShowImage(lastTrackedLeftCam.c_str(), lastLeftImage.Ipl());
#endif
        lastLeftImage.CloneFrom(leftImage);

        break;
      }
      default:
        break;
      }

#ifdef DISPLAY
      CvPoint3D64f euler;
      CvMatUtils::eulerAngle(rot, euler);
      sprintf(info, "%04d, KyPt %d, TrckPir %d, Inlrs %d, eulr=(%4.2f,%4.2f,%4.2f), d=%4.1f",
          frameIndex, keyPointsCurr.size(), numTrackablePairs, numInliers, euler.x, euler.y, euler.z, cvNorm((const CvMat *)&shift));
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
    sprintf(rightCamWithMarks, "%s/rightCamwithMarks-%04d.png", outputDirname.c_str(), frameIndex);
    sprintf(poseEstFilename, "%s/poseEst-%04d.png", outputDirname.c_str(), frameIndex);
    cvSaveImage(leftCamWithMarks,  leftimgC3);
    cvSaveImage(poseEstFilename,   leftimgC3a);


    // wait for a while for opencv to draw stuff on screen
    cvWaitKey(25);  //  milliseconds
//    cvWaitKey(0);  //  wait indefinitely
#endif

    //
    //  getting ready for next iteration
    //

    if (i==mStartFrameIndex) {
      keyPointsLast = keyPointsCurr;
      lastDispMap.CloneFrom(dispMap);
      lastLeftImage.CloneFrom(leftImage);
    }
  }

  int numKeyFrames = mEndFrameIndex - mStartFrameIndex - mNumFramesSkipped;
  double scale   = 1./(double)(mEndFrameIndex - mStartFrameIndex);
  double kfScale = 1./(double)numKeyFrames;
  cout <<"Num of frames skipped: " << mNumFramesSkipped <<endl;
  cout <<"Total distance covered: "<< mPathLength <<" mm"<<endl;
  cout <<"Total/Average keypoints: "<< mTotalKeypoints << ","<< (double)mTotalKeypoints*scale <<endl;
  cout <<"Total/Average trackable pairs: "<< mTotalTrackablePairs << ","<< (double)mTotalTrackablePairs*scale <<endl;
  cout <<"Total/Average inliers: "<< mTotalInliers << ","<< (double)mTotalInliers*scale <<endl;
  cout << "Total/Average keypoints with no disparity: "<< mPoseEstimator.mNumKeyPointsWithNoDisparity <<","<<
  (double)mPoseEstimator.mNumKeyPointsWithNoDisparity*kfScale <<endl;

  cout << "In Key Frames: "<<endl;
  cout <<"Total/Average keypoints: "<< mTotalKeypointsInKeyFrames << ","<< (double)mTotalKeypointsInKeyFrames*kfScale <<endl;
  cout <<"Total/Average trackable pairs: "<< mTotalTrackablePairsInKeyFrames << ","<< (double)mTotalTrackablePairsInKeyFrames*kfScale <<endl;
  cout <<"Total/Average inliers: "<< mTotalInliersInKeyFrames << ","<< (double)mTotalInliersInKeyFrames*kfScale <<endl;

  saveFramePoses(string("Output/indoor1/"));

  return status;
}
