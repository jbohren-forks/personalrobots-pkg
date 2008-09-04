#include "CvTest3DPoseEstimate.h"

#include <iostream>
#include <vector>
#include <queue>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
// the google wrapper
#include <opencv/cvwimage.h>

#include <opencv/cxcore.hpp>

#include <stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily

// WG Ext of OpenCV
#include <CvPoseEstErrMeasDisp.h>
#include <Cv3DPoseEstimateStereo.h>
#include "Cv3DPoseEstimateDisp.h"
#include "Cv3DPoseEstimate.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"
#include <CvPathRecon.h>

// VTK headers
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>


// star detector
//#include <detector.h>

using namespace cv;
using namespace std;

#define GAUSSIANNOISE

#define ShowTemplateMatching 0

#define DEBUG 1

CvTest3DPoseEstimate::CvTest3DPoseEstimate():
	Parent(),
    mRng(cvRNG(time(NULL))),
    mDisturbScale(0.001),
    mOutlierScale(100.0),
    mOutlierPercentage(0.0),
    mStop(false)
{
	_init();
};
CvTest3DPoseEstimate::CvTest3DPoseEstimate(double Fx, double Fy, double Tx, double Clx, double Crx, double Cy):
    Parent(Fx, Fy, Tx, Clx, Crx, Cy),
    mRng(cvRNG(time(NULL))),
    mDisturbScale(0.001),
    mOutlierScale(100.0),
    mOutlierPercentage(0.0),
    mStop(false)
{
	_init();
}

CvTest3DPoseEstimate::~CvTest3DPoseEstimate()
{
}

void CvTest3DPoseEstimate::_init(){
	cvInitMatHeader(&mRot,   3, 3, CV_64FC1, mRotData);
	cvInitMatHeader(&mTrans, 3, 1, CV_64FC1, mTransData);

#if 0 // TODO: to be removed
	mMinNumInliersForGoodFrame  = defMinNumInliersForGoodFrame;
	mMinNumInliers  = defMinNumInliers;
	mMinInlierRatio = defMinInlierRatio;
	mMaxAngleAlpha  = defMaxAngleAlpha;
	mMaxAngleBeta   = defMaxAngleBeta;
	mMaxAngleGamma  = defMaxAngleGamma;
	mMaxShift       = defMaxShift;

//	mMinNumInliers  = 40;
	mMaxAngleAlpha  = 15;
	mMaxAngleBeta   = 15;
	mMaxAngleGamma  = 15;
	mMaxShift       = 300;
#endif
}

/**
 *  points0  - numPoints x 3
 *  points1  - numPoints x 3
 */
void CvTest3DPoseEstimate::transform(CvMat *points0, CvMat *points1) {
	// create a translated/rotated copy of the 3D points
    // the euler angle of the rotation matrix is given in z-y-x form, aka roll-yaw-pitch, Tait-Bryan angles
	double roll, pitch, yaw;
	pitch = mEulerAngle.x;
	yaw   = mEulerAngle.y;
	roll  = mEulerAngle.z;

#if 0 // ZYX
	CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::ZYX);
	cout << "Rotation Matrix:= Pitch * Yaw* Roll"<< endl;
#else // XYZ euler angle
	CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::XYZ);
	cout << "Rotation Matrix:= Roll * Yaw * Pitch: "<< mEulerAngle.x/CV_PI*180. << ","<<mEulerAngle.y/CV_PI*180.<<","<<mEulerAngle.z/CV_PI*180.<< endl;
#endif
#if 1
	CvMatUtils::printMat(&mRot);
#endif

	cvmSet(&mTrans, 0, 0, mTranslation.x);
	cvmSet(&mTrans, 1, 0, mTranslation.y);
	cvmSet(&mTrans, 2, 0, mTranslation.z);

#if 1
	cout << "Translation Matrix: "<< endl;
	CvMatUtils::printMat(&mTrans);
#endif

	// R * transpose(points0) + T
	transform(&mRot, points0, &mTrans, points1);
}

void CvTest3DPoseEstimate::transform(CvMat* R, CvMat *points0, CvMat* T, CvMat *points1) {
	int numPoints = points0->rows;
	CvMat * points1t =  cvCreateMat(3,  numPoints, CV_64FC1);

	CvMat *Temp = cvCreateMat(3, points0->rows, CV_64FC1);
	cvRepeat(T, Temp);
	cvGEMM(R, points0, 1.0, Temp, 1.0, points1t, CV_GEMM_B_T);
	cvTranspose(points1t, points1);

#if 0
	cout << "original points: " << endl;
	CvMatUtils::printMat(points0);
	cout << "transformed points: "<<endl;
	CvMatUtils::printMat(points1);
	cout << "Rot and Trans: "<< endl;
	CvMatUtils::printMat(R);
	CvMatUtils::printMat(T);
#endif
}

double CvTest3DPoseEstimate::randReal(double min, double max) {
	double r = cvRandReal(&mRng);
	r = r*(max-min) + min;
	return r;
}

void CvTest3DPoseEstimate::disturb(const CvMat *xyzs, CvMat* xyzsNoised) {
#ifdef GAUSSIANNOISE
//	CvMat* Noise = cvCreateMat(xyzs->rows, xyzs->cols, xyzs->type);
	double sigma = mDisturbScale/2.0; // ~ 95%
	cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
	cvAdd(xyzs, xyzsNoised, xyzsNoised);
//	cvReleaseMat(&Noise);
#else
	double s = mDisturbScale;
	if (s<=0.0)
		return;
	for (int i=0; i<xyzs->rows; i++){
		cvSetReal2D(xyzs, i, 0, cvGetReal2D(xyzs, i, 0)+randReal(-s, s));
		cvSetReal2D(xyzs, i, 1, cvGetReal2D(xyzs, i, 1)+randReal(-s, s));
		cvSetReal2D(xyzs, i, 2, cvGetReal2D(xyzs, i, 2)+randReal(-s, s));
	}
#endif
}

void CvTest3DPoseEstimate::randomize(CvMat *xyzs, int num, double maxVal){
	// fill in the random number in (-maxVal and maxVal)
	double r;
	for (int i=0; i<num; i++){
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 0, r);
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 1, r);
		r = randReal(-maxVal, maxVal);
		cvSetReal2D(xyzs, i, 2, r);
	}
}

bool CvTest3DPoseEstimate::test() {
	switch (this->mTestType) {
	case Cartesian:
	case Disparity:
	case CartAndDisp:
		return testPointClouds();
		break;
	case Video:
		return testVideos();
		break;
	default:
		cout << "Unknown test type: "<<  mTestType << endl;
	}
	return false;
}

void CvTest3DPoseEstimate::MyMouseCallback(int event, int x, int y, int flagsm, void* param){
	CvTest3DPoseEstimate *pe = (CvTest3DPoseEstimate *)param;
	switch(event) {
	case CV_EVENT_LBUTTONDBLCLK:
		if (flagsm & CV_EVENT_FLAG_CTRLKEY) {
			pe->mStop = true;
		}
		break;
	default:
		;
	}
}
#if 0
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
#endif

/** display disparity map */
bool CvTest3DPoseEstimate::showDisparityMap(WImageBuffer1_16s& dispMap, string& dispWindowName,
		string& outputDirname, int frameIndex, int maxDisp){
	char dispMapFilename[PATH_MAX];
	sprintf(dispMapFilename, "%s/dispMap-%04d.png", outputDirname.c_str(), frameIndex);
	string _dispMapFilename(dispMapFilename);
	CvMatUtils::showDisparityMap(dispMap, dispWindowName, _dispMapFilename, maxDisp);
	return true;
}

bool CvTest3DPoseEstimate::drawKeypoints(WImage3_b& image, vector<Keypoint>& keyPointsLast,
		vector<Keypoint>& keyPointsCurr){
	// draw the key points
	IplImage* img = image.Ipl();
	for (vector<Keypoint>::const_iterator ikp = keyPointsCurr.begin(); ikp != keyPointsCurr.end(); ikp++) {
		cvCircle(img, cvPoint(ikp->x, ikp->y), 4, CvMatUtils::green, 1, CV_AA, 0);
	}
	// draw the key points from last key frame
	for (vector<Keypoint>::const_iterator ikp = keyPointsLast.begin(); ikp != keyPointsLast.end(); ikp++) {
		// draw cross instead of circle
		CvMatUtils::cvCross(img, cvPoint(ikp->x, ikp->y), 4, CvMatUtils::yellow, 1, CV_AA, 0);
	}
	return true;
}

bool CvTest3DPoseEstimate::drawTrackablePairs(
		WImage3_b& image,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs){
	for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = trackablePairs.begin();
	iter != trackablePairs.end(); iter++) {
		const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
		CvPoint p0 = cvPoint(p.first.x, p.first.y);
		CvPoint p1 = cvPoint(p.second.x, p.second.y);
		int thickness =1;
		cvLine(image.Ipl(), p0, p1, CvMatUtils::red, thickness, CV_AA);
	}
	return true;
}

// the following function is a clean-up version of testVideos(), which serves as
// an intermediate form for re-factorization of  code into pose estimate or 3d reconstruction
void CvTest3DPoseEstimate::loadStereoImagePair(string & dirname, int & frameIndex, WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage)
{
    char leftfilename[PATH_MAX];
    char rightfilename[PATH_MAX];
    sprintf(leftfilename, "%s/left-%04d.ppm", dirname.c_str(), frameIndex);
    sprintf(rightfilename, "%s/right-%04d.ppm", dirname.c_str(), frameIndex);
    cout << "loading " << leftfilename << " and " << rightfilename << endl;
    IplImage* leftimg  = cvLoadImage(leftfilename,  CV_LOAD_IMAGE_GRAYSCALE);
    leftImage.SetIpl(leftimg);
    IplImage* rightimg = cvLoadImage(rightfilename, CV_LOAD_IMAGE_GRAYSCALE);
    rightImage.SetIpl(rightimg);
}

bool CvTest3DPoseEstimate::testVideos() {
	bool status = false;
	int numImages = 1509;
	double ransacInlierthreshold = 2.0;
	int numRansacIterations = 200;
//	int numImages = 1;
	char leftCamWithMarks[PATH_MAX];
	char rightCamWithMarks[PATH_MAX];
	char poseEstFilename[PATH_MAX];


	CvSize imgSize = cvSize(640, 480);
	CvPathRecon pathRecon(imgSize);

	// The following parameters are from indoor1/proj.txt
	// note that B (or Tx) is in mm
	this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
	pathRecon.mPoseEstimator.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);
	pathRecon.mPoseEstimator.setInlierErrorThreshold(ransacInlierthreshold);
	pathRecon.mPoseEstimator.setNumRansacIterations(numRansacIterations);

	string poseEstWinName = string("Pose Estimated");
	string leftCamWinName = string("Left  Cam");
	string lastTrackedLeftCam = string("Last Tracked Left Cam");
	string dispWindowName = string("Disparity Map");


	// create a list of windows to display results
	cvNamedWindow(poseEstWinName.c_str(), CV_WINDOW_AUTOSIZE);
	cvNamedWindow(leftCamWinName.c_str(), CV_WINDOW_AUTOSIZE);
	cvNamedWindow(dispWindowName.c_str(), CV_WINDOW_AUTOSIZE);
	cvNamedWindow(lastTrackedLeftCam.c_str(), CV_WINDOW_AUTOSIZE);

	cvMoveWindow(poseEstWinName.c_str(), 0, 0);
	cvMoveWindow(leftCamWinName.c_str(), 650, 0);
	cvMoveWindow(dispWindowName.c_str(), 650, 530);
	cvMoveWindow(lastTrackedLeftCam.c_str(), 0, 530);

	cvSetMouseCallback(leftCamWinName.c_str(), MyMouseCallback, (void*)this);

	// input and output directory
	string dirname       = "Data/indoor1";
	string outputDirname = "Output/indoor1";

	vector<Keypoint>& keyPointsLast = pathRecon.mKeyPointsLast;

	WImageBuffer1_b&   lastLeftImage = pathRecon.mLastKeyFrameImage;
	WImageBuffer1_16s& lastDispMap   = pathRecon.mLastKeyFrameDispMap;

	int maxDisp = (int)pathRecon.mPoseEstimator.getD(400); // the closest point we care is at least 1000 mm away
	cout << "Max disparity is: "<< maxDisp<<endl;

	pathRecon.mErrMeas.setParams(mFx, mFy, mTx, mClx, mCrx, mCy);

	pathRecon.mStartFrameIndex = 0;
	pathRecon.mNumFrames       = numImages;
	pathRecon.mEndFrameIndex   = numImages;
	pathRecon.mFrameStep       = 1;

	// current transformation w.r.t. to the current frame

	bool& reversed = pathRecon.mReversed;

	// prepare for the text in the pose estimate window
	char info[256];
	CvPoint org = cvPoint(0, 475);
	CvFont font;
	cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, .5, .4);

	CvMat& rot   = pathRecon.mRot;
	CvMat& shift = pathRecon.mShift;


	int fBackTracked = false;

	WImageBuffer1_16s dispMap(imgSize.width, imgSize.height);
	WImageBuffer1_b leftImage;
	WImageBuffer1_b rightImage;
	WImageBuffer3_b leftImageC3(imgSize.width, imgSize.height);
	WImageBuffer3_b leftImageC3a(imgSize.width, imgSize.height);
	IplImage *leftimgC3a = leftImageC3a.Ipl();
	IplImage *leftimgC3  = leftImageC3.Ipl();
	vector<Keypoint>& keyPointsCurr = pathRecon.mKeyPointsCurr;

	char inliersFilename[256];

	for (int i=pathRecon.mStartFrameIndex;
		i<pathRecon.mEndFrameIndex && mStop == false;
		i+= (fBackTracked==false)?pathRecon.mFrameStep:0
	) {
		int frameIndex=i;
//		fBackTracked = false;

		if (fBackTracked==true){
			// do not need to load the image or compute the key points again
			leftimgC3a = leftImageC3a.Ipl();
			fBackTracked = false;
		} else {
			loadStereoImagePair(dirname, frameIndex, leftImage, rightImage);
			pathRecon.mPoseEstimator.getDisparityMap(leftImage, rightImage, dispMap);
			keyPointsCurr = pathRecon.mPoseEstimator.goodFeaturesToTrack(leftImage, &dispMap);
			pathRecon.mTotalKeypoints += keyPointsCurr.size();
			cout << "Found " << keyPointsCurr.size() << " good features in left  image" << endl;
			cvCvtColor(leftImage.Ipl(),  leftimgC3,  CV_GRAY2RGB);
			leftimgC3a = leftImageC3a.Ipl();
			cvCvtColor(leftImage.Ipl(),  leftimgC3a, CV_GRAY2RGB);

			showDisparityMap(dispMap, dispWindowName, outputDirname, frameIndex, maxDisp);
			drawKeypoints(leftImageC3, keyPointsLast, keyPointsCurr);
		}
		//
		// match the good feature points between this iteration and last
		//
		vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs =
			pathRecon.mPoseEstimator.getTrackablePairs(lastLeftImage, leftImage, lastDispMap, dispMap, keyPointsLast, keyPointsCurr);
		pathRecon.mTotalTrackablePairs += trackablePairs.size();

		cout << "Num of trackable pairs for pose estimate: "<<trackablePairs.size() <<endl;

		int numTrackablePairs = trackablePairs.size();

		if (numTrackablePairs<10) {
			cout << "Too few trackable pairs" <<endl;
			sprintf(info, "%04d, #TrackablePair: %d, Too few to track",
					i, numTrackablePairs);
		} else {

			// Draw all the trackable pairs
			this->drawTrackablePairs(leftImageC3, trackablePairs);

			//  pose estimation given the feature point pairs
			int numInliers =
				pathRecon.mPoseEstimator.estimate(trackablePairs, rot, shift, reversed);

			pathRecon.mTotalInliers += numInliers;

			CvMat *inliers0 = NULL;
			CvMat *inliers1 = NULL;
			pathRecon.getInliers(inliers0, inliers1);

			cout << "num of inliers: "<< numInliers <<endl;

			CvPathRecon::KeyFramingDecision kfd =
				pathRecon.keyFrameEval(i, trackablePairs, keyPointsCurr, numInliers, inliers0, inliers1, rot, shift);

			switch (kfd) {
			case CvPathRecon::KeyFrameSkip:	{
				// skip this frame
				pathRecon.mNumFramesSkipped++;
				break;
			}
			case CvPathRecon::KeyFrameBackTrack: 	{
				// go back to the last good frame
				fBackTracked = true;
#if DEBUG==1
				cerr << "Going back to last good frame  from frame "<<i<<endl;
#endif
				assert(pathRecon.mLastGoodFrame != NULL);

				cerr << "Last good frame is "<<pathRecon.mLastGoodFrame->mFrameIndex << endl;
				pathRecon.mNumFramesSkipped--;
				// show the inlier
				inliers0 = pathRecon.mLastGoodFrame->mInliers0;
				inliers1 = pathRecon.mLastGoodFrame->mInliers1;
				cvCopy(&pathRecon.mLastGoodFrame->mRot, &rot);
				cvCopy(&pathRecon.mLastGoodFrame->mShift, &shift);
				frameIndex = pathRecon.mLastGoodFrame->mFrameIndex;
				numTrackablePairs = pathRecon.mLastGoodFrame->mNumTrackablePairs;
				numInliers = pathRecon.mLastGoodFrame->mNumInliers;
				leftimgC3a = pathRecon.mLastGoodFrame->mImageC3a.Ipl();

				// keep track of the trajectory
				pathRecon.appendTransform(rot, shift);
				pathRecon.mTotalInliersInKeyFrames += numInliers;
				pathRecon.mTotalKeypointsInKeyFrames += pathRecon.mLastGoodFrame->mKeypoints.size();
				pathRecon.mTotalTrackablePairsInKeyFrames += pathRecon.mLastGoodFrame->mNumTrackablePairs;

				// save the inliers into a file
				sprintf(inliersFilename, "Output/indoor1/inliers1_%04d.xml", frameIndex);
				pathRecon.saveKeyPoints(*inliers1, string(inliersFilename));

				// stores rotation mat and shift vector in rods and shifts
				pathRecon.storeTransform(rot, shift, frameIndex - pathRecon.mStartFrameIndex);


				CvMatUtils::drawMatchingPairs(*inliers0, *inliers1, pathRecon.mLastGoodFrame->mImageC3a,
						rot, shift,
						(Cv3DPoseEstimateDisp&)pathRecon.mPoseEstimator, reversed);

				// measure the errors
				pathRecon.measureErr(inliers0, inliers1);

				cvShowImage(lastTrackedLeftCam.c_str(), lastLeftImage.Ipl());

				// getting ready for next key frame
				// TODO: shall replace the next 3 lines with more efficient implementation
				keyPointsLast = pathRecon.mLastGoodFrame->mKeypoints;
				lastDispMap.CloneFrom(pathRecon.mLastGoodFrame->mDispMap);
				lastLeftImage.CloneFrom(pathRecon.mLastGoodFrame->mImage);

				// next we are supposed to try current image, frame i again with
				// last good frame
				pathRecon.mLastGoodFrameAvailable = false;

				break;
			}
			case CvPathRecon::KeyFrameKeep: 	{
				int numTrackablePairs = trackablePairs.size();
				pathRecon.keepGoodFrame(leftImage, dispMap, keyPointsCurr, rot, shift,
						numTrackablePairs, numInliers, frameIndex, leftImageC3a, inliers0, inliers1);
				break;
			}
			case CvPathRecon::KeyFrameUse:	{
				// show the inliers
				frameIndex = i;

				// keep track of the trajectory
				pathRecon.appendTransform(rot, shift);
				pathRecon.mTotalInliersInKeyFrames   += numInliers;
				pathRecon.mTotalKeypointsInKeyFrames += keyPointsCurr.size();
				pathRecon.mTotalTrackablePairsInKeyFrames += trackablePairs.size();

				// save the inliers into a file
				sprintf(inliersFilename, "Output/indoor1/inliers1_%04d.xml", frameIndex);
				pathRecon.saveKeyPoints(*inliers1, string(inliersFilename));

				// stores rotation mat and shift vector in rods and shifts
				pathRecon.storeTransform(rot, shift, frameIndex - pathRecon.mStartFrameIndex);

				CvMatUtils::drawMatchingPairs(*inliers0, *inliers1, leftImageC3a,
						rot, shift, (Cv3DPoseEstimateDisp&)pathRecon.mPoseEstimator, reversed);

				// measure the errors
				pathRecon.measureErr(inliers0, inliers1);
				// getting ready for next key frame
				keyPointsLast = keyPointsCurr;
				lastDispMap.CloneFrom(dispMap);

				cvShowImage(lastTrackedLeftCam.c_str(), lastLeftImage.Ipl());
				lastLeftImage.CloneFrom(leftImage);

				break;
			}
			default:
				break;
			}

			CvPoint3D64f euler;
			CvMatUtils::eulerAngle(rot, euler);
			sprintf(info, "%04d, KyPt %d, TrckPir %d, Inlrs %d, eulr=(%4.2f,%4.2f,%4.2f), d=%4.1f",
					frameIndex, keyPointsCurr.size(), numTrackablePairs, numInliers, euler.x, euler.y, euler.z, cvNorm((const CvMat *)&shift));

		}

		cvPutText(leftimgC3a, info, org, &font, CvMatUtils::yellow);
		cvShowImage(poseEstWinName.c_str(), leftimgC3a);
		cvShowImage(leftCamWinName.c_str(), leftimgC3);

		// save the marked images
#if 1
		sprintf(leftCamWithMarks, "%s/leftCamWithMarks-%04d.png", outputDirname.c_str(), frameIndex);
		sprintf(rightCamWithMarks, "%s/rightCamwithMarks-%04d.png", outputDirname.c_str(), frameIndex);
		sprintf(poseEstFilename, "%s/poseEst-%04d.png", outputDirname.c_str(), frameIndex);
		cvSaveImage(leftCamWithMarks,  leftimgC3);
		cvSaveImage(poseEstFilename,   leftimgC3a);
#endif


		// wait for a while for opencv to draw stuff on screen
		cvWaitKey(25);  //  milliseconds
//		cvWaitKey(0);  //  wait indefinitely

		//
		//  getting ready for next iteration
		//

		if (i==pathRecon.mStartFrameIndex) {
			keyPointsLast = keyPointsCurr;
			lastDispMap.CloneFrom(dispMap);
			lastLeftImage.CloneFrom(leftImage);
		}
	}

	int numKeyFrames = pathRecon.mEndFrameIndex - pathRecon.mStartFrameIndex - pathRecon.mNumFramesSkipped;
	double scale   = 1./(double)(pathRecon.mEndFrameIndex - pathRecon.mStartFrameIndex);
	double kfScale = 1./(double)numKeyFrames;
	cout <<"Num of frames skipped: " << pathRecon.mNumFramesSkipped <<endl;
	cout <<"Total distance covered: "<< pathRecon.mPathLength <<" mm"<<endl;
	cout <<"Total/Average keypoints: "<< pathRecon.mTotalKeypoints << ","<< (double)pathRecon.mTotalKeypoints*scale <<endl;
	cout <<"Total/Average trackable pairs: "<< pathRecon.mTotalTrackablePairs << ","<< (double)pathRecon.mTotalTrackablePairs*scale <<endl;
	cout <<"Total/Average inliers: "<< pathRecon.mTotalInliers << ","<< (double)pathRecon.mTotalInliers*scale <<endl;
	cout << "Total/Average keypoints with no disparity: "<< pathRecon.mPoseEstimator.mNumKeyPointsWithNoDisparity <<","<<
	(double)pathRecon.mPoseEstimator.mNumKeyPointsWithNoDisparity*kfScale <<endl;

	cout << "In Key Frames: "<<endl;
	cout <<"Total/Average keypoints: "<< pathRecon.mTotalKeypointsInKeyFrames << ","<< (double)pathRecon.mTotalKeypointsInKeyFrames*kfScale <<endl;
	cout <<"Total/Average trackable pairs: "<< pathRecon.mTotalTrackablePairsInKeyFrames << ","<< (double)pathRecon.mTotalTrackablePairsInKeyFrames*kfScale <<endl;
	cout <<"Total/Average inliers: "<< pathRecon.mTotalInliersInKeyFrames << ","<< (double)pathRecon.mTotalInliersInKeyFrames*kfScale <<endl;

	pathRecon.saveFramePoses(string("Output/indoor1/"));

	return status;
}

bool CvTest3DPoseEstimate::testPointClouds(){
    bool status = true;
	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped_2000_adjusted.xml");

#if 0
	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped_2000.xml");
	int numPoints = points0->rows;
	// adjusting the data set
	// turn it from meters to mm
	cvScale(points0, points0, 1000.0);
#if 0
	double _v[3] = {0, 100., 100.};
	CvMat v = cvMat(1, 3, CV_64F, _v);
	double _Temp[3*points0->rows];
	CvMat Temp = cvMat(points0->rows, 3, CV_64F, _Temp);
	cvRepeat(&v, &Temp);
	cvAdd(points0, &Temp, points0);
#endif
	// a rotation matrix to turn from x forward, z up to
	// z forward and y down
	double _X2Z[9] = {
			0, -1,  0,
			0,  0, -1,
			1,  0,  0
	};
	CvMat X2Z = cvMat(3, 3, CV_64F, _X2Z);
	CvMat *points00 = cvCreateMat(numPoints, 3, CV_64FC1);
	cvGEMM(points0, &X2Z, 1.0, NULL, 0, points00, CV_GEMM_B_T);
	cvReleaseMat(&points0);
	points0 = points00;
//	cvMatMul(&X2Z, points0, points00);
	cvSave("Data/obj1_cropped_2000_adjusted.xml", points0, "adjusted", "...so that it is more like real data");
#endif
//	CvMat * points0 =  (CvMat *)cvLoad("Data/3dPointClouds0.xml");
//	CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped.xml");
	int numPoints = points0->rows;


	CvMat * points1   = cvCreateMat(numPoints,  3, CV_64FC1);
	CvMat * points1d  = cvCreateMat(numPoints,  3, CV_64FC1); // to hold the disturbed version of points1
	CvMat * points1r  = cvCreateMat(numPoints,  3, CV_64FC1);

	mEulerAngle.x = CV_PI/4.;
	mEulerAngle.y = CV_PI/3.;
	mEulerAngle.z = CV_PI/6;

	mTranslation.x = 100.;
	mTranslation.y = 10.0;
	mTranslation.z = 50.0;

	CvMat *rot    = cvCreateMat(3, 3, CV_64FC1);
	CvMat *trans  = cvCreateMat(3, 1, CV_64FC1);

//	Cv3DPoseEstimateRef peCart;
	Cv3DPoseEstimate peCart;
//	Cv3DPoseEstimateDispSpaceRef peDisp;
	Cv3DPoseEstimateDisp peDisp;
	CvMat *uvds0=NULL, *uvds1=NULL;
	CvScalar mean, std;

	if (this->mTestType == Cartesian) {
		cout << "Testing in Cartesian Space"<<endl;
		cvAvgSdv(points0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;
		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 1.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peCart.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;
	} else if (this->mTestType == Disparity){
		cout << "Testing in disparity space"<<endl;
		uvds0 = cvCreateMat(numPoints, 3, CV_64FC1);
		uvds1 = cvCreateMat(numPoints, 3, CV_64FC1);
		this->projection(points0, uvds0);
		cvAvgSdv(uvds0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;

		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 10.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peDisp.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;

		peDisp.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);
	} else if (this->mTestType == CartAndDisp) {
		cout << "testing mixed of cartesian and disparity space"<<endl;
		uvds1 = cvCreateMat(numPoints, 3, CV_64FC1);
		cvAvgSdv(points0, &mean, &std);
		cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;

		this->mDisturbScale = std.val[0]*0.015;
		this->mOutlierScale = 1.0;
		mOutlierPercentage = 0.0;
		// set threshold
		double threshold = std.val[0]*0.01;
		peDisp.configureErrorMeasurement(NULL, threshold);
		cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;

		peDisp.setCameraParams(this->mFx, this->mFy, this->mTx, this->mClx, this->mCrx, this->mCy);

	} else {
		cerr << "Unknown test type: "<< this->mTestType<<endl;
	}

	double percentageOfOutliers = mOutlierPercentage;
	double numOutliers = percentageOfOutliers*numPoints;


	double numIters = 1000;
	double maxErrorAfterLevMarq = 0.0;
	int numInliers_maxErrorAfterLevMarq=0;
	double maxErrorBeforeLevMarq = 0.0;
	int numInliers_maxErrorBeforeLevMarq=0;
	int maxNumInliers = 0;
	double errAfterLevMarq_maxNumInliers=0.0;
	double maxImprovementAfterLevMarq = 0;
	int numInliers_maxImprovementAfterLevMarq;

	double maxErrorRod = 0.0;
	int numInliers_maxErrorRod = 0;
	int testCaseNum = -1;

	int numGoodIters=0;
	for (int i=0; i<numIters; i++)
	{
		cout << "Test Case Number:  "<<i<<endl;
		if (i>0) {
			// first iter use preset value
			mEulerAngle.x = CV_PI/4.0*randReal(-1., 1.);
			mEulerAngle.y = CV_PI/4.0*randReal(-1., 1.);
			mEulerAngle.z = CV_PI/4.0*randReal(-1., 1.);

			mTranslation.x = 50. + 100.*cvRandReal(&mRng);
			mTranslation.y = 50. + 100.*cvRandReal(&mRng);
			mTranslation.z = 50. + 100.*cvRandReal(&mRng);
		}
		transform(points0, points1);

		// now randomize the first percentageOfOutliers points to make them outliers
		randomize(points1, numOutliers, mOutlierScale);

		int numInLiers;
		CvMat *inliers0=NULL, *inliers1=NULL;

		CvMat *TransformBestBeforeLevMarq;
		CvMat *TransformAfterLevMarq;
		if (this->mTestType == Cartesian) {
			disturb(points1, points1d);
			int64 t = cvGetTickCount();
			numInLiers = peCart.estimate(points0, points1d, rot, trans);
			peDisp.getInliers(inliers0, inliers1);
			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;

			TransformBestBeforeLevMarq = peCart.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peCart.getFinalTransformation();
		} else if (mTestType == Disparity){
			// convert both set of points into disparity color space
			this->projection(points1, points1d);
			disturb(points1d, uvds1);

			int64 t = cvGetTickCount();
			numInLiers = peDisp.estimate(uvds0, uvds1, rot, trans);
			peDisp.getInliers(inliers0, inliers1);

			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
			TransformBestBeforeLevMarq = peDisp.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peDisp.getFinalTransformation();
		} else if (mTestType == CartAndDisp) {
			// convert both set of points into disparity color space
			this->projection(points1, points1d);
			disturb(points1d, uvds1);

			int64 t = cvGetTickCount();
			numInLiers = peDisp.estimateMixedPointClouds(points0, uvds1, 0, NULL, rot, trans);

			peDisp.getInliers(inliers0, inliers1);

			CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
			TransformBestBeforeLevMarq = peDisp.getBestTWithoutNonLinearOpt();
			TransformAfterLevMarq      = peDisp.getFinalTransformation();

		} else {
			cerr << "Unknown test type "<< mTestType << endl;
		}
		if (numInLiers < 6) {
			continue;
		}
		numGoodIters++;

#if 1
		cout << "Max num of InLiers: "<< numInLiers << endl;
		cout << "Reconstructed Rotation Matrix" << endl;
		CvMatUtils::printMat(rot);
		cout << "Reconstructed Translation Matrix" << endl;
		CvMatUtils::printMat(trans);

		// calculate the relative L2 norm of the diff between true transformation and predicted transformation
		// compute rodrigues from rot and mRot
		double _rod0[3], _rod1[3];
		CvMat rod0 = cvMat(3, 1, CV_64F, _rod0);
		CvMat rod1 = cvMat(3, 1, CV_64F, _rod1);
		cvRodrigues2(&mRot, &rod0);
		cvRodrigues2(rot, &rod1);

		cout << "Rodrigues:"<<endl;
		CvMatUtils::printMat(&rod0);
		CvMatUtils::printMat(&rod1);

		double errRod   = cvNorm(&rod0, &rod1, CV_L2);
		double errTransMat = cvNorm(trans, &mTrans, CV_RELATIVE_L2);

		cout << "L2 -norm of the diff of rodrigues and trans mat are: "<<errRod<<" , "<<errTransMat<<endl;

		if (maxErrorRod < errRod) {
			maxErrorRod = errRod;
			numInliers_maxErrorRod = numInLiers;
			testCaseNum = i;
		}

		CvMat P0, P1r;

		cvReshape(points0,  &P0,  3, 0);
		cvReshape(points1r, &P1r, 3, 0);

		cvPerspectiveTransform(&P0, &P1r, TransformBestBeforeLevMarq);

		// calculate the relative L2 norm of the diff between observed and predicted points;
		double errorBeforeLevMarq = cvNorm(points1, points1r, CV_RELATIVE_L2)/numPoints;
		cout << "Average of the L2-norm of the diff between observed and prediction (Before LevMarq):  "<< errorBeforeLevMarq << endl;

		if (maxErrorBeforeLevMarq < errorBeforeLevMarq) {
			maxErrorBeforeLevMarq = errorBeforeLevMarq;
			numInliers_maxErrorBeforeLevMarq = numInLiers;
		}


		this->transform(rot, points0, trans, points1r);
		//	cout << "Reconstructed tranformed points: "<<endl;
		//	CvMatUtils::printMat(points1r);

		// calculate the relative L2 norm of the diff between observed and predicted points;
		double errorAfterLevMarq = cvNorm(points1, points1r, CV_RELATIVE_L2)/numPoints;
		cout << "Average of the L2-norm of the diff between observed and prediction (After  LevMarq):  "<< errorAfterLevMarq << endl;

		if (maxErrorAfterLevMarq < errorAfterLevMarq) {
			maxErrorAfterLevMarq = errorAfterLevMarq;
			numInliers_maxErrorAfterLevMarq = numInLiers;
		}

		if (maxNumInliers < numInLiers){
			maxNumInliers = numInLiers;
			errAfterLevMarq_maxNumInliers = errorAfterLevMarq;
		}

		if (maxImprovementAfterLevMarq < errorBeforeLevMarq - errorAfterLevMarq){
			maxImprovementAfterLevMarq = errorBeforeLevMarq - errorAfterLevMarq;
			numInliers_maxImprovementAfterLevMarq = numInLiers;
		}

		if (inliers0){
			CvMat *inliers1r = cvCreateMat(numInLiers, 3, CV_64FC1);

			CvMat P0, P1r;

			cvReshape(inliers0, &P0, 3, 0);
			cvReshape(inliers1r, &P1r, 3, 0);

			cvPerspectiveTransform(&P0, &P1r, TransformBestBeforeLevMarq);
			double errorInliersBefore = cvNorm(inliers1, inliers1r, CV_RELATIVE_L2)/numInLiers;
			cout << "Average of the L2-norm of the diff between observed and prediction of inliers (Before LevMarq):  "<< errorInliersBefore << endl;
#if 0
			if (maxErrorBeforeLevMarq < errorInliersBefore) {
				maxErrorBeforeLevMarq = errorInliersBefore;
				numInliers_maxErrorBeforeLevMarq = numInLiers;
			}
#endif

			cvPerspectiveTransform(&P0, &P1r, TransformAfterLevMarq);
//			this->transform(rot, inliers0, trans, inliers1r);
			//	cout << "Reconstructed tranformed points of inliers: "<<endl;
			// calculate the relative L2 norm of the diff between observed and predicted points;
			double errorInliersAfter = cvNorm(inliers1, inliers1r, CV_RELATIVE_L2)/numInLiers;
			cout << "Average of the L2-norm of the diff between observed and prediction of inliers (After  LevMarq):  "<< errorInliersAfter << endl;
#if 0
			if (maxErrorAfterLevMarq < errorInliersAfter) {
				maxErrorAfterLevMarq = errorInliersAfter;
				numInliers_maxErrorAfterLevMarq = numInLiers;
			}

			if (maxNumInliers < numInLiers){
				maxNumInliers = numInLiers;
				errAfterLevMarq_maxNumInliers = errorInliersAfter;
			}

			if (maxImprovementAfterLevMarq < errorInliersBefore - errorInliersAfter){
				maxImprovementAfterLevMarq = errorInliersBefore - errorInliersAfter;
				numInliers_maxImprovementAfterLevMarq = numInLiers;
			}
#endif

			cvReleaseMat(&inliers1r);
		}

#endif
	}
	cout << "max error of rodrigues: "<<maxErrorRod <<endl;
	cout << "num of inliers for the test case: "<< numInliers_maxErrorRod<<endl;
	cout << "test Case number: "<< testCaseNum <<endl;
	cout << "max error before levmarq: "<< maxErrorBeforeLevMarq << endl;
	cout << "num of inliers for the test case: "<< numInliers_maxErrorBeforeLevMarq << endl;

	cout << "max error after levmarq:  " << maxErrorAfterLevMarq  << endl;
	cout << "num of inliers for the test case: " << numInliers_maxErrorAfterLevMarq<< endl;

	cout << "max improvement after levmarq: " << maxImprovementAfterLevMarq << endl;
	cout << "num of inliers for the test case: "<<numInliers_maxImprovementAfterLevMarq << endl;

	cout << "max num of inliers: "<< maxNumInliers << endl;
	cout << "error after levmarq for the test case: "<< errAfterLevMarq_maxNumInliers << endl;

	CvTestTimer& timer = CvTestTimer::getTimer();

	timer.mNumIters = numGoodIters;
	timer.printStat();

	cvReleaseMat(&points1);
	cvReleaseMat(&points1d);
	cvReleaseMat(&points1r);

    return status;
}

int main(int argc, char **argv){
	CvTest3DPoseEstimate test3DPoseEstimate;

	if (argc >= 2) {
		char *option = argv[1];
		if (strcasecmp(option, "cartesian")==0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Cartesian;
		} else if (strcasecmp(option, "disparity")==0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Disparity;
		} else if (strcasecmp(option, "cartanddisp") == 0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::CartAndDisp;
		} else if (strcasecmp(option, "video") == 0) {
			test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Video;
		} else {
			cerr << "Unknown option: "<<option<<endl;
			exit(1);
		}
	} else {
		test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Cartesian;
	}

	cout << "Testing wg3DPoseEstimate ..."<<endl;


    test3DPoseEstimate.test();

	cout << "Done testing wg3DPoseEstimate ..."<<endl;
	return 0;
}
