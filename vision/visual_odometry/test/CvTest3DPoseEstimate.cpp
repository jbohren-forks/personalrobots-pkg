#include "CvTest3DPoseEstimate.h"

#include <iostream>
#include <limits>
#include <vector>
#include <queue>
#include <boost/foreach.hpp>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
// the google wrapper
#include <opencv/cvwimage.h>

#include <opencv/cxcore.hpp>

//#include <stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily

// WG Ext of OpenCV
#include <CvPoseEstErrMeasDisp.h>
#include <Cv3DPoseEstimateStereo.h>
#include "PoseEstimateDisp.h"
#include "PoseEstimate.h"
#include "CvMatUtils.h"
#include "CvMat3X3.h"
#include "CvTestTimer.h"
#include <PathRecon.h>
#include <VOSparseBundleAdj.h>

#include <VisOdom.h>

using namespace cv::willow;

// star detector
//#include <detector.h>

using namespace cv;
using namespace std;


#define GAUSSIANNOISE

#define ShowTemplateMatching 0

#define DEBUG 1
#define DISPLAY 1

#if CHECKTIMING == 1
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
  if (mem_storage_) cvReleaseMemStorage(&mem_storage_);
}

void CvTest3DPoseEstimate::_init(){
  cvInitMatHeader(&mRot,   3, 3, CV_64FC1, mRotData);
  cvInitMatHeader(&mTrans, 3, 1, CV_64FC1, mTransData);
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
  CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::EulerZYX);
  cout << "Rotation Matrix:= Pitch * Yaw* Roll"<< endl;
#else // XYZ euler angle
  CvMat3X3<double>::rotMatrix(mEulerAngle.x, mEulerAngle.y, mEulerAngle.z, mRotData, CvMat3X3<double>::EulerXYZ);
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

}

double CvTest3DPoseEstimate::randReal(double min, double max) {
  double r = cvRandReal(&mRng);
  r = r*(max-min) + min;
  return r;
}

void CvTest3DPoseEstimate::disturb(const CvMat *xyzs, CvMat* xyzsNoised) {
#ifdef GAUSSIANNOISE
//  CvMat* Noise = cvCreateMat(xyzs->rows, xyzs->cols, xyzs->type);
  double sigma = mDisturbScale/2.0; // ~ 95%
  cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
  cvAdd(xyzs, xyzsNoised, xyzsNoised);
//  cvReleaseMat(&Noise);
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
    return testVideo();
    break;
  case Video2:
    return testVideo2();
  case Video3:
    return testVideo3();
    break;
  case VideoBundleAdj:
    return testVideoBundleAdj();
    break;
  case BundleAdj:
    return testBundleAdj(true, true);
    break;
  default:
    cout << "Unknown test type: "<<  mTestType << endl;
  }
  return false;
}

void CvTest3DPoseEstimate::loadStereoImagePair(string & dirname, int & frameIndex, WImageBuffer1_b & leftImage, WImageBuffer1_b & rightImage)
{
    char leftfilename[PATH_MAX];
    char rightfilename[PATH_MAX];
    sprintf(leftfilename,  "%s/left-%04d.ppm",  dirname.c_str(), frameIndex);
    sprintf(rightfilename, "%s/right-%04d.ppm", dirname.c_str(), frameIndex);
    cout << "loading " << leftfilename << " and " << rightfilename << endl;
    IplImage* leftimg  = cvLoadImage(leftfilename,  CV_LOAD_IMAGE_GRAYSCALE);
    leftImage.SetIpl(leftimg);
    IplImage* rightimg = cvLoadImage(rightfilename, CV_LOAD_IMAGE_GRAYSCALE);
    rightImage.SetIpl(rightimg);
}

bool CvTest3DPoseEstimate::testVideoBundleAdj() {
  bool status = false;
  CvSize imgSize = cvSize(640, 480);
  VOSparseBundleAdj sba(imgSize, 5, 10);

  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm
  this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  // parameterize the post estimator
  sba.setCameraParams(Fx_, Fy_, Tx_, Clx_, Crx_, Cy_, Du_);

  string dirname("Data/indoor1");
  string leftimgfmt("/left-%04d.ppm");
  string rightimgfmt("/right-%04d.ppm");
  string dispimgfmt(".dispmap-%04d.xml");
  int start = 0;
  int end   = 1509;
  int step  = 1;

  start = 100;
  end   = 500;

  // @todo if we run from frame 30 to frame 600. at frame 535 there is a weird behavior - two of the estimated
  // point get map to the observed points.

  // set up a FileSeq
  FileSeq fileSeq;
  fileSeq.setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);
  // visualization
#if DISPLAY
  // Optionally, set up the visualizer
  vector<FramePose*>* fp = sba.getFramePoses();
  const PointTracks& tracks = sba.getTracks();

  sba.mVisualizer = new SBAVisualizer(sba.mPoseEstimator, *fp, tracks, &sba.map_index_to_FramePose_);
#endif

  if (fileSeq.getStartFrame() == false) {
    return false;
  }

  do {
#if 0
    StereoFrame& sf = fileSeq.mInputImageQueue.front();
    sf.mDispMap = new WImageBuffer1_16s(sf.mImage->Width(), sf.mImage->Height());
    getDisparityMap(*sf.mImage, *sf.mRightImage, *sf.mDispMap);
#endif
    sba.track(fileSeq.mInputImageQueue);
  } while(fileSeq.getNextFrame() == true);

  vector<FramePose*>* framePoses = sba.getFramePoses();
  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = fileSeq.mNumFrames/fileSeq.mFrameStep;

  saveFramePoses(string("Output/indoor1/"), *framePoses);

  sba.printStat();
  sba.mStat2.print();

  CvTestTimer::getTimer().printStat();
  return status;
}

static bool getDisparityMap( WImageBuffer1_b& leftImage, WImageBuffer1_b& rightImage, WImageBuffer1_16s& dispMap) {
#if 0
  bool status = true;

  int w = leftImage.Width();
  int h = leftImage.Height();

  if (w != rightImage.Width() || h != rightImage.Height()) {
    cerr << __PRETTY_FUNCTION__ <<"(): size of images incompatible. "<< endl;
    return false;
  }

  //
  // Try Kurt's dense stereo pair
  //
  const uint8_t *lim = leftImage.ImageData();
  const uint8_t *rim = rightImage.ImageData();

  int16_t* disp    = dispMap.ImageData();
  int16_t* textImg = NULL;

  static const int mFTZero       = 31;    //< max 31 cutoff for prefilter value
  static const int mDLen         = 64;    //< 64 disparities
  static const int mCorr         = 15;    //< correlation window size
  static const int mTextThresh   = 10;    //< texture threshold
  static const int mUniqueThresh = 15;    //< uniqueness threshold

  // scratch images
  uint8_t *mBufStereoPairs     = new uint8_t[h*mDLen*(mCorr+5)]; // local storage for the stereo pair algorithm
  uint8_t *mFeatureImgBufLeft  = new uint8_t[w*h];
  uint8_t *mFeatureImgBufRight = new uint8_t[w*h];

  // prefilter
  do_prefilter((uint8_t *)lim, mFeatureImgBufLeft, w, h, mFTZero, mBufStereoPairs);
  do_prefilter((uint8_t *)rim, mFeatureImgBufRight, w, h, mFTZero, mBufStereoPairs);

  // stereo
  do_stereo(mFeatureImgBufLeft, mFeatureImgBufRight, disp, textImg, w, h,
      mFTZero, mCorr, mCorr, mDLen, mTextThresh, mUniqueThresh, mBufStereoPairs);

  return status;
#endif
}

bool CvTest3DPoseEstimate::testVideo() {
  bool status = false;
  CvSize imgSize = cvSize(640, 480);
  PathRecon pathRecon(imgSize);
  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm
  this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95, 1.0);
  // parameterize the pose estimator
  pathRecon.setCameraParams(Fx_, Fy_, Tx_, Clx_, Crx_, Cy_, Du_);

  string dirname("Data/indoor1");
  string leftimgfmt("/left-%04d.ppm");
  string rightimgfmt("/right-%04d.ppm");
  string dispimgfmt(".dispmap-%04d.xml");
  int start = 1150;
  int end   = 1160;
  int step  = 1;

  // set up a FileSeq
  FileSeq fileSeq;
  fileSeq.setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);
  // visualization
#if DISPLAY
  // Optionally, set up the visualizer
  pathRecon.mVisualizer = new F2FVisualizer(pathRecon.mPoseEstimator);
#endif

  if (fileSeq.getStartFrame() == false) {
    return false;
  }

  do {
    StereoFrame& sf = fileSeq.mInputImageQueue.front();
#if 0
    sf.mDispMap = new WImageBuffer1_16s(sf.mImage->Width(), sf.mImage->Height());
    getDisparityMap(*sf.mImage, *sf.mRightImage, *sf.mDispMap);
#endif
    pathRecon.track(fileSeq.mInputImageQueue);
  } while(fileSeq.getNextFrame() == true);


  vector<FramePose*>* framePoses = pathRecon.getFramePoses();
  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = fileSeq.mNumFrames/fileSeq.mFrameStep;

  saveFramePoses(string("Output/indoor1/"), *framePoses);

  pathRecon.printStat();

  CvTestTimer::getTimer().printStat();
  return status;
}

bool CvTest3DPoseEstimate::testVideo2() {
  bool status = false;
  CvSize imgSize = cvSize(640, 480);
  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm
  this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  CamTracker* tracker = CamTracker::getCamTracker(Pairwise, imgSize,
      Fx_, Fy_, Tx_, Clx_, Crx_, Cy_);

  string dirname("Data/indoor1");
  string leftimgfmt("/left-%04d.ppm");
  string rightimgfmt("/right-%04d.ppm");
  string dispimgfmt("/dispmap-%04d.xml");
  int start = 0;
  int end   = 1509;
  int step  = 1;
  // set up a FileSeq
  FileSeq fileSeq;
  fileSeq.setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

  if (fileSeq.getStartFrame() == false) {
    return false;
  }
  do {
    tracker->track(fileSeq.mInputImageQueue);
  } while(fileSeq.getNextFrame() == true);


  vector<FramePose*>* framePoses = tracker->getFramePoses();
  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = fileSeq.mNumFrames/fileSeq.mFrameStep;

  saveFramePoses(string("Output/indoor1/"), *framePoses);

  tracker->printStat();
#if 0
  pathRecon.mStat.mNumKeyPointsWithNoDisparity = pathRecon.mPoseEstimator.mNumKeyPointsWithNoDisparity;
  pathRecon.mStat.mPathLength = pathRecon.mPathLength;
//  mStat.print();
  pathRecon.mStat.print();
#endif

  CvTestTimer::getTimer().printStat();
  return status;
}

bool CvTest3DPoseEstimate::testVideo3() {
  bool status = false;
  CvSize imgSize = cvSize(640, 480);
  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm
  this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  CamTracker* tracker = CamTracker::getCamTracker(Pairwise, imgSize,
      Fx_, Fy_, Tx_, Clx_, Crx_, Cy_);

  string dirname("Data/indoor1");
  string leftimgfmt("/left-%04d.ppm");
  string rightimgfmt("/right-%04d.ppm");
  string dispmapfmt("/dispmap-%04d.xml");
  int start = 0;
  int end   = 1509;
  int step  = 1;

  // set up a FileSeq
  FileSeq fileSeq;
  fileSeq.setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispmapfmt, start, end, step);

  if (fileSeq.getStartFrame() == false) {
    return false;
  }
  do {
//    tracker->track(fileSeq.mInputImageQueue);
    while (fileSeq.mInputImageQueue.size()>0) {
      bool newKeyFrame = tracker->trackOneFrame(fileSeq.mInputImageQueue,
          tracker->getFrameSeq());
      if (newKeyFrame == true) {
        // only keep the last frame in the queue
        tracker->reduceWindowSize(1);
      }
//      visualize();
    }

  } while(fileSeq.getNextFrame() == true);


  vector<FramePose*>* framePoses = tracker->getFramePoses();
  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = fileSeq.mNumFrames/fileSeq.mFrameStep;



  framePoses = tracker->getFramePoses();

  saveFramePoses(string("Output/indoor1/"), *framePoses);

  tracker->printStat();

  CvTestTimer::getTimer().printStat();
  return status;
}

bool CvTest3DPoseEstimate::testVideo4OneFrame(queue<StereoFrame> inputImageQueue,
    FrameSeq& frameSeq, CamTracker& tracker)
{
  bool insertNewKeyFrame = false;
  TIMERSTART(Total);

  // currFrame is a reference to frameSeq.mCurrentFrame;
  PoseEstFrameEntry*& currFrame = frameSeq.mCurrentFrame;
  bool gotCurrFrame = false;
  bool stop = false;

  if (frameSeq.mNextFrame.get()){
    // do not need to load new images nor compute the key points again
    frameSeq.mCurrentFrame = frameSeq.mNextFrame.release();
    gotCurrFrame = true;
  } else {
    int numWaits;
    // wait at most for 30 seconds
    const int maxNumWaits = 30*30;
    for (numWaits=0; numWaits<maxNumWaits && inputImageQueue.size()==0; numWaits++){
      // wait for 33 milliseconds
      usleep(33000);
    }
    if (numWaits>=maxNumWaits) {
      stop = true;
    } else {
      // process the next stereo pair of images.
      StereoFrame stereoFrame = inputImageQueue.front();
      if (stereoFrame.mFrameIndex == -1) {
        // done
        stop = true;
        gotCurrFrame = false;
      } else {
        currFrame = new PoseEstFrameEntry(stereoFrame.mFrameIndex);
        currFrame->mImage = stereoFrame.mImage;
        currFrame->mRightImage = stereoFrame.mRightImage;
        currFrame->mDispMap = stereoFrame.mDispMap;
        // compute disparity map
        TIMERSTART2(DisparityMap);
        currFrame->mDispMap = new WImageBuffer1_16s(
            currFrame->mImage->Width(), currFrame->mImage->Height());

        tracker.getPoseEstimator()->getDisparityMap(*currFrame->mImage,
            *currFrame->mRightImage, *currFrame->mDispMap);
        TIMEREND2(DisparityMap);
#if SAVEDISPMAP
        {
          char dispfilename[256];
          char dispname[256];
          sprintf(dispfilename, "Output/indoor1/dispmap-%04d.xml",
              currFrame->frame_index_);
          sprintf(dispname, "dispmap-%04d.xml", currFrame->frame_index_);
          cvSave(dispfilename, currFrame->mDispMap, dispname, "disparity map 16s");
        }
#endif

        // counting how many frames we have processed
        frameSeq.mNumFrames++;

        TIMERSTART2(FeaturePoint);
        tracker.goodFeaturesToTrack(*currFrame->mImage, currFrame->mDispMap,
            currFrame->mKeypoints);
        TIMEREND2(FeaturePoint);
//        if (mVisualizer) mVisualizer->drawDispMap(*currFrame);
      }
      inputImageQueue.pop();
    }
  }

  KeyFramingDecision kfd = KeyFrameBackTrack;

  if (stop == false) {
    if (frameSeq.mNumFrames==1) {
      // First frame ever, do not need to do anything more.
      frameSeq.mStartFrameIndex = currFrame->mFrameIndex;
      kfd = KeyFrameUse;
    } else {
      //
      // match the good feature points between this iteration and last key frame
      //
      vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;
      if (currFrame->mTrackableIndexPairs == NULL) {
        currFrame->mTrackableIndexPairs = new vector<pair<int, int> >();
      } else {
        currFrame->mTrackableIndexPairs->clear();
      }
      tracker.matchKeypoints(&trackablePairs, currFrame->mTrackableIndexPairs);
      assert(currFrame->mTrackableIndexPairs->size() == trackablePairs.size());
#ifdef DEBUG
      cout << "Num of trackable pairs for pose estimate: "<<trackablePairs.size() << endl;
#endif

      // if applicable, pass a reference of the current frame for visualization
      //      if (mVisualizer)  mVisualizer->drawKeypoints(*getLastKeyFrame(), *currFrame, trackablePairs);

      if (currFrame->mNumTrackablePairs< defMinNumTrackablePairs) {
#ifdef DEBUG
        cout << "Too few trackable pairs" <<endl;
#endif
        // shall backtrack
        kfd = KeyFrameBackTrack;
      } else {

        //  pose estimation given the feature point pairs
        //  note we do not do Levenberg-Marquardt here, as we are not sure if
        //  this is key frame yet.
        TIMERSTART2(PoseEstimateRANSAC);
        currFrame->mNumInliers =
          tracker.getPoseEstimator()->estimate(*currFrame->mKeypoints,
              *tracker.getLastKeyFrame()->mKeypoints,
              *currFrame->mTrackableIndexPairs,
              currFrame->mRot, currFrame->mShift, false);
        TIMEREND2(PoseEstimateRANSAC);

        //        mStat.mHistoInliers.push_back(currFrame->mNumInliers);

        currFrame->mInliers0 = NULL;
        currFrame->mInliers1 = NULL;
        tracker.fetchInliers(currFrame->mInliers1, currFrame->mInliers0);
        currFrame->mInlierIndices = tracker.fetchInliers();
        currFrame->mLastKeyFrameIndex = tracker.getLastKeyFrame()->mFrameIndex;
#ifdef DEBUG
        cout << "num of inliers: "<< currFrame->mNumInliers <<endl;
#endif
        assert(tracker.getLastKeyFrame());
        //        if (mVisualizer) mVisualizer->drawTracking(*getLastKeyFrame(), *currFrame);

        // Decide if we need select a key frame by now
        kfd =
          tracker.keyFrameEval(currFrame->mFrameIndex, currFrame->mKeypoints->size(),
              currFrame->mNumInliers,
              currFrame->mRot, currFrame->mShift);

        // key frame action
        //        insertNewKeyFrame = keyFrameAction(kfd, frameSeq);
      }
    } // not first frame;
  } // stop == false

  insertNewKeyFrame = tracker.keyFrameAction(kfd, frameSeq);

  TIMEREND(Total);
  return insertNewKeyFrame;
}

bool CvTest3DPoseEstimate::testVideo4() {
  bool status = false;
  CvSize imgSize = cvSize(640, 480);
  // The following parameters are from indoor1/proj.txt
  // note that B (or Tx) is in mm
  this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  CamTracker* tracker = CamTracker::getCamTracker(Pairwise, imgSize,
      Fx_, Fy_, Tx_, Clx_, Crx_, Cy_);

  string dirname("Data/indoor1");
  string leftimgfmt("/left-%04d.ppm");
  string rightimgfmt("/right-%04d.ppm");
  string dispmapfmt("/dispmap-%04d.xml");
  int start = 0;
  int end   = 1509;
  int step  = 1;

  // set up a FileSeq
  FileSeq fileSeq;
  fileSeq.setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispmapfmt, start, end, step);

  if (fileSeq.getStartFrame() == false) {
    return false;
  }
  do {
//    tracker->track(fileSeq.mInputImageQueue);
    while (fileSeq.mInputImageQueue.size()>0) {
      bool newKeyFrame = tracker->trackOneFrame(fileSeq.mInputImageQueue,
          tracker->getFrameSeq());
      if (newKeyFrame == true) {
        // only keep the last frame in the queue
        tracker->reduceWindowSize(1);
      }
//      visualize();
    }

  } while(fileSeq.getNextFrame() == true);


  vector<FramePose*>* framePoses = tracker->getFramePoses();
  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = fileSeq.mNumFrames/fileSeq.mFrameStep;



  framePoses = tracker->getFramePoses();

  saveFramePoses(string("Output/indoor1/"), *framePoses);

  tracker->printStat();

  CvTestTimer::getTimer().printStat();
  return status;
}

bool CvTest3DPoseEstimate::testBundleAdj(bool disturb_frames, bool disturb_points) {
  bool status = true;
  // rows of 3d points in Cartesian space
  CvMat *points = (CvMat *)cvLoad("Data/cartesianPoints.xml");
  // rows of euler angle and shift
  CvMat *frames = (CvMat *)cvLoad("Data/frames.xml");
  CvMat cartToDisp;
  CvMat dispToCart;

  this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
  this->getProjectionMatrices(&cartToDisp, &dispToCart);

//  cvSetIdentity(&cartToDisp);
//  cvSetIdentity(&dispToCart);


  // set up cameras
  vector<FramePose* > frame_poses;
  double rod_shift[6];
//  int index_offset = 9;
  int oldest_index = numeric_limits<int>::max();
  CvMat mat_rod_shift = cvMat(6, 1, CV_64FC1, rod_shift);
  for (int r=0; r<frames->rows; r++) {
    CvMat param1x6;
    CvMat param6x1;
    cvGetSubRect(frames, &param1x6, cvRect(1,r,6,1));
//    cvGetRow(frames, &param1x6, r);
    cvReshape(&param1x6, &param6x1, 1, 6);
    int frame_index = cvmGet(frames, r, 0);
    if (frame_index<oldest_index) {
      oldest_index = frame_index;
    }

    FramePose* fp = new FramePose(frame_index);
    CvMatUtils::transformFromEulerAndShift(&param6x1, &fp->transf_local_to_global_);
    CvMatUtils::transformToRodriguesAndShift(fp->transf_local_to_global_, mat_rod_shift);
    fp->mRod.x = rod_shift[0];
    fp->mRod.y = rod_shift[1];
    fp->mRod.z = rod_shift[2];

    fp->mShift.x = rod_shift[3];
    fp->mShift.y = rod_shift[4];
    fp->mShift.z = rod_shift[5];
    frame_poses.push_back(fp);
  }

  // set up tracks
  PointTracks tracks;
  CvPoint3D64f coord;
  CvPoint3D64f disp_coord0;
  CvPoint3D64f disp_coord1;
  CvMat global_point = cvMat(1, 1, CV_64FC3, &coord);
  CvMat disp_point0 = cvMat(1, 1, CV_64FC3, &disp_coord0);
  CvMat disp_point1 = cvMat(1, 1, CV_64FC3, &disp_coord1);
  CvMat* global_to_local = cvCreateMat(4, 4, CV_64FC1);
  CvMat* global_to_disp = cvCreateMat(4, 4, CV_64FC1);
  for (int ipt = 0; ipt < points->rows; ipt++) {
    coord.x = cvmGet(points, ipt, 0);
    coord.y = cvmGet(points, ipt, 1);
    coord.z = cvmGet(points, ipt, 2);
    // convert from coord to disp_coord0 and disp_coord1
    CvMatUtils::invertRigidTransform(&frame_poses[0]->transf_local_to_global_, global_to_local);
    cvMatMul(&cartToDisp, global_to_local, global_to_disp);
    cvPerspectiveTransform(&global_point, &disp_point0, global_to_disp);
    CvMatUtils::invertRigidTransform(&frame_poses[1]->transf_local_to_global_, global_to_local);
    cvMatMul(&cartToDisp, global_to_local, global_to_disp);
    cvPerspectiveTransform(&global_point, &disp_point1, global_to_disp);

    PointTrackObserv* obsv0 = new PointTrackObserv(frame_poses[0]->mIndex, disp_coord0, ipt);
    PointTrackObserv* obsv1 = new PointTrackObserv(frame_poses[1]->mIndex, disp_coord1, ipt);
    PointTrack* point = new PointTrack(obsv0, obsv1, coord, ipt);

    for (int iframe = 2; iframe < frames->rows; iframe++) {
      // extend the track
      CvMatUtils::invertRigidTransform(&frame_poses[iframe]->transf_local_to_global_, global_to_local);
      cvMatMul(&cartToDisp, global_to_local, global_to_disp);
      cvPerspectiveTransform(&global_point, &disp_point0, global_to_disp);
      PointTrackObserv* obsv = new PointTrackObserv(frame_poses[iframe]->mIndex, disp_coord0, ipt);
      point->extend(obsv);
    }

    tracks.tracks_.push_back(point);
    tracks.oldest_frame_index_in_tracks_ = oldest_index;
  }

  int full_free_window_size  = 5;
  int full_fixed_window_size = 5;
  int num_good_updates = 5000;
  double epsilon = DBL_EPSILON;
//  epsilon = FLT_EPSILON;
//  epsilon = LDBL_EPSILON;
//  epsilon = .1e-10;
  CvTermCriteria term_criteria =
    cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,num_good_updates,epsilon);

  LevMarqSparseBundleAdj sba(&dispToCart, &cartToDisp,
      full_free_window_size, full_fixed_window_size, term_criteria);

  vector<FramePose*> free_frames;
  vector<FramePose*> fixed_frames;

  // the fixed windows
  fixed_frames.push_back(frame_poses[0]);

  // the free windows
  free_frames.push_back(frame_poses[1]);
  free_frames.push_back(frame_poses[2]);
  free_frames.push_back(frame_poses[3]);
  free_frames.push_back(frame_poses[4]);
  free_frames.push_back(frame_poses[5]);

  // disturb the point parameters.
  if (disturb_points == true) {
    printf("Disturbed Points:\n");
    double disturb_scale = .001;
    double sigma = disturb_scale/2.0; // ~ 95%
    int numPoints = tracks.tracks_.size();
    CvMat* xyzsNoised = cvCreateMat(numPoints, 3, CV_64FC1);
    cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
    int iPoints = 0;
    BOOST_FOREACH(PointTrack* p, tracks.tracks_) {
      p->coordinates_.x += cvmGet(xyzsNoised, iPoints, 0);
      p->coordinates_.y += cvmGet(xyzsNoised, iPoints, 1);
      p->coordinates_.z += cvmGet(xyzsNoised, iPoints, 2);
      printf("point %3d, [%9.4f, %9.4f, %9.4f]\n", p->id_, p->coordinates_.x,
          p->coordinates_.y, p->coordinates_.z);
      iPoints++;
    }
    cvReleaseMat(&xyzsNoised);
  }

  string output_dir("Output");
  tracks.save(output_dir);

  int numFreeFrames = free_frames.size();
  CvMat* frame_params_input = cvCreateMat(numFreeFrames, 6, CV_64FC1);
  CvMat* frame_params_est   = cvCreateMat(numFreeFrames, 6, CV_64FC1);

  // make a copy of the input params
  {
    int iFrames = 0;
    BOOST_FOREACH(FramePose *fp, free_frames){
      CvMat rot, transl;
      cvGetSubRect(&fp->transf_local_to_global_, &rot, cvRect(0,0,3,3));
      cvGetSubRect(&fp->transf_local_to_global_, &transl, cvRect(3,0,1,3));
      CvPoint3D64f euler;
      CvMatUtils::rotMatToEuler(rot, euler);
      cvmSet(frame_params_input, iFrames, 0, euler.x );
      cvmSet(frame_params_input, iFrames, 1, euler.y );
      cvmSet(frame_params_input, iFrames, 2, euler.z );
      cvmSet(frame_params_input, iFrames, 3, cvmGet(&transl, 0, 0));
      cvmSet(frame_params_input, iFrames, 4, cvmGet(&transl, 1, 0));
      cvmSet(frame_params_input, iFrames, 5, cvmGet(&transl, 2, 0));
      iFrames++;
    }
  }

  if (disturb_frames == true) {
    // disturb the frame parameters

    double disturb_scale = .001;
    double sigma = disturb_scale/2.0; // ~ 95%
    CvMat* xyzsNoised = cvCreateMat(numFreeFrames, 6, CV_64FC1);
    cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
    int iFrames = 0;
    BOOST_FOREACH(FramePose *fp, free_frames){
      CvMatUtils::transformToRodriguesAndShift(fp->transf_local_to_global_, mat_rod_shift);
      for (int i=0; i<6; i++) {
        rod_shift[i] += cvmGet(xyzsNoised, iFrames, i);
      }
      iFrames++;
    }

    cvReleaseMat(&xyzsNoised);
  }

  sba.optimize(&free_frames, &fixed_frames, &tracks);

  // print some output
  printf("Number of good updates: %d\n", sba.num_good_updates_);
  printf("Number of retractions:  %d\n", sba.num_retractions_);

  double transpose_transl_data[3];
  CvMat transpose_transl = cvMat(1, 3, CV_64FC1, transpose_transl_data);
  BOOST_FOREACH(const FramePose* fp, free_frames) {
    printf("transf of frame: %d\n", fp->mIndex);
    CvMatUtils::printMat(&fp->transf_local_to_global_);
    // in Euler angle and translation
    CvMat rot, transl;
    cvGetSubRect(&fp->transf_local_to_global_, &rot, cvRect(0,0,3,3));
    CvPoint3D64f euler;
    CvMatUtils::rotMatToEuler(rot, euler);
    cvGetSubRect(&fp->transf_local_to_global_, &transl, cvRect(3,0,1,3));
    printf("In Euler angle and translation:\n");
    printf("Euler angle: [%9.4f, %9.4f, %9.4f]\n", euler.x, euler.y, euler.z);
    cvTranspose(&transl, &transpose_transl);
    CvMatUtils::printMat(&transpose_transl, "%9.4f");
  }

  // compare frame params
  {
    int iFrames = 0;
    BOOST_FOREACH(FramePose *fp, free_frames){
      CvMat rot, transl;
      cvGetSubRect(&fp->transf_local_to_global_, &rot, cvRect(0,0,3,3));
      cvGetSubRect(&fp->transf_local_to_global_, &transl, cvRect(3,0,1,3));
      CvPoint3D64f euler;
      CvMatUtils::rotMatToEuler(rot, euler);
      cvmSet(frame_params_est, iFrames, 0, euler.x );
      cvmSet(frame_params_est, iFrames, 1, euler.y );
      cvmSet(frame_params_est, iFrames, 2, euler.z );
      cvmSet(frame_params_est, iFrames, 3, cvmGet(&transl, 0, 0));
      cvmSet(frame_params_est, iFrames, 4, cvmGet(&transl, 1, 0));
      cvmSet(frame_params_est, iFrames, 5, cvmGet(&transl, 2, 0));
      iFrames++;
    }

    double error_norm_frames = cvNorm(frame_params_input, frame_params_est, CV_RELATIVE_L2);
    printf("Relative L2 Norm of the Errors for the frames: %e\n", error_norm_frames);

    status = status && (error_norm_frames < 1.e-8);

  }



  CvMat* points_est = cvCreateMat(points->rows, points->cols, CV_64FC1);
  int iPoints=0;
  BOOST_FOREACH(const PointTrack* p, tracks.tracks_) {
//    printf("point %3d, [%9.4f, %9.4f, %9.4f]\n",
    printf("point %3d, [%9.7e, %9.7e, %9.7e]\n",
        p->id_, p->coordinates_.x, p->coordinates_.y, p->coordinates_.z);
    cvmSet(points_est, iPoints, 0, p->coordinates_.x);
    cvmSet(points_est, iPoints, 1, p->coordinates_.y);
    cvmSet(points_est, iPoints, 2, p->coordinates_.z);
    iPoints++;
  }

  // compare input points and estimated points.
  double error_norm_points = cvNorm(points, points_est, CV_RELATIVE_L2);
  printf("Relative L2 Norm of Errors for point params: %e\n", error_norm_points);


  status = status && (error_norm_points < 1.e-8);

  // clean up
  // remove the points;

  // remove the frames
  BOOST_FOREACH(FramePose* fp, frame_poses) {
    delete fp;
  }

  cvReleaseMat(&frame_params_input);
  cvReleaseMat(&frame_params_est  );
  cvReleaseMat(&points_est);
  cvReleaseMat(&global_to_disp);
  cvReleaseMat(&global_to_local);
  cvReleaseMat(&frames);
  cvReleaseMat(&points);

  if (status == true) {
    printf("testBundleAdj() passed\n");
  } else {
    printf("testBundleAdj() failed\n");
  }

  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = 1;
  CvTestTimer::getTimer().printStat();

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
//  cvMatMul(&X2Z, points0, points00);
  cvSave("Data/obj1_cropped_2000_adjusted.xml", points0, "adjusted", "...so that it is more like real data");
#endif
//  CvMat * points0 =  (CvMat *)cvLoad("Data/3dPointClouds0.xml");
//  CvMat * points0 =  (CvMat *)cvLoad("Data/obj1_cropped.xml");
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

//  Cv3DPoseEstimateRef peCart;
  PoseEstimate peCart;
//  Cv3DPoseEstimateDispSpaceRef peDisp;
  PoseEstimateDisp peDisp;
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
    this->cartToDisp(points0, uvds0);
    cvAvgSdv(uvds0, &mean, &std);
    cout << "mean and std of point cloud: "<<mean.val[0] << ","<<std.val[0]<<endl;

    this->mDisturbScale = std.val[0]*0.015;
    this->mOutlierScale = 10.0;
    mOutlierPercentage = 0.0;
    // set threshold
    double threshold = std.val[0]*0.01;
    peDisp.configureErrorMeasurement(NULL, threshold);
    cout << "set disturb scale, threshold to be: "<< this->mDisturbScale<<","<<threshold<<endl;

    peDisp.setCameraParams(Fx_, Fy_, Tx_, Clx_, Crx_, Cy_);
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

    peDisp.setCameraParams(Fx_, Fy_, Tx_, Clx_, Crx_, Cy_);

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
      this->cartToDisp(points1, points1d);
      disturb(points1d, uvds1);

      int64 t = cvGetTickCount();
      numInLiers = peDisp.estimate(uvds0, uvds1, rot, trans, true);
      peDisp.getInliers(inliers0, inliers1);

      CvTestTimer::getTimer().mTotal += cvGetTickCount() - t;
      TransformBestBeforeLevMarq = peDisp.getBestTWithoutNonLinearOpt();
      TransformAfterLevMarq      = peDisp.getFinalTransformation();
    } else if (mTestType == CartAndDisp) {
      // convert both set of points into disparity color space
      this->cartToDisp(points1, points1d);
      disturb(points1d, uvds1);

      int64 t = cvGetTickCount();
      numInLiers = peDisp.estimateMixedPointClouds(points0, uvds1, 0, NULL, rot, trans, true);

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
    //  cout << "Reconstructed tranformed points: "<<endl;
    //  CvMatUtils::printMat(points1r);

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
//      this->transform(rot, inliers0, trans, inliers1r);
      //  cout << "Reconstructed tranformed points of inliers: "<<endl;
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
    } else if (strcasecmp(option, "video2") == 0) {
      test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Video2;
    } else if (strcasecmp(option, "video3") == 0) {
      test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::Video3;
    } else if (strcasecmp(option, "bundle") == 0) {
      test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::VideoBundleAdj;
    } else if (strcasecmp(option, "bundle1") == 0) {
      test3DPoseEstimate.mTestType = CvTest3DPoseEstimate::BundleAdj;
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

