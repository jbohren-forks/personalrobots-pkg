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
#define SAVE_FRAMES_POINTS 0

#if CHECKTIMING == 1
#define TIMERSTART(x)
#define TIMEREND(x)
#define TIMERSTART2(x)
#define TIMEREND2(x)
#else
#define TIMERSTART(x)  CvTestTimerStart1(x)
#define TIMEREND(x)    CvTestTimerEnd1(x)
#define TIMERSTART2(x) CvTestTimerStart2(x)
#define TIMEREND2(x)   CvTestTimerEnd2(x)
#endif


CvTest3DPoseEstimate::CvTest3DPoseEstimate():
  Parent(),
  verbose_(true),
  img_size_(cvSize(640, 480)),
  input_file_sequence_(NULL),
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
  verbose_(true),
  img_size_(cvSize(640, 480)),
  input_file_sequence_(NULL),
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
  case BundleAdjUTest: {
    string frame_file("frames10straight.xml");
    string point_file("cartesianPoints.xml");
    return testBundleAdj(point_file, frame_file, 5, 5, 10, 1, true, true, false);
    break;
  }
  case BundleAdj: {
    string frame_file("frames3.xml");
//    string frame_file("frames.xml");
//    string point_file("cartesianPoints.xml");
//    string frame_file("frames101.xml");
    string point_file("points260.xml");

    int num_free_frames  = 2;
    int num_fixed_frames = 1;
    int num_iterations   = 300;
    int num_repeats      = 1;
    return testBundleAdj(point_file, frame_file, num_free_frames,
        num_fixed_frames, num_iterations, num_repeats,
        false, false, false);
    break;
  }
  case BundleAdjSeq: {
    testBundleAdjSeq(true, true, true);
    break;
  }
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

void CvTest3DPoseEstimate::setInputData(DataSet data_set) {
  switch(data_set) {
  case SyntheticDiskLoop: {
    img_size_ = cvSize(640, 480);
    setCameraParams(389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95);
//    setCameraParams(432.0, 432.0, .088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001);
    string dirname("/home/jdchen/Data/SyntheticLoopDisk");
    string leftimgfmt("/loop-%06d-L.png");
    string rightimgfmt("/loop-%06d-R.png");
    string dispimgfmt("/dispmap-%06d.xml");
    int start = 0;
    int end   = 400;
    int step  = 1;

    // set up a FileSeq
    delete input_file_sequence_;
    input_file_sequence_ = new FileSeq;
    input_file_sequence_->setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

    // output directory
    output_data_path_ = string("Output/SyntheticDiskLoop/");
    break;
  }
  case SyntheticDiskLoopNoisy: {
    img_size_ = cvSize(640, 480);
    setCameraParams(389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95);
//    setCameraParams(432.0, 432.0, .088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001);
    string dirname("/home/jdchen/Data/SyntheticLoopDiskNoisy");
    string leftimgfmt("/loop-noisy-%06d-L.png");
    string rightimgfmt("/loop-noisy-%06d-R.png");
    string dispimgfmt("/dispmap-%06d.xml");
    int start = 0;
    int end   = 400;
    int step  = 1;

    // set up a FileSeq
    delete input_file_sequence_;
    input_file_sequence_ = new FileSeq;
    input_file_sequence_->setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

    // output directory
    output_data_path_ = string("Output/SyntheticDiskLoopNoisy/");
    break;
  }
  case SyntheticDiskLoopNoisy2: {
    img_size_ = cvSize(640, 480);
    setCameraParams(389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95);
//    setCameraParams(432.0, 432.0, .088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001);
    string dirname("/home/jdchen/Data/SyntheticLoopDiskNoisy2");
    string leftimgfmt("/loop-noisy2-%06d-L.png");
    string rightimgfmt("/loop-noisy2-%06d-R.png");
    string dispimgfmt("/dispmap-%06d.xml");
    int start = 0;
    int end   = 400;
    int step  = 1;

    // set up a FileSeq
    delete input_file_sequence_;
    input_file_sequence_ = new FileSeq;
    input_file_sequence_->setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

    // output directory
    output_data_path_ = string("Output/SyntheticDiskLoopNoisy2/");
    break;
  }
  case SyntheticLoop1: {
    img_size_ = cvSize(640, 480);
    setCameraParams(389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95);

    string dirname("/home/jdchen/Data/SyntheticLoop1");
    string leftimgfmt("/loop-%06d-L.png");
    string rightimgfmt("/loop-%06d-R.png");
    string dispimgfmt("/dispmap-%06d.xml");
    int start = 0;
    int end   = 400;
    int step  = 1;

    // set up a FileSeq
    delete input_file_sequence_;
    input_file_sequence_ = new FileSeq;
    input_file_sequence_->setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

    // output directory
    output_data_path_ = string("Output/SyntheticLoop1/");
    break;
  }
  case James4: {
    img_size_ = cvSize(640, 480);
    // The following parameters are from indoor1/proj.txt
    // note that B (or Tx) is in mm
//    setCameraParams(432.0, 432.0, 88.981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001);
    // now Tx is in meters
    setCameraParams(432.0, 432.0, .088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001);
//    string dirname("/home/jdchen/Data/VisOdom/Data/james4");
    string dirname("/u/prdata/videre-bags/james4");
    string leftimgfmt("/im.%06d.left_rectified.tiff");
    string rightimgfmt("/im.%06d.right_rectified.tiff");
    string dispimgfmt("/dispmap-%06d.xml");
    int start = 0;
    int end   = 2324;
    int step  = 1;

    // set up a FileSeq
    delete input_file_sequence_;
    input_file_sequence_ = new FileSeq;
    input_file_sequence_->setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

    // output directory
    output_data_path_ = string("Output/james4/");

    break;
  }

  default:
  case Indoor1: {
    img_size_ = cvSize(640, 480);
    // The following parameters are from indoor1/proj.txt
    // note that B (or Tx) is in mm
    setCameraParams(389.0, 389.0, 89.23*1.e-3, 323.42, 323.42, 274.95);
    string dirname("Data/indoor1");
    string leftimgfmt("/left-%04d.ppm");
    string rightimgfmt("/right-%04d.ppm");
    string dispimgfmt(".dispmap-%04d.xml");
    int start = 0;
    int end   = 1508;
    int step  = 1;

//    step = 1508;

    // set up a FileSeq
    delete input_file_sequence_;
    input_file_sequence_ = new FileSeq;
    input_file_sequence_->setInputVideoParams(dirname, leftimgfmt, rightimgfmt, dispimgfmt, start, end, step);

    // output directory
    output_data_path_ = string("Output/indoor1/");
    break;
  }
  }
}

bool CvTest3DPoseEstimate::testVideoBundleAdj() {
  bool status = false;

//  setInputData(Indoor1);
//  setInputData(James4);
//  setInputData(SyntheticDiskLoop);
    setInputData(SyntheticDiskLoopNoisy);
//    setInputData(SyntheticDiskLoopNoisy2);
  //  setInputData(SyntheticLoop1);


//  VOSparseBundleAdj sba(img_size_, 100, 100);
  VOSparseBundleAdj sba(img_size_, 3, 10);
//  VOSparseBundleAdj sba(img_size_, 1, 1);

  // parameterize the post estimator
  sba.setCameraParams(Fx_, Fy_, Tx_, Clx_, Crx_, Cy_, Du_);
  sba.mPoseEstimator.setInlierErrorThreshold(1.5);

  // visualization
#if DISPLAY
  // Optionally, set up the visualizer
  vector<FramePose*>* fp = sba.getFramePoses();
  const PointTracks& tracks = sba.getTracks();

  sba.mVisualizer = new SBAVisualizer(sba.mPoseEstimator, fp, (PointTracks *)&tracks,
      &sba.map_index_to_FramePose_);
  sba.mVisualizer->outputDirname = output_data_path_;
#endif

  if (input_file_sequence_ == NULL ||
      input_file_sequence_->getStartFrame() == false) {
    return false;
  }

  do {
   sba.track(input_file_sequence_->mInputImageQueue);
  } while(input_file_sequence_->getNextFrame() == true);

  vector<FramePose*>* framePoses = sba.getFramePoses();
  CvTestTimer& timer = CvTestTimer::getTimer();
  timer.mNumIters = input_file_sequence_->mNumFrames/input_file_sequence_->mFrameStep;

  saveFramePoses(output_data_path_, *framePoses);

  sba.printStat();
  sba.mStat2.print();

  CvTestTimer::getTimer().printStat();
  return status;
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

void synthesizePoints(const string& data_path) {
  int num_points = 260;
  double points[num_points*3];
  CvMat threeDPoints = cvMat(num_points, 3, CV_64FC1, points);
  CvRNG rng_state = cvRNG(0xffffffff);

  CvMat values;
  cvReshape(&threeDPoints, &values, 3, 0);
  cvRandArr(&rng_state, &values, CV_RAND_UNI,
      cvScalar(-5,-5,2,0), // inclusive lower bound
      cvScalar(5,5,32,0) // exclusive upper bound
  );

  string file_path = data_path+string("randomPoints.xml");
  cvSave(file_path.c_str(), &threeDPoints, "randomPoitns", "random 3d points");
}

void synthesizeFrames(const string& data_path) {
  int num_frames = 101;
  double frames[num_frames*7];
  CvMat  mat_frames = cvMat(num_frames, 7, CV_64FC1, frames);
  cvSetZero(&mat_frames);
  for (int i=0; i<num_frames; i++ ) {
    // frame id
    cvmSet(&mat_frames, i, 0, (double)i);
    // set z value
    cvmSet(&mat_frames, i, 6, (double)i*.01);
  }

  string file_path = data_path+string("frames101.xml");
  cvSave(file_path.c_str(), &mat_frames, "frames101",
      "frames in euler/meter, straight forward, .01 meter each step");
}

void setupSceeneTorusFrames(vector <FramePose*>* frames) {
  double param_data[6];
  CvMat  param = cvMat(6, 1, CV_64FC1, param_data);
  double transf_global_to_local_data[16];
  CvMat  transf_global_to_local = cvMat(4, 4, CV_64FC1, transf_global_to_local_data);
  // make sure that the last pose is back to where it starts.
  // Hence we need one extra cam.
  int numCams = 73;
  double R = 10.0; // 10  meters
  for (int iu= 0; iu < numCams; iu++) {
    double u = 2.0*CV_PI*(double)iu/(double)(numCams-1);

    // rodrigues: rotate around Y axis for angle of radian u
    param_data[0] = 0.0;
    param_data[1] = u;
    param_data[2] = 0.0;

    // as the camera has been rotated, Z axis points to tangent direction
    // and x axis is parallel to the line connects origin and camera center.
    // hence the shift shall be along X only.
    param_data[3] = R;
    param_data[4] = 0.;
    param_data[5] = 0.;

    FramePose* fp = new FramePose();
    fp->mIndex = iu;
//    CvMatUtils::transformFromEulerAndShift(&param, &transf_global_to_local);
    CvMatUtils::transformFromRodriguesAndShift(&param, &transf_global_to_local);
    CvMatUtils::invertRigidTransform(&transf_global_to_local, &(fp->transf_local_to_global_));
    frames->push_back(fp);
    printf("iu %d, fi %d, u=%f, cos=%f, sin=%f\n", iu, fp->mIndex, u, cos(u), sin(u));
    CvMatUtils::printMat(&fp->transf_local_to_global_);
  }
}

void setupSceneTorusPointsRegular(CvMat **points) {
  int numPtsPerCircle = 36;
  int numStepsForPts = 36;
  CvMat *pt = cvCreateMat(numStepsForPts*numPtsPerCircle, 3, CV_64FC1);
  *points = pt;
  double R = 10.0; // 10  meters
  double r = 1.0;  // 1.0 meters
  int i=0;
  for (int iu= 0; iu < numStepsForPts; iu++) {
    double u = 2.*CV_PI*(double)iu/(double)numStepsForPts;
    // the ones on this circle are between those on the last circle in angles.
    double offset = (iu%2==0)?0.:(.5* 2.*CV_PI/(double)numStepsForPts);
    for (int iv=0; iv < numPtsPerCircle; iv++) {
      double v = 2.0*CV_PI*(double)iv/numPtsPerCircle + offset;
      CvPoint3D64f point;
      point.x = (R + r * cos(v)) * cos(u);
      point.y = r * sin(v);
      point.z = (R + r * cos(v)) * sin(u);
      cvmSet(pt, i, 0, point.x);
      cvmSet(pt, i, 1, point.y);
      cvmSet(pt, i, 2, point.z);
      i++;
    }
  }
}

void setupSceneTorus(vector<FramePose*>* frames, CvMat** points) {
  setupSceeneTorusFrames(frames);
  setupSceneTorusPointsRegular(points);
}

void CvTest3DPoseEstimate::setUpTracks(const vector<FramePose* >& frame_poses,
    const CvMat *points,
    const CvMat& cartToDisp, int oldest_index,
    PointTracks* tracks)
{
  double left = 0;
  double upper = 0;
  double right = 640;
  double lower = 480;
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

    int fi1 = 1;
    // find the first pair that is visible in consecutive frames

    PointTrackObserv* obsv0=NULL;
    PointTrackObserv* obsv1=NULL;
    for (fi1=1; fi1<frame_poses.size(); fi1++) {
      // convert from coord to disp_coord0 and disp_coord1
      CvMatUtils::invertRigidTransform(&frame_poses[fi1-1]->transf_local_to_global_, global_to_local);
      cvMatMul(&cartToDisp, global_to_local, global_to_disp);
      cvPerspectiveTransform(&global_point, &disp_point0, global_to_disp);
      CvMatUtils::invertRigidTransform(&frame_poses[fi1  ]->transf_local_to_global_, global_to_local);
      cvMatMul(&cartToDisp, global_to_local, global_to_disp);
      cvPerspectiveTransform(&global_point, &disp_point1, global_to_disp);
      if (left  <= disp_coord0.x && disp_coord0.x < right  &&
          upper <= disp_coord0.y && disp_coord0.y < lower &&
          0< disp_coord0.z &&
          left  <= disp_coord1.x && disp_coord1.x < right  &&
          upper <= disp_coord1.y && disp_coord1.y < lower &&
          0 < disp_coord1.z ) {
        // found the first pair for a new track.
        obsv0 = new PointTrackObserv(frame_poses[fi1-1]->mIndex, disp_coord0, -1);
        obsv1 = new PointTrackObserv(frame_poses[fi1  ]->mIndex, disp_coord1, -1);
        break;
      }
    }
    if (fi1 < frame_poses.size()) {
      PointTrack* point = new PointTrack(obsv0, obsv1, coord, ipt);

      for (int iframe = fi1+1; iframe < frame_poses.size(); iframe++) {
        // extend the track
        CvMatUtils::invertRigidTransform(&frame_poses[iframe]->transf_local_to_global_, global_to_local);
        cvMatMul(&cartToDisp, global_to_local, global_to_disp);
        cvPerspectiveTransform(&global_point, &disp_point0, global_to_disp);
        if (left  <= disp_coord0.x && disp_coord0.x < right  &&
            upper <= disp_coord0.y && disp_coord0.y < lower &&
            0 < disp_coord0.z) {
          PointTrackObserv* obsv = new PointTrackObserv(frame_poses[iframe]->mIndex, disp_coord0, -1);
          point->extend(obsv);
        } else {
          // this track is not visible anymore.
          break;
        }
      }
      tracks->tracks_.push_back(point);
    } else {
      if (verbose_) {
        printf("point %d [%5.2f, %5.2f, %5.2f] is not visible\n", ipt, coord.x, coord.y, coord.z);
      }
    }

    tracks->oldest_frame_index_in_tracks_ = oldest_index;
  }
  cvReleaseMat(&global_to_disp);
  cvReleaseMat(&global_to_local);
}

/// disturb the frame parameters
void CvTest3DPoseEstimate::disturbFrames(
    vector<FramePose*>& free_frames) {
  double rod_shift[6];
  CvMat mat_rod_shift = cvMat(6, 1, CV_64FC1, rod_shift);
  double transpose_transl_data[3];
  CvMat transpose_transl = cvMat(1, 3, CV_64FC1, transpose_transl_data);

  if (verbose_)
    printf("Disturbed Free Frames\n");

  double disturb_scale = .0001;
  double sigma = disturb_scale/2.0; // ~ 95%
  int numFreeFrames = free_frames.size();
  CvMat* xyzsNoised = cvCreateMat(numFreeFrames, 6, CV_64FC1);
  cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
  int iFrames = 0;
  BOOST_FOREACH(FramePose *fp, free_frames){
    CvMatUtils::transformToRodriguesAndShift(fp->transf_local_to_global_, mat_rod_shift);
    for (int i=0; i<6; i++) {
      CV_MAT_ELEM( mat_rod_shift, double, i, 0 )
        += CV_MAT_ELEM(*xyzsNoised, double, iFrames, i);
    }
    // update the transformation matrix of fp
    CvMatUtils::transformFromRodriguesAndShift(&mat_rod_shift, &fp->transf_local_to_global_);

    if (verbose_) {
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

    iFrames++;
  }

  cvReleaseMat(&xyzsNoised);
}

void CvTest3DPoseEstimate::disturbPoints(PointTracks* tracks) {
  double disturb_scale = .0001;
  double sigma = disturb_scale/2.0; // ~ 95%
  int numPoints = tracks->tracks_.size();
  CvMat* xyzsNoised = cvCreateMat(numPoints, 3, CV_64FC1);
  cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
  int iPoints = 0;
  BOOST_FOREACH(PointTrack* p, tracks->tracks_) {
    p->coordinates_.x += cvmGet(xyzsNoised, iPoints, 0);
    p->coordinates_.y += cvmGet(xyzsNoised, iPoints, 1);
    p->coordinates_.z += cvmGet(xyzsNoised, iPoints, 2);
    if (verbose_)
      printf("point %3d, [%9.4f, %9.4f, %9.4f]\n", p->id_, p->coordinates_.x,
          p->coordinates_.y, p->coordinates_.z);
    iPoints++;
  }
  cvReleaseMat(&xyzsNoised);

  if (verbose_) {
    printf("Disturbed Points:\n");
    tracks->print();
  }
}

void CvTest3DPoseEstimate::disturbPoints(const CvMat* points, CvMat* disturbed_points) {
  double disturb_scale = .0001;
  double sigma = disturb_scale/2.0; // ~ 95%
  int numPoints = points->rows;
  CvMat* xyzsNoised = cvCreateMat(numPoints, 3, CV_64FC1);
  cvRandArr( &mRng, xyzsNoised, CV_RAND_NORMAL, cvScalar(0.0), cvScalar(sigma));
  cvAdd(points, xyzsNoised, disturbed_points);
  cvReleaseMat(&xyzsNoised);
}

void CvTest3DPoseEstimate::disturbObsvs(PointTracks* tracks) {
  CvRNG rng_state = cvRNG(0xffffffff);
  double disturb[3];
  CvMat mat_disturb = cvMat(1, 1, CV_64FC3, disturb);
//  double sigma = 1.0; // 99.99% within [-3. , +3]
  double sigma = .5; // 99.99% within [-1.5 , +1.5]

  BOOST_FOREACH(PointTrack* pt, tracks->tracks_) {
    BOOST_FOREACH(PointTrackObserv* obsv, *pt) {
      cvRandArr(&rng_state, &mat_disturb, CV_RAND_NORMAL,
          cvScalar(0.,0.,0.,0.), // mean
          cvScalar(sigma,sigma,sigma,0) // sigma
      );
      obsv->disp_coord_.x += disturb[0];
      obsv->disp_coord_.y += disturb[1];
      obsv->disp_coord_.z += disturb[2];
    }
  }
}


bool CvTest3DPoseEstimate::testBundleAdj(
    string& points_file, string& frames_file,
    int num_free_frames, int num_fixed_frames, int num_iterations,
    int num_repeats,
    bool disturb_frames, bool disturb_points, bool disturb_obsvs) {
  bool status = true;

#if 0 // set it to 1 to generate a set of random 3d points
  synthesizePoints(input_data_path_);
#endif

#if 0 // set it to 1 to generate a set of random 3d points
  synthesizeFrames(input_data_path_);
#endif

  // rows of 3d points in Cartesian space
  string points_file_path(input_data_path_);
//  point_file.append("cartesianPoints.xml");
  points_file_path.append(points_file);
  CvMat *points = (CvMat *)cvLoad(points_file_path.c_str());

  // rows of euler angle and shift
  string frames_file_path(input_data_path_);
//  frames_file.append("frames.xml");
  frames_file_path.append(frames_file);
  CvMat *frames = (CvMat *)cvLoad(frames_file_path.c_str());
  CvMat cartToDisp;
  CvMat dispToCart;

  this->setCameraParams(389.0, 389.0, 89.23*1.e-3, 323.42, 323.42, 274.95);
  this->getProjectionMatrices(&cartToDisp, &dispToCart);

  // set up cameras
  vector<FramePose* > frame_poses;
//  int index_offset = 9;
  int oldest_index = numeric_limits<int>::max();
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
    frame_poses.push_back(fp);
  }

  // set up tracks
  PointTracks tracks;
  setUpTracks(frame_poses, points, cartToDisp, oldest_index, &tracks);

  int full_free_window_size  = num_free_frames;
  int full_fixed_window_size = num_fixed_frames;
  int max_num_iters = num_iterations;
  double epsilon = DBL_EPSILON;
//  epsilon = FLT_EPSILON;
//  epsilon = LDBL_EPSILON;
//  epsilon = .1e-12;
  CvTermCriteria term_criteria =
    cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,max_num_iters,epsilon);

  LevMarqSparseBundleAdj sba(&dispToCart, &cartToDisp,
      full_fixed_window_size, full_free_window_size, term_criteria);

  vector<FramePose*> free_frames;
  vector<FramePose*> fixed_frames;

  // the fixed windows
  int fi=0;
  for (fi=0; fi<full_fixed_window_size; fi++) {
    fixed_frames.push_back(frame_poses[fi]);
  }
  // the free windows
  for (; fi < full_fixed_window_size + full_free_window_size; fi++) {
    free_frames.push_back(frame_poses[fi]);
  }

  for (int k=0; k < num_repeats; k++ ) {
    // disturb the point parameters.
    if (disturb_points == true) {
      disturbPoints(&tracks);
    }

    int numFreeFrames = free_frames.size();
    CvMat* frame_params_input = cvCreateMat(numFreeFrames, 6, CV_64FC1);

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

    double transpose_transl_data[3];
    CvMat transpose_transl = cvMat(1, 3, CV_64FC1, transpose_transl_data);

    if (disturb_frames == true) {
      disturbFrames(free_frames);
    }

    if (disturb_obsvs == true) {
      disturbObsvs(&tracks);
    }

#if SAVE_FRAMES_POINTS==1
    string track_file("tracks.txt");
    string output_dir(output_data_path_);
    tracks.save(output_dir);

    tracks.saveInOneFile(string(output_dir).append(track_file), true);
    tracks.print();

    // for experimental purpose, save all framepose here
    saveFramePosesNonXML(output_dir, frame_poses);
#endif

    LevMarqSparseBundleAdj::ErrorCode err_code = sba.optimize(&fixed_frames, &free_frames, &tracks);

    if (err_code == LevMarqSparseBundleAdj::InputError) {
      cout << "Input error: either fixed win or free win is empty" << endl;
    }
    cout << "initial cost: " << sba.initial_cost_ << " final cost: " << sba.accepted_cost_ << endl;

    if (verbose_) {
      // print some output
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
    }

    // compare frame params
    double error_norm_frames;
    CvMat* frame_params_est   = (CvMat*) cvClone(frame_params_input);
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

      error_norm_frames = cvNorm(frame_params_input, frame_params_est, CV_RELATIVE_L2);

      status = status && (error_norm_frames < 1.e-7);

    }

    CvMat* points_est = (CvMat*)cvClone(points);
    int iPoints=0;
    // note: not all the points are in the tracks. Some of them may not
    // be visible.
    BOOST_FOREACH(const PointTrack* p, tracks.tracks_) {
      if (verbose_) {
        printf("point %3d, [%17.10f, %17.10f, %17.10f]\n",
            p->id_, p->coordinates_.x, p->coordinates_.y, p->coordinates_.z);
      }
      cvmSet(points_est, p->id_, 0, p->coordinates_.x);
      cvmSet(points_est, p->id_, 1, p->coordinates_.y);
      cvmSet(points_est, p->id_, 2, p->coordinates_.z);
      iPoints++;
    }

    // compare input points and estimated points.
    double error_norm_points = cvNorm(points, points_est, CV_RELATIVE_L2);

    status = status && (error_norm_points < 1.e-8);

    if (verbose_ == true) {
      if (status == true) {
        printf("testBundleAdj() passed\n");
      } else {
        printf("testBundleAdj() failed\n");
      }
      printf("Number of good updates: %d\n", sba.num_good_updates_);
      printf("Number of retractions:  %d\n", sba.num_retractions_);

      printf("Relative L2 Norm of Error vector for frames: %e\n", error_norm_frames);
      printf("Relative L2 Norm of Error vector for points: %e\n", error_norm_points);
    }

    // cleanup
    cvReleaseMat(&frame_params_input);
    cvReleaseMat(&frame_params_est  );
    cvReleaseMat(&points_est);
  } // num_tries

  // clean up
  /// @todo remove the points;

  // remove the frames
  BOOST_FOREACH(FramePose* fp, frame_poses) {
    delete fp;
  }

  cvReleaseMat(&frames);
  cvReleaseMat(&points);

  if (verbose_ == true) {
    CvTestTimer& timer = CvTestTimer::getTimer();
    timer.mNumIters = 1;
    CvTestTimer::getTimer().printStat();
  }

  return status;
}

int CvTest3DPoseEstimate::selectTracks(const vector<FramePose*>* frames,
    const PointTracks* all_tracks, PointTracks* tracks){
  int numSelectedTracks = 0;
  boost::unordered_set<int> frame_index_set;
  BOOST_FOREACH(const FramePose* fp, *frames) {
    frame_index_set.insert(fp->mIndex);
  }
  BOOST_FOREACH(const PointTrack* pt, all_tracks->tracks_) {
    // 0: not finding the first frame in track yet
    // 1: found the first frame in track
    // 2: found the next frame in track
    // 3: found the first frame not in track after some in track
    //    frames.
    int state = 0;
    PointTrackObserv* obsv0 = NULL;
    PointTrackObserv* obsv1 = NULL;
    PointTrack* point_track = NULL;
    BOOST_FOREACH(const PointTrackObserv* obsv, *pt) {
      if (frame_index_set.find(obsv->frame_index_) == frame_index_set.end()) {
        switch(state) {
        case 0:
          // still not finding the first frame in the track yet.
          break;
        case 1:
          // unfortunately the next frame is not in track.
          state = 0;
          delete obsv0;
          obsv0 = NULL;
          break;
        case 2:
          // found the first frame not in track after some in track.
          state = 3;
          break;
        }
      } else {
        switch(state) {
        case 0:
          // found the first frame in track
          state = 1;
          // make a copy of obsv
          obsv0 = new PointTrackObserv(obsv->frame_index_, obsv->disp_coord_, obsv->keypoint_index_);
          break;
        case 1:
          // found the next frame in track. Create a track in tracks
          state = 2;
          obsv1 = new PointTrackObserv(obsv->frame_index_, obsv->disp_coord_, obsv->keypoint_index_);
          point_track = new PointTrack(obsv0, obsv1, pt->coordinates_, pt->id_);
          tracks->tracks_.push_back(point_track);
          numSelectedTracks++;
          break;
        case 2:
          // extend the track.
          obsv0 = new PointTrackObserv(obsv->frame_index_, obsv->disp_coord_, obsv->keypoint_index_);
          point_track->extend(obsv0);
          break;
        }
      }
      if (state == 3) {
        // done with this track
        break;
      }
    }
  }
  return numSelectedTracks++;
}

bool CvTest3DPoseEstimate::testBundleAdjSeq(
    bool disturb_frames, bool disturb_points, bool disturb_obsvs) {
  bool status = true;

  CvMat cartToDisp;
  CvMat dispToCart;

  this->setCameraParams(389.0, 389.0, 89.23*1.e-3, 323.42, 323.42, 274.95);
  this->getProjectionMatrices(&cartToDisp, &dispToCart);
  PoseEstimateDisp peDisp;
  peDisp.setCameraParams(Fx_, Fy_, Tx_, Clx_, Crx_, Cy_);

  boost::unordered_map<int, FramePose*> map_index_to_framepose;
  SBAVisualizer vis(peDisp, NULL, NULL, NULL);
  // note that SBAVisualizer by design does not own map_index_to_FramePose
  vis.map_index_to_FramePose_ = &map_index_to_framepose;
  vis.outputDirname = string("/tmp");

  // set up cameras and points
  vector<FramePose* > frame_poses;
  CvMat* points;

  setupSceneTorus(&frame_poses, &points);

  // set up all the tracks w.r.t. all the frames
  PointTracks all_tracks;
  setUpTracks(frame_poses, points, cartToDisp, 0, &all_tracks);

  cout << "All Tracks"<<endl;
  all_tracks.print();

  vector<FramePose*> disturbed_frames;
  BOOST_FOREACH(FramePose* fp, frame_poses){
    FramePose* fp_disturbed = new FramePose(*fp);
    disturbed_frames.push_back(fp_disturbed);
  }
  if (disturb_frames == true) {
    disturbFrames(disturbed_frames);
  }

  if (disturb_points == true) {
    disturbPoints(&all_tracks);
  }

  if (disturb_obsvs == true) {
    disturbObsvs(&all_tracks);
  }

  int full_fixed_win_size = 1;
  int full_free_win_size  = 1;
  int max_num_iters = 100;
  double epsilon = DBL_EPSILON;
//  epsilon = FLT_EPSILON;
//  epsilon = LDBL_EPSILON;
//  epsilon = .1e-12;
  CvTermCriteria term_criteria =
    cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,max_num_iters,epsilon);

  LevMarqSparseBundleAdj sba(&dispToCart, &cartToDisp,
      full_fixed_win_size, full_free_win_size, term_criteria);

  vector<FramePose*> free_frames;
  vector<FramePose*> fixed_frames;
  vector<FramePose*> active_frames;
  for (unsigned int i=1; i< frame_poses.size(); i++) {
    free_frames.clear();
    fixed_frames.clear();
    active_frames.clear();
    active_frames.clear();
    if (i >= full_fixed_win_size+full_free_win_size-1) {
      // full windows
      // first the fixed frames
      int offset = i-full_fixed_win_size-full_free_win_size+1;
      for (int j=0; j<full_fixed_win_size; j++) {
        fixed_frames.push_back(frame_poses[offset+j]);
        active_frames.push_back(frame_poses[offset+j]);
      }
      offset = i-full_free_win_size+1;
      for (int j=0; j<full_free_win_size; j++) {
        free_frames.push_back(frame_poses[offset+j]);
        active_frames.push_back(frame_poses[offset+j]);
      }
    } else {
      // frame 0 shall be in as a fixed frame
      fixed_frames.push_back(frame_poses[0]);
      int num_fixed_frames = i+1 - full_free_win_size;
      if (num_fixed_frames>1) {
        for (int j=1; j<num_fixed_frames; j++) {
          fixed_frames.push_back(frame_poses[j]);
        }
        for (int j=num_fixed_frames; j<=i; j++) {
          free_frames.push_back(frame_poses[j]);
        }
      } else {
        for (int j=1; j<=i; j++) {
          free_frames.push_back(frame_poses[j]);
        }
      }
      for (int j=0; j<=i; j++) {
        active_frames.push_back(frame_poses[j]);
      }
    }
    cout << "fixed frames size " << fixed_frames.size() << " [";
    BOOST_FOREACH(FramePose* fp, fixed_frames) {
      cout << fp->mIndex << " ";
    }
    cout << "]"<<endl;
    cout << "free frames size "<<free_frames.size() <<" [";
    BOOST_FOREACH(FramePose* fp, free_frames) {
      cout << fp->mIndex << " ";
    }
    cout << "]"<<endl;
    // select the tracks that are visible to current frame windows
    PointTracks tracks;
    int numSelectedTracks = selectTracks(&active_frames, &all_tracks, &tracks);
    cout << "selected "<< numSelectedTracks << " tracks" << endl;

    if (numSelectedTracks == 0) {
      all_tracks.print();
    }

    sba.optimize(&fixed_frames, &free_frames, &tracks);

    vis.show(NULL, fixed_frames, free_frames, tracks);
    vis.save();

    BOOST_FOREACH(PointTrack* pt, tracks.tracks_) {
      delete pt;
    }
  }

  cout << "Final Pose"<<endl;
  CvMatUtils::printMat(&frame_poses.back()->transf_local_to_global_);

  BOOST_FOREACH(FramePose* fp, frame_poses){
    delete fp;
  }
  BOOST_FOREACH(FramePose* fp, disturbed_frames) {
    delete fp;
  }
  //    remove the tracks
  BOOST_FOREACH(PointTrack* pt, all_tracks.tracks_) {
    delete pt;
  }
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
    this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
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
    this->setCameraParams(389.0, 389.0, 89.23, 323.42, 323.42, 274.95);
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

