/*
 * VisOdom.cpp
 *
 *  Created on: Oct 1, 2008
 *      Author: jdchen
 */

#include "VisOdom.h"

#include "KeypointDescriptors.h"

#include "PathRecon.h"
#include "PoseEstimateDisp.h"

using namespace cv::willow;

#include "iostream"
using namespace std;

namespace cv { namespace willow {
CamTracker* CamTracker::getCamTracker(
    const CamTrackerType type,
    CvSize& imgSize,
    double Fx,  double Fy,  double Tx,
    double Clx, double Crx, double Cy) {
  switch(type) {
  case Pairwise:
  {
    PathRecon* pathRecon = new PathRecon(imgSize);
    pathRecon->mPoseEstimator.setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
    return pathRecon;
  }
  default:
    cerr << "not implement yet for this type of camtracker: "<< type<<endl;
  }
  return NULL;
}

#if 0 //TODO: remove it
bool goodFeaturesToTrack(
    CamTracker* tracker,
    /// input image
    const WImage1_b& img,
    /// disparity map of the input image
    const WImage1_16s* dispMap,
    /// (OUTPUT) key point detected
    Keypoints*& keypoints
) {
  PathRecon* t = (PathRecon*)tracker;
  return t->goodFeaturesToTrack(img, dispMap, keypoints);
}
#endif

/// Use Harris corner to extrack keypoints.
/// @return the number of key points detected
int goodFeaturesToTrackHarrisCorner(
    /// input image
    const WImage1_b& image,
    /// Temporary floating-point 32-bit image of the same size as image.
    WImage1_f& eigImg,
    /// Another temporary image of the same size and same format as eig_image.
    WImage1_f& tempImg,
    /// (OUTPUT) key point detected
    Keypoints& keypoints,
    /// (on entry) max number of key points
    /// (on exit ) the number of key points returned in keypoints
    int numKeypoints,
    /// threshold value (between 0.0 and 1.0)
    double qualityThreshold,
    /// minimum distance between the key points
    double minDistance,
    /// Region of interest. The function selects points either in the specified
    /// region or in the whole image if the mask is NULL.
    CvArr* mask,
    /// neighborhood size @see cvCornerHarris. @see CornerEigenValsAndVecs.
    /// The default value was 3 in cvGoodFeatuesToTrack()
    int blockSize,
    /// the free parameter in Harris corner @see cvGoodFeaturesToTrack()
    /// @see cvCornerHarris(). The default value is .04 in cvGoodFeaturesToTrack()
    double k
) {
  return PoseEstimateStereo::goodFeaturesToTrackHarrisCorner(image, eigImg, tempImg, keypoints,
      numKeypoints, qualityThreshold, minDistance, mask, blockSize, k);
}

bool matchKeypoints(
    /// type of the key point matcher
    MatchMethod matcherType,
    /// input image 0
    WImage1_b& img0,
    /// input image 1
    WImage1_b& img1,
    /// disparity map of input image 0
    WImage1_16s& dispMap0,
    /// disparity map of input image 1
    WImage1_16s& dispMap1,
    /// Detected key points in image 0
    Keypoints& keyPoints0,
    /// Detected Key points in image 1
    Keypoints& keyPoints1,
    /// (Output) pairs of corresponding 3d locations for possibly the same
    /// 3d features. Used for pose estimation.
    /// Set it to NULL if not interested.
    vector<pair<CvPoint3D64f, CvPoint3D64f> >* matchPairs,
    /// (Output) pairs of indices, to the input keypoints, of the corresponding
    /// 3d locations for possibly the same 3d features. Used for pose estimation.
    /// Set it to NULL if not interested.
    vector<pair<int, int> >* matchIndexPairs
) {
  return PoseEstimateStereo::getTrackablePairs(matcherType, img0, img1,
      dispMap0, dispMap1, keyPoints0, keyPoints1, matchPairs, matchIndexPairs);
}

#if 0 // TODO: delete it
PoseEstimator* getPoseEstimator(
    /// The size of the image this pose estimator is expected to see
    CvSize& imgSize,
    /// focal length in x
    double Fx,
    /// focal length in y
    double Fy,
    /// baseline length
    double Tx,
    /// x coordinate of the optical center of the left cam
    double Clx,
    /// x coordinate of the optical center of the right cam
    double Crx,
    /// y coordinate optical center
    double Cy){
  PoseEstimateDisp* pe = new PoseEstimateDisp();
  pe->setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
  // when we use harris corner detector, we need to
  // set the threshold larger so that we can pick up enough key point match.
  pe->setInlierErrorThreshold(4.0);

  return pe;
}
#endif
#if 0 //TODO: delete it
PoseEstimator* getPoseEstimator(CamTracker* tracker) {
  PathRecon *t = (PathRecon *)tracker;
  return &(t->mPoseEstimator);
}
#endif

#if 0 // TODO: delete it
int poseEstimate(
    PoseEstimator* poseEstimator,
    Keypoints& keypoints0,
    Keypoints& keypoints1,
    vector<pair<int, int> >& matchIndexPairs,
    CvMat& rot,
    CvMat& shift,
    bool smoothed) {
  return ((PoseEstimateDisp*)poseEstimator)->estimate(keypoints0, keypoints1,
      matchIndexPairs,  rot, shift, smoothed);
}
#endif

void estimateWithLevMarq(
    /// inlier list 0
    const CvMat& points0inlier,
    /// inlier list 1
    const CvMat& points1inlier,
    /// transformation matrix from Cartesian coordinates to disparity coordinates
    const CvMat& CartToDisp,
    /// transformation matrix from disparity coordinates to Cartesian coordinates.
    const CvMat& DispToCart,
    CvMat& rot, CvMat& trans) {
  PoseEstimateDisp::estimateWithLevMarq(points0inlier, points1inlier, CartToDisp,
      DispToCart, rot, trans);
}
#if 0 // TODO: remove it
KeyFramingDecision keyFrameEval(
    CamTracker* camTracker,
    /// the index of this frame
    int frameIndex,
    /// number of key points detected in this frame.
    int numKeypoints,
    /// number of inliers detected in this frame with respect to last key frame.
    int numInliers,
    /// estimated rotation matrix.
    const CvMat & rot,
    /// estimated translation matrix.
    const CvMat & shift){
  assert(camTracker);
  PathRecon* t = (PathRecon *)camTracker;
  return t->keyFrameEval(frameIndex, numKeypoints, numInliers, rot, shift);
}
#endif

bool FrameSeq::backTrack(){
  bool status;
  assert(mNextFrame.get() == NULL);
  assert(mCurrentFrame != NULL);
#ifdef DEBUG
  cerr << "Going back to last good frame  from frame "<<mCurrentFrame->mFrameIndex<<endl;
  cerr << "Last good frame is "<<mLastGoodFrame->mFrameIndex << endl;
#endif
  if (mLastGoodFrame == NULL) {
    // nothing the backtrack to, may at the begining
    status = false;
  } else {
    mNextFrame.reset(mCurrentFrame);
    mCurrentFrame  = mLastGoodFrame;
    mLastGoodFrame = NULL;
    status = true;
  }
  return status;
}
void FrameSeq::releasePoseEstFrameEntry( PoseEstFrameEntry** frameEntry ) {
  if (*frameEntry)
  delete *frameEntry;
  *frameEntry = NULL;
}
void PoseEstFrameEntry::clear() {
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
  delete mTrackableIndexPairs;
  mTrackableIndexPairs = NULL;
  delete mInlierIndices;
  mInlierIndices = NULL;
}

PoseEstFrameEntry::~PoseEstFrameEntry(){
  clear();
}
#if 0 // TODO: delete them
void setLastKeyFrame(CamTracker* tracker, PoseEstFrameEntry* keyFrame){
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->setLastKeyFrame(keyFrame);
}
void matchKeypoints(
    /// pointer to the tracker
    CamTracker* tracker,
    /// (Output) coordinates of trackable pairs
    vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
    /// (Output and Optional) indices of the trackable pairs into
    /// their corresponding key points.
    vector<pair<int, int> >*  trackableIndexPairs){
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->matchKeypoints(trackablePairs, trackableIndexPairs);
}
PoseEstFrameEntry* getLastKeyFrame(CamTracker* tracker) {
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  if (t->mActiveKeyFrames.size() == 0)
    return NULL;
  return t->mActiveKeyFrames.back();
}
bool fetchInliers(CamTracker* tracker, CvMat *& inliers0, CvMat *& inliers1){
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->fetchInliers(inliers0, inliers1);
}
int *fetchInliers(CamTracker* tracker) {
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->fetchInliers();
}
bool keyFrameAction(CamTracker* tracker, KeyFramingDecision kfd, FrameSeq& frameSeq){
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->keyFrameAction(kfd, frameSeq);
}

FrameSeq& getFrameSeq(const CamTracker* tracker) {
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->mFrameSeq;
}

vector<FramePose>* getFramePoses(CamTracker* tracker){
  assert(tracker);
    PathRecon* t = (PathRecon*)tracker;
    return t->getFramePoses();
}
#endif

void saveFramePoses(const string& dirname, vector<FramePose>& framePoses) {
  // TODO: for now, turn poses into a CvMat of numOfKeyFrames x 7 (index, rod[3], shift[3])
  double _poses[framePoses.size()*7];
  CvMat  _framePoses = cvMat(framePoses.size(), 7, CV_64FC1, _poses);
  int i=0;
  for (vector<FramePose>::const_iterator iter= framePoses.begin(); iter!=framePoses.end(); iter++,i++) {
    _poses[i*7 + 0] = iter->mIndex;
    _poses[i*7 + 1] = iter->mRod.x;
    _poses[i*7 + 2] = iter->mRod.y;
    _poses[i*7 + 3] = iter->mRod.z;
    _poses[i*7 + 4] = iter->mShift.x;
    _poses[i*7 + 5] = iter->mShift.y;
    _poses[i*7 + 6] = iter->mShift.z;
  }
  if (i>0) {
    string framePosesFilename("framePoses.xml");
    cvSave((dirname+framePosesFilename).c_str(), &_framePoses, "index-rod3-shift3", "indices, rodrigues and shifts w.r.t. starting frame");
  }
}

CvMat* dispMapToMask(const WImage1_16s& dispMap) {
  return CvMatUtils::dispMapToMask(dispMap);
}

void FileSeq::setInputVideoParams(const string& dirname, const string& leftFileFmt,
    const string& rightFileFmt, const string& dispmapFileFmt, int start, int end, int step)
{
    mDirname = dirname;
    mLeftImageFilenameFmt = dirname;
    mLeftImageFilenameFmt += leftFileFmt;
    mRightImageFilenameFmt = dirname;
    mRightImageFilenameFmt += rightFileFmt;
    mDisparityMapFilenameFmt = dirname;
    mDisparityMapFilenameFmt += dispmapFileFmt;
    mStartFrameIndex = start;
    mNumFrames = end - start;
    mEndFrameIndex = end;
    mFrameStep = step;
}

bool FileSeq::getCurrentFrame() {
  bool status = true;
  StereoFrame stereoFrame;
  stereoFrame.mImage = new WImageBuffer1_b;
  stereoFrame.mRightImage = new WImageBuffer1_b;
  stereoFrame.mDispMap = NULL;
  CvMatUtils::loadStereoImagePair(mDirname, mLeftImageFilenameFmt, mRightImageFilenameFmt,
      mDisparityMapFilenameFmt, mCurrentFrameIndex, stereoFrame.mImage,
      stereoFrame.mRightImage,
      stereoFrame.mDispMap);
  stereoFrame.mFrameIndex = mCurrentFrameIndex;
  mInputImageQueue.push(stereoFrame);
  return status;
}
bool FileSeq::getStartFrame() {
  // load the first stereo pair and put them into the queue
  mCurrentFrameIndex = mStartFrameIndex;
  return getCurrentFrame();
}

bool FileSeq::getNextFrame() {
  // load next frame only when the queue is empty
  if (mInputImageQueue.size()==0) {
    mCurrentFrameIndex += mFrameStep;

    if (mCurrentFrameIndex < mEndFrameIndex ) {
      return getCurrentFrame();
    } else {
      return false;
    }
  } else
    return false;
}

PoseEstimator* getStereoPoseEstimator(
      /// The size of the image this pose estimator is expected to see
      CvSize& imgSize,
      /// focal length in x
      double Fx,
      /// focal length in y
      double Fy,
      /// baseline length
      double Tx,
      /// x coordinate of the optical center of the left cam
      double Clx,
      /// x coordinate of the optical center of the right cam
      double Crx,
      /// y coordinate optical center
      double Cy) {
  PoseEstimateStereo* pe = new PoseEstimateStereo(imgSize.width, imgSize.height);
  pe->setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy);
  return pe;
}

void KeypointDescriptor::constructTemplateDescriptors(
    /// input image
    const uint8_t* img,
    int width,
    int height,
    /// The list of keypoints
    Keypoints& keypoints,
    int matchMethod
){
  KeypointTemplateDescriptor::constructDescriptors(img, width, height, keypoints);
}

void KeypointDescriptor::constructSADDescriptors(
     /// input image
     const uint8_t* img,
     int width,
     int height,
     /// The list of keypoints
     Keypoints& keypoints,
     /// buffer used by this function. Same size as img
     uint8_t* bufImg1,
     /// buffer used by this function. Same size as img
     uint8_t* bufImg2
 ){
  KeypointSADDescriptor::constructDescriptors(img, width, height, keypoints, bufImg1, bufImg2);
}

}  // namespace willow
}  // namespace cv
