/*
 * VisOdom.cpp
 *
 *  Created on: Oct 1, 2008
 *      Author: jdchen
 */

#include "VisOdom.h"

#include "PathRecon.h"
#include "PoseEstimateDisp.h"

using namespace cv::willow;

#include "iostream"
using namespace std;

namespace cv { namespace willow {
CamTracker* getCamTracker(
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


bool trackCameras(
    const CamTracker* tracker,
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
    int step,
    vector<FramePose>*& framePoses){
  if (tracker==NULL) {
    cerr << "no tracker"<<endl;
    return false;
  }
  PathRecon* t = (PathRecon *)tracker;
  return t->recon(dirname, leftFileFmt, rightFileFmt, start, end, step, framePoses);
}

bool goodFeaturesToTrack(
    const WImage1_b& img,
    const WImage1_16s* dispMap,
    double disparityUnitInPixels,
    Keypoints& keypoints,
    CvMat* eigImg,
    CvMat* tempImg
) {
  return PoseEstimateStereo::goodFeaturesToTrack(img, dispMap, disparityUnitInPixels, keypoints,
      eigImg, tempImg);
}

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

PoseEstimator* getPoseEstimator(CamTracker* tracker) {
  PathRecon *t = (PathRecon *)tracker;
  return &(t->mPoseEstimator);
}

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

void estimateWithLevMarq(
    PoseEstimator* poseEstimator,
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

void printStat(const CamTracker*tracker){
  assert(tracker);
  PathRecon* pr = (PathRecon*)tracker;
  pr->mStat.print();
}

void FrameSeq::backTrack(){
  assert(mNextFrame.get() == NULL);
  assert(mLastGoodFrame != NULL);
  assert(mCurrentFrame != NULL);
#ifdef DEBUG
  cerr << "Going back to last good frame  from frame "<<mCurrentFrame->mFrameIndex<<endl;
  cerr << "Last good frame is "<<mLastGoodFrame->mFrameIndex << endl;
#endif
  mNextFrame.reset(mCurrentFrame);
  mCurrentFrame  = mLastGoodFrame;
  mLastGoodFrame = NULL;
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

bool loadStereoFrame(
    /// pointer to the tracker object
    CamTracker* camTracker,
    /// frame index
    int frameIndex,
    /// (Output) the left image loaded
    WImageBuffer1_b* & leftImage,
    /// (Output) disparity map.
    WImageBuffer1_16s* & dispMap) {
  assert(camTracker);
  PathRecon* t = (PathRecon*)camTracker;
  return t->loadStereoFrame(frameIndex, leftImage, dispMap);
}
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
  return t->getTrackablePairs(trackablePairs, trackableIndexPairs);
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

void setInputVideoParams(
    /// The tracker
    const CamTracker* tracker,
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
    int step
) {
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->setInputVideoParams(dirname, leftFileFmt, rightFileFmt, start, end, step);
}

FrameSeq& getFrameSeq(const CamTracker* tracker) {
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->mFrameSeq;
}

void deleteAllButLastKeyFrame(CamTracker* tracker) {
  assert(tracker);
  PathRecon* t = (PathRecon*)tracker;
  return t->deleteAllButLastFrame();
}

vector<FramePose>* getFramePoses(CamTracker* tracker){
  assert(tracker);
    PathRecon* t = (PathRecon*)tracker;
    return &t->mFramePoses;
}

}  // namespace willow
}  // namespace cv
