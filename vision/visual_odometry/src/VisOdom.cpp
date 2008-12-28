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

#include <boost/foreach.hpp>
#include <Eigen/Geometry>
// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#include <iostream>
#include <fstream>
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

bool FrameSeq::backTrack(){
  bool status;
  assert(mNextFrame.get() == NULL);
#if DEBUG==1
  if (mCurrentFrame) {
    cout << "Going back to last good frame  from frame "<<mCurrentFrame->mFrameIndex<<endl;
  } else {
    cout << "Going back to last good frame  from off the end"<<endl;
  }
  if (mLastGoodFrame) {
    cout << "Last good frame is "<<(mLastGoodFrame?mLastGoodFrame->mFrameIndex:-1) << endl;
  }
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

void FrameSeq::print() {
  printf("FrameSeq:  NumFrames=%d, LastGoodFrame=%d, CurrentFrame=%d, NextFrame=%d\n",
      mNumFrames, (mLastGoodFrame)?mLastGoodFrame->mFrameIndex:-1,
      (mCurrentFrame)?mCurrentFrame->mFrameIndex:-1,
      mNextFrame.get()? mNextFrame.get()->mFrameIndex:-1);
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

void saveFramePoses(const string& dirname, const vector<FramePose*>& framePoses) {

  // @TODO: for now, turn poses into a CvMat of numOfKeyFrames x 7 (index, rod[3], shift[3])
  double _poses[framePoses.size()*7];
  CvMat  _framePoses = cvMat(framePoses.size(), 7, CV_64FC1, _poses);
  int i=0;
  for (vector<FramePose*>::const_iterator iter= framePoses.begin(); iter!=framePoses.end(); iter++,i++) {
    FramePose* fp = *iter;

    // make sure the transformation matrix and the rodrigues and shift vectors are in sync
    double params_local_to_global_data[6];
    CvMat  params_local_to_global = cvMat(6, 1, CV_64FC1, params_local_to_global_data);
    CvMatUtils::transformToRodriguesAndShift(fp->transf_local_to_global_, params_local_to_global);

    _poses[i*7 + 0] = fp->mIndex;
    _poses[i*7 + 1] = params_local_to_global_data[0];
    _poses[i*7 + 2] = params_local_to_global_data[1];
    _poses[i*7 + 3] = params_local_to_global_data[2];
    _poses[i*7 + 4] = params_local_to_global_data[3];
    _poses[i*7 + 5] = params_local_to_global_data[4];
    _poses[i*7 + 6] = params_local_to_global_data[5];
  }
  if (i>0) {
    string framePosesFilename("framePoses.xml");
    cvSave((dirname+framePosesFilename).c_str(), &_framePoses, "index-rod3-shift3", "indices, rodrigues and shifts w.r.t. starting frame");
  }
}

/// save frame poses from global to local, in plain text format.
void saveFramePosesInPlainText(
    const string& filename,
    const vector<FramePose*>& framePoses,
    bool global_to_local) {
  std::fstream out(filename.c_str(), ios::out);

  cout << "saving frames (global to local) in file "<< filename.c_str() << endl;

  out << "# global to local transformations." << std::endl;
  out << "# quaternion (w x y z) translation (x y z)"<<std::endl;
  int i=0;
  double transf_global_to_local_data[16];
  CvMat transf_global_to_local = cvMat(4,4, CV_64FC1, transf_global_to_local_data);
  for (vector<FramePose*>::const_iterator iter= framePoses.begin(); iter!=framePoses.end(); iter++,i++) {
    FramePose* fp = *iter;

    double* transf_data=NULL;
    if (global_to_local) {
      CvMatUtils::invertRigidTransform(&fp->transf_local_to_global_, &transf_global_to_local);
      transf_data = transf_global_to_local_data;
    } else {
      transf_data = fp->transf_local_to_global_data_;
    }

    /// @todo why that Matrix is column-major while the RowMajorBit is set?
    // it seems to me that Matrix is column-major. Hence we need to transpose it.
//    Eigen::Matrix4d transf_eg = Eigen::Map<Eigen::Matrix4d>(transf_data);
    Eigen::Matrix4d transf_eg(transf_data);
    Eigen::Quaterniond quatd(transf_eg.transpose().block<3,3>(0,0));

    // quaternion
    out<<quatd.w()<<" "<<quatd.x()<<" "<<quatd.y()<<" "<<quatd.z()<<" ";
    // translation
    out<< transf_eg.transpose()(0,3) << " ";
    out<< transf_eg.transpose()(1,3) << " ";
    out<< transf_eg.transpose()(2,3) << endl;

#if 0 /// @todo remove me. Just to check if eigen matrix is row-major or col-major
    // it seems to me that it is col-major. However it reports that it is row-major
    // make sure the transformation matrix and the rodrigues and shift vectors are in sync
    double params_local_to_global_data[7];
    CvMat  params_local_to_global = cvMat(7, 1, CV_64FC1, params_local_to_global_data);
    CvMat  transf = cvMat(4, 4, CV_64FC1, transf_data);

    cout << "quaternion and shift"<<endl;
    CvMatUtils::transformToQuaternionAndShift(&transf, &params_local_to_global);
    CvMatUtils::printMat(&params_local_to_global);
    cout << endl;
    CvMatUtils::printMat(&transf_global_to_local);
    cout << endl;
    for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
        printf("%f ", transf_eg(i,j));
      }
      cout<<endl;
    }
    cout<<endl;
    for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
        printf("%f ", transf_global_to_local_data[i*4+j]);
      }
      cout<<endl;
    }
    cout<< "Row Major Bit: "<<Eigen::RowMajorBit << endl;
#endif
  }
}

/// print frame pose to screen
void printFramePoses(
    vector<FramePose*>& frames) {
  double rod_shift[6];
  CvMat mat_rod_shift = cvMat(6, 1, CV_64FC1, rod_shift);
  double transpose_transl_data[3];
  CvMat transpose_transl = cvMat(1, 3, CV_64FC1, transpose_transl_data);

  BOOST_FOREACH(FramePose *fp, frames){
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

    if (mCurrentFrameIndex <= mEndFrameIndex ) {
      return getCurrentFrame();
    } else {
      if (mCurrentFrameIndex <= mEndFrameIndex + mFrameStep) {
        // signaling the end of the seq
        StereoFrame stereoFrame;
        stereoFrame.mFrameIndex = -1;
        mInputImageQueue.push(stereoFrame);
        return true;
      } else {
        // last time we signal the end of the sequence.
        // now no more frame
        return false;
      }
    }
  } else
    return true;
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
