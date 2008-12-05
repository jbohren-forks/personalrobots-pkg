/*
 * VisOdom.h
 *
 *  Created on: Oct 1, 2008
 *      Author: jdchen
 */

#ifndef VISODOM_H_
#define VISODOM_H_

#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <memory>
using namespace std;

#include <opencv/cxcore.h>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>


namespace cv { namespace willow {

/// Default value of disparity unit (in pixels) in disparity map
static const double DefDisparityUnitInPixels = 16.;
/// The minimum number needed to do tracking between two cams
static const int defMinNumTrackablePairs = 10;

class Keypoint;

/// Disparity coordinates of the key points
/// x, y, z in class CvPoint3D64f is used to represent x, y, d (or u, v, d)
//typedef vector<CvPoint3D64f> Keypoints;
typedef vector<Keypoint> Keypoints;

/// An abstract class for keypoint descriptors;
class KeypointDescriptor {
public:
  virtual ~KeypointDescriptor(){};

  /// @return the "distance" between the two
  /// descriptors
  virtual float compare(const KeypointDescriptor& kpd) const = 0 ;
  static void constructTemplateDescriptors(
      /// input image
      const uint8_t* img,
      int width,
      int height,
      /// The list of keypoints
      Keypoints& keypoints,
      /// match method, @see cvMatchTemplate()
      int matchMethod = CV_TM_CCORR_NORMED
  );
  static void constructSADDescriptors(
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
  );
};

/// Key point extracted from (stereo) images.
/// - For one single 2D image, x, y are interpreted as u and v, the image location
/// of the feature point.
/// - For stereo pair, the class members x, y, z are interpreted as
/// u, v, d, where u and v are the image coordinate on the left image and d
/// is the disparity.
/// - For spin image, or other representations that may want to specify a key point
/// in space, x,y,z can be interpreted as the Cartesian coordinates.
class Keypoint: public CvPoint3D64f
{
public:
  Keypoint(double _x, double _y, double _z, double response, double scale,
      KeypointDescriptor* descriptor):
    r(response), s(scale), desc(descriptor){
    x = _x; y=_y; z=_z;
  }
  ~Keypoint() {
    delete desc;
  }
  /// the response of the keypoint
  double r;
  /// scale of the keypoint
  double s;
  /// the key point descriptor
  KeypointDescriptor* desc;
};


/// A struct to keep input images,
class StereoFrame {
public:
  StereoFrame(int frameIndex):mFrameIndex(frameIndex),
  mImage(NULL), mRightImage(NULL),mDispMap(NULL){}
  StereoFrame():mFrameIndex(-1),mImage(NULL),mRightImage(NULL), mDispMap(NULL){}
  /// index of this frame in the video sequence
  int mFrameIndex;
  /// left camera image
  WImageBuffer1_b* mImage;
  /// right camera image
  WImageBuffer1_b* mRightImage;
  /// disparity map
  WImageBuffer1_16s * mDispMap;
};

/// Information of pose estimation for one frame.
/// @todo consolidate PoseEstFrameEntry and FramePose by making FramePose
/// a member (or base class) of PoseEstFrameEntry
class PoseEstFrameEntry: public StereoFrame  {
public:
  /**
   * The object takes ownership of the images, keypoints and inliers
   */
  PoseEstFrameEntry(int frameIndex):
    StereoFrame(frameIndex), mKeypoints(NULL),
    mRot(cvMat(3, 3, CV_64FC1, _mRot)),
    mShift(cvMat(3, 1, CV_64FC1, _mShift)),
    transf_local_to_global_(cvMat(4, 4, CV_64FC1, transf_local_to_global_data_)),
    mNumTrackablePairs(0),
    mNumInliers(0),
    mImageC3a(NULL),
    mTrackableIndexPairs(NULL),
    mInlierIndices(NULL),
    mInliers0(NULL), mInliers1(NULL){}

  ~PoseEstFrameEntry();

  Keypoints* mKeypoints;
  /// Estimated rotation matrix from this frame to last key frame
  CvMat mRot;
  /// Estimated translation matrix from this frame to last key frame.
  CvMat mShift;
  /// Estimated global transformation matrix from this frame to the
  /// reference frame (this first frame), both in Cartesian space.
  CvMat transf_local_to_global_;
  /// Number of trackable pairs
  int mNumTrackablePairs;
  /// Number of inliers
  int mNumInliers;
  /// Index of last key frame, with which the transformation is estimated
  /// Use mostly for sanity checking
  int mLastKeyFrameIndex;
  WImageBuffer3_b *mImageC3a;
  /// index pairs of the trackable pairs w.r.t the keypoint list
  vector<pair<int, int> >* mTrackableIndexPairs;
  /// indices of the inliers in the trackable pairs
  int* mInlierIndices;
  /// inliers from keypoint0, in disparity space.
  CvMat *mInliers0;
  /// inliers from keypoint1, in disparity space.
  CvMat *mInliers1;
protected:

  void clear();
  // buffers
  double _mRot[9];
  double _mShift[3];
  double transf_local_to_global_data_[16];
};


/// transformation of each frame.
class FramePose   {
public:
  FramePose()
  :mIndex(-1),
  transf_local_to_global_(cvMat(4,4,CV_64FC1, transf_local_to_global_data_)),
  transf_global_to_disp_(NULL)
  {}

  FramePose(int i)
  :mIndex(i),
  transf_local_to_global_(cvMat(4,4,CV_64FC1, transf_local_to_global_data_)),
  transf_global_to_disp_(NULL)
  {}

  ~FramePose() {
    if (transf_global_to_disp_)
      cvReleaseMat(&transf_global_to_disp_);
  }

  /// Index of the frame
  int mIndex;
  /// Transformation matrix from local frame to global frame
  CvMat transf_local_to_global_;
  /// optional matrix to convert from global to disparity space
  CvMat* transf_global_to_disp_;
  double transf_local_to_global_data_[16];
};

void saveFramePoses(const string& dirname, const vector<FramePose*>& framePoses);

typedef enum {
  Pairwise,
  BundleAdjust
} CamTrackerType;

class FrameSeq;
class PoseEstimator;

typedef enum {
  /// This frame shall be skipped.
  KeyFrameSkip      = 0x0,
  /// This frame shall be kept. Maybe used as key frame later.
  KeyFrameKeep      = 0x1,
  /// Use this frame as key frame right now.
  KeyFrameUse       = 0x2,
  /// Backtrack to last kept frame and use it as key frame. This frame shall
  /// still be used moving forward.
  KeyFrameBackTrack = 0x3
} KeyFramingDecision;

/// A Camera tracker interface
class CamTracker {
public:
  virtual ~CamTracker(){};
  /// construct a camera tracker object
  static CamTracker* getCamTracker(
      /// type of the tracker
      const CamTrackerType type,
      /// size of the image the tracker expects to track on
      CvSize& imgSize,
      /// focal length in x
      double Fx,
      /// focal length in y
      double Fy,
      /// baseline
      double Tx,
      /// optical center of left cam, x coordinate
      double Clx,
      /// optical center of right cam, x coorindate
      double Crx,
      /// optical center, y cooridinate
      double Cy);
  /// Process all the stereo images in the queue.
  virtual bool track(
      /// queue of input stereo images
      queue<StereoFrame>& inputImages
  )=0;
  /// Use the tracker to track one single frame.
  /// process one frame, either from the queue, or backtracking to
  /// a previously kept one.
  /// @return true if this frame is used as a key frame.
  virtual bool trackOneFrame(
      /// input image queue
      queue<StereoFrame>& inputImageQueue,
      /// reference to a frame sequence management object
      FrameSeq& frameSeq)=0;
  /// Get a reference of  frame sequence management object
  virtual FrameSeq& getFrameSeq()=0;

  /// detects 3D keypoints from a stereo image, given as an image and its disparity mag.
  virtual bool goodFeaturesToTrack(
      /// input image
      const WImage1_b& img,
      /// disparity map of the input image
      const WImage1_16s* dispMap,
      /// (OUTPUT) key point detected
      Keypoints*& keypoints
  ) = 0;

  /// Find matching keypoints in between the current frame and the last key frame
  /// of the tracker.
  virtual void matchKeypoints(
      /// (Output) coordinates of trackable pairs
      vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
      /// (Output and Optional) indices of the trackable pairs into
      /// their corresponding key points.
      vector<pair<int, int> >*  trackableIndexPairs = NULL)=0;

  /// Evaluate the stat from the current frame and the tracker and decide
  /// appropriate action to the frame.
  /// @return recommended action for this frame.
  virtual KeyFramingDecision keyFrameEval(
      int frameIndex,
      /// number of key points detected in this frame.
      int numKeypoints,
      /// number of inliers detected in this frame with respect to last key frame.
      int numInliers,
      /// estimated rotation matrix.
      const CvMat & rot,
      /// estimated translation matrix.
      const CvMat & shift)=0;

  /// perform the recommended action on the current frame
  virtual bool keyFrameAction(
      /// recommended key frame action
      KeyFramingDecision kfd,
      /// the frame sequence management module
      FrameSeq& frameSeq) = 0;

  /// set the keyFrame as the last key frame
  virtual void setLastKeyFrame(
      /// the frame to be set as last key frame
      PoseEstFrameEntry* keyFrame) = 0;
  /// get the pointer to the last key frame.
  /// @return a pointer to the last key frame
  virtual PoseEstFrameEntry* getLastKeyFrame()=0;

  /// fetch the inliers of the current frame
  virtual bool fetchInliers(
      CvMat *& inliers0,
      /// inlier list 1 from current key frame
      CvMat *& inliers1)=0;
  /// fetch the list of index of inlier, indexed into the matchIndexPairs
  /// in cv::willow::poseEstimate
  /// @return the index of the inlier pairs
  virtual int *fetchInliers()=0;


  /// get a pointer to the pose estimator used by this tracker
  virtual PoseEstimator* getPoseEstimator()=0;

  virtual vector<FramePose*>* getFramePoses()=0;
  /// Reduce the key frame window size to winSize.
  virtual void reduceWindowSize(unsigned int winSize)=0;
  /// print statistics of the last tracking run to std out.
  virtual void printStat()=0;
protected:
  CamTracker(){}
};

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
    CvArr* mask = NULL,
    /// neighborhood size @see cvCornerHarris. @see CornerEigenValsAndVecs.
    /// The default value was 3 in cvGoodFeatuesToTrack()
    int blockSize = 5,
    /// the free parameter in Harris corner @see cvGoodFeaturesToTrack()
    /// @see cvCornerHarris(). The default value is .04 in cvGoodFeaturesToTrack()
    double k = .01
);

CvMat* dispMapToMask(const WImage1_16s& dispMap);

/**
 * matching method to find a best match of a key point in a neighborhood
 */
typedef enum {
  /// best location by cross correlation
  CrossCorrelation,
  /// best key point cross correlation
  KeyPointCrossCorrelation,
  /// best match in terms of sum of absolute differences
  KeyPointSumOfAbsDiff,
  /// best key point by Calonder descriptor
  CalonderDescriptor
} MatchMethod;

/**
  *  Match up two list of key points and output a list of matching pairs.
  *  @return true if the status of execution is normal.
  */
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
);

/// Forward declaration of a class for pose estimation.
class PoseEstimator {
public:
  virtual ~PoseEstimator(){};
  /// Construct a pose estimator.
  static PoseEstimator* getStereoPoseEstimator(
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
      double Cy);
  /// Method to estimate transformation based pairs of 3D points, in
  /// disparity coordinates
  /// @return number of inliers
  virtual int estimate(
      /// key point list 0
      const Keypoints& keypoints0,
      /// key point list 1
      const Keypoints& keypoints1,
      /// index pairs of matching key points
      const vector<pair<int, int> >& matchIndexPairs,
      /// (Output) rotation matrix
      CvMat& rot,
      /// (Output) translation vector
      CvMat& shift,
      /// If true, Levenberg-Marquardt is used at the end for smoothing
      bool smoothed)=0;

  virtual bool getDisparityMap(const WImage1_b& leftImage, const WImage1_b& rightImage,
      WImage1_16s& dispMap) {return false;}
protected:
  PoseEstimator(){}
};

/// estimate the transformation with Levenberg-Marquardt.
void estimateWithLevMarq(
    /// inlier list 0
    const CvMat& points0inlier,
    /// inlier list 1
    const CvMat& points1inlier,
    /// transformation matrix from Cartesian coordinates to disparity coordinates
    const CvMat& CartToDisp,
    /// transformation matrix from disparity coordinates to Cartesian coordinates.
    const CvMat& DispToCart,
    CvMat& rot, CvMat& trans);

/// A class for tracking the filenames
class FileSeq {
public:
  FileSeq():
    mNumFrames(-1),
    mStartFrameIndex(0),
    mEndFrameIndex(0),
    mFrameStep(1),
    mStop(false),
    mCurrentFrameIndex(0)
    {}

  int mNumFrames;
  /// index of the first frame
  int mStartFrameIndex;
  /// index of the last frame, inclusive.
  int mEndFrameIndex;
  int mFrameStep;
  bool mStop;
  int mCurrentFrameIndex;
  queue<StereoFrame> mInputImageQueue;

  /// input directory name
  string mDirname;
  /// Format of the filename of the left images, e.g. "left-%04d"
  string mLeftImageFilenameFmt;
  /// Format of the filename of the right images, e.g. "right-%04d"
  string mRightImageFilenameFmt;
  /// Format fo the filename of the disparity map, e.g. dispmap-%04d"
  string mDisparityMapFilenameFmt;

  void setInputVideoParams(
      /// The directory of the video files
      const string & dirname,
      /// Format of the filename of the left images, e.g. "left-%04d.ppm"
      const string & leftFileFmt,
      /// Format of the filename of the right images, e.g. "right-%04d.ppm"
      const string & rightFileFmt,
      /// Format of the filename of the disparity maps, e.g. dispmap-%04d.ppm"
      const string & dispFileFmt,
      /// Starting index of the image sequence
      int start,
      /// Ending index (exclusive) of the image sequence
      int end,
      /// increment to add from the index of one image pair to next one
      int step);
  bool getStartFrame();
  bool getNextFrame();
private:
  bool getCurrentFrame();
};

/// a data structure for tracking image sequence
class FrameSeq {
public:
  FrameSeq():
    mNumFrames(0),
    mStartFrameIndex(-1),
    mLastGoodFrame(NULL),
    mCurrentFrame(NULL),
    mNextFrame(NULL){}

  int mNumFrames;
  int mStartFrameIndex;
  /// Last good frame
  PoseEstFrameEntry *mLastGoodFrame;
  /// Current frame
  PoseEstFrameEntry *mCurrentFrame;
  /// Next frame. Used in back tracking to hold the current frame
  auto_ptr<PoseEstFrameEntry> mNextFrame;
  bool backTrack();
  void keepCurrentAsGoodFrame(){
    assert(mCurrentFrame != NULL);
    releasePoseEstFrameEntry( &mLastGoodFrame);
    mLastGoodFrame = mCurrentFrame;
    mCurrentFrame = NULL;
  }
  void reset() {
    releasePoseEstFrameEntry( &mCurrentFrame);
    releasePoseEstFrameEntry( &mLastGoodFrame);
  }
  void releasePoseEstFrameEntry( PoseEstFrameEntry** frameEntry );
  void print();
};

}
}

#endif /* VISODOM_H_ */
