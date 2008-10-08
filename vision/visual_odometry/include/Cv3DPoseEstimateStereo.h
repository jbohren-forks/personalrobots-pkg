/*
 * Cv3DPoseEstimateStereo.h
 *
 *  Created on: Aug 22, 2008
 *      Author: jdchen
 */

#ifndef CV3DPOSEESTIMATESTEREO_H_
#define CV3DPOSEESTIMATESTEREO_H_

#include <vector>
using namespace std;

#include <opencv/cvwimage.h>
using namespace cv;
#include "PoseEstimateDisp.h"
#include "CvMatUtils.h"

#if STAR_DETECTOR
// star detector
#include <star_detector/include/detector.h>
#endif

#if CALONDER_DESCRIPTOR
// Calonder descriptor
#include <calonder_descriptor/include/signature.h>
#include <calonder_descriptor/include/matcher.h>
#include <calonder_descriptor/include/rtree_classifier.h>
using namespace features;
#endif

namespace cv { namespace willow {

/**
 * Pose estimation based on two frames of stereo image pairs.
 */
class PoseEstimateStereo: public PoseEstimateDisp {
public:
	typedef PoseEstimateDisp Parent;
	static const int DefWidth        = 640;
	static const int DefHeight       = 480;
	// constants used in getDisparityMap
	static const int DefFTZero       = 31;		//< max 31 cutoff for prefilter value
	static const int DefDLen         = 64;		//< 64 disparities
	static const int DefCorr         = 15;		//< correlation window size
	static const int DefTextThresh   = 10;		//< texture threshold
	static const int DefUniqueThresh = 15;		//< uniqueness threshold

	// constants used in harris corner detector
	static const double HarrisCornerQualityLevel = 0.01;  // what should I use? [0., 1.0]
	static const double HarrisCornerMinDistance  = 10.0;  // 10 pixels?
	static const int    HarrisCornerBlockSize    = 5;     // default is 3
	static const int    HarrisCornerDetectorFlag = 1;     // default is 0;
	static const double HarrisCornerFreeParam    = 0.01;  // default is 0.04;


	// constants used in goodFeaturesToTrack/star_detector
	static const int DefNumScales    = 7;
	static const int DefThreshold    = 15;
	static const int DefMaxNumKeyPoints = 500;

	// constants used in getting trackable pairs
	static const CvPoint DefNeighborhoodSize;
	static const CvPoint DefTemplateSize;
	static const double  DefTemplateMatchThreshold = 0;

	// constants used in RANSAC
	static const int DefNumRansacIter = 500; //< default num of iterations in RANSAC
	/// Default threshold for inliers checking in RANSAC
	static const double DefInlierThreshold = 1.5;

	static const MatchMethod DefMatchMethod = CrossCorrelation;

  static const size_t SSEAlignment = 16;

	PoseEstimateStereo(int width=DefWidth, int height=DefHeight);
	virtual ~PoseEstimateStereo();

	typedef enum  {
		Star,			// use star detector
		HarrisCorner    // use Harris corner
	} KeyPointDetector;
	void setKeyPointDector(KeyPointDetector detector) {	mKeyPointDetector = detector;}
	KeyPointDetector getKeyPointDetector() {return mKeyPointDetector;}
	void setKeyPointMatcher(MatchMethod matcher) { mMatchMethod = matcher;}
	MatchMethod getKeyPointMatcher() { return mMatchMethod;}

	CvSize& getSize() {	return mSize; }

  virtual bool getDisparityMap(const WImage1_b& leftImage,
      const WImage1_b& rightImage, WImage1_16s& dispMap);
	/**
	 * Compute a list of good feature point for tracking
	 * img   -- input image
	 * mask  -- mask of regions where feature points shall be found, positive value for yes
	 *          and negative value for no. Typically, mask is disMap, the output of
	 *          getDisparityMap()
	 * keypoints  -- Output. A vector of feature points that are believed to be
	 *               good for tracking.
	 */
//  bool goodFeaturesToTrack(WImage1_b& img, WImage1_16s* dispMap, vector<Keypoint>& keypoints);
  bool goodFeaturesToTrack(
      const WImage1_b& img,
      const WImage1_16s* dispMap,
      Keypoints& keypoints);

	static bool goodFeaturesToTrack(
	    const WImage1_b& img,
	    const WImage1_16s* dispMap,
	    double disparityUnitInPixels,
	    Keypoints& keypoints,
	    WImage1_f& eigImg,
	    WImage1_f& tempImg
	);

	/// Use Harris corner to extract keypoints.
	/// @return the number of feature points.
	static int goodFeaturesToTrackHarrisCorner(
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
	    double qualityThreshold = HarrisCornerQualityLevel,
	    /// minimum distance between the key points
	    double minDistance = HarrisCornerMinDistance,
	    /// Region of interest. The function selects points either in the specified
	    /// region or in the whole image if the mask is NULL.
	    CvArr* mask = NULL,
	    /// neighborhood size @see cvCornerHarris, @see CornerEigenValsAndVecs
	    int blockSize = HarrisCornerBlockSize,
	    /// the free parameter in Harris corner @see cvGoodFeaturesToTrack()
	    /// @see cvCornerHarris()
	    double k = HarrisCornerFreeParam
	);

	/**
	 *  Data structure for the Calonder matcher.
	 */
	class CalonderMatcher {
	public:
		// load random forests classifier
		CalonderMatcher(string& modelfilename);
		~CalonderMatcher(){}
		// random forests classifier
#if CALONDER_DESCRIPTOR
		RTreeClassifier mClassifier;
#endif
	protected:
		// A threshold of 0 is safest (but slowest), as the signatures are
		// effectively dense vectors. Increasing the threshold makes the
		// signatures sparser, increasing the speed of matching at some cost
		// in the recognition rate. Reasonable thresholds are in [0, 0.01].
		static const float SIG_THRESHOLD = 0.0;
	};
	/// Construct descriptors for each key points. The descriptors are filled
	/// into the keypoint objects.
	bool constructKeypointDescriptors(
	    /// left input image,
	    const WImage1_b& leftImage,
	    /// key point list
	    Keypoints& keypoints);
	/**
	 *  Match up two list of key points and output a list of trackable pairs.
	 *  @return true if the status of execution is normal.
	 */
	bool getTrackablePairs(
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
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			/// (Output) pairs of indices, to the input keypoints, of the corresponding
			/// 3d locations for possibly the same 3d features. Used for pose estimation.
			/// Set it to NULL if not interested.
			vector<pair<int, int> >* trackableIndexPairs
			);

	 /**
	   *  Match up two list of key points and output a list of trackable pairs.
	   *  @return true if the status of execution is normal.
	   */
	  static bool getTrackablePairs(
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
	      vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
	      /// (Output) pairs of indices, to the input keypoints, of the corresponding
	      /// 3d locations for possibly the same 3d features. Used for pose estimation.
	      /// Set it to NULL if not interested.
	      vector<pair<int, int> >* trackbleIndexPairs
	      );

	/// Make a CvRect of size rectSize around faturePt (at the center).
	/// Check if rect is within the bounds of the image. Cut it back if necessary.
	static bool makePatchRect(const CvPoint& rectSize, const CvPoint3D64f& featurePt,
	    const CvSize& imgSize, CvRect& rect);

	int mNumKeyPointsWithNoDisparity;  // a convenient counter for analysis

protected:
	/**
	 * convenient function to call cvMatchTemplate with normalized cross correlation
	 * for template matching over a neighborhood
	 */
	static double matchTemplate(const CvMat& neighborhood, const CvMat& templ,
	    CvMat& res, CvPoint& loc);
	bool getTrackablePairsByCalonder(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			Keypoints& keyPoints0, Keypoints& keyPoint1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> > * trackablePairs,
			vector<pair<int, int> >* trackbleIndexPairs = NULL
	);
	static bool getTrackablePairsByCrossCorr(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			Keypoints& keyPoints0, Keypoints& keyPoint1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			vector<pair<int, int> >* trackbleIndexPairs
	);
	static bool  getTrackablePairsByKeypointCrossCorr(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			Keypoints& keyPoints0, Keypoints& keyPoint1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			vector<pair<int, int> >* trackbleIndexPairs
	);

	CvSize mSize;

	// some parameters for Kurt's stereo pair code
	int mFTZero;		// max 31 cutoff for prefilter value
	int mDLen;			// 64 disparities
	int mCorr;			// correlation window size
	int mTextThresh;	// texture threshold
	int mUniqueThresh;	// uniqueness threshold

	uint8_t* mBufStereoPairs;			// local storage for the stereo pair algorithm
	uint8_t* mFeatureImgBufLeft;		// feature image buffer for left image
	uint8_t* mFeatureImgBufRight;		// feature image buffer for right image

	KeyPointDetector mKeyPointDetector;

	// parameters for the star detector
	int mNumScales;
	int mThreshold;
	unsigned int mMaxNumKeyPoints; // if greater than zero, get the top mMaxNumKeyPoints key points

#if STAR_DETECTOR
	StarDetector mStarDetector;
#endif
	/// buffer use by Harris Corner
	WImage1_f* mEigImage;
	WImage1_f* mTempImage;


	MatchMethod mMatchMethod;
	CalonderMatcher* mCalonderMatcher;

	double mDisparityUnitInPixels;  // disparity unit in pixels
};
} // namespace willow
} // namespace cv
#endif /* CV3DPOSEESTIMATESTEREO_H_ */
