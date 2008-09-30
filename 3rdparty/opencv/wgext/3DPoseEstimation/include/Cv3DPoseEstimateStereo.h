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

#include "PoseEstimateDisp.h"
#include <opencv/cvwimage.h>
using namespace cv;

// star detector
#include <star_detector/include/detector.h>

// Calonder descriptor
#include <calonder_descriptor/include/signature.h>
#include <calonder_descriptor/include/matcher.h>
#include <calonder_descriptor/include/rtree_classifier.h>
using namespace features;

namespace cv { namespace willow {
/**
 * Pose estimation based on two frames of stereo image pairs.
 */
class Cv3DPoseEstimateStereo: public PoseEstimateDisp {
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

	/**
	 * matching method to find a best match of a key point in a neighborhood
	 */
	typedef enum {
		CrossCorrelation,   // best location by cross correlation
		KeyPointCrossCorrelation, // best key point cross correlation
		CalonderDescriptor  // best key point by Calonder descriptor
	} MatchMethod;
	static const MatchMethod DefMatchMethod = CrossCorrelation;

	// misc constants
	static const double DefDisparityUnitInPixels = 16.;

	Cv3DPoseEstimateStereo(int width=DefWidth, int height=DefHeight);
	virtual ~Cv3DPoseEstimateStereo();

	typedef enum  {
		Star,			// use star detector
		HarrisCorner    // use Harris corner
	} KeyPointDetector;
	void setKeyPointDector(KeyPointDetector detector) {	mKeyPointDetector = detector;}
	KeyPointDetector getKeyPointDetector() {return mKeyPointDetector;}
	void setKeyPointMatcher(MatchMethod matcher) { mMatchMethod = matcher;}
	MatchMethod getKeyPointMatcher() { return mMatchMethod;}

	CvSize& getSize() {	return mSize; }

	bool getDisparityMap(WImage1_b& leftImage, WImage1_b& rightImage, WImage1_16s& dispMap);
	/**
	 * Compute a list of good feature point for tracking
	 * img   -- input image
	 * mask  -- mask of regions where feature points shall be found, positive value for yes
	 *          and negative value for no. Typically, mask is disMap, the output of
	 *          getDisparityMap()
	 * keypoints  -- Output. A vector of feature points that are believed to be
	 *               good for tracking.
	 */
	bool goodFeaturesToTrack(WImage1_b& img, WImage1_16s* mask, vector<Keypoint>& keypoints);

	/**
	 *  Data structure for the Calonder matcher.
	 */
	class CalonderMatcher {
	public:
		// load random forests classifier
		CalonderMatcher(string& modelfilename);
		~CalonderMatcher(){}
		// random forests classifier
		RTreeClassifier mClassifier;
	protected:
		// A threshold of 0 is safest (but slowest), as the signatures are
		// effectively dense vectors. Increasing the threshold makes the
		// signatures sparser, increasing the speed of matching at some cost
		// in the recognition rate. Reasonable thresholds are in [0, 0.01].
		static const float SIG_THRESHOLD = 0.0;
	};
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
			vector<Keypoint>& keyPoints0,
			/// Detected Key points in image 1
			vector<Keypoint>& keyPoints1,
			/// (Output) pairs of corresponding 3d locations for possibly the same
			/// 3d features. Used for pose estimation.
			/// Set it to NULL if not interested.
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			/// (Output) pairs of indices, to the input keypoints, of the corresponding
			/// 3d locations for possibly the same 3d features. Used for pose estimation.
			/// Set it to NULL if not interested.
			vector<pair<int, int> >* trackbleIndexPairs
			);

	bool makePatchRect(const CvPoint& rectSize, const CvPoint2D32f& featurePt, CvRect& rect);

	int mNumKeyPointsWithNoDisparity;  // a convenient counter for analysis

protected:
	/**
	 * convenient function to call cvMatchTemplate with normalized cross correlation
	 * for template matching over a neighborhood
	 */
	double matchTemplate(const CvMat& neighborhood, const CvMat& templ, CvMat& res, CvPoint& loc);
	bool getTrackablePairsByCalonder(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoint1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> > * trackablePairs,
			vector<pair<int, int> >* trackbleIndexPairs = NULL
	);
	bool getTrackablePairsByCrossCorr(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoint1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			vector<pair<int, int> >* trackbleIndexPairs
	);
	bool  getTrackablePairsByKeypointCrossCorr(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoint1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			vector<pair<int, int> >* trackbleIndexPairs
	);
	double 	getDisparity(WImage1_16s& dispMap, CvPoint& pt) {
		// the unit of disp is 1/16 of a pixel - mDisparityUnitInPixels
		return CV_IMAGE_ELEM(dispMap.Ipl(), int16_t, pt.y, pt.x)/mDisparityUnitInPixels;
	}

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

	StarDetector mStarDetector;
	/// buffer use by Harris Corner
  CvMat* mEigImage;
  CvMat* mTempImage;


	MatchMethod mMatchMethod;
	CalonderMatcher* mCalonderMatcher;

	double mTemplateMatchThreshold; // minimum threshold for template matching

	double mDisparityUnitInPixels;  // disparity unit in pixels
};
} // namespace willow
} // namespace cv
#endif /* CV3DPOSEESTIMATESTEREO_H_ */
