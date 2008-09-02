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

#include "Cv3DPoseEstimateDisp.h"
#include <opencv/cvwimage.h>
using namespace cv;

// star detector
#include <star_detector/include/detector.h>


class Cv3DPoseEstimateStereo: public Cv3DPoseEstimateDisp {
public:
	typedef Cv3DPoseEstimateDisp Parent;
	static const int DefWidth        = 640;
	static const int DefHeight       = 480;
	// constants used in getDisparityMap
	static const int DefFTZero       = 31;		// max 31 cutoff for prefilter value
	static const int DefDLen         = 64;		// 64 disparities
	static const int DefCorr         = 15;		// correlation window size
	static const int DefTextThresh   = 10;		// texture threshold
	static const int DefUniqueThresh = 15;		// uniqueness threshold

	// constants used in goodFeaturesToTrack
	static const int DefNumScales    = 7;
	static const int DefThreshold    = 15;
//	static const int DefThreshold    = 5;
	static const int DefMaxNumKeyPoints = 0;

	// constants used in getting trackable pairs
	static const CvPoint DefNeighborhoodSize;
	static const CvPoint DefTemplateSize;

	Cv3DPoseEstimateStereo(int width=DefWidth, int height=DefHeight);
	virtual ~Cv3DPoseEstimateStereo();

	typedef enum  {
		Star,
		HarrisCorner
	} KeyPointDetector;
	void setKeyPointDector(KeyPointDetector detector) {	mKeyPointDetector = detector;}
	KeyPointDetector getKeyPointDetector() {return mKeyPointDetector;}

	CvSize& getSize() {	return mSize; }

	bool getDisparityMap(WImage1_b& leftImage, WImage1_b& rightImage, WImage1_16s& dispMap);
	vector<Keypoint> goodFeaturesToTrack(WImage1_b& img, WImage1_16s* mask);

	vector<pair<CvPoint3D64f, CvPoint3D64f> > getTrackablePairs(
			WImage1_b& img0, WImage1_b& img1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoint1
			);
	using Parent::estimate;
	int estimate(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs, CvMat& rot, CvMat& shift,
			bool reversed=false);

protected:
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
	int mMaxNumKeyPoints; // if greater than zero, get the top mMaxNumKeyPoints key points

	StarDetector mStarDetector;
};

#endif /* CV3DPOSEESTIMATESTEREO_H_ */
