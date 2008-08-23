/*
 * Cv3DPoseEstimateStereo.h
 *
 *  Created on: Aug 22, 2008
 *      Author: jdchen
 */

#ifndef CV3DPOSEESTIMATESTEREO_H_
#define CV3DPOSEESTIMATESTEREO_H_

#include "Cv3DPoseEstimateDisp.h"
#include <opencv/cvwimage.h>
using namespace cv;

class Cv3DPoseEstimateStereo: public Cv3DPoseEstimateDisp {
public:
	Cv3DPoseEstimateStereo(int width=640, int height=480);
	virtual ~Cv3DPoseEstimateStereo();

	void setSize(CvSize& size);
	CvSize& getSize() {	return mSize; }

	bool getDisparityMap(WImage1_b& leftImage, WImage1_b& rightImage, WImage1_16s& dispMap);

	static const int DefFTZero       = 31;		// max 31 cutoff for prefilter value
	static const int DefDLen         = 64;		// 64 disparities
	static const int DefCorr         = 15;		// correlation window size
	static const int DefTextThresh   = 10;		// texture threshold
	static const int DefUniqueThresh = 15;		// uniqueness threshold

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
};

#endif /* CV3DPOSEESTIMATESTEREO_H_ */
