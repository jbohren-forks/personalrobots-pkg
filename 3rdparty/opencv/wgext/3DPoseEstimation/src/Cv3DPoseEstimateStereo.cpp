/*
 * Cv3DPoseEstimateStereo.cpp
 *
 *  Created on: Aug 22, 2008
 *      Author: jdchen
 */

#include "Cv3DPoseEstimateStereo.h"
#include <stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily

#include <iostream>
using namespace std;

//opencv
#include <cv.h>
#include <cvwimage.h>

// boost
#include <boost/foreach.hpp>
#include <cassert>

// star detector
#include <star_detector/include/detector.h>

#undef DEBUG

// change from 61 to 101 to accommodate the sudden change around frame 0915 in
// the indoor sequence "indoor1"
const CvPoint Cv3DPoseEstimateStereo::DefNeighborhoodSize = cvPoint(128, 48);
// increasing the neighborhood size does not help very much. In fact, I have been
// degradation of performance.
//const CvPoint Cv3DPoseEstimateStereo::DefNeighborhoodSize = cvPoint(256, 48);
const CvPoint Cv3DPoseEstimateStereo::DefTemplateSize     = cvPoint(16, 16);

Cv3DPoseEstimateStereo::Cv3DPoseEstimateStereo(int width, int height):
	mNumKeyPointsWithNoDisparity(0),
	mSize(cvSize(width, height)),
	mFTZero(DefFTZero),
	mDLen(DefDLen),
	mCorr(DefCorr),
	mTextThresh(DefTextThresh),
	mUniqueThresh(DefUniqueThresh),
	mBufStereoPairs(NULL), mFeatureImgBufLeft(NULL), mFeatureImgBufRight(NULL),
	mKeyPointDetector(Star),
	mNumScales(DefNumScales),
	mThreshold(DefThreshold),
	mMaxNumKeyPoints(DefMaxNumKeyPoints),
	mStarDetector(mSize, mNumScales, mThreshold),
	mMatchMethod(CalonderDescriptor),
	mCalonderMatcher(NULL),
	mTemplateMatchThreshold(DefTemplateMatchThreshold),
	mDisparityUnitInPixels(DefDisparityUnitInPixels)
{
	mBufStereoPairs     = new uint8_t[mSize.height*mDLen*(mCorr+5)]; // local storage for the stereo pair algorithm
	mFeatureImgBufLeft  = new uint8_t[mSize.width*mSize.height];
	mFeatureImgBufRight = new uint8_t[mSize.width*mSize.height];

	setNumRansacIterations(DefNumRansacIter);
	setInlierErrorThreshold(DefInlierThreshold);


	switch(mMatchMethod) {
	case CalonderDescriptor:{
    string modelFile("land30.trees");
//    string modelFile("land50.trees");
		mCalonderMatcher = new CalonderMatcher(modelFile);
		cout << "loaded model file "<<modelFile.c_str()<<" for Calonder descriptor"<<endl;
		break;
	}
	case CrossCorrelation:
		break;
	case KeyPointCrossCorrelation:
		break;
	}
}

Cv3DPoseEstimateStereo::~Cv3DPoseEstimateStereo() {
	delete [] mBufStereoPairs;
	delete [] mFeatureImgBufLeft;
	delete mCalonderMatcher;
}

bool Cv3DPoseEstimateStereo::getDisparityMap(
		WImage1_b& leftImage, WImage1_b& rightImage, WImage1_16s& dispMap) {
	bool status = true;

	if (leftImage.Width()  != mSize.width || leftImage.Height()  != mSize.height ||
			rightImage.Width() != mSize.width || rightImage.Height() != mSize.height ) {
		cerr << __PRETTY_FUNCTION__ <<"(): size of images incompatible. "<< endl;
		return false;
	}

	//
	// Try Kurt's dense stereo pair
	//
	uint8_t *lim = leftImage.ImageData();
	uint8_t *rim = rightImage.ImageData();

	int16_t* disp    = dispMap.ImageData();
	int16_t* textImg = NULL;

	// prefilter
	do_prefilter(lim, mFeatureImgBufLeft, mSize.width, mSize.height, mFTZero, mBufStereoPairs);
	do_prefilter(rim, mFeatureImgBufRight, mSize.width, mSize.height, mFTZero, mBufStereoPairs);

	// stereo
	do_stereo(mFeatureImgBufLeft, mFeatureImgBufRight, disp, textImg, mSize.width, mSize.height,
			mFTZero, mCorr, mCorr, mDLen, mTextThresh, mUniqueThresh, mBufStereoPairs);

	return status;
}

bool Cv3DPoseEstimateStereo::goodFeaturesToTrack(WImage1_b& img, WImage1_16s* mask, vector<Keypoint>& keypoints){
  bool status = true;
	KeyPointDetector keyPointDetector = getKeyPointDetector();
	switch(keyPointDetector) {
	case HarrisCorner: {
		//
		//  Try cvGoodFeaturesToTrack
		//
		int maxKeypoints = 500;
		CvPoint2D32f kps[maxKeypoints];
		CvMat* eig_image  = cvCreateMat(mSize.height, mSize.width, CV_32FC1);
		CvMat* temp_image = cvCreateMat(mSize.height, mSize.width, CV_32FC1);
		//cvConvert(rightimg, tmp);
//		cvCornerHarris(tmp, harrisCorners, 21);
		double quality_level =  0.01; // what should I use? [0., 1.0]
		double min_distance  = 10.0; // 10 pixels?

		// misc params to cvGoodFeaturesToTrack
		CvMat *mask1 = NULL;
		CvMat mask0;
		if (mask) {
			int16_t *disp = mask->ImageData();
			int8_t _mask[mSize.width*mSize.height];
			for (int v=0; v<mSize.height; v++) {
				for (int u=0; u<mSize.width; u++) {
					int16_t d = disp[v*mSize.width+u];
					if (d>0) {
						_mask[v*mSize.width+u] = 1;
					} else {
						_mask[v*mSize.width+u] = 0;
					}
				}
			}
			mask0= cvMat(mSize.height, mSize.width, CV_8SC1, _mask);
			mask1 = &mask0;
		}

		const int block_size=5;  // default is 3
		const int use_harris=1;   // default is 0;
		const double k=0.01; // default is 0.04;
		int numkeypoints = maxKeypoints;
		cvGoodFeaturesToTrack(img.Ipl(), eig_image, temp_image,
				kps, &numkeypoints,
				quality_level, min_distance, mask1, block_size, use_harris, k);
		cvReleaseMat(&eig_image);
		cvReleaseMat(&temp_image);
		for (int i=0; i<numkeypoints; i++) {
			keypoints.push_back(Keypoint(kps[i].x, kps[i].y, 0., 0., 0));
		}
		return status;
		break;
	}
	case Star: {
		//
		// Try Star Detector
		//
		std::vector<Keypoint> kps = mStarDetector.DetectPoints(img.Ipl());
#ifdef DEBUG
    cout << "Found "<< kps.size() << " good keypoints by Star Detector"<<endl;
#endif
		if (kps.size() > mMaxNumKeyPoints) {
	    std::nth_element(kps.begin(), kps.begin()+mMaxNumKeyPoints, kps.end());
	    kps.erase(kps.begin()+mMaxNumKeyPoints, kps.end());
		}

		// filter out keypoints that do not have disparity value
		if (mask) {
			int16_t* _mask = mask->ImageData();
			int numKeyPointsHasNoDisp=0;
			BOOST_FOREACH( Keypoint &pt, kps ) {
				int16_t d = _mask[pt.y*mSize.width+pt.x];
				if (d>0) {
					keypoints.push_back(pt);
				} else {
					numKeyPointsHasNoDisp++;
				}
			}
#ifdef DEBUG
			cout << "Num of keypoints have no disparity: "<< numKeyPointsHasNoDisp <<endl;
#endif
			mNumKeyPointsWithNoDisparity = numKeyPointsHasNoDisp;
			return status;
		} else {
		  // make a copy, will compiler be smart enough to save real copying
		  // since kps is going out of scope anyway.
		  keypoints = kps;
			return status;
		}

	}
	default:
		cerr << __PRETTY_FUNCTION__ <<"() Not implemented yet";
		return false;
	}
	return status;
}

bool Cv3DPoseEstimateStereo::makePatchRect(const CvPoint& rectSize, const CvPoint2D32f& featurePt, CvRect& rect) {
	rect = cvRect(
			(int)(.5 + featurePt.x - rectSize.x/2),
			(int)(.5 + featurePt.y - rectSize.y/2),
			rectSize.x, rectSize.y
	);
	// check if rectTempl is all within bound
	// cut it back if yes

	bool fOutOfBound = false;
	if (rect.x < 0 ) {
		rect.x = 0;
		fOutOfBound = true;
	} else if ( rect.x + rect.width  > mSize.width ) {
		rect.width = mSize.width - rect.x;
		fOutOfBound = true;
	}
	if (rect.y < 0 ) {
		rect.y = 0;
		fOutOfBound = true;
	} else if ( rect.y + rect.height > mSize.height ) {
		rect.height = mSize.height - rect.y;
		fOutOfBound = true;
	}
	return fOutOfBound;
}

bool
Cv3DPoseEstimateStereo::getTrackablePairsByCalonder(
		WImage1_b& image0, WImage1_b& image1,
		WImage1_16s& dispMap0, WImage1_16s& dispMap1,
		vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoints1,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs
		) {
	assert(this->mCalonderMatcher!=NULL);
	bool status = true;
	CvPoint neighborhoodSize = DefNeighborhoodSize;
	CvPoint templSize        = DefTemplateSize;
	// a buffer for neighborhood template matching
	float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
	CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
	int numTrackablePairs=0;

	// for image1Brute
	// Extract patches and add their signatures to matcher database
	BruteForceMatcher<DenseSignature, CvPoint> matcher;

	BOOST_FOREACH( Keypoint &pt, keyPoints1 ) {
		cv::WImageView1_b view = extractPatch(image1.Ipl(), pt);
		DenseSignature sig = mCalonderMatcher->mClassifier.getDenseSignature(view.Ipl());
		matcher.addSignature(sig, cvPoint(pt.x, pt.y));
	}

	// For each key point in image0, find its best match in matcher
	float distance;
	int index = 0;
	CvPoint bestloc;
	BOOST_FOREACH( Keypoint &pt, keyPoints0 ) {
	  CvRect rectNeighborhood;
	  CvPoint2D32f pt2D32f = cvPoint2D32f(pt.x, pt.y);
	  makePatchRect(neighborhoodSize, pt2D32f, rectNeighborhood);

		cv::WImageView1_b view = extractPatch(image0.Ipl(), pt);
		DenseSignature sig = mCalonderMatcher->mClassifier.getDenseSignature(view.Ipl());
//		int match = matcher.findMatch(sig, &distance);
		int match = matcher.findMatchInWindow(sig, rectNeighborhood, &distance);
		if (match<0) {		  // no match
		  continue;
		}

		Keypoint& kp = keyPoints1.at(match);
		bestloc.x = kp.x;
		bestloc.y = kp.y;

		double disp = getDisparity(dispMap1, bestloc);
		if (disp<0) {
			mNumKeyPointsWithNoDisparity++;
			continue;
		}
		CvPoint pt0Int = cvPoint(pt.x, pt.y);
		double disp0 = getDisparity(dispMap0, pt0Int);
		CvPoint3D64f pt0 = cvPoint3D64f(pt0Int.x, pt0Int.y, disp0);
		CvPoint3D64f pt1 = cvPoint3D64f(bestloc.x, bestloc.y, disp);
#ifdef DEBUG
		cout << "best match: "<<pt1.x<<","<<pt1.y<<","<<pt1.z<<endl;
#endif
		pair<CvPoint3D64f, CvPoint3D64f> p(pt0, pt1);
		trackablePairs.push_back(p);
		numTrackablePairs++;

		++index;
	}
	assert(numTrackablePairs == (int)trackablePairs.size());
	return status;
}


bool
Cv3DPoseEstimateStereo::getTrackablePairsByCrossCorr(
		WImage1_b& image0, WImage1_b& image1,
		WImage1_16s& dispMap0, WImage1_16s& dispMap1,
		vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoints1,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs
		) {
	bool status = true;
	CvPoint neighborhoodSize = DefNeighborhoodSize;
	CvPoint templSize        = DefTemplateSize;
	// a buffer for neighborhood template matching
	float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
	CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
	int numTrackablePairs=0;

	// loop thru the keypoints of image0 and look for best match from image1
	for (vector<Keypoint>::const_iterator ikp = keyPoints0.begin(); ikp!=keyPoints0.end(); ikp++) {
		CvPoint2D32f featurePtLastLeft = cvPoint2D32f(ikp->x, ikp->y);
		CvPoint fPtLastLeft = cvPoint(ikp->x, ikp->y);

		double disp = getDisparity(dispMap0, fPtLastLeft);
		// we should have check if disparity is available already for all key points
		assert( disp>=0);
		CvPoint3D64f ptLast = cvPoint3D64f(featurePtLastLeft.x, featurePtLastLeft.y, disp);

#ifdef DEBUG
		cout << "Feature at "<< featurePtLastLeft.x<<","<<featurePtLastLeft.y<<","<<disp<<endl;
#endif
		// find the closest (in distance and appearance) feature
		// to it in current feature list
		// In a given neighborhood, if there is at least a good feature
		// in the left cam image,
		// we search for a location in current
		// left cam image is that is closest to this one in appearance
		vector<CvPoint2D32f> featurePtsInNeighborhood;
		assert(featurePtsInNeighborhood.size()==0);
#ifdef DEBUG
		bool goodFeaturePtInCurrentLeftImg = false;
		for (vector<Keypoint>::const_iterator jkp = keyPoints1.begin();
		jkp!=keyPoints1.end(); jkp++) {
			CvPoint2D32f featurePtLeft = cvPoint2D32f(jkp->x, jkp->y);
			float dx = featurePtLeft.x - featurePtLastLeft.x;
			float dy = featurePtLeft.y - featurePtLastLeft.y;
			if (fabs(dx)<neighborhoodSize.x && fabs(dy)<neighborhoodSize.y) {
				goodFeaturePtInCurrentLeftImg = true;
				featurePtsInNeighborhood.push_back(featurePtLeft);
				cout << "Good candidate at "<< featurePtLeft.x <<","<< featurePtLeft.y<<endl;
			}
		}
		if (goodFeaturePtInCurrentLeftImg == false)
			continue;
		cout <<"Found "<< featurePtsInNeighborhood.size() << " candidates in the neighborhood"<<endl;
#endif
		/* find the best correlation in the neighborhood */
		/** make a template center around featurePtLastLeft; */
		CvRect rectTempl;

		bool fOutOfBound = makePatchRect(templSize, featurePtLastLeft, rectTempl);
		if (fOutOfBound == true) {
			continue;
		}
		CvRect rectNeighborhood;

		fOutOfBound = makePatchRect(neighborhoodSize, featurePtLastLeft, rectNeighborhood);
		if (fOutOfBound == true) {
			// (partially) out of bound.
			if (rectNeighborhood.width < rectTempl.width || rectNeighborhood.height < rectTempl.height) {
				// skip this
				continue;
			}
		}
		CvMat templ, neighborhood;
		cvGetSubRect(image0.Ipl(), &templ, rectTempl);
		cvGetSubRect(image1.Ipl(), &neighborhood, rectNeighborhood);

		CvPoint bestloc;
		CvMat res0;
		CvRect rectRes = cvRect(0, 0,
				rectNeighborhood.width - rectTempl.width + 1, rectNeighborhood.height - rectTempl.height + 1);
		cvGetSubRect(&res, &res0, rectRes);
		double  matchingScore = matchTemplate(neighborhood, templ, res0, bestloc);
		if (matchingScore < mTemplateMatchThreshold) {
			// best matching is not good enough, skip this key point
			continue;
		}

		bestloc.x += rectNeighborhood.x + rectTempl.width/2;
		bestloc.y += rectNeighborhood.y + rectTempl.height/2;

#if 0
		// shift bestloc to  coordinates w.r.t the neighborhood
		bestloc.x += rectTempl.width/2;  // please note that they are integer
		bestloc.y += rectTempl.height/2;
		bestloc.x -= rectNeighborhood.width/2;
		bestloc.y -= rectNeighborhood.height/2;

#ifdef DEBUG
		cout << "best match offset: "<< bestloc.x <<","<<bestloc.y<<endl;
#endif
		// further shift to the coordinates of the left image
		bestloc.x += featurePtLastLeft.x;
		bestloc.y += featurePtLastLeft.y;
#endif

		disp = getDisparity(dispMap1, bestloc);

		if (disp<0) {
			this->mNumKeyPointsWithNoDisparity++;
			continue;
		}
		CvPoint3D64f pt = cvPoint3D64f(bestloc.x, bestloc.y, disp);
#ifdef DEBUG
		cout << "best match: "<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
#endif
		pair<CvPoint3D64f, CvPoint3D64f> p(ptLast, pt);
		trackablePairs.push_back(p);
		numTrackablePairs++;
	}
	assert(numTrackablePairs == (int)trackablePairs.size());
	return status;
}

bool
Cv3DPoseEstimateStereo::getTrackablePairsByKeypointCrossCorr(
		WImage1_b& image0, WImage1_b& image1,
		WImage1_16s& dispMap0, WImage1_16s& dispMap1,
		vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoints1,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs
		) {
	bool status = true;
	CvPoint neighborhoodSize = DefNeighborhoodSize;
	CvPoint templSize        = DefTemplateSize;
	// a buffer for neighborhood template matching
	float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
	CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
	int numTrackablePairs=0;

	// loop thru the keypoints of image0 and look for best match from image1
	for (vector<Keypoint>::const_iterator ikp = keyPoints0.begin(); ikp!=keyPoints0.end(); ikp++) {
		CvPoint2D32f featurePtLastLeft = cvPoint2D32f(ikp->x, ikp->y);
		CvPoint fPtLastLeft = cvPoint(ikp->x, ikp->y);

		double disp = getDisparity(dispMap0, fPtLastLeft);
		assert( disp>=0);
		CvPoint3D64f ptLast = cvPoint3D64f(featurePtLastLeft.x, featurePtLastLeft.y, disp);

#ifdef DEBUG
		cout << "Feature at "<< featurePtLastLeft.x<<","<<featurePtLastLeft.y<<","<<disp<<endl;
#endif
		// find the closest (in distance and appearance) feature
		// to it in current feature list
		// In a given neighborhood, if there is at least a good feature
		// in the left cam image,
		// we search for a location in current
		// left cam image is that is closest to this one in appearance
		bool goodFeaturePtInCurrentLeftImg = false;
		vector<CvPoint2D32f> featurePtsInNeighborhood;
		assert(featurePtsInNeighborhood.size()==0);

		// make a template center around featurePtLastLeft;
		CvRect rectTempl = cvRect(
				(int)(.5 + featurePtLastLeft.x - templSize.x/2),
				(int)(.5 + featurePtLastLeft.y - templSize.y/2),
				templSize.x, templSize.y
		);
		// check if rectTempl is all within bound
		if (rectTempl.x < 0 || rectTempl.y < 0 ||
				rectTempl.x + rectTempl.width  > mSize.width ||
				rectTempl.y + rectTempl.height > mSize.height ) {
			// (partially) out of bound.
			// skip this
			continue;
		}
		CvMat templ;
		cvGetSubRect(image0.Ipl(), &templ, rectTempl);
		CvMat templ2;
		float _res[0];
		CvMat res = cvMat(1, 1, CV_32FC1, _res);
		cvMatchTemplate(&templ, &templ, &res, CV_TM_CCORR_NORMED );
		double threshold = _res[0]*.75;
		CvPoint bestloc = cvPoint(-1,-1);
		double maxScore = threshold;
		for (vector<Keypoint>::const_iterator jkp = keyPoints1.begin();
		jkp!=keyPoints1.end(); jkp++) {
			CvPoint2D32f featurePtLeft = cvPoint2D32f(jkp->x, jkp->y);
			float dx = featurePtLeft.x - featurePtLastLeft.x;
			float dy = featurePtLeft.y - featurePtLastLeft.y;
			if (fabs(dx)<neighborhoodSize.x && fabs(dy)<neighborhoodSize.y) {
				goodFeaturePtInCurrentLeftImg = true;
				featurePtsInNeighborhood.push_back(featurePtLeft);
#ifdef DEBUG
				cout << "Good candidate at "<< featurePtLeft.x <<","<< featurePtLeft.y<<endl;
#endif
				CvRect rectTempl2 = cvRect(
						(int)(.5 + featurePtLeft.x - templSize.x/2),
						(int)(.5 + featurePtLeft.y - templSize.y/2),
						templSize.x, templSize.y
				);
				// check if rectTempl is all within bound
				if (rectTempl2.x < 0 || rectTempl2.y < 0 ||
						rectTempl2.x + rectTempl2.width  > mSize.width ||
						rectTempl2.y + rectTempl2.height > mSize.height ) {
					// (partially) out of bound.
					// skip this
					continue;
				}
				cvGetSubRect(image1.Ipl(), &templ2, rectTempl2);
				cvMatchTemplate(&templ2, &templ, &res, CV_TM_CCORR_NORMED );
				double score = _res[0];
				if (score > maxScore) {
					bestloc.x = featurePtLeft.x;
					bestloc.y = featurePtLeft.y;
					maxScore = score;
				}
			}
		}
		if (maxScore <= threshold ) {
			continue;
		}

		disp = getDisparity(dispMap1, bestloc);
		if (disp<0) {
			mNumKeyPointsWithNoDisparity++;
			continue;
		}
		CvPoint3D64f pt = cvPoint3D64f(bestloc.x, bestloc.y, disp);
#ifdef DEBUG
		cout << "best match: "<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
#endif
		pair<CvPoint3D64f, CvPoint3D64f> p(ptLast, pt);
		trackablePairs.push_back(p);
		numTrackablePairs++;
	}
	assert(numTrackablePairs == (int)trackablePairs.size());
	return status;
}

bool Cv3DPoseEstimateStereo::getTrackablePairs(
			WImage1_b& image0, WImage1_b& image1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoints1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs
			) {
	bool status = true;
	switch(mMatchMethod){
	case CrossCorrelation:
		status = getTrackablePairsByCrossCorr(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
				trackablePairs);
		break;
	case KeyPointCrossCorrelation:
		status = getTrackablePairsByKeypointCrossCorr(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
				trackablePairs);
		break;
	case CalonderDescriptor:
		status = getTrackablePairsByCalonder(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
						trackablePairs);
		break;
	default:
		status = false;
		break;
	}
	return status;
}

double Cv3DPoseEstimateStereo::matchTemplate(const CvMat& neighborhood, const CvMat& templ, CvMat& res,
		CvPoint& loc){
	cvMatchTemplate(&neighborhood, &templ, &res, CV_TM_CCORR_NORMED );
	double		minval, maxval;
	cvMinMaxLoc( &res, &minval, &maxval, NULL, &loc, NULL);
	return maxval;
}

Cv3DPoseEstimateStereo::CalonderMatcher::CalonderMatcher(string& modelfilename){
	  mClassifier.read(modelfilename.c_str());
	  mClassifier.setThreshold(SIG_THRESHOLD);
}

