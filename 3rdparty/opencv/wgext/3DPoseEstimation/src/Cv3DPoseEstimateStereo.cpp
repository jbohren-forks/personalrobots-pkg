/*
 * Cv3DPoseEstimateStereo.cpp
 *
 *  Created on: Aug 22, 2008
 *      Author: jdchen
 */

#include "Cv3DPoseEstimateStereo.h"
#include <stereolib.h> // from 3DPoseEstimation/include. The header file is there temporarily

#include <iostream>
#include <star_detector/include/detector.h>
using namespace std;

#define DEBUG

const CvPoint Cv3DPoseEstimateStereo::DefNeighborhoodSize = cvPoint(128, 48);
// increasing the neighborhood size does not help very much. In fact, I have been
// degradation of performance.
//const CvPoint Cv3DPoseEstimateStereo::DefNeighborhoodSize = cvPoint(256, 48);
const CvPoint Cv3DPoseEstimateStereo::DefTemplateSize     = cvPoint(16, 16);

Cv3DPoseEstimateStereo::Cv3DPoseEstimateStereo(int width, int height):
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
	mTemplateMatchThreshold(DefTemplateMatchThreshold)
{
	mBufStereoPairs     = new uint8_t[mSize.height*mDLen*(mCorr+5)]; // local storage for the stereo pair algorithm
	mFeatureImgBufLeft  = new uint8_t[mSize.width*mSize.height];
	mFeatureImgBufRight = new uint8_t[mSize.width*mSize.height];
}

Cv3DPoseEstimateStereo::~Cv3DPoseEstimateStereo() {
	delete [] mBufStereoPairs;
	delete [] mFeatureImgBufLeft;
}

bool Cv3DPoseEstimateStereo::getDisparityMap(
		WImage1_b& leftImage, WImage1_b& rightImage, WImage1_16s& dispMap) {
	bool status = true;

	if (leftImage.Width() != mSize.width || leftImage.Height() != mSize.height ||
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

vector<Keypoint> Cv3DPoseEstimateStereo::goodFeaturesToTrack(WImage1_b& img, WImage1_16s* mask){
	KeyPointDetector keyPointDetector = getKeyPointDetector();
	switch(keyPointDetector) {
	case HarrisCorner: {
		//
		//  Try cvGoodFeaturesToTrack
		//
		vector<Keypoint> keypoints;
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
		return keypoints;
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
		if (mask) {
			vector<Keypoint> keypoints;
			int16_t* _mask = mask->ImageData();
			int numKeyPointsHasNoDisp=0;
			for (vector<Keypoint>::const_iterator ikp = kps.begin(); ikp != kps.end(); ikp++) {
				int16_t d = _mask[ikp->y*mSize.width+ikp->x];
				if (d>0) {
					keypoints.push_back(*ikp);
				} else {
					numKeyPointsHasNoDisp++;
				}
			}
#ifdef DEBUG
			cout << "Num of keypoints have no disparity: "<< numKeyPointsHasNoDisp <<endl;
#endif
			return keypoints;
		} else {
			return kps;
		}

	}
	default:
		cerr << __PRETTY_FUNCTION__ <<"() Not implemented yet";
		return vector<Keypoint>();
	}
	return vector<Keypoint>();
}

vector<pair<CvPoint3D64f, CvPoint3D64f> > Cv3DPoseEstimateStereo::getTrackablePairs(
			WImage1_b& image0, WImage1_b& image1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			vector<Keypoint>& keyPoints0, vector<Keypoint>& keyPoints1
			) {
	// change from 61 to 101 to accommodate the sudden change around 0915 in
	// the indoor sequence
	CvPoint neighborhoodSize = DefNeighborhoodSize;
	CvPoint templSize        = DefTemplateSize;
	int numTrackablePairs=0;
	vector<pair<CvPoint3D64f, CvPoint3D64f> > trackablePairs;

	for (vector<Keypoint>::const_iterator ikp = keyPoints0.begin(); ikp!=keyPoints0.end(); ikp++) {
		CvPoint2D32f featurePtLastLeft = cvPoint2D32f(ikp->x, ikp->y);
		CvPoint fPtLastLeft = cvPoint(ikp->x, ikp->y);

		CvScalar s = cvGet2D(dispMap0.Ipl(), fPtLastLeft.y, fPtLastLeft.x);
		double d = s.val[0];
		double d1 = CV_IMAGE_ELEM(dispMap0.Ipl(), int16_t, fPtLastLeft.y, fPtLastLeft.x);
		assert(d == d1);
		if (d<0) {
			printf("%d, %d, %d\n", fPtLastLeft.y, fPtLastLeft.x, (int)d);
		}
		assert( d>=0);
		d /= 16.;
		CvPoint3D64f ptLast = cvPoint3D64f(featurePtLastLeft.x, featurePtLastLeft.y, d);

#ifdef DEBUG
		cout << "Feature at "<< featurePtLastLeft.x<<","<<featurePtLastLeft.y<<","<<d<<endl;
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
		if (mKeypointVsKeyPoint == true) {
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

			CvScalar s = cvGet2D(dispMap1.Ipl(), bestloc.y, bestloc.x);
			double d = s.val[0];

			if (d<0) {
#ifdef DEBUG
				cout << "disparity missing: "<<d<<endl;
#endif
				continue;
			}
			d /= 16.;
			CvPoint3D64f pt = cvPoint3D64f(bestloc.x, bestloc.y, d);
#ifdef DEBUG
			cout << "best match: "<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
#endif
			pair<CvPoint3D64f, CvPoint3D64f> p(ptLast, pt);
			trackablePairs.push_back(p);
			numTrackablePairs++;
		} else {
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

				}
			}
			if (goodFeaturePtInCurrentLeftImg == false)
				continue;
#ifdef DEBUG
			cout <<"Found "<< featurePtsInNeighborhood.size() << " candidates in the neighborhood"<<endl;
#endif
			/* find the best correlation in the neighborhood */
			/** make a template center around featurePtLastLeft; */
			CvRect rectTempl;

			bool fOutOfBound = false;
			rectTempl = cvRect(
					(int)(.5 + featurePtLastLeft.x - templSize.x/2),
					(int)(.5 + featurePtLastLeft.y - templSize.y/2),
					templSize.x, templSize.y
			);
			// check if rectTempl is all within bound
			if (rectTempl.x < 0 || rectTempl.y < 0 ||
					rectTempl.x + rectTempl.width  > mSize.width ||
					rectTempl.y + rectTempl.height > mSize.height ) {
				/* (partially) out of bound. */
				/* skip this */
				fOutOfBound = true;
			}
			if (fOutOfBound == true) {
				continue;
			}
			CvRect rectNeighborhood = cvRect(
					(int)(.5 + featurePtLastLeft.x - neighborhoodSize.x/2),
					(int)(.5 + featurePtLastLeft.y - neighborhoodSize.y/2),
					neighborhoodSize.x, neighborhoodSize.y
			);
			if (rectNeighborhood.x < 0 || rectNeighborhood.y < 0 ||
					rectNeighborhood.x + rectNeighborhood.width  > mSize.width ||
					rectNeighborhood.y + rectNeighborhood.height > mSize.height ) {
				// (partially) out of bound.
				// skip this
				continue;
			}
			CvMat templ, neighborhood;
			float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
			CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
			cvGetSubRect(image0.Ipl(), &templ, rectTempl);
			cvGetSubRect(image1.Ipl(), &neighborhood, rectNeighborhood);

#if 0
			cvMatchTemplate(&neighborhood, &templ, &res, CV_TM_SQDIFF );
			CvPoint		minloc, maxloc;
			double		minval, maxval;
			cvMinMaxLoc( &res, &minval, &maxval, &minloc, &maxloc, 0 );
			// minloc goes with CV_TM_SQDIFF
			// maxloc goes with CV_TM_CCORR and CV_TM_CCOEFF
			// and I guess CV_TM_CCORR_NORMED and CV_TM_CCOEFF_NORMED too
			CvPoint     bestloc = minloc;
#else
			cvMatchTemplate(&neighborhood, &templ, &res, CV_TM_CCORR_NORMED );
			CvPoint		minloc, maxloc;
			double		minval, maxval;
			cvMinMaxLoc( &res, &minval, &maxval, &minloc, &maxloc, 0 );
			CvPoint     bestloc = maxloc;
			if (maxval < mTemplateMatchThreshold) {
				// best matching is not good enough, skip this key point
				continue;
			}
#endif

			// shift bestloc to image coordinates
			bestloc.x += rectTempl.width/2;  // please note that they are integer
			bestloc.y += rectTempl.height/2;
			bestloc.x -= rectNeighborhood.width/2;
			bestloc.y -= rectNeighborhood.height/2;

#ifdef DEBUG
			cout << "best match offset: "<< bestloc.x <<","<<bestloc.y<<endl;
#endif
			bestloc.x += featurePtLastLeft.x;
			bestloc.y += featurePtLastLeft.y;

			CvScalar s = cvGet2D(dispMap1.Ipl(), bestloc.y, bestloc.x);
			double d = s.val[0];

			if (d<0) {
#ifdef DEBUG
				cout << "disparity missing: "<<d<<endl;
#endif
				continue;
			}
			d /= 16.;
			CvPoint3D64f pt = cvPoint3D64f(bestloc.x, bestloc.y, d);
#ifdef DEBUG
			cout << "best match: "<<pt.x<<","<<pt.y<<","<<pt.z<<endl;
#endif
			pair<CvPoint3D64f, CvPoint3D64f> p(ptLast, pt);
			trackablePairs.push_back(p);
			numTrackablePairs++;
		}
	}
	assert(numTrackablePairs == (int)trackablePairs.size());
	return trackablePairs;
}

int Cv3DPoseEstimateStereo::estimate(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
		CvMat& rot, CvMat& shift, bool reversed) {
	int numTrackablePairs = trackablePairs.size();
	double _uvds0[3*numTrackablePairs];
	double _uvds1[3*numTrackablePairs];
	CvMat uvds0 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds0);
	CvMat uvds1 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds1);

	int iPt=0;
	for (vector<pair<CvPoint3D64f, CvPoint3D64f> >::const_iterator iter = trackablePairs.begin();
	iter != trackablePairs.end(); iter++) {
		const pair<CvPoint3D64f, CvPoint3D64f>& p = *iter;
		_uvds0[iPt*3 + 0] = p.first.x;
		_uvds0[iPt*3 + 1] = p.first.y;
		_uvds0[iPt*3 + 2] = p.first.z;

		_uvds1[iPt*3 + 0] = p.second.x;
		_uvds1[iPt*3 + 1] = p.second.y;
		_uvds1[iPt*3 + 2] = p.second.z;
		iPt++;
	}

	// estimate the transform the observed points from current back to last position
	// it should be equivalent to the transform of the camera frame from
	// last position to current position
	int numInliers;
	if (reversed == true) {
		numInliers = this->estimate(&uvds1, &uvds0, &rot, &shift);
	} else {
		numInliers = this->estimate(&uvds0, &uvds1, &rot, &shift);
	}
	return numInliers;
}
