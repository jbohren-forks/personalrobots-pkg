/*
 * Cv3DPoseEstimateStereo.cpp
 *
 *  Created on: Aug 22, 2008
 *      Author: jdchen
 */

#include "Cv3DPoseEstimateStereo.h"
#include "CvMatUtils.h"
using namespace cv::willow;
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
const CvPoint PoseEstimateStereo::DefNeighborhoodSize = cvPoint(128, 48);
// increasing the neighborhood size does not help very much. In fact, I have been
// degradation of performance.
//const CvPoint Cv3DPoseEstimateStereo::DefNeighborhoodSize = cvPoint(256, 48);
const CvPoint PoseEstimateStereo::DefTemplateSize     = cvPoint(16, 16);

PoseEstimateStereo::PoseEstimateStereo(int width, int height):
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
  mEigImage(NULL),
  mTempImage(NULL),
  mMatchMethod(CalonderDescriptor),
	mCalonderMatcher(NULL),
	mTemplateMatchThreshold(DefTemplateMatchThreshold),
	mDisparityUnitInPixels(CvMatUtils::DefDisparityUnitInPixels)
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

PoseEstimateStereo::~PoseEstimateStereo() {
	delete [] mBufStereoPairs;
	delete [] mFeatureImgBufLeft;
	delete mCalonderMatcher;
	if (mEigImage)  cvReleaseMat(&mEigImage);
	if (mTempImage) cvReleaseMat(&mTempImage);
}

bool PoseEstimateStereo::getDisparityMap(
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

bool PoseEstimateStereo::goodFeaturesToTrack(WImage1_b& img, WImage1_16s* dispMap,
    vector<CvPoint3D64f>& keypoints){
  bool status = true;
  switch(getKeyPointDetector()) {
  case HarrisCorner:
  {
    if (mEigImage  == NULL)
      mEigImage  = cvCreateMat(mSize.height, mSize.width, CV_32FC1);
    if (mTempImage == NULL )
      mTempImage = cvCreateMat(mSize.height, mSize.width, CV_32FC1);
    return goodFeaturesToTrack(img, dispMap, mDisparityUnitInPixels, keypoints, mEigImage, mTempImage);
  }
  case Star:
  {
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
    if (dispMap) {
      int16_t* _mask = dispMap->ImageData();
      BOOST_FOREACH( Keypoint &pt, kps ) {
        int16_t d = _mask[pt.y*mSize.width+pt.x];
        if (d>0) {
          keypoints.push_back(cvPoint3D64f(pt.x, pt.y, (double)d/mDisparityUnitInPixels));
        }
      }
    } else {
      // make a copy, will compiler be smart enough to save real copying
      // leave the disparity value as -1
      BOOST_FOREACH( Keypoint &pt, kps ) {
        keypoints.push_back(cvPoint3D64f((double)pt.x, (double)pt.y, -1.));
      }
    }
  }
  default:
    cerr << __PRETTY_FUNCTION__ <<"() Not implemented yet";
    status = false;
  }
  return status;
}

bool PoseEstimateStereo::goodFeaturesToTrack(WImage1_b& img, WImage1_16s* dispMap,
    double disparityUnitInPixels,
    vector<CvPoint3D64f>& keypoints,
    CvMat* eigImg, CvMat* tempImg
){
  bool status = true;
  CvSize sz = cvSize(img.Width(), img.Height());

  //
  //  Try cvGoodFeaturesToTrack
  //
  int maxKeypoints = 500;
  CvPoint2D32f kps[maxKeypoints];

  //cvConvert(rightimg, tmp);
  //    cvCornerHarris(tmp, harrisCorners, 21);
  double quality_level =  0.01; // what should I use? [0., 1.0]
  double min_distance  = 10.0; // 10 pixels?

  // misc params to cvGoodFeaturesToTrack
  CvMat *mask1 = NULL;
  CvMat mask0;
  if (dispMap) {
    int16_t *disp = dispMap->ImageData();
    int8_t _mask[sz.width*sz.height];
    for (int v=0; v<sz.height; v++) {
      for (int u=0; u<sz.width; u++) {
        int16_t d = disp[v*sz.width+u];
        if (d>0) {
          _mask[v*sz.width+u] = 1;
        } else {
          _mask[v*sz.width+u] = 0;
        }
      }
    }
    mask0= cvMat(sz.height, sz.width, CV_8SC1, _mask);
    mask1 = &mask0;
  }

  const int block_size=5;  // default is 3
  const int use_harris=1;  // default is 0;
  const double k=0.01;     // default is 0.04;
  int numkeypoints = maxKeypoints;
  cvGoodFeaturesToTrack(img.Ipl(), eigImg, tempImg,
      kps, &numkeypoints,
      quality_level, min_distance, mask1, block_size, use_harris, k);
  if (dispMap) {
    for (int i=0; i<numkeypoints; i++) {
      int16_t *disp = dispMap->ImageData();
      double d = disp[(int)(kps[i].y+.5) * sz.width + (int)(kps[i].x+.5)];
      d /= disparityUnitInPixels;
      keypoints.push_back(cvPoint3D64f(kps[i].x, kps[i].y, d));
    }
  } else {
    // disparity map is not available, fill out -1 in place of displarity value.
    for (int i=0; i<numkeypoints; i++) {
      keypoints.push_back(cvPoint3D64f(kps[i].x, kps[i].y, -1.0));
    }
  }
  return status;
}

#if 0
bool PoseEstimateStereo::goodFeaturesToTrack(WImage1_b& img, WImage1_16s* mask, vector<Keypoint>& keypoints){
  bool status = true;
	KeyPointDetector keyPointDetector = getKeyPointDetector();
	switch(keyPointDetector) {
	case HarrisCorner: {
		//
		//  Try cvGoodFeaturesToTrack
		//
		int maxKeypoints = 500;
		CvPoint2D32f kps[maxKeypoints];

		if (mEigImage  == NULL)
		  mEigImage  = cvCreateMat(mSize.height, mSize.width, CV_32FC1);
		if (mTempImage == NULL )
		  mTempImage = cvCreateMat(mSize.height, mSize.width, CV_32FC1);

		CvMat* eig_image  = mEigImage;
		CvMat* temp_image = mTempImage;
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
		const int use_harris=1;  // default is 0;
		const double k=0.01;     // default is 0.04;
		int numkeypoints = maxKeypoints;
		cvGoodFeaturesToTrack(img.Ipl(), eig_image, temp_image,
				kps, &numkeypoints,
				quality_level, min_distance, mask1, block_size, use_harris, k);
		for (int i=0; i<numkeypoints; i++) {
			keypoints.push_back(Keypoint((int)(kps[i].x+.5), (int)(kps[i].y+.5), 0., 0., 0));
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
			mNumKeyPointsWithNoDisparity += numKeyPointsHasNoDisp;
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
#endif

bool PoseEstimateStereo::makePatchRect(const CvPoint& rectSize,
    const CvPoint3D64f& featurePt, CvRect& rect) {
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
PoseEstimateStereo::getTrackablePairsByCalonder(
		WImage1_b& image0, WImage1_b& image1,
		WImage1_16s& dispMap0, WImage1_16s& dispMap1,
		Keypoints& keyPoints0, Keypoints& keyPoints1,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
    vector<pair<int, int> >* trackableIndexPairs
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

	BOOST_FOREACH( CvPoint3D64f &pt, keyPoints1 ) {
		cv::WImageView1_b view = extractPatch(image1.Ipl(), pt);
		DenseSignature sig = mCalonderMatcher->mClassifier.getDenseSignature(view.Ipl());
		matcher.addSignature(sig, cvPoint(pt.x, pt.y));
	}

	// For each key point in image0, find its best match in matcher
	float distance;
	int index = 0;
	CvPoint bestloc;
	BOOST_FOREACH( CvPoint3D64f& pt, keyPoints0 ) {
	  CvRect rectNeighborhood;
	  CvPoint2D64f pt2D64f = cvPoint2D64f(pt.x, pt.y);
	  makePatchRect(neighborhoodSize, pt, rectNeighborhood);

		cv::WImageView1_b view = extractPatch(image0.Ipl(), pt);
		DenseSignature sig = mCalonderMatcher->mClassifier.getDenseSignature(view.Ipl());
//		int match = matcher.findMatch(sig, &distance);
		int match = matcher.findMatchInWindow(sig, rectNeighborhood, &distance);

		if (match>=0) { // got match

		  CvPoint3D64f& kp = keyPoints1.at(match);
		  bestloc.x = kp.x;
		  bestloc.y = kp.y;

		  double disp = getDisparity(dispMap1, bestloc);
		  if (disp<0) {
		    mNumKeyPointsWithNoDisparity++;
		  } else {
		    if (trackablePairs) {
		      CvPoint pt0Int = cvPoint(pt.x, pt.y);
		      double disp0 = getDisparity(dispMap0, pt0Int);
		      CvPoint3D64f pt0 = cvPoint3D64f(pt0Int.x,  pt0Int.y,  disp0);
		      CvPoint3D64f pt1 = cvPoint3D64f(bestloc.x, bestloc.y, disp);
		      pair<CvPoint3D64f, CvPoint3D64f> p(pt0, pt1);
		      trackablePairs->push_back(p);
		    }
		    if (trackableIndexPairs) {
		      trackableIndexPairs->push_back(make_pair(index, match));
		    }
		    numTrackablePairs++;
		  }
		}
		++index;
	}
	assert(numTrackablePairs == (int)(trackablePairs)?trackablePairs->size():
	  (trackableIndexPairs?trackableIndexPairs->size():-1));
	return status;
}


bool
PoseEstimateStereo::getTrackablePairsByCrossCorr(
		WImage1_b& image0, WImage1_b& image1,
		WImage1_16s& dispMap0, WImage1_16s& dispMap1,
		Keypoints& keyPoints0, Keypoints& keyPoints1,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
    vector<pair<int, int> >* trackableIndexPairs
		) {
	bool status = true;
	CvPoint neighborhoodSize = DefNeighborhoodSize;
	CvPoint templSize        = DefTemplateSize;
	// a buffer for neighborhood template matching
	float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
	CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
	int numTrackablePairs=0;

	// loop thru the keypoints of image0 and look for best match from image1
	for (Keypoints::const_iterator ikp = keyPoints0.begin(); ikp!=keyPoints0.end(); ikp++) {
		const CvPoint3D64f& ptLast = *ikp;
		CvPoint fPtLastLeft = cvPoint(ikp->x, ikp->y);

#ifdef DEBUG
		cout << "Feature at "<< ptLast.x<<","<<ptLast.y<<","<<ptLast.z<<endl;
#endif
		// find the closest (in distance and appearance) feature
		// to it in current feature list
		// In a given neighborhood, if there is at least a good feature
		// in the left cam image,
		// we search for a location in current
		// left cam image is that is closest to this one in appearance

		/* find the best correlation in the neighborhood */
		/** make a template center around featurePtLastLeft; */
		CvRect rectTempl;

		bool fOutOfBound = makePatchRect(templSize, ptLast, rectTempl);
		if (fOutOfBound == true) {
			continue;
		}
		CvRect rectNeighborhood;

		fOutOfBound = makePatchRect(neighborhoodSize, ptLast, rectNeighborhood);
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

		double disp = getDisparity(dispMap1, bestloc);

		if (disp<0) {
			this->mNumKeyPointsWithNoDisparity++;
			continue;
		}
		if (trackablePairs) {
		  CvPoint3D64f pt = cvPoint3D64f(bestloc.x, bestloc.y, disp);
		  pair<CvPoint3D64f, CvPoint3D64f> p(ptLast, pt);
		  trackablePairs->push_back(p);
		}
		// Because we search for best match in image instead of keypoint,
		// index pair does not make sense here.
		// TODO: The closest approx maybe to find closes point in keyPoints1
		if (trackableIndexPairs) {
		  status = false;
		}
		numTrackablePairs++;
	}
	assert(numTrackablePairs == (int)(trackablePairs)?trackablePairs->size():-1);
	return status;
}

bool
PoseEstimateStereo::getTrackablePairsByKeypointCrossCorr(
		WImage1_b& image0, WImage1_b& image1,
		WImage1_16s& dispMap0, WImage1_16s& dispMap1,
		Keypoints& keyPoints0, Keypoints& keyPoints1,
		vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
    vector<pair<int, int> >* trackableIndexPairs
) {
	bool status = true;
	CvPoint neighborhoodSize = DefNeighborhoodSize;
	CvPoint templSize        = DefTemplateSize;
	// a buffer for neighborhood template matching
	float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
	CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
	int numTrackablePairs=0;

	// loop thru the keypoints of image0 and look for best match from image1
	int iKeypoint0 = 0; //< the index of the current key point in keypoints0
	for (Keypoints::const_iterator ikp = keyPoints0.begin(); ikp!=keyPoints0.end(); ikp++, iKeypoint0++) {
		const CvPoint3D64f& ptLast = *ikp;
		assert(ptLast.z>=0);

		CvPoint fPtLastLeft = cvPoint(ikp->x, ikp->y);

#ifdef DEBUG
		cout << "Feature at "<< ptLast.x<<","<<ptLast.y<<","<<ptLast.z<<endl;
#endif
		// find the closest (in distance and appearance) feature
		// to it in current feature list
		// In a given neighborhood, if there is at least a good feature
		// in the left cam image,
		// we search for a location in current
		// left cam image is that is closest to this one in appearance
		bool goodFeaturePtInCurrentLeftImg = false;

		// make a template center around featurePtLastLeft;
		CvRect rectTempl = cvRect(
				(int)(.5 + ptLast.x - templSize.x/2),
				(int)(.5 + ptLast.y - templSize.y/2),
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
		float _res[1];
		CvMat res = cvMat(1, 1, CV_32FC1, _res);
		// just to get an idea what is the score for the best possible match
		cvMatchTemplate(&templ, &templ, &res, CV_TM_CCORR_NORMED );
		double threshold = _res[0]*.75;
		double maxScore = threshold;
		int     bestlocIndex = -1;
		int iKeypoint1=0; //< index of the current key point in keypoints1
		for (Keypoints::const_iterator jkp = keyPoints1.begin();
		jkp!=keyPoints1.end();
		jkp++, iKeypoint1++) {
		  CvPoint3D64f pt = *jkp;
			double dx = pt.x - ptLast.x;
			double dy = pt.y - ptLast.y;
			if (fabs(dx)<neighborhoodSize.x && fabs(dy)<neighborhoodSize.y) {
				goodFeaturePtInCurrentLeftImg = true;
#ifdef DEBUG
        cout << "Good candidate at "<< pt.x <<","<< pt.y<<endl;
#endif
				CvRect rectTempl2 = cvRect(
						(int)(.5 + pt.x - templSize.x/2),
						(int)(.5 + pt.y - templSize.y/2),
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
					bestlocIndex = iKeypoint1;
					maxScore = score;
				}
			}
		}
		if (maxScore <= threshold ) {
			continue;
		}

    if (trackablePairs) {
      CvPoint3D64f& pt0 = keyPoints1[bestlocIndex];
      assert(pt0.z>=0);
      trackablePairs->push_back(make_pair(ptLast, pt0));
#ifdef DEBUG
        cout << "trackable pair "<< ptLast.x <<","<< ptLast.y<<"  <==> "<< pt.x <<","<< pt.y<<endl;
#endif
		}
		if (trackableIndexPairs) {
		  trackableIndexPairs->push_back(make_pair(iKeypoint0, bestlocIndex));
#ifdef DEBUG
        cout << "trackable pair "<< iKeypoint0 <<"  <==> "<< bestlocIndex <<endl;
#endif
		}
		numTrackablePairs++;
	}
  assert((trackablePairs?      numTrackablePairs == (int)trackablePairs->size()      : true));
  assert((trackableIndexPairs? numTrackablePairs == (int)trackableIndexPairs->size() : true));

	return status;
}

bool PoseEstimateStereo::getTrackablePairs(
			WImage1_b& image0, WImage1_b& image1,
			WImage1_16s& dispMap0, WImage1_16s& dispMap1,
			Keypoints& keyPoints0, Keypoints& keyPoints1,
			vector<pair<CvPoint3D64f, CvPoint3D64f> >* trackablePairs,
			vector<pair<int, int> >* trackableIndexPairs
			) {
	bool status = true;
	switch(mMatchMethod){
	case CrossCorrelation:
		status = getTrackablePairsByCrossCorr(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
				trackablePairs, trackableIndexPairs);
		break;
	case KeyPointCrossCorrelation:
		status = getTrackablePairsByKeypointCrossCorr(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
				trackablePairs, trackableIndexPairs);
		break;
	case CalonderDescriptor:
		status = getTrackablePairsByCalonder(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
						trackablePairs, trackableIndexPairs);
		break;
	default:
		status = false;
		break;
	}
	return status;
}

double PoseEstimateStereo::matchTemplate(const CvMat& neighborhood, const CvMat& templ, CvMat& res,
		CvPoint& loc){
	cvMatchTemplate(&neighborhood, &templ, &res, CV_TM_CCORR_NORMED );
	double		minval, maxval;
	cvMinMaxLoc( &res, &minval, &maxval, NULL, &loc, NULL);
	return maxval;
}

PoseEstimateStereo::CalonderMatcher::CalonderMatcher(string& modelfilename){
	  mClassifier.read(modelfilename.c_str());
	  mClassifier.setThreshold(SIG_THRESHOLD);
}

