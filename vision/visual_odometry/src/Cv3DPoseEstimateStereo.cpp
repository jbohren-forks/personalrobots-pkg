/*
 * Cv3DPoseEstimateStereo.cpp
 *
 *  Created on: Aug 22, 2008
 *      Author: jdchen
 */

#include "Cv3DPoseEstimateStereo.h"
#include "CvMatUtils.h"
#include "VisOdom.h"
#include "KeypointDescriptors.h"
using namespace cv::willow;
#include "stereolib.h" // from 3DPoseEstimation/include. The header file is there temporarily

#include <iostream>
using namespace std;

//opencv
#include <opencv/cv.h>
#include <opencv/cvwimage.h>

// boost
#include <boost/foreach.hpp>
#include <cassert>

#if STAR_DETECTOR
// star detector
#include <star_detector/detector.h>
#endif

// sse intrinsic
#include <emmintrin.h>

#include <stdlib.h>

#undef DEBUG
//#define DEBUG 1

#if CHECKTIMING == 0
#define TIMERSTART(x)
#define TIMEREND(x)
#define TIMERSTART2(x)
#define TIMEREND2(x)
#else
#define TIMERSTART(x)  CvTestTimerStart(x)
#define TIMEREND(x)    CvTestTimerEnd(x)
#define TIMERSTART2(x) CvTestTimerStart2(x)
#define TIMEREND2(x)   CvTestTimerEnd2(x)
#endif

// change from 61 to 101 to accommodate the sudden change around frame 0915 in
// the indoor sequence "indoor1"
/// matching key points must not be further than these two values, in respective
/// directions.
//const CvPoint PoseEstimateStereo::DefNeighborhoodSize = cvPoint(128, 48);
const CvPoint PoseEstimateStereo::DefNeighborhoodSize = cvPoint(64, 32);
// increasing the neighborhood size does not help very much. In fact, I have been
// degradation of performance.
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
  mKeyPointDetector(HarrisCorner),
	mNumScales(DefNumScales),
	mThreshold(DefThreshold),
	mMaxNumKeyPoints(DefMaxNumKeyPoints),
#if STAR_DETECTOR
	mStarDetector(mSize, mNumScales, mThreshold),
#endif
  mEigImage(NULL),
  mTempImage(NULL),
  mMatchMethod(KeyPointCrossCorrelation),
	mCalonderMatcher(NULL),
	mDisparityUnitInPixels(DefDisparityUnitInPixels)
{
  mBufStereoPairs     = new uint8_t[mSize.height*mDLen*(mCorr+5)]; // local storage for the stereo pair algorithm
  // allocate the following buffer aligned with 16 byte block
  // so that they can be faster to access in sse
  size_t bufsz = mSize.width*mSize.height;
  posix_memalign((void **)&mFeatureImgBufLeft,  SSEAlignment, bufsz);
  posix_memalign((void **)&mFeatureImgBufRight, SSEAlignment, bufsz);
//  mFeatureImgBufLeft  = new uint8_t[mSize.width*mSize.height];
//  mFeatureImgBufRight = new uint8_t[mSize.width*mSize.height];

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
	case KeyPointSumOfAbsDiff:
		break;
	}
}

PoseEstimateStereo::~PoseEstimateStereo() {
  delete [] mBufStereoPairs;
  free(mFeatureImgBufLeft);
  free(mFeatureImgBufRight);
//  delete [] mBufStereoPairs;
//  delete [] mFeatureImgBufLeft;
  delete mCalonderMatcher;
	if (mEigImage)  delete mEigImage;
	if (mTempImage) delete mTempImage;
}

bool PoseEstimateStereo::getDisparityMap(
		const WImage1_b& leftImage, const WImage1_b& rightImage, WImage1_16s& dispMap) {
	bool status = true;

	if (leftImage.Width()  != mSize.width || leftImage.Height()  != mSize.height ||
			rightImage.Width() != mSize.width || rightImage.Height() != mSize.height ) {
		cerr << __PRETTY_FUNCTION__ <<"(): size of images incompatible. "<< endl;
		return false;
	}

	//
	// Try Kurt's dense stereo pair
	//
	const uint8_t *lim = leftImage.ImageData();
	const uint8_t *rim = rightImage.ImageData();

	int16_t* disp    = dispMap.ImageData();
	int16_t* textImg = NULL;

	// prefilter
	do_prefilter((uint8_t *)lim, mFeatureImgBufLeft, mSize.width, mSize.height, mFTZero, mBufStereoPairs);
	do_prefilter((uint8_t *)rim, mFeatureImgBufRight, mSize.width, mSize.height, mFTZero, mBufStereoPairs);

	// stereo
	do_stereo(mFeatureImgBufLeft, mFeatureImgBufRight, disp, textImg, mSize.width, mSize.height,
			mFTZero, mCorr, mCorr, mDLen, mTextThresh, mUniqueThresh, mBufStereoPairs);

	return status;
}

bool PoseEstimateStereo::constructKeypointDescriptors(
    const WImage1_b& leftImage,
    Keypoints& keypoints) {
  bool status = false;
  switch(mMatchMethod){
  case CrossCorrelation:
  case KeyPointCrossCorrelation:
    KeypointDescriptor::constructTemplateDescriptors(leftImage.ImageData(),
        leftImage.Width(), leftImage.Height(), keypoints);
    break;
  case KeyPointSumOfAbsDiff:
    // share two buffers with getDisparityMap()
    KeypointDescriptor::constructSADDescriptors(leftImage.ImageData(),
        leftImage.Width(), leftImage.Height(), keypoints, mFeatureImgBufLeft, mBufStereoPairs);
    break;
  default:
    cerr << "Not implement yet for matcher: "<<mMatchMethod<<endl;
    status = false;
    break;
  }
  return status;
}

bool PoseEstimateStereo::computeKeypointDisparity(
    const WImage1_b& rightImage,
    Keypoints& keypoints) {
  bool status = false;
  switch(mMatchMethod){
  case CrossCorrelation:
  case KeyPointCrossCorrelation:
    cerr << "Not implement yet for matcher: "<<mMatchMethod<<endl;
    status = false;
    break;
  case KeyPointSumOfAbsDiff:
    // share two buffers with getDisparityMap()
    KeypointSADDescriptor::computeDisparity(rightImage.ImageData(),
        rightImage.Width(), rightImage.Height(), keypoints, mFeatureImgBufLeft,
        mBufStereoPairs);
    break;
  default:
    cerr << "Not implement yet for matcher: "<<mMatchMethod<<endl;
    status = false;
    break;
  }
  return status;
}

bool PoseEstimateStereo::goodFeaturesToTrack(
    const WImage1_b& img, const WImage1_16s* dispMap,
    Keypoints& keypoints){
  bool status = true;
  switch(getKeyPointDetector()) {
  case HarrisCorner:
  {
    if (mEigImage  == NULL)
      mEigImage  = new WImageBuffer1_f(mSize.width, mSize.height);
    if (mTempImage == NULL )
      mTempImage = new WImageBuffer1_f(mSize.width, mSize.height);
    return goodFeaturesToTrack(img, dispMap, mDisparityUnitInPixels, keypoints, *mEigImage, *mTempImage);
  }
  case Star:
  {
#if STAR_DETECTOR
    //
    // Try Star Detector
    //
    std::vector<Keypoint> kps = mStarDetector.DetectPoints((IplImage*)img.Ipl());
#ifdef DEBUG
    cout << "Found "<< kps.size() << " good keypoints by Star Detector"<<endl;
#endif
    if (kps.size() > mMaxNumKeyPoints) {
      std::nth_element(kps.begin(), kps.begin()+mMaxNumKeyPoints, kps.end());
      kps.erase(kps.begin()+mMaxNumKeyPoints, kps.end());
    }

    // filter out keypoints that do not have disparity value
    if (dispMap) {
      const int16_t* _mask = dispMap->ImageData();
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
#else
    cerr << __PRETTY_FUNCTION__ <<"() Star detector not include in this build"<<endl;
#endif
  }
  default:
    cerr << __PRETTY_FUNCTION__ <<"() Not implemented yet"<<endl;
    status = false;
  }
  return status;
}

bool PoseEstimateStereo::goodFeaturesToTrack(
    const WImage1_b& img,
    const WImage1_16s* dispMap,
    double disparityUnitInPixels,
    Keypoints& keypoints,
    WImage1_f& eigImg, WImage1_f& tempImg
){
  bool status = true;
  CvSize sz = cvSize(img.Width(), img.Height());

  //
  //  Try cvGoodFeaturesToTrack
  //
  int maxKeypoints = 500;

  // misc params to cvGoodFeaturesToTrack
  CvMat *mask1 = NULL;
  if (dispMap) {
    mask1 = CvMatUtils::dispMapToMask(*dispMap);
  }

  int numkeypoints = goodFeaturesToTrackHarrisCorner(img, eigImg, tempImg, keypoints,
      maxKeypoints, HarrisCornerQualityLevel, HarrisCornerMinDistance, (CvArr*)mask1,
      HarrisCornerBlockSize, HarrisCornerFreeParam);
#if DEBUG
  int numKeypointWithNoDisp=0;
#endif
  if (dispMap) {
    const int16_t *disp = dispMap->ImageData();
    BOOST_FOREACH( Keypoint& kp, keypoints) {
      double d = disp[(int)(kp.y+.5) * sz.width + (int)(kp.x+.5)];
      if (d>=0) { // this could happen due to rounding
        d /= disparityUnitInPixels;
        kp.z = d;
      } else {
        // no disparity
        // this could happen due to rounding
#if DEBUG
        numKeypointWithNoDisp++;
#endif
      }
    }
#if DEBUG
    printf("%d key points have not disp\n", numKeypointWithNoDisp);
#endif
  } else {
#if DEBUG
    printf("no disp map\n");
#endif
    // disparity map is not available, do nothing
  }
  return status;
}

/// Use Harris corner to extrack keypoints.
/// @return the number of detected feature points
int PoseEstimateStereo::goodFeaturesToTrackHarrisCorner(
    /// input image
    const WImage1_b& image,
    /// Temporary floating-point 32-bit image of the same size as image.
    WImage1_f& eigImg,
    /// Another temporary image of the same size and same format as eig_image.
    WImage1_f& tempImg,
    /// (OUTPUT) key point detected
    Keypoints& keypoints,
    /// (input) max number of key points
    int maxNumKeypoints,
    /// threshold value (between 0.0 and 1.0), -1 if don't care
    double qualityThreshold,
    /// mininum distance between the keypoints
    double minDistance,
    /// Region of interest. The function selects points either in the specified
    /// region or in the whole image if the mask is NULL.
    CvArr* mask,
    /// neighborhood size @see cvCornerHarris, @see CornerEigenValsAndVecs
    int blockSize,
    /// the free parameter in Harris corner @see cvGoodFeaturesToTrack()
    /// @see cvCornerHarris()
    double k
) {
  int numkeypoints = maxNumKeypoints;
  CvPoint2D32f kps[numkeypoints];
  cvGoodFeaturesToTrack(image.Ipl(), eigImg.Ipl(), tempImg.Ipl(),
      kps, &numkeypoints, qualityThreshold, minDistance, mask, blockSize,
      HarrisCornerDetectorFlag, k);

  // unfortunately, we do not have reponses from this interface to sort them
  // create an array of Keypoint objects,
  for (int i=0; i<numkeypoints; i++) {
    keypoints.push_back(Keypoint(kps[i].x, kps[i].y, -1, 0, -1, NULL));
  }

  return numkeypoints;
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

bool PoseEstimateStereo::makePatchRect(
    const CvPoint& rectSize,
    const CvPoint3D64f& featurePt,
    const CvSize& imgSize,
    CvRect& rect) {
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
	} else if ( rect.x + rect.width  > imgSize.width ) {
		rect.width = imgSize.width - rect.x;
		fOutOfBound = true;
	}
	if (rect.y < 0 ) {
		rect.y = 0;
		fOutOfBound = true;
	} else if ( rect.y + rect.height > imgSize.height ) {
		rect.height = imgSize.height - rect.y;
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
#if CALONDER_DESCRIPTOR
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
	  makePatchRect(neighborhoodSize, pt, mSize, rectNeighborhood);

		cv::WImageView1_b view = extractPatch(image0.Ipl(), pt);
		DenseSignature sig = mCalonderMatcher->mClassifier.getDenseSignature(view.Ipl());
//		int match = matcher.findMatch(sig, &distance);
		int match = matcher.findMatchInWindow(sig, rectNeighborhood, &distance);

		if (match>=0) { // got match

		  CvPoint3D64f& kp = keyPoints1.at(match);
		  bestloc.x = kp.x;
		  bestloc.y = kp.y;

		  // use the default value for unit in disparity map, 1/16 pixel
		  double disp = CvMatUtils::getDisparity(dispMap1, bestloc);
		  if (disp<0) {
		    mNumKeyPointsWithNoDisparity++;
		  } else {
		    if (trackablePairs) {
		      CvPoint pt0Int = cvPoint(pt.x, pt.y);
		      double disp0 = CvMatUtils::getDisparity(dispMap0, pt0Int);
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
#else
	cerr << __PRETTY_FUNCTION__ <<"(): Calonder matcher (based on Calonder descriptor) is not included in this build. "<< endl;
	return false;
#endif
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
	CvSize imgSize = cvSize(image0.Width(), image0.Height());
	// TODO: check if other image buffers are consistent.
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

		bool fOutOfBound = makePatchRect(templSize, ptLast, imgSize, rectTempl);
		if (fOutOfBound == true) {
			continue;
		}
		CvRect rectNeighborhood;

		fOutOfBound = makePatchRect(neighborhoodSize, ptLast, imgSize, rectNeighborhood);
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
		if (matchingScore < DefTemplateMatchThreshold) {
			// best matching is not good enough, skip this key point
			continue;
		}

		bestloc.x += rectNeighborhood.x + rectTempl.width/2;
		bestloc.y += rectNeighborhood.y + rectTempl.height/2;

		// we assume the unit in disparity map is 1/16 of a pixel
		double disp = CvMatUtils::getDisparity(dispMap1, bestloc);

		if (disp<0) {
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
	CvSize imgSize = cvSize(image0.Width(), image0.Height());
	// TODO: check the sizes of other image buffers to see if they are the same.
	// a buffer for neighborhood template matching
	float _res[(neighborhoodSize.y-templSize.y+1)*(neighborhoodSize.x-templSize.x+1)];
	CvMat res = cvMat(neighborhoodSize.y-templSize.y+1, neighborhoodSize.x-templSize.x+1, CV_32FC1, _res);
	int numTrackablePairs=0;

	// loop thru the keypoints of image0 and look for best match from image1
	int iKeypoint0 = 0; //< the index of the current key point in keypoints0
	for (Keypoints::const_iterator ikp = keyPoints0.begin(); ikp!=keyPoints0.end(); ikp++, iKeypoint0++) {
//    const CvPoint3D64f& ptLast = *ikp;
    const Keypoint& ptLast = *ikp;
		if (ptLast.z<0) { // disparity value maybe still missing
		  continue;
		}

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
				rectTempl.x + rectTempl.width  > imgSize.width ||
				rectTempl.y + rectTempl.height > imgSize.height ) {
			// (partially) out of bound.
			// skip this
			continue;
		}

		double minDist = DBL_MAX;

    // loop thru keypoints1 and get the best match
		int     bestlocIndex = -1;
		int iKeypoint1=0; //< index of the current key point in keypoints1
		for (Keypoints::const_iterator jkp = keyPoints1.begin();
		jkp!=keyPoints1.end();
		jkp++, iKeypoint1++) {
//      const CvPoint3D64f pt& = *jkp;
      const Keypoint& pt = *jkp;
		  if (pt.z < 0) { // disparity value is missing
		    continue;
		  }
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
						rectTempl2.x + rectTempl2.width  > imgSize.width ||
						rectTempl2.y + rectTempl2.height > imgSize.height ) {
					// (partially) out of bound.
					// skip this
					continue;
				}

				double dist = ptLast.desc->compare(*pt.desc);
        if (dist < minDist) {
          bestlocIndex = iKeypoint1;
          minDist = dist;
        }
			}
		}

		if (bestlocIndex < 0) {
		  continue;
		}

		// fill in the output buffer
    if (trackablePairs) {
      CvPoint3D64f& pt0 = keyPoints1[bestlocIndex];
      CvPoint3D64f pt1 = cvPoint3D64f(ptLast.x, ptLast.y, ptLast.z);
      assert(pt0.z>=0);
      trackablePairs->push_back(make_pair(pt1, pt0));
#ifdef DEBUG
        cout << "trackable pair "<< ptLast.x <<","<< ptLast.y<<"  <==> "<< pt0.x <<","<< pt0.y<<endl;
#endif
		}
		if (trackableIndexPairs) {
		  trackableIndexPairs->push_back(make_pair(iKeypoint0, bestlocIndex));
#ifdef DEBUG
        cout << "trackable pair indices "<< iKeypoint0 <<"  <==> "<< bestlocIndex <<endl;
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
	case KeyPointSumOfAbsDiff:
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

bool PoseEstimateStereo::getTrackablePairs(
    /// type of the key point matcher
    MatchMethod matcherType,
    /// input image 0
    WImage1_b& image0,
    /// input image 1
    WImage1_b& image1,
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
) {
  bool status = true;
  switch(matcherType){
  case CrossCorrelation:
    status = getTrackablePairsByCrossCorr(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
        trackablePairs, trackableIndexPairs);
    break;
  case KeyPointCrossCorrelation:
    status = getTrackablePairsByKeypointCrossCorr(image0, image1, dispMap0, dispMap1, keyPoints0, keyPoints1,
        trackablePairs, trackableIndexPairs);
    break;
  default:
    cerr << "Not implement yet for matcher: "<<matcherType<<endl;
    status = false;
    break;
  }
  return status;
}

double PoseEstimateStereo::matchTemplate(const CvMat& neighborhood, const CvMat& templ, CvMat& res,
		CvPoint& loc)  {
	cvMatchTemplate(&neighborhood, &templ, &res, CV_TM_CCORR_NORMED );
	double		minval, maxval;
	cvMinMaxLoc( &res, &minval, &maxval, NULL, &loc, NULL);
	return maxval;
}

PoseEstimateStereo::CalonderMatcher::CalonderMatcher(string& modelfilename){
#if CALONDER_DESCRIPTOR
	  mClassifier.read(modelfilename.c_str());
	  mClassifier.setThreshold(SIG_THRESHOLD);
#endif
}

