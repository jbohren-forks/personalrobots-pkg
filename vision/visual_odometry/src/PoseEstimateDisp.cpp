
#include <iostream>
using namespace std;

#include <boost/foreach.hpp>

#include <opencv/cv.h>

#include "PoseEstimateDisp.h"

#include "CvMat3X3.h"
#include "LevMarqTransformDispSpace.h"

#include "CvTestTimer.h"
using namespace cv::willow;

#undef DEBUG
#define USE_LEVMARQ
//#define DEBUG_DISTURB_PARAM

#if 0
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


PoseEstimateDisp::PoseEstimateDisp():
  PoseParent(), Parent()
{
  // overide parent default
  mErrThreshold = 1.5;
}

PoseEstimateDisp::~PoseEstimateDisp()
{
}

int PoseEstimateDisp::checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation){
	int numInLiers = 0;
	int numPoints = points0->rows;

	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows,
	// and num of col is 3
	double *_P0 = points0->data.db;
	double *_P1 = points1->data.db;
	double thresholdM =  this->mErrThreshold;
	double thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;
	for (int i=0; i<numPoints; i++) {

		double p0x, p0y, p0z;

#if 1
		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		double w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
#else
		// not worth using the following code
		// 1) not sure if it helps on speed.
		// 2) not sure if the order of evaluation is preserved as left to right.
		double w3 =
			_T[12]*(p0x=*_P0++) +
			_T[13]*(p0y=*_P0++) +
			_T[14]*(p0z=*_P0++) +
			_T[15];
#endif

		double scale = 1.0/w3;

		double rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		double ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		double rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
		if (rz>thresholdM || rz< thresholdm) {
			continue;
		}

		numInLiers++;
	}
#ifdef DEBUG
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;
}

// almost the same as the function above
int PoseEstimateDisp::getInLiers(const CvMat *points0,
    const CvMat *points1, const CvMat* transformation,
    CvMat* points0Inlier, CvMat* points1Inlier,
    int inlierIndices[]) {
	int numInLiers = 0;
	int numPoints = points0->rows;

	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows,
	// and num of col is 3
	double *_P0 = points0->data.db;
	double *_P1 = points1->data.db;
	double thresholdM =  this->mErrThreshold;
	double thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;
	for (int i=0; i<numPoints; i++) {

		double p0x, p0y, p0z;

		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		double w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
		double scale = 1.0/w3;

		double rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		double ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		double rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
		if (rz>thresholdM || rz< thresholdm) {
			continue;
		}

		// store the inlier
		if (points0Inlier) {
		  cvmSet(points0Inlier, numInLiers, 0, p0x);
		  cvmSet(points0Inlier, numInLiers, 1, p0y);
		  cvmSet(points0Inlier, numInLiers, 2, p0z);
		}
		if (points1Inlier) {
		  cvmSet(points1Inlier, numInLiers, 0, *(_P1-3));
		  cvmSet(points1Inlier, numInLiers, 1, *(_P1-2));
		  cvmSet(points1Inlier, numInLiers, 2, *(_P1-1));
		}
		if (inlierIndices) {
		  inlierIndices[numInLiers] = i;
		}
		numInLiers++;
	}
#ifdef DEBUG
	cout << "Num of Inliers: "<<numInLiers<<endl;
#endif
	return numInLiers;
}

int PoseEstimateDisp::estimate(const Keypoints& keypoints0, const Keypoints& keypoints1,
    const vector<pair<int, int> >& matchIndexPairs, CvMat& rot, CvMat& shift,
    bool smoothed){
  int numTrackablePairs = matchIndexPairs.size();
  double _uvds0[3*numTrackablePairs];
  double _uvds1[3*numTrackablePairs];
  CvMat uvds0 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds0);
  CvMat uvds1 = cvMat(numTrackablePairs, 3, CV_64FC1, _uvds1);

  int iPt=0;
  typedef const pair<int, int> intpair;
  BOOST_FOREACH( intpair& p, matchIndexPairs ) {
    const CvPoint3D64f& p0 = keypoints0[p.first];
    const CvPoint3D64f& p1 = keypoints1[p.second];
    _uvds0[iPt*3 + 0] = p0.x;
    _uvds0[iPt*3 + 1] = p0.y;
    _uvds0[iPt*3 + 2] = p0.z;

    _uvds1[iPt*3 + 0] = p1.x;
    _uvds1[iPt*3 + 1] = p1.y;
    _uvds1[iPt*3 + 2] = p1.z;
    iPt++;
  }

  // estimate the transform the observed points from current back to last position
  // it should be equivalent to the transform of the camera frame from
  // last position to current position
  int numInliers;
  numInliers = this->estimate(&uvds0, &uvds1, &rot, &shift, smoothed);

  return numInliers;
}

int PoseEstimateDisp::estimate(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
    CvMat& rot, CvMat& shift, bool smoothed) {
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
  numInliers = this->estimate(&uvds0, &uvds1, &rot, &shift, smoothed);

  return numInliers;
}

int PoseEstimateDisp::estimateMixedPointClouds(
    CvMat *xyzs0, CvMat *uvds1,
    int numRefGrps, int refPoints[],
    CvMat *rot, CvMat *shift, bool smoothed) {

  // convert the first point cloud, xyzs0, which is in cartesian space
  // into disparity space
  int numPoints = xyzs0->rows;

  CvMat* uvds0 = cvCreateMat(numPoints, 3, CV_64FC1);
  CvMat* xyzs1 = cvCreateMat(numPoints, 3, CV_64FC1);

  dispToCart(uvds1, xyzs1);
  cartToDisp(xyzs0, uvds0);

  int numInLiers = estimate(xyzs0, xyzs1, uvds0, uvds1,
      numRefGrps, refPoints,
      rot, shift, smoothed);

  cvReleaseMat(&uvds0);
  cvReleaseMat(&xyzs1);
  return numInLiers;
}

int PoseEstimateDisp::estimate(CvMat *uvds0, CvMat *uvds1,
    CvMat *rot, CvMat *shift, bool smoothed) {
  int numInLiers = 0;

  int numPoints = uvds0->rows;

  CvMat* xyzs0 = cvCreateMat(numPoints, 3, CV_64FC1);
  CvMat* xyzs1 = cvCreateMat(numPoints, 3, CV_64FC1);

  // project from disparity space back to XYZ
  dispToCart(uvds0, xyzs0);
  dispToCart(uvds1, xyzs1);

  numInLiers = estimate(xyzs0, xyzs1, uvds0, uvds1, 0, NULL, rot, shift, smoothed);

  cvReleaseMat(&xyzs0);
  cvReleaseMat(&xyzs1);
  return numInLiers;
}

int PoseEstimateDisp::estimate(CvMat *xyzs0, CvMat *xyzs1,
    CvMat *uvds0, CvMat *uvds1,
    int numRefGrps, int refPoints[],
    CvMat *rot, CvMat *shift, bool smoothed) {
  int numPoints = xyzs0->rows;

  // Sanity checks
  if (numPoints < 3) {
    cerr << __PRETTY_FUNCTION__ <<"too few points to do RANSAC: "<<numPoints<<endl;
    return 0;
  }

  if (numPoints != uvds1->rows ||
      numPoints != xyzs1->rows ||
      numPoints != uvds0->rows ) {
    cerr << "number of points mismatched in input" << endl;
    return 0;
  }

  // make sure if the disparity values in uvds0 and uvds1 are greater than
  // zero. Column 2 is the disparity col
  double min_val, max_val;
  CvMat dispCol;
  cvGetCol(uvds0, &dispCol, 2);
  cvMinMaxLoc(&dispCol, &min_val, &max_val, NULL, NULL, NULL);
  if (min_val<=0) {
    cerr << "please make sure disparity values are greater than zero\n";
    return 0;
  }
  cvGetCol(uvds1, &dispCol, 2);
  cvMinMaxLoc(&dispCol, &min_val, &max_val, NULL, NULL, NULL);
  if (min_val<=0) {
    cerr << "please make sure disparity values are greater than zero\n";
    return 0;
  }
  // done with sanity checks

  int numInLiers = 0;
  double _P0[3*3], _P1[3*3], _R[3*3], _T[3*1], _H[4*4];
  CvMat P0, P1;
  cvInitMatHeader(&P0, 3, 3, CV_64FC1, _P0); // 3 random points from camera0, stored in columns
  cvInitMatHeader(&P1, 3, 3, CV_64FC1, _P1); // 3 random points from caemra1, stored in columns

  CvMat R, T, H;
  cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);  // rotation matrix, R = V * tranpose(U)
  cvInitMatHeader(&T,  3, 1, CV_64FC1, _T);  // translation matrix
  cvInitMatHeader(&H,  4, 4, CV_64FC1, _H);  // disparity space homography

  int maxNumInLiers=0;
  mRandomTripletSetGenerator.reset(0, numPoints-1);
  if (numRefGrps == 0) {
    for (int i=0; i< mNumRansacIter; i++) {
#ifdef DEBUG
      cout << "Iteration: "<< i << endl;
#endif
      // randomly pick 3 points. make sure they are not
      // tooCloseToColinear
      if (pick3RandomPoints(xyzs0, xyzs1, &P0, &P1)== false) {
        // we have exhausted all possible combinations of triplet sets
        // get out of the loop
        break;
      }

      TIMERSTART2(SVD);
      this->estimateLeastSquareInCol(&P0, &P1, &R, &T);
      TIMEREND2(SVD);

      this->constructDisparityHomography(&R, &T, &H);

#ifdef DEBUG
      cout << "R, T, and H: "<<endl;
      CvMatUtils::printMat(&R);
      CvMatUtils::printMat(&T);
      CvMatUtils::printMat(&H);
#endif

      CvTestTimerStart(CheckInliers);
      // scoring against all points
      numInLiers = checkInLiers(uvds0, uvds1, &H);
      CvTestTimerEnd(CheckInliers);

      // keep the best R and T
      if (maxNumInLiers < numInLiers) {
        maxNumInLiers = numInLiers;
        cvCopy(&R, rot);
        cvCopy(&T, shift);
      }
    }
  } else {
    for (int j=0; j<3; j++) {

      _P0[      j] = cvmGet(xyzs0, refPoints[j], 0);
      _P0[1*3 + j] = cvmGet(xyzs0, refPoints[j], 1);
      _P0[2*3 + j] = cvmGet(xyzs0, refPoints[j], 2);

      _P1[      j] = cvmGet(xyzs1, refPoints[j], 0);
      _P1[1*3 + j] = cvmGet(xyzs1, refPoints[j], 1);
      _P1[2*3 + j] = cvmGet(xyzs1, refPoints[j], 2);
    }

    this->estimateLeastSquareInCol(&P0, &P1, rot, shift);
    this->constructDisparityHomography(rot, shift, &H);
    maxNumInLiers = checkInLiers(uvds0, uvds1, &H);
  }

  if (maxNumInLiers<6) {
    cout << "Too few inliers: "<< maxNumInLiers << endl;
    return maxNumInLiers;
  }

  // get a copy of all the inliers, original and transformed
  CvMat *uvds0Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
  CvMat *uvds1Inlier = cvCreateMat(maxNumInLiers, 3, CV_64FC1);
  int   *inlierIndices = new int[maxNumInLiers];
  // construct homography matrix
  constructDisparityHomography(rot, shift, &H);
  int numInliers0 = getInLiers(uvds0, uvds1, &H, uvds0Inlier, uvds1Inlier, inlierIndices);

  // make a copy of the best Transformation before nonlinear optimization
  cvCopy(&H, &mRTBestWithoutLevMarq);

  if (smoothed == true) {
    estimateWithLevMarq(*uvds0Inlier, *uvds1Inlier, *rot, *shift);
  }

  updateInlierInfo(uvds0Inlier, uvds1Inlier, inlierIndices);

  // construct the final transformation matrix
  this->constructDisparityHomography(rot, shift, &mT);
  return numInliers0;
}

bool PoseEstimateDisp::estimateWithLevMarq(const CvMat& uvds0Inlier,
    const CvMat& uvds1Inlier,
    CvMat& rot, CvMat& shift) {
  return estimateWithLevMarq(uvds0Inlier, uvds1Inlier,
      mat_cart_to_disp_, mat_disp_to_cart_, rot, shift);
}

bool PoseEstimateDisp::estimateWithLevMarq(
    const CvMat& uvds0Inlier, const CvMat& uvds1Inlier,
    const CvMat& CartToDisp,  const CvMat& DispToCart,
    CvMat& rot, CvMat& shift) {

  int numInliers = uvds0Inlier.rows;

  if (numInliers<6) {
    cerr << "WARNING: Too few inliers: "<< numInliers << endl;
    return false;
  }

  // nonlinear optimization by Levenberg-Marquardt
  cv::willow::LevMarqTransformDispSpace
  levMarq(&DispToCart, &CartToDisp, numInliers);

  double param[6];

  //initialize the parameters
  //initialize the parameters
  levMarq.rotAndShiftMatsToParams(rot, shift, param);

  CvTestTimerStart(LevMarqDoit);
  levMarq.optimize(&uvds0Inlier, &uvds1Inlier, param);
  CvTestTimerEnd(LevMarqDoit);

  // TODO: construct matrix with parameters from nonlinear optimization
  levMarq.paramsToRotAndShiftMats(param, rot, shift);
  return true;
}

bool PoseEstimateDisp::constructDisparityHomography(const CvMat *R, const CvMat *T,
    CvMat *H){
  if (R == NULL || T == NULL) {
    return false;
  }
  return this->constructHomography(*R, *T, mat_disp_to_cart_, mat_cart_to_disp_,
      *H);
}

bool PoseEstimateDisp::constructHomography(const CvMat& R, const CvMat& T,
    const CvMat& dispToCart, const CvMat& cartToDisp, CvMat& H){
    bool status = true;
    // Transformation matrix RT:
    //         R  t
    // RT = (       )
    //         0  1
    // Disparity Space Homography
    // H = Gamma RT inv(Gamma)
    //

    double _RT[16], _G[16];
    CvMat RT, G;
    cvInitMatHeader(&RT, 4, 4, CV_64FC1, _RT); // transformation matrix (including rotation and translation)
    cvInitMatHeader(&G,  4, 4, CV_64FC1, _G);  // a temp matrix

    CvMatUtils::transformFromRotationAndShift(R, T, RT);

    cvMatMul(&cartToDisp, &RT, &G);
    cvMatMul(&G, (CvMat *)&dispToCart, &H);
    return status;
}





