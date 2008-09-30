
#include <iostream>
using namespace std;

#include <opencv/cv.h>

#include "Cv3DPoseEstimateDisp.h"

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


Cv3DPoseEstimateDisp::Cv3DPoseEstimateDisp():
  PoseParent(), Parent()
{
  // overide parent default
  mErrThreshold = 1.5;
}

Cv3DPoseEstimateDisp::~Cv3DPoseEstimateDisp()
{
}

int Cv3DPoseEstimateDisp::checkInLiers(CvMat *points0, CvMat *points1, CvMat* transformation){
	int numInLiers = 0;
	int numPoints = points0->rows;

	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows,
	// and num of col is 3
	CvMyReal *_P0 = points0->data.db;
	CvMyReal *_P1 = points1->data.db;
	CvMyReal thresholdM =  this->mErrThreshold;
	CvMyReal thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;
	for (int i=0; i<numPoints; i++) {

		CvMyReal p0x, p0y, p0z;

#if 1
		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		CvMyReal w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
#else
		// not worth using the following code
		// 1) not sure if it helps on speed.
		// 2) not sure if the order of evaluation is preserved as left to right.
		CvMyReal w3 =
			_T[12]*(p0x=*_P0++) +
			_T[13]*(p0y=*_P0++) +
			_T[14]*(p0z=*_P0++) +
			_T[15];
#endif

		CvMyReal scale = 1.0/w3;

		CvMyReal rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		CvMyReal ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		CvMyReal rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
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
int Cv3DPoseEstimateDisp::getInLiers(CvMat *points0, CvMat *points1, CvMat* transformation,
    CvMat* points0Inlier, CvMat* points1Inlier, int *inlierIndices) {
	int numInLiers = 0;
	int numPoints = points0->rows;

	// TODO: I hope this is okay. We need to make sure that the matrix data is in rows,
	// and num of col is 3
	CvMyReal *_P0 = points0->data.db;
	CvMyReal *_P1 = points1->data.db;
	CvMyReal thresholdM =  this->mErrThreshold;
	CvMyReal thresholdm = -this->mErrThreshold;
	double *_T = transformation->data.db;
	for (int i=0; i<numPoints; i++) {

		CvMyReal p0x, p0y, p0z;

		p0x = *_P0++;
		p0y = *_P0++;
		p0z = *_P0++;

		CvMyReal w3 = _T[15] + _T[14]*p0z + _T[13]*p0y + _T[12]*p0x;
		CvMyReal scale = 1.0/w3;

		CvMyReal rx = *_P1++ - (_T[3] + _T[2]*p0z + _T[1]*p0y + _T[0]*p0x)*scale;
		if (rx> thresholdM || rx< thresholdm) {
			(++_P1)++;
			continue;
		}
		CvMyReal ry = *_P1++ - (_T[7] + _T[6]*p0z + _T[5]*p0y + _T[4]*p0x)*scale;
		if (ry>thresholdM || ry< thresholdm) {
			_P1++;
			continue;
		}

		CvMyReal rz = *_P1++ - (_T[11] + _T[10]*p0z + _T[9]*p0y + _T[8]*p0x)*scale;
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

int Cv3DPoseEstimateDisp::estimate(vector<pair<CvPoint3D64f, CvPoint3D64f> >& trackablePairs,
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

int Cv3DPoseEstimateDisp::estimateMixedPointClouds(
    CvMat *xyzs0, CvMat *uvds1,
    int numRefGrps, int refPoints[],
    CvMat *rot, CvMat *shift) {

  // convert the first point cloud, xyzs0, which is in cartesian space
  // into disparity space
  int numPoints = xyzs0->rows;

  CvMat* uvds0 = cvCreateMat(numPoints, 3, CV_64FC1);
  CvMat* xyzs1 = cvCreateMat(numPoints, 3, CV_64FC1);

  reprojection(uvds1, xyzs1);
  projection(xyzs0, uvds0);
#ifdef DEBUG
  cout << "Cv3DPoseEstimateDisp::estimateMixedPointClouds()"<<endl;
  cout << "xyzs0:"<<endl;
  CvMatUtils::printMat(xyzs0);
  cout << "uvds0"<<endl;
  CvMatUtils::printMat(uvds0);
  cout << "xyzs1"<<endl;
  CvMatUtils::printMat(xyzs1);
  cout << "uvds1"<<endl;
  CvMatUtils::printMat(uvds1);
#endif

  int numInLiers = estimate(xyzs0, xyzs1, uvds0, uvds1,
      numRefGrps, refPoints,
      rot, shift);

  cvReleaseMat(&uvds0);
  cvReleaseMat(&xyzs1);
  return numInLiers;
}

int Cv3DPoseEstimateDisp::estimate(CvMat *uvds0, CvMat *uvds1,
    CvMat *rot, CvMat *shift) {
  int numInLiers = 0;

  int numPoints = uvds0->rows;

  if (numPoints != uvds1->rows) {
    cerr << "number of points mismatched in input" << endl;
    return 0;
  }

  CvMat* xyzs0 = cvCreateMat(numPoints, 3, CV_64FC1);
  CvMat* xyzs1 = cvCreateMat(numPoints, 3, CV_64FC1);

  // project from disparity space back to XYZ
  reprojection(uvds0, xyzs0);
  reprojection(uvds1, xyzs1);

  numInLiers = estimate(xyzs0, xyzs1, uvds0, uvds1, 0, NULL, rot, shift);

  cvReleaseMat(&xyzs0);
  cvReleaseMat(&xyzs1);
  return numInLiers;
}

int Cv3DPoseEstimateDisp::estimate(CvMat *xyzs0, CvMat *xyzs1,
    CvMat *uvds0, CvMat *uvds1,
    int numRefGrps, int refPoints[],
    CvMat *rot, CvMat *shift) {
  int numPoints = xyzs0->rows;
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
        cerr << "Cannot find points that are non colinear enough"<<endl;
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

  // get the euler angle from rot
  CvPoint3D64f eulerAngles;
  {
    double _R[9], _Q[9];
    CvMat R, Q;
    CvMat *pQx=NULL, *pQy=NULL, *pQz=NULL;  // optional. For debugging.
    cvInitMatHeader(&R,  3, 3, CV_64FC1, _R);
    cvInitMatHeader(&Q,  3, 3, CV_64FC1, _Q);

#ifdef DEBUG // all debugging stuff
    double _Qx[9], _Qy[9], _Qz[9], _Qyz[9], _Q1T[9];
    CvMat Qx, Qy, Qz, Qyz, Q1T;  // optional. For debugging.
    double _Q1[9];
    CvMat Q1;
    cvInitMatHeader(&Qx, 3, 3, CV_64FC1, _Qx);
    cvInitMatHeader(&Qy, 3, 3, CV_64FC1, _Qy);
    cvInitMatHeader(&Qz, 3, 3, CV_64FC1, _Qz);
    cvInitMatHeader(&Qyz, 3, 3, CV_64FC1, _Qyz);
    cvInitMatHeader(&Q1T, 3, 3, CV_64FC1, _Q1T);
    cvInitMatHeader(&Q1, 3, 3, CV_64FC1, _Q1);
    pQx = &Qx;
    pQy = &Qy;
    pQz = &Qz;
#endif

    cvRQDecomp3x3((const CvMat*)rot, &R, &Q, pQx, pQy, pQz, &eulerAngles);

#ifdef DEBUG
    cout << "RQ decomposition rot => R, Q"<< endl;
    CvMatUtils::printMat(rot);
    CvMatUtils::printMat(&R);
    CvMatUtils::printMat(&Q);
    cout << "reconstruct rot matrices. Qx, Qy and Qz"<<endl;
    CvMatUtils::printMat(&Qx);
    CvMatUtils::printMat(&Qy);
    CvMatUtils::printMat(&Qz);
    cout <<"euler angles"<<endl;
    cout << eulerAngles.x<<","<<eulerAngles.y<<","<<eulerAngles.z<<endl;
    cvGEMM(&Qy, &Qz, 1, NULL, 0, &Qyz);
    cvGEMM(&Qx, &Qyz, 1, NULL, 0, &Q1T);
    cvTranspose(&Q1T, &Q1);
    cout <<"(Qx * Qy * Qz)^T => Q1: "<< endl;
    CvMatUtils::printMat(&Q1);
#endif
  }

  // nonlinear optimization by Levenberg-Marquardt
  cv::willow::LevMarqTransformDispSpace
  levMarq(&mMatDispToCart, &mMatCartToDisp, numInliers0);

  double param[6];

  //initialize the parameters
  param[0] = eulerAngles.x/180. * CV_PI;
  param[1] = eulerAngles.y/180. * CV_PI;
  param[2] = eulerAngles.z/180. * CV_PI;
  param[3] = cvmGet(shift, 0, 0);
  param[4] = cvmGet(shift, 1, 0);
  param[5] = cvmGet(shift, 2, 0);

#ifdef DEBUG
  printf("initial parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n",
      param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2],
      param[3], param[4], param[5]);
#endif

#ifdef    DEBUG_DISTURB_PARAM
  for (int i=0; i<6; i++) param[i] *= 1.01;
  printf("disturbed parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n",
      param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2],
      param[3], param[4], param[5]);
#endif

  CvTestTimerStart(LevMarqDoit);
  levMarq.optimize(uvds0Inlier, uvds1Inlier, param);
  CvTestTimerEnd(LevMarqDoit);

#ifdef DEBUG
  printf("optimized parameters: %f(%f), %f(%f), %f(%f), %f, %f, %f\n",
      param[0]/CV_PI*180., param[0], param[1]/CV_PI*180., param[1], param[2]/CV_PI*180., param[2],
      param[3], param[4], param[5]);
#endif

  // TODO: construct matrix with parameters from nonlinear optimization
  double _rot[9];

  CvMat3X3<double>::rotMatrix(param[0], param[1], param[2], _rot,
      CvMat3X3<double>::EulerXYZ);
  for (int i=0;i<3;i++)
    for (int j=0;j<3;j++) {
      cvmSet(rot, i, j, _rot[i*3+j]);
    }
  cvmSet(shift, 0, 0, param[3]);
  cvmSet(shift, 1, 0, param[4]);
  cvmSet(shift, 2, 0, param[5]);

  updateInlierInfo(uvds0Inlier, uvds1Inlier, inlierIndices);

  // construct the final transformation matrix
  this->constructDisparityHomography(rot, shift, &mT);
  return numInliers0;
}

bool Cv3DPoseEstimateDisp::constructDisparityHomography(const CvMat *R, const CvMat *T,
    CvMat *H){
  if (R == NULL || T == NULL) {
    return false;
  }
  return this->constructHomography(*R, *T, mMatDispToCart, mMatCartToDisp, *H);
}

bool Cv3DPoseEstimateDisp::constructHomography(const CvMat& R, const CvMat& T,
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

    constructRT((CvMat *)&R, (CvMat *)&T, &RT);

    cvMatMul(&cartToDisp, &RT, &G);
    cvMatMul(&G, (CvMat *)&dispToCart, &H);
    return status;
}

/*
 * A Convenient function to map z to d, at the optical center
 */
double Cv3DPoseEstimateDisp::getD(double z) const {
  double _xyz[] = {0., 0., z};
  double _uvd[3];
  CvMat xyz = cvMat(1, 3, CV_64FC1, _xyz);
  CvMat uvd = cvMat(1, 3, CV_64FC1, _uvd);
  projection(&xyz, &uvd);
  return _uvd[2];
}
/*
 * A convenient function to map disparity d to Z, at the optical center
 */
double Cv3DPoseEstimateDisp::getZ(double d) const {
  double _uvd[] = {this->mClx, this->mCy, d};
  double _xyz[3];
  CvMat xyz = cvMat(1, 3, CV_64FC1, _xyz);
  CvMat uvd = cvMat(1, 3, CV_64FC1, _uvd);
  reprojection(&uvd, &xyz);
  return _xyz[2];
}



