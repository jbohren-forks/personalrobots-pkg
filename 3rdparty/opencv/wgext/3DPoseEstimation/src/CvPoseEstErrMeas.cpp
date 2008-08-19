/*
 * PoseEstErrMeas.cpp
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#include "CvPoseEstErrMeas.h"
#include <opencv/cxcore.h>

#include <iostream>
using namespace std;

CvPoseEstErrMeas::CvPoseEstErrMeas() :
	mRotation(cvMat(3, 3, CV_64FC1, _mRotData)),
	mShift(cvMat(3, 1, CV_64FC1,    _mShiftData))
{
	// TODO Auto-generated constructor stub

}

CvPoseEstErrMeas::~CvPoseEstErrMeas() {
	// TODO Auto-generated destructor stub
}

void CvPoseEstErrMeas::transform(const CvMat& src, CvMat& dst) {
	CvMat xyzs0Reshaped;
	CvMat xyzs1Reshaped;
	cvReshape(&src, &xyzs0Reshaped,  3, 0);
	cvReshape(&dst, &xyzs1Reshaped,  3, 0);
	cvTransform(&xyzs0Reshaped, &xyzs1Reshaped, &mRotation, &mShift);
}

bool CvPoseEstErrMeas::setTransform(const CvMat& rot, const CvMat& shift){
	int status = true;
	cvCopy(&rot, &mRotation);
	cvCopy(&shift, &mShift);
	return status;
}

void CvPoseEstErrMeas::measure(const CvMat& xyzs0, const CvMat& xyzs1){
	cout << "entering "<<__PRETTY_FUNCTION__<< endl;
	int n = xyzs0.rows; // number of points

	// transform xyz0 into xyz1 using the estimated rot and shift matrices
	double _xyzs11[3*n];
	CvMat xyzs11 = cvMat(n, 3, CV_64F, _xyzs11);
	// transform the point cloud of pose 0 to pose 1
	transform(xyzs0, xyzs11);
	this->compare(xyzs11, xyzs1);
}

void CvPoseEstErrMeas::compare(const CvMat& xyzs11, const CvMat& xyzs1){

	cout << "entering "<< __PRETTY_FUNCTION__ << endl;

	int n = xyzs1.rows; // number of points

	// now compute the diff
//	double err3DL2 = cvNorm(&xyzs11, &xyzs1, CV_L2)/n;
	double err3DL1 = cvNorm(&xyzs11, &xyzs1, CV_L1)/n;
	double err3DC  = cvNorm(&xyzs11, &xyzs1, CV_C);

#if 0
	cout << "Errors in Cartesian Space"<<endl;
	cout << "Inf norm of error in mm of all corners: " << err3DC <<endl;
	cout << "Average L-1 error in mm of all corners: " << err3DL1 <<endl;
	cout << "Average L-2 error in mm of all corners: " << err3DL2 <<endl;
#endif

	double _xyzs1AbsDiff[3*n];
	double _xyzs1Diff[3*n];
	CvMat xyzs1AbsDiff = cvMat(n, 3, CV_64F, _xyzs1AbsDiff);
	CvMat xyzs1Diff = cvMat(n, 3, CV_64F, _xyzs1Diff);
	cvAbsDiff(&xyzs11, &xyzs1, &xyzs1AbsDiff);
	// compute the diff vectors from xyzs1 to xyzs11
	cvSub(&xyzs11, &xyzs1, &xyzs1Diff);

	double _err3DL2 = 0;
	for (int k=0; k<n; k++){
		_err3DL2 += cvSqrt(
		_xyzs1AbsDiff[3*k  ]*_xyzs1AbsDiff[3*k  ] +
		_xyzs1AbsDiff[3*k+1]*_xyzs1AbsDiff[3*k+1] +
		_xyzs1AbsDiff[3*k+2]*_xyzs1AbsDiff[3*k+2]);
	}
	_err3DL2 /= n;
//	CvScalar sum = cvSum(&xyzs1AbsDiff);
//	double _err3DL1 = (sum.val[0]+sum.val[1]+sum.val[2])/n;
//	double _err3DC  = cvNorm(&xyzs1AbsDiff, NULL, CV_C);

	cout << "Errors  in Cartesian Space"<<endl;
	cout << "Inf norm of error in mm of all corners: " << err3DC <<endl;
	cout << "Average L-1 error in mm of all corners: " << err3DL1 <<endl;
	cout << "Average L-2 error in mm of all corners: " << _err3DL2 <<endl;

	// measure errors in x-y and z (depth) separately, as the lrf tends to have larger error
	// in depth
	CvMat xy1AbsDiff;
	CvMat z1AbsDiff;

	cvGetCols(&xyzs1AbsDiff, &xy1AbsDiff, 0, 1);
	cvGetCol(&xyzs1AbsDiff,  &z1AbsDiff,  2);

	double errC_3DXY = cvNorm(&xy1AbsDiff, NULL, CV_C);
	double errC_3DZ  = cvNorm(&z1AbsDiff,  NULL, CV_C);

	double errL1_3DXY = cvNorm(&xy1AbsDiff, NULL, CV_L1)/n;
	double errL1_3DZ  = cvNorm(&z1AbsDiff, NULL, CV_L1)/n;

	double errL2_3DXY = 0;
	double errL2_3DZ  = 0;
	for (int k=0; k<n; k++){
		errL2_3DXY += cvSqrt(
		_xyzs1AbsDiff[3*k  ]*_xyzs1AbsDiff[3*k  ] +
		_xyzs1AbsDiff[3*k+1]*_xyzs1AbsDiff[3*k+1]);

		errL2_3DZ += _xyzs1AbsDiff[3*k+2];
	}
	errL2_3DXY /= n;
	errL2_3DZ  /= n;

	cout << "Errors  in Cartesian Space (measured separately in X-Y and Z"<<endl;
	cout << "Inf norm of error in mm of all corners: " << errC_3DXY << ","<<errC_3DZ <<endl;
	cout << "Average L-1 error in mm of all corners: " << errL1_3DXY << ","<<errL1_3DZ <<endl;
	cout << "Average L-2 error in mm of all corners: " << errL2_3DXY << ","<<errL2_3DZ <<endl;

	// measure bias in cartesian space
	// simply compute the average of the error vectors of each data point
	CvMat xyzs1Diff_3c;
	cvReshape(&xyzs1Diff, &xyzs1Diff_3c, 3, 0);
	CvScalar avgErrorVector = cvAvg(&xyzs1Diff_3c);

	cout << "Average error vector:"<<avgErrorVector.val[0]<<","<<avgErrorVector.val[1]<<","<<avgErrorVector.val[2]<<endl;

}


