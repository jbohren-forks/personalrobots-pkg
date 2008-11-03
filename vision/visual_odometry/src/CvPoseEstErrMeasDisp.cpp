/*
 * CvPoseEstErrMeasDisp.cpp
 *
 *  Created on: Aug 18, 2008
 *      Author: jdchen
 */

#include <iostream>
using namespace std;

#include <opencv/cxcore.h>

#include "CvPoseEstErrMeasDisp.h"
#include "PoseEstimateDisp.h"
#include "CvMatUtils.h"
using namespace cv::willow;

#undef DEBUG

CvPoseEstErrMeasDisp::CvPoseEstErrMeasDisp():
	mHomography(cvMat(4, 4, CV_64FC1, _mHomography)){
}

CvPoseEstErrMeasDisp::~CvPoseEstErrMeasDisp() {
}

void CvPoseEstErrMeasDisp::transform(const CvMat& src, CvMat& dst){
	CvMat srcC3;
	CvMat dstC3;
	cvReshape(&src, &srcC3, 3, 0);
	cvReshape(&dst, &dstC3, 3, 0);
	cvPerspectiveTransform(&srcC3, &dstC3, &mHomography);
}


// compute the error in disparity space
void CvPoseEstErrMeasDisp::measure(const CvMat& uvds0, const CvMat& uvds1) {
#ifdef DEBUG
	cout << "Entering "<< __PRETTY_FUNCTION__ <<endl;
#endif

	int n = uvds0.rows;
	double _xyzs0[3*n], _xyzs1[3*n];
	CvMat xyzs0 = cvMat(n, 3, CV_64FC1, _xyzs0);
	CvMat xyzs1 = cvMat(n, 3, CV_64FC1, _xyzs1);

	dispToCart(&uvds0, &xyzs0);
	dispToCart(&uvds1, &xyzs1);

	// compare in Cartesian Space
	CvPoseEstErrMeas::compare(xyzs0, xyzs1);

	// compare in Disparity Space
	PoseEstimateDisp::constructHomography(mRotation, mShift, mat_disp_to_cart_, mat_cart_to_disp_,
			mHomography);

	double _uvds11[3*n];
	CvMat uvds11 = cvMat(n, 3, CV_64FC1, _uvds11);
	this->transform(uvds0, uvds11);

	this->compare(uvds11, uvds1);
}

void CvPoseEstErrMeasDisp::compare(const CvMat& uvds11, const CvMat& uvds1){
	cout << "entering "<<__PRETTY_FUNCTION__<<endl;
	int n = uvds1.rows; // num of points;

	CvMat uvds11_img;
	CvMat uvds1_img;
	cvReshape(&uvds11, &uvds11_img, 3, 0);
	cvReshape(&uvds1,  &uvds1_img,  3, 0);
//	double err = cvNorm(&uvds11, &uvds1, CV_L2)/n;

	double _uvsRight1[2*n];
	double _uvsRight11[2*n];
	double _uvdToUvRight[] = {
			 1., 0.,
			 0., 1.,
			-1., 0.
	};
	double _uvds1AbsDiff[3*n];
	double _uvLeftAbsDiff[2*n];
	double _uvRightAbsDiff[2*n];
	double _uvds1Diff[3*n];
	CvMat uvsLeft1;  cvGetCols(&uvds1,  &uvsLeft1,  0, 2);
	CvMat uvsLeft11; cvGetCols(&uvds11, &uvsLeft11, 0, 2);
	CvMat uvsRight1  = cvMat(n, 2, CV_64FC1, _uvsRight1);
	CvMat uvsRight11 = cvMat(n, 2, CV_64FC1, _uvsRight11);

	const CvMat uvdToUvRight = cvMat(3, 2, CV_64FC1, _uvdToUvRight);
	cvMatMul(&uvds1,   &uvdToUvRight, &uvsRight1);
	cvMatMul(&uvds11, &uvdToUvRight, &uvsRight11);

	CvMat uvds1AbsDiff   = cvMat(n, 3, CV_64F, _uvds1AbsDiff);
	CvMat uvds1Diff      = cvMat(n, 3, CV_64F, _uvds1Diff);
	CvMat uvLeftAbsDiff  = cvMat(n, 2, CV_64FC1, _uvLeftAbsDiff);
	CvMat uvRightAbsDiff = cvMat(n, 2, CV_64FC1, _uvRightAbsDiff);

	cvAbsDiff(&uvds11, &uvds1, &uvds1AbsDiff);
	cvSub(&uvds11, &uvds1, &uvds1Diff);
	cvAbsDiff(&uvsLeft11,  &uvsLeft1,  &uvLeftAbsDiff);
	cvAbsDiff(&uvsRight11, &uvsRight1, &uvRightAbsDiff);

	double _errL2 = 0;
	double errLeftL2 = 0;
	double errRightL2 = 0;
	for (int k=0; k<n; k++){
		_errL2 += cvSqrt(
				_uvds1AbsDiff[3*k  ]*_uvds1AbsDiff[3*k  ] +
				_uvds1AbsDiff[3*k+1]*_uvds1AbsDiff[3*k+1] +
				_uvds1AbsDiff[3*k+2]*_uvds1AbsDiff[3*k+2]);

		errLeftL2 += cvSqrt(
				_uvLeftAbsDiff[2*k]*_uvLeftAbsDiff[2*k] +
				_uvLeftAbsDiff[2*k+1]*_uvLeftAbsDiff[2*k+1]);
		errRightL2 += cvSqrt(
				_uvRightAbsDiff[2*k]*_uvRightAbsDiff[2*k] +
				_uvRightAbsDiff[2*k+1]*_uvRightAbsDiff[2*k+1]);
	}
	_errL2     /= n;
	errLeftL2  /= n;
	errRightL2 /= n;

//	double errL2 = cvNorm(&uvds11_img, &uvds1_img, CV_L2)/n;
	double errL1 = cvNorm(&uvds11_img, &uvds1_img, CV_L1)/n;
	double errC  = cvNorm(&uvds11_img, &uvds1_img, CV_C);

	cout << "Errors in Disparity Space"<<endl;
	cout << "Inf norm of error in pixels of all corners: " << errC   <<endl;
	cout << "Average L-1 error in pixels of all corners: " << errL1  <<endl;
	cout << "Average L-2 error in pixels of all corners: " << _errL2 <<endl;

	double errLeftL1  = cvNorm(&uvLeftAbsDiff,  NULL, CV_L1)/n;
	double errRightL1 = cvNorm(&uvRightAbsDiff, NULL, CV_L1)/n;
	double errLeftC   = cvNorm(&uvLeftAbsDiff,  NULL, CV_C);
	double errRightC  = cvNorm(&uvRightAbsDiff, NULL, CV_C);

	cout << "Errors in Screen Spaces (Left, Right)"<<endl;
	cout << "Inf norm of error in pixels of all corners: " << errLeftC  <<","<< errRightC <<endl;
	cout << "Average L-1 error in pixels of all corners: " << errLeftL1 <<","<<errRightL1 <<endl;
	cout << "Average L-2 error in pixels of all corners: " << errLeftL2 <<","<<errRightL2 << endl;

}

void CvPoseEstErrMeasDisp::measureMixed(const CvMat& xyzs0, const CvMat& uvds1) {
	cout << "entering "<<__PRETTY_FUNCTION__<<endl;
	int n = xyzs0.rows; // num of points;
	// transform xyz0 into xyz1 using the estimated rot and shift matrices
	double _xyzs1[3*n];
	double _uvds0[3*n];
	CvMat xyzs1 = cvMat(n, 3, CV_64F, _xyzs1);
	CvMat uvds0 = cvMat(n, 3, CV_64F, _uvds0);

	cout << endl;
	cout << "Convert Cartesian points to disparity and compute errors"<<endl;
	cout << endl;
	this->cartToDisp(&xyzs0, &uvds0);
	this->measure(uvds0, uvds1);

	cout << endl;
	cout << "Convert disparity  points to cartesian space and compute errors"<<endl;
	cout << endl;
	this->dispToCart(&uvds1, &xyzs1);

	this->CvPoseEstErrMeas::measure(xyzs0, xyzs1);
}

